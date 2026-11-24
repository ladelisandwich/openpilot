"""Private exact-created namespace cleanup, only after caller reaps all actors."""

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import time


def atomic(path, value):
  path = Path(path)
  temporary = path.with_name(path.name + '.tmp-' + str(os.getpid()))
  with temporary.open('x') as stream:
    stream.write(json.dumps(value, indent=2) + '\n')
  os.replace(temporary, path)


def journal_created(path, prefix, destination):
  path = Path(path)
  assert re.fullmatch(r'replay-bench-(?:camera-)?[0-9a-f]{12}', prefix)
  assert path == Path('/dev/shm') / ('msgq_' + prefix)
  info = path.lstat()
  assert stat.S_ISDIR(info.st_mode) and info.st_uid == 0
  fields = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
  value = {
    'path': str(path),
    'prefix': prefix,
    'device': info.st_dev,
    'inode': info.st_ino,
    'creatorPid': os.getpid(),
    'creatorStartTicks': int(fields[19]),
    'createdMonoNs': time.monotonic_ns(),
    'bootId': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
    'leaseTokenSha256': hashlib.sha256(os.environ['SP_PRIVATE_IRQ_LEASE_TOKEN'].encode()).hexdigest(),
  }
  assert not Path(destination).exists()
  atomic(destination, value)


def identity(pid):
  try:
    return int(Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19])
  except (FileNotFoundError, ProcessLookupError):
    return None


def no_users(path, prefix, owned_inodes):
  users, inspected = [], 0
  for proc in Path('/proc').iterdir():
    if not proc.name.isdecimal():
      continue
    pid = int(proc.name)
    start = identity(pid)
    if start is None:
      continue
    try:
      environ = (proc / 'environ').read_bytes().split(b'\0')
      if ('OPENPILOT_PREFIX=' + prefix).encode() in environ:
        users.append({'pid': pid, 'startTicks': start, 'kind': 'environment'})
      for line in (proc / 'maps').read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) == 6 and (fields[5] == str(path) or fields[5].startswith(str(path) + '/')):
          users.append({'pid': pid, 'startTicks': start, 'kind': 'mapping'})
      for fd in (proc / 'fd').iterdir():
        try:
          target = os.readlink(fd)
          info = fd.stat()
        except FileNotFoundError:
          continue  # descriptor closed during scan
        if target == str(path) or target.startswith(str(path) + '/') or (info.st_dev, info.st_ino) in owned_inodes:
          users.append({'pid': pid, 'startTicks': start, 'kind': 'descriptor'})
      assert identity(pid) == start or identity(pid) is None, 'PID changed while scanning'
      inspected += 1
    except (FileNotFoundError, ProcessLookupError):
      assert identity(pid) is None, 'Live process incompletely scanned'
  return {'monoNs': time.monotonic_ns(), 'inspectedProcesses': inspected, 'users': users}


def cleanup(journal, receipt):
  result = {'removed': False, 'failure': None, 'scans': []}
  try:
    value = json.loads(Path(journal).read_text())
    result['ownership'] = value
    assert value['bootId'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    assert value['leaseTokenSha256'] == hashlib.sha256(os.environ['SP_PRIVATE_IRQ_LEASE_TOKEN'].encode()).hexdigest()
    prefix, path = value['prefix'], Path(value['path'])
    assert re.fullmatch(r'replay-bench-(?:camera-)?[0-9a-f]{12}', prefix)
    assert path == Path('/dev/shm') / ('msgq_' + prefix)
    info = path.lstat()
    assert stat.S_ISDIR(info.st_mode) and info.st_uid == 0
    assert (info.st_dev, info.st_ino) == (value['device'], value['inode'])
    children = list(path.iterdir())
    states = {child.name: child.lstat() for child in children}
    assert all(stat.S_ISREG(item.st_mode) and item.st_uid == 0 and item.st_dev == info.st_dev for item in states.values())
    inodes = {(item.st_dev, item.st_ino) for item in states.values()} | {(info.st_dev, info.st_ino)}
    result['allocatedBytesBefore'] = sum(item.st_blocks * 512 for item in states.values())
    for _ in range(2):
      scan = no_users(path, prefix, inodes)
      result['scans'].append(scan)
      assert not scan['users'], 'Namespace has active users'
      time.sleep(0.05)
    current = path.lstat()
    assert (current.st_dev, current.st_ino) == (info.st_dev, info.st_ino)
    assert {child.name: (child.lstat().st_dev, child.lstat().st_ino) for child in path.iterdir()} == {
      name: (item.st_dev, item.st_ino) for name, item in states.items()
    }
    assert shutil.rmtree.avoids_symlink_attacks
    shutil.rmtree(path)
    assert not path.exists()
    result['removed'] = True
  except BaseException as error:
    result['failure'] = type(error).__name__ + ': ' + str(error)
    raise
  finally:
    atomic(receipt, result)
