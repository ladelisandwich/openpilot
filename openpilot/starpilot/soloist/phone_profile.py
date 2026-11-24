"""Temporary AGNOS BlueALSA receiving profile. Invoked as root with a fixed action.

Only a feature-owned /run drop-in is changed; Bluetooth power, bonds, bluetoothd,
HFP and the on-disk service definition are untouched. Unknown service commands
are rejected rather than replaced. The override disappears on reboot.
"""
from __future__ import annotations

import fcntl
from pathlib import Path
import subprocess
import sys

if __package__:
  from . import bluealsa_runtime
else:
  import bluealsa_runtime

DROP_IN = Path('/run/systemd/system/bluealsa.service.d/90-starpilot-phone.conf')
# BlueALSA is tiny but must drain the Bluetooth socket on time; at nice 0 it competes with driving
# processes on busy cores and the music chain (nice 10) cannot make up for lost incoming packets.
LEGACY_CONTENTS = '[Service]\nExecStart=\nExecStart=/usr/bin/bluealsa -p a2dp-source -p a2dp-sink\n'
CONTENTS = LEGACY_CONTENTS + 'Nice=-5\n'


def systemctl(*args):
  return subprocess.run(['/usr/bin/systemctl', *args], check=True, capture_output=True, text=True, timeout=15).stdout.strip()


def configure(action: str, path: Path = DROP_IN, run=systemctl) -> None:
  if action not in ('acquire', 'release'):
    raise ValueError('Unknown profile action')
  previous = path.read_text() if path.exists() else None
  if previous is not None and previous not in (CONTENTS, LEGACY_CONTENTS):
    raise RuntimeError('Phone audio profile override was changed externally')
  bluealsa_runtime.check_owned(path)
  override, _ = bluealsa_runtime.paths(path)
  if action == 'release':
    if previous is not None or override.exists():
      bluealsa_runtime.remove(path)
      path.unlink(missing_ok=True)
      run('daemon-reload')
      run('try-restart', 'bluealsa.service')
    return
  bluealsa_runtime.validate()
  if previous == CONTENTS and override.exists():
    run('start', 'bluealsa.service')
    return
  if previous is None:
    command = run('show', '--property=ExecStart', '--value', 'bluealsa.service')
    if not any(f'argv[]=/usr/bin/bluealsa {profiles} ;' in command for profiles in
               ('-p a2dp-source', '-p a2dp-source -p a2dp-sink')):
      raise RuntimeError('Unsupported BlueALSA service configuration; phone audio was not enabled')
  path.parent.mkdir(parents=True, exist_ok=True)
  try:
    # Keep this file byte-for-byte compatible with old commits recovery code.
    path.write_text(CONTENTS)
    bluealsa_runtime.install(path)
    run('daemon-reload')
    run('restart', 'bluealsa.service')
  except Exception:
    bluealsa_runtime.remove(path)
    if previous is None:
      path.unlink(missing_ok=True)
    else:
      path.write_text(previous)
    run('daemon-reload')
    run('try-restart', 'bluealsa.service')
    raise


if __name__ == '__main__':
  # Serializes recovery and normal shutdown; never takes the adapter's pairing lease.
  with open('/run/starpilot-phone-profile.lock', 'w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    configure(sys.argv[1])
