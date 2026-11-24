"""A bounded replay MP4, separate from the seekable preview's rolling cache."""

import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import stat
import subprocess
import uuid

from openpilot.starpilot.replay.media import MediaMux

BITRATE_KBPS = 4000
MAX_EXPORT_BYTES = 32 * 1024 ** 3
EXPORT_TTL = 30 * 60
MAX_FRAME_GAP = .5


def disk_budget(directory: Path) -> int:
  usage = shutil.disk_usage(directory)
  reserve = max(5 * 1024 ** 3, math.ceil(usage.total * .1))
  return max(0, min(MAX_EXPORT_BYTES, (usage.free - reserve) // 2))


def export_ranges(status: dict, segment: int | None) -> list[tuple[float, float]]:
  rows = status.get('segmentRanges')
  if not rows:
    raise ValueError('Replay engine needs an update before exporting')
  ranges = []
  found = set()
  previous_end = status['start']
  for row in rows:
    if (type(row) is not list or len(row) != 3 or type(row[0]) is not int or row[0] in found or
        not all(type(v) in (int, float) and math.isfinite(v) for v in row[1:]) or
        not status['start'] <= row[1] < row[2] <= status['end'] or row[1] < previous_end):
      raise ValueError('Invalid recorded segment ranges')
    found.add(row[0])
    previous_end = row[2]
    if segment is None or segment == row[0]:
      begin, end = row[1:]
      if ranges and abs(ranges[-1][1] - begin) < 1e-6:
        ranges[-1] = ranges[-1][0], end
      else:
        ranges.append((begin, end))
  if not ranges:
    raise ValueError('Selected segment has no replayable coverage')
  return ranges


class ExportVideo:
  def __init__(self, directory: Path, width: int, height: int, ranges, budget: int):
    self.directory, self.width, self.height = Path(directory), width, height
    self.ranges, self.budget = ranges, budget
    self.duration = sum(end - begin for begin, end in ranges)
    self.index = 0
    self.elapsed = 0.
    self.frames = self.written = self.disk_check_at = 0
    self.pending = self.mux = None
    self.seen = self.running = self.pausing = False
    self.first_position = self.last_position = None
    self.file = (self.directory / 'export.part').open('xb')
    os.chmod(self.directory / 'export.part', 0o600)

  @property
  def progress(self):
    current = 0 if self.last_position is None else max(0., self.last_position - self.ranges[self.index][0])
    return min(1., (self.elapsed + current) / self.duration)

  def write(self, data):
    if self.written + len(data) > self.budget:
      raise OSError('Replay exceeded its export storage budget')
    if self.written >= self.disk_check_at:
      if disk_budget(self.directory) <= 0:
        raise OSError('Not enough free recording storage for this export')
      self.disk_check_at = self.written + 4 * 1024 ** 2
    self.file.write(data)
    self.written += len(data)

  def frame(self, packet: bytes, *, position: float, keyframe: bool):
    begin, end = self.ranges[self.index]
    if position < begin - 1e-6 or position >= end:
      return
    if not self.seen:
      if not keyframe or position - begin > MAX_FRAME_GAP:
        raise ValueError('Replay could not render the beginning of the selected coverage')
      self.first_position = position
      position = begin
      self.seen = True
    if self.last_position is not None and position <= self.last_position:
      return
    if self.last_position is not None and position - self.last_position > MAX_FRAME_GAP:
      raise ValueError('Replay lost video coverage; export was not completed')
    pts = round((self.elapsed + position - begin) * 1e6)
    if self.mux is None:
      self.mux = MediaMux(self.width, self.height, packet)
      self.write(self.mux.init)
    if self.pending is not None:
      self.flush(pts)
    self.pending = packet, pts, keyframe
    self.last_position = position
    self.frames += 1

  def flush(self, end_us):
    if self.pending is None:
      raise ValueError('Replay export is missing its final frame')
    packet, pts, keyframe = self.pending
    duration = end_us - pts
    if not 0 < duration <= round(MAX_FRAME_GAP * 1e6):
      raise ValueError('Replay export has incomplete video coverage')
    self.write(self.mux.frame(packet, pts_us=pts, duration_us=duration, keyframe=keyframe))
    self.pending = None

  def advance(self, engine, status, readback_pending: bool) -> bool:
    begin, end = self.ranges[self.index]
    boundary = status['position'] >= end - 1e-6 or (status.get('state') == 'ended' and end == status['end'])
    if self.seen and not self.running and not boundary:
      engine.command({'op': 'pause', 'paused': False, 'requestId': uuid.uuid4().hex})
      self.running = True
    if not boundary:
      return False
    if not status['paused']:
      if not self.pausing:
        engine.command({'op': 'pause', 'paused': True, 'requestId': uuid.uuid4().hex})
        self.pausing = True
      return False
    if readback_pending:
      return False
    if not self.seen or end - self.last_position > MAX_FRAME_GAP:
      raise ValueError('Replay could not render the end of the selected coverage')
    self.elapsed += end - begin
    self.flush(round(self.elapsed * 1e6))
    if self.index + 1 == len(self.ranges):
      return True
    self.index += 1
    self.seen = self.running = self.pausing = False
    self.first_position = self.last_position = None
    engine.command({'op': 'seek', 'position': self.ranges[self.index][0], 'requestId': uuid.uuid4().hex})
    return False

  def finalize(self):
    from openpilot.starpilot.galaxy.recording_media import _ffmpeg_binary
    self.file.flush()
    os.fsync(self.file.fileno())
    self.file.close()
    usage = shutil.disk_usage(self.directory)
    if usage.free - self.written < max(5 * 1024 ** 3, math.ceil(usage.total * .1)):
      raise OSError('Not enough free storage to finalize replay')
    binary = _ffmpeg_binary()
    output = self.directory / 'export.mp4'
    timeout = max(30, self.written / (16 * 1024 ** 2) + 10)
    subprocess.run([str(binary), '-nostdin', '-hide_banner', '-loglevel', 'error', '-i', str(self.directory / 'export.part'),
                    '-map', '0:v:0', '-an', '-c:v', 'copy', '-movflags', '+faststart', '-f', 'mp4', str(output)],
                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=timeout, check=True)
    os.chmod(output, 0o600)
    probe = subprocess.run([str(binary.with_name('ffprobe')), '-v', 'error', '-show_entries',
                            'stream=codec_name,nb_frames:format=duration', '-of', 'json', str(output)],
                           stdin=subprocess.DEVNULL, capture_output=True, timeout=10, check=True)
    if len(probe.stdout) > 8192:
      raise ValueError('Invalid replay export metadata')
    info = json.loads(probe.stdout)
    streams = info.get('streams', [])
    if (len(streams) != 1 or streams[0].get('codec_name') != 'h264' or int(streams[0].get('nb_frames', 0)) != self.frames or
        abs(float(info.get('format', {}).get('duration', 0)) - self.duration) > .1 or
        not 0 < output.stat().st_size <= self.budget):
      raise ValueError('Replay export did not contain the complete requested video')
    (self.directory / 'export.part').unlink()

  def close(self):
    self.file.close()
    if self.mux is not None:
      self.mux.close()


class ExportSource:
  """A finished private file; no source-recording lease survives rendering."""
  def __init__(self, root: Path, export_id: str, size: int):
    self.fds = []
    try:
      parent = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
      self.fds.append(parent)
      parent = os.open('exports', os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent)
      self.fds.append(parent)
      parent = os.open(export_id, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent)
      self.fds.append(parent)
      for fd in self.fds:
        info = os.fstat(fd)
        if info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o700:
          raise ValueError('Replay export directory changed')
      self.fd = os.open('replay.mp4', os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent)
      self.fds.append(self.fd)
      fcntl.flock(self.fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
      info = os.fstat(self.fd)
      if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o600 or info.st_size != size:
        raise ValueError('Replay export file changed')
      self.identity = info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns
      self.parent = parent
    except BaseException:
      self.close()
      raise

  def current(self):
    try:
      info = os.stat('replay.mp4', dir_fd=self.parent, follow_symlinks=False)
      return self.identity == (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns)
    except OSError:
      return False

  def close(self):
    for fd in reversed(self.fds):
      os.close(fd)
    self.fds.clear()
