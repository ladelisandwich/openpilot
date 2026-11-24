"""Private bounded RAM capture; no disk writes while producers are timed."""

from pathlib import Path
import time


class BoundedCapture:
  def __init__(self, path, maximum_bytes=128 * 1024 * 1024):
    self.path = Path(path)
    assert not self.path.exists() and 1 <= maximum_bytes <= 256 * 1024 * 1024
    self.storage = bytearray(maximum_bytes)
    self.used = 0
    self.export_ns = None

  def __enter__(self):
    return self

  def __exit__(self, *_):
    return False

  def write(self, value):
    size = len(value)
    assert self.used + size <= len(self.storage), 'Bounded raw capture full; no drop/truncation permitted'
    self.storage[self.used : self.used + size] = value
    self.used += size

  def export(self):
    # Caller must reap all producers first. A slow export invalidates wrapper
    # completion, but cannot delay CAN/peer producers or timed IPC observations.
    start = time.monotonic_ns()
    with self.path.open('xb') as stream:
      view = memoryview(self.storage)
      for offset in range(0, self.used, 1024 * 1024):
        stream.write(view[offset : min(offset + 1024 * 1024, self.used)])
    self.export_ns = time.monotonic_ns() - start
    return {'bytes': self.used, 'capacityBytes': len(self.storage), 'postReapExportNs': self.export_ns, 'timedDiskWrites': False, 'drops': 0}
