"""Lets the native UI drop its map while Android Auto is drawing one; both need GPS and GPU tile work."""
import os
import time

DEFAULT_PATH = '/tmp/starpilot-projection-map.lease'
MAX_AGE_S = 3.0
TOUCH_EVERY_S = 1.0


class MapLeaseWriter:
  def __init__(self, path=DEFAULT_PATH):
    self.path = str(path)
    self._touched = -TOUCH_EVERY_S

  def touch(self, now=None):
    now = time.monotonic() if now is None else now
    if now - self._touched < TOUCH_EVERY_S:
      return
    self._touched = now
    try:
      with open(self.path, 'a'):
        os.utime(self.path)
    except OSError:
      pass

  def release(self):
    self._touched = -TOUCH_EVERY_S
    try:
      os.unlink(self.path)
    except OSError:
      pass


def projection_map_active(path=DEFAULT_PATH, now=None):
  try:
    # Filesystem mtimes use the realtime clock; monotonic time has a different epoch.
    age = (time.time() if now is None else now) - os.stat(path).st_mtime  # noqa: TID251
  except OSError:
    return False
  return 0 <= age <= MAX_AGE_S
