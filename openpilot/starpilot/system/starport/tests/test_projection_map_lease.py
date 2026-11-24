import os
import tempfile
import unittest

from openpilot.starpilot.system.starport.projection_map_lease import MAX_AGE_S, MapLeaseWriter, projection_map_active


class MapLeaseTests(unittest.TestCase):
  def test_lease_is_active_only_while_fresh_and_ends_on_release(self):
    with tempfile.TemporaryDirectory() as directory:
      path = os.path.join(directory, 'lease')
      writer = MapLeaseWriter(path)
      self.assertFalse(projection_map_active(path))
      writer.touch(now=100.0)
      self.assertTrue(projection_map_active(path))
      mtime = os.stat(path).st_mtime
      self.assertTrue(projection_map_active(path, now=mtime + MAX_AGE_S))
      self.assertFalse(projection_map_active(path, now=mtime + MAX_AGE_S + 1))
      writer.release()
      self.assertFalse(projection_map_active(path))
      writer.release()

  def test_touch_is_throttled(self):
    with tempfile.TemporaryDirectory() as directory:
      path = os.path.join(directory, 'lease')
      writer = MapLeaseWriter(path)
      writer.touch(now=10.0)
      os.utime(path, (1, 1))
      writer.touch(now=10.5)
      self.assertEqual(os.stat(path).st_mtime, 1)
      writer.touch(now=11.5)
      self.assertGreater(os.stat(path).st_mtime, 1)
