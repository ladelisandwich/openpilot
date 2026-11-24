"""Pure diagnostic wiring selection; no vehicle identity or control authority."""
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest

# Load this pure production function without initializing native car bindings.
# The normal discovery caller uses this same source; no substitute is supplied.
discovery = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'discovery.py'))
normal_bus_topology = discovery['normal_bus_topology']
FRAMES = (
  (0, 593, '700361a6008cc77f'),
  (0, 902, '4cc749475a475f47'),
  (0, 916, '71a4000071041d83'),
  (0, 1056, '401460cadc916b00'),
  (0, 1057, '000000ffe37f2126'),
  (0, 1265, 'e04666d0'),
  (2, 832, '000000246400a018'),
)


def frames():
  return [SimpleNamespace(src=bus, address=address, dat=bytes.fromhex(data)) for bus, address, data in FRAMES]


class TestClassicNormalDiscovery(unittest.TestCase):
  def test_recorded_normal_wiring_layout_selects_queries(self):
    self.assertTrue(normal_bus_topology(lambda **kwargs: [frames()]))

  def test_each_required_source_rejects_missing_length_bus_and_echo(self):
    for index in range(len(FRAMES)):
      for defect in ('missing', 'length', 'bus', 'echo'):
        with self.subTest(index=index, defect=defect):
          packets = frames()
          frame = packets.pop(index)
          if defect != 'missing':
            packets.append(SimpleNamespace(src=frame.src + (128 if defect == 'echo' else 1) if defect != 'length' else frame.src,
                                           address=frame.address, dat=frame.dat[:-1] if defect == 'length' else frame.dat))
          self.assertFalse(normal_bus_topology(lambda packets=packets, **kwargs: [packets]))

  def test_existing_canfd_layout_and_bounded_reads(self):
    layout = discovery['NORMAL_BUS_LAYOUT']
    packets = [SimpleNamespace(src=bus, address=address, dat=bytes(length)) for (bus, address), length in layout.items()]
    self.assertTrue(normal_bus_topology(lambda packets=packets, **kwargs: [packets]))
    for missing in layout:
      with self.subTest(missing=missing):
        subset = [frame for frame in packets if (frame.src, frame.address) != missing]
        self.assertFalse(normal_bus_topology(lambda subset=subset, **kwargs: [subset]))
    calls = []
    self.assertFalse(normal_bus_topology(lambda **kwargs: calls.append(kwargs) or [], clock=lambda: 1.))
    self.assertEqual(len(calls), 24)
    times = iter((1., 1.13))
    self.assertFalse(normal_bus_topology(lambda **kwargs: [], clock=lambda: next(times)))


if __name__ == '__main__':
  unittest.main()
