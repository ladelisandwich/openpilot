import json
from pathlib import Path
import unittest
from unittest.mock import patch

from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import fingerprint
from opendbc.car.hyundai.discovery import NORMAL_BUS_LAYOUT, normal_bus_topology
from opendbc.car.vin import VIN_UNKNOWN
from opendbc.car import fw_versions


def recorded_frames():
  fixture = json.loads((Path(__file__).parent / 'testdata/ioniq5pe_discovery.json').read_text())
  return [CanData(f['address'], bytes.fromhex(f['data']), f['bus']) for f in fixture['frames']]


class TestPEDiscovery(unittest.TestCase):
  def test_recorded_live_layout_uses_normal_wiring(self):
    self.assertTrue(normal_bus_topology(lambda **kw: [recorded_frames()]))

  def test_incomplete_layout_retains_generic_discovery(self):
    for missing in NORMAL_BUS_LAYOUT:
      with self.subTest(missing=missing):
        frames = [f for f in recorded_frames() if (f.src, f.address) != missing]
        self.assertFalse(normal_bus_topology(lambda frames=frames, **kw: [frames]))

  def test_invalid_layout_cannot_select_query_family(self):
    for bad in ('bus', 'length', 'echo'):
      with self.subTest(bad=bad):
        frames = recorded_frames()
        frame = frames[0]
        replacement = CanData(frame.address, frame.dat[:-1] if bad == 'length' else frame.dat,
                              frame.src + (128 if bad == 'echo' else 1) if bad != 'length' else frame.src)
        self.assertFalse(normal_bus_topology(lambda replacement=replacement, frames=frames, **kw: [[replacement, *frames[1:]]]))

  def test_probe_has_bounded_reads(self):
    calls = []
    self.assertFalse(normal_bus_topology(lambda **kw: calls.append(kw) or [], clock=lambda: 1.))
    self.assertEqual(len(calls), 24)

  def test_fresh_firmware_still_owns_identity_and_no_obd_switch_occurs(self):
    switches = []
    with patch('opendbc.car.car_helpers.get_vin', return_value=(-1, -1, VIN_UNKNOWN)), \
         patch('opendbc.car.car_helpers.get_fw_versions', return_value=[]) as query, \
         patch('opendbc.car.car_helpers.get_present_ecus') as present, \
         patch('opendbc.car.car_helpers.match_fw_to_car', return_value=(True, set())), \
         patch('opendbc.car.car_helpers.can_fingerprint', return_value=(None, {})):
      actual = fingerprint(lambda **kw: [recorded_frames()], lambda frames: None, switches.append, None)
    self.assertIsNone(actual[0])
    self.assertTrue(switches)
    self.assertFalse(any(switches))
    query.assert_called_once()
    self.assertEqual(query.call_args.kwargs, {'query_brand': 'hyundai', 'normal_bus_only': True})
    present.assert_not_called()

  def test_normal_query_filter_never_executes_obd_request(self):
    switches = []
    buses = []

    class Query:
      def __init__(self, send, recv, bus, *args):
        buses.append(bus)

      def get_data(self, timeout):
        return {}

    with patch.object(fw_versions, 'IsoTpParallelQuery', Query):
      fw_versions.get_fw_versions(lambda **kw: [], lambda frames: None, switches.append,
                                  query_brand='hyundai', normal_bus_only=True)
    self.assertTrue({0, 1} <= set(buses))
    self.assertTrue(switches)
    self.assertFalse(any(switches))

  def test_other_layouts_keep_original_vin_and_firmware_discovery(self):
    switches = []
    with patch('opendbc.car.car_helpers.normal_bus_topology', return_value=False), \
         patch('opendbc.car.car_helpers.get_vin', return_value=(-1, -1, VIN_UNKNOWN)) as vin, \
         patch('opendbc.car.car_helpers.get_fw_versions_ordered', return_value=[]) as query, \
         patch('opendbc.car.car_helpers.get_present_ecus', return_value=set()) as present, \
         patch('opendbc.car.car_helpers.match_fw_to_car', return_value=(True, set())), \
         patch('opendbc.car.car_helpers.can_fingerprint', return_value=(None, {})):
      fingerprint(lambda **kw: [], lambda frames: None, switches.append, None)
    self.assertEqual(switches, [True, False])
    self.assertEqual(vin.call_args.args[2], (0, 1))
    present.assert_called_once()
    query.assert_called_once()

  def test_deadline_stops_probe_without_waiting_for_all_reads(self):
    calls = []
    times = iter((1., 1.13))
    self.assertFalse(normal_bus_topology(lambda **kw: calls.append(kw) or [], clock=lambda: next(times)))
    self.assertEqual(calls, [])
