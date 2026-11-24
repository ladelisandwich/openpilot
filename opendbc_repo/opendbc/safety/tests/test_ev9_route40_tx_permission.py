"""Recorded ordinary engagement must use its admitted combined-axis permission."""
import json
from pathlib import Path
import pytest
from opendbc.safety.tests.libsafety import libsafety_py

CAPTURE = Path(__file__).parent / 'testdata/ev9_route40_active_cb.json'


def ready(param=0x5c95, experience=32):
  safety = libsafety_py.libsafety
  safety.init_tests()
  safety.set_alternative_experience(experience)
  assert safety.set_safety_hooks(28, param) == 0
  first = None
  active = []
  for row in json.loads(CAPTURE.read_text())['rows']:
    stamp = int(row['t'] * 1e6)
    safety.set_timer(stamp)
    mask = 3 if row['t'] >= 92.48 else 0
    safety.aol_set_host_request(mask)
    safety.set_aol_test_heartbeat(bool(mask))
    for frame in row['f']:
      packet = libsafety_py.make_CANPacket(frame['a'], frame['b'], bytes.fromhex(frame['d']))
      if row['k'] == 'can':
        assert safety.safety_rx_hook(packet)
      elif packet[0].data[3] >> 4 == 2:
        first = first or (stamp, frame)
        active.append((stamp, frame))
        # The first active packet is tested separately before any active TX.
        if len(active) == 1:
          assert safety.get_controls_allowed()
          assert safety.aol_get_permission_mask() == (3 if experience else 0)
          return safety, first
      else:
        safety.safety_tx_hook(packet)
  raise AssertionError('No captured active CB')


@pytest.mark.parametrize('param', [0x5c95, 0x5495])
def test_captured_ev9_ordinary_cb_uses_advertised_combined_permission(param):
  safety, (_, frame) = ready(param)
  packet = libsafety_py.make_CANPacket(frame['a'], frame['b'], bytes.fromhex(frame['d']))
  assert safety.safety_tx_hook(packet), 'PERMITTED_EV9_CB_MUST_REACH_EPS'
  assert safety.safety_test_selected_fwd(2, 0x110)
  assert safety.safety_test_selected_fwd(2, 0x362)
  assert not safety.safety_tx_hook(packet), 'Duplicate counter still rejected'
  assert not safety.safety_test_selected_fwd(2, 0x110)


@pytest.mark.parametrize('param', [0x5c95, 0x5495])
@pytest.mark.parametrize('fault', ['no_request', 'no_heartbeat', 'ordinary_disabled', 'stale_sources', 'bad_crc', 'wrong_bus'])
def test_captured_ev9_combined_permission_keeps_all_refusals(fault, param):
  safety, (stamp, frame) = ready(param, experience=0 if fault == 'ordinary_disabled' else 32)
  data = bytearray.fromhex(frame['d'])
  bus = frame['b']
  if fault == 'no_request':
    safety.aol_set_host_request(0)
  elif fault == 'no_heartbeat':
    safety.set_aol_test_heartbeat(False)
  elif fault == 'ordinary_disabled':
    # With independent AOL disabled, ordinary disengagement must refuse TX.
    safety.set_controls_allowed(False)
  elif fault == 'stale_sources':
    safety.set_timer(stamp + 100_001)
  elif fault == 'bad_crc':
    data[0] ^= 1
  elif fault == 'wrong_bus':
    bus = 0
  assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(frame['a'], bus, data))


@pytest.mark.parametrize('param', [0x5c95, 0x5495])
def test_captured_physically_armed_aol_retains_lateral_without_ordinary_controls(param):
  safety, (_, frame) = ready(param)
  safety.set_controls_allowed(False)
  assert not safety.get_controls_allowed()
  assert safety.aol_get_permission_mask() == 1
  packet = libsafety_py.make_CANPacket(frame['a'], frame['b'], bytes.fromhex(frame['d']))
  assert safety.safety_tx_hook(packet)
  assert not safety.safety_tx_hook(packet), 'Duplicate counter still rejected'


@pytest.mark.parametrize('param', [0x5c95, 0x5495])
def test_captured_ordinary_engagement_without_independent_aol_can_transmit(param):
  safety, (_, frame) = ready(param, experience=0)
  packet = libsafety_py.make_CANPacket(frame['a'], frame['b'], bytes.fromhex(frame['d']))
  assert safety.safety_tx_hook(packet)
  safety.set_controls_allowed(False)
  assert not safety.safety_tx_hook(packet)
