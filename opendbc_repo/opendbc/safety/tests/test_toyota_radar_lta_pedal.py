"""Genuine native LTA/pedal checksum, counter, fault and warm-switch checks."""

import pytest
from opendbc.car import structs
from opendbc.car.toyota import pedal
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_toyota_angle_aol import Physical
from openpilot.starpilot.car.toyota.tests import test_pedal_caller_native as helpers

source_matched_native = helpers.source_matched_native


@pytest.mark.parametrize('word', (0x2449, 0x2649))
@pytest.mark.parametrize('fault', ('checksum', 'counter', 'state', 'expiry', 'main', 'eps'))
def test_lta_pedal_independent_sources_refuse_and_recover(word, fault):
  case = Physical(word)
  counter = 0
  safety = case.native

  def tick(bad=None):
    nonlocal counter
    case.tick(lta=9 if bad == 'eps' else 1, skip_main=bad == 'main')
    a, d, b = case.packer.make_can_msg('PCM_CRUISE', 0, {'CRUISE_ACTIVE': 1, 'GAS_RELEASED': 1})
    safety.safety_rx_hook(libsafety_py.make_CANPacket(a, b, d))
    if bad != 'expiry':
      a, d, b = case.packer.make_can_msg('GAS_SENSOR', 0, {'STATE': 1 if bad == 'state' else 0, 'COUNTER_PEDAL': 0 if bad == 'counter' else counter})
      counter = (counter + 1) & 15
      d = bytearray(d)
      d[-1] = pedal.checksum(d) ^ int(bad == 'checksum')
      safety.safety_rx_hook(libsafety_py.make_CANPacket(a, b, d))
    safety.safety_tick()
    safety.aol_set_host_request(3)

  def tx(value):
    a, d, b = pedal.command(case.packer, value, 0)
    return bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d)))

  try:
    assert safety.aol_get_permission_mask() == 0 and not tx(0.1)
    for _ in range(20):
      tick()
    assert safety.safety_config_valid() and safety.aol_get_permission_mask() == (1 if word == 0x2649 else 3)
    assert bool(safety.safety_tx_hook(case.angle()))
    assert tx(0.1) == (word == 0x2449) and tx(0)
    for _ in range(40):
      tick(fault)
    assert not tx(0.1) and tx(0)
    assert safety.aol_get_permission_mask() == 0
    for _ in range(25):
      tick()
    assert safety.aol_get_permission_mask() == (1 if word == 0x2649 else 3)
  finally:
    safety.set_alternative_experience(0)
    safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
