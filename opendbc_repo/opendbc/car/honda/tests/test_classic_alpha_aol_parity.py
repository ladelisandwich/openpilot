"""Actual normal startup for every established classic Bosch alpha identity."""
import os
from unittest.mock import patch

import pytest
from opendbc.car import structs, gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.stock_aol import CLASSIC_BOSCH, qualified
from opendbc.car.honda.values import CAR, HondaFlags
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.params import Params


@pytest.mark.parametrize('identity', sorted(CLASSIC_BOSCH, key=str))
@pytest.mark.parametrize('alt', (False, True))
def test_actual_normal_card_sd_controls_startup(identity, alt):
  from openpilot.selfdrive.car.card import Car
  from openpilot.selfdrive.controls.controlsd import Controls
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  fp = gen_empty_fingerprint()
  fp[1][0x1A3] = 8
  if alt:
    fp[1][0x1BE] = 3
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '0'}):
    params = Params()
    params.put_bool('OpenpilotEnabledToggle', True, block=True)
    params.put_bool('AlwaysOnLateral', True, block=True)
    cp = CarInterface.get_params(identity, fp, [], True, False, False)
    ci = CarInterface(cp)
    card = Car(CI=ci, RI=ci.RadarInterface(cp))
    assert card.aol_qualified
    sd, controls = SelfdriveD(CP=card.CP), Controls()
    assert sd.aol_replay and controls.aol_replay  # Normal settings, no replay override.
    assert card.CP.alternativeExperience == 32 and qualified(card.CP, marked_only=True)
    expected_alt = bool(identity.config.flags & HondaFlags.BOSCH_ALT_BRAKE) or (alt and identity in (CAR.HONDA_ACCORD, CAR.ACURA_RDX_3G))
    assert bool(card.CP.flags & HondaFlags.BOSCH_ALT_BRAKE) == expected_alt
    expected = 2082 | int(expected_alt) | (128 if identity == CAR.ACURA_RDX_3G else 0)
    assert card.CP.safetyConfigs[0].safetyParam == expected
    assert not card.aol_card_intent.allowed_latch
    with structs.CarParams.from_bytes(params.get('CarParams')) as published:
      assert published.to_dict() == card.CP.to_dict() == controls.CP.to_dict()
    assert CarInterface(card.CP).CC.bosch_longitudinal is not None and ci.CC.bosch_longitudinal is not None
    from opendbc.safety.tests.libsafety import libsafety_py
    from openpilot.starpilot.aol.tests.honda_can import HondaCan
    safety = libsafety_py.libsafety
    codec = HondaCan(ci)
    safety.init_tests()
    safety.set_alternative_experience(32)
    assert safety.set_safety_hooks(20, expected) == 0
    safety.set_aol_test_heartbeat(True)

    def observe(tick, **values):
      now = 1_000_000 + tick * 10_000
      safety.set_timer(now)
      frames = codec.frames(**values)
      for address, payload, bus in frames:
        assert safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, payload))
      safety.aol_set_host_request(3)
      return ci.update([(now * 1000, frames)])

    try:
      for tick in range(40):
        observe(tick)
      assert safety.aol_get_permission_mask() == 0
      observe(40, cruise_button=3)
      observe(41)
      assert safety.get_controls_allowed() and safety.aol_get_permission_mask() == 3
      observed = observe(42, brake=True)
      assert observed.brakePressed and not safety.get_controls_allowed()
      assert safety.aol_get_permission_mask() == 1
      observe(43, eps=6)
      assert safety.aol_get_permission_mask() == 0
      observe(44)
      assert safety.aol_get_permission_mask() == 1
      reverse = [value for value, name in ci.CS.shifter_values.items() if name == 'R']
      assert len(reverse) == 1
      observed = observe(45, gear=reverse[0])
      assert observed.gearShifter == structs.CarState.GearShifter.reverse
      assert safety.aol_get_permission_mask() == 0
      observe(46)
      assert safety.aol_get_permission_mask() == 1
      observe(47, eps=7)
      observe(48)
      assert safety.aol_get_permission_mask() == 0
    finally:
      safety.set_alternative_experience(0)


@pytest.mark.parametrize('identity', sorted(CLASSIC_BOSCH, key=str))
def test_classic_alpha_rejects_wrong_namespace_manual_and_diagnostic_borrowing(identity):
  from openpilot.starpilot.car.honda.aol import policy_for
  from openpilot.starpilot.aol.panda_faults import blocking_fault_mask
  fp = gen_empty_fingerprint()
  fp[1][0x1A3] = 8
  cp = CarInterface.get_params(identity, fp, [], True, False, False)
  assert qualified(cp)
  cp.safetyConfigs[0].safetyParam |= policy_for(cp).safety_param_addition
  cp.alternativeExperience = 32
  assert qualified(cp, marked_only=True)
  assert blocking_fault_mask(1 << 3, cp)
  for word in (34, 35, 163, 1058, 1570, 2080, 2084, 2210):
    bad = cp.as_reader().as_builder()
    bad.safetyConfigs[0].safetyParam = word
    assert not qualified(bad, marked_only=True)
  cp.transmissionType = 'manual'
  assert not qualified(cp)


@pytest.mark.parametrize('word,ceiling', ((2082, 2000), (2083, 2000), (2211, 2200)))
def test_actual_native_classic_alpha_gas_ceiling_and_brake_withdrawal(word, ceiling):
  from opendbc.safety.tests.test_honda_stock_aol import TestHondaStockAol, CASES
  case = TestHondaStockAol()
  try:
    case.configure(next(row for row in CASES if row[2] == word))
    case.stream()
    case.normal_engage()
    case.safety.aol_set_host_request(3)
    assert case.safety.aol_get_permission_mask() == 3
    for gas, allowed in ((ceiling, True), (ceiling + 1, False)):
      packet = case.packet('ACC_CONTROL', {'GAS_COMMAND': gas, 'ACCEL_COMMAND': 0, 'BRAKE_REQUEST': 0})
      assert bool(case.safety.safety_tx_hook(packet)) == allowed
    case.press_brake()
    assert case.safety.aol_get_permission_mask() == 1
    assert not case.safety.safety_tx_hook(case.packet('ACC_CONTROL', {'GAS_COMMAND': 1, 'ACCEL_COMMAND': 0, 'BRAKE_REQUEST': 0}))
  finally:
    case.tearDown()
