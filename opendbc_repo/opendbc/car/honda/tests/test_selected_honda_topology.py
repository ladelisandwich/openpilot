"""Exact current selected CP and two-Panda owner, never a selection-made grant."""

from dataclasses import replace

import pytest

from opendbc.car import gen_empty_fingerprint, structs, Bus, car_helpers
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.values import CAR, DBC
from opendbc.car.honda.stock_aol import CANFD_STOCK, CLASSIC_BOSCH, NIDEC, RADARLESS, ALT_RADAR_STOCK, qualified
from opendbc.car.honda.tests.test_manual_honda_port_aol import IDS
from openpilot.starpilot.aol.vehicle import native_matches_cp
from openpilot.starpilot.aol.runtime import native_inventory_matches_cp
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState
from openpilot.starpilot.car.honda.aol import policy_for


EXPECTED_PORTS = CLASSIC_BOSCH | NIDEC | RADARLESS | CANFD_STOCK | ALT_RADAR_STOCK | {CAR.HONDA_ACCORD_11G}


MANUAL_SELECTION_PORTS = CLASSIC_BOSCH | NIDEC | RADARLESS | CANFD_STOCK | ALT_RADAR_STOCK | {CAR.HONDA_ACCORD_11G}


@pytest.mark.parametrize('identity', sorted(EXPECTED_PORTS - set(IDS), key=str))
def test_remaining_registered_selection_uses_actual_factory_card_controls_native_zero(identity):
  import os
  from unittest.mock import patch
  from opendbc.can.dbc import DBC as CANDBC
  from opendbc.safety.tests.libsafety import libsafety_py
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.common.params import Params
  from openpilot.selfdrive.car.card import Car
  from openpilot.selfdrive.controls.controlsd import Controls

  fp = gen_empty_fingerprint()
  preliminary = CarInterface.get_params(identity, fp, [], False, False, False)
  messages = CANDBC(DBC[identity][Bus.pt]).name_to_msg
  gear = messages.get('GEARBOX_AUTO') or messages.get('GEARBOX_CVT')
  fp[CanBus(preliminary).pt][gear.address] = gear.size
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '1', 'FINGERPRINT': ''}):
    params = Params()
    params.put_bool('OpenpilotEnabledToggle', True, block=True)
    params.put_bool('AlwaysOnLateral', True, block=True)
    observation = (None, fp, '0' * 17, [], structs.CarParams.FingerprintSource.can, False)
    with patch.object(car_helpers, 'fingerprint', return_value=observation):
      ci = car_helpers.get_car(lambda *a: [], lambda *a: None, lambda *a: None, False, False, forced_candidate=str(identity))
    assert ci.CP.fingerprintSource == structs.CarParams.FingerprintSource.fixed
    assert qualified(ci.CP)
    assert identity in MANUAL_SELECTION_PORTS
    card = Car(CI=ci, RI=ci.RadarInterface(ci.CP))
    assert card.aol_qualified and not card.aol_card_intent.allowed_latch
    assert Controls().CP.to_dict() == card.CP.to_dict()
    safety = libsafety_py.libsafety
    try:
      safety.init_tests()
      safety.set_alternative_experience(32)
      cfg = card.CP.safetyConfigs[0]
      assert safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam) == 0
      safety.set_aol_test_heartbeat(True)
      safety.aol_set_host_request(1)
      assert safety.aol_get_permission_mask() == 0
    finally:
      safety.set_alternative_experience(0)


@pytest.mark.parametrize('identity', sorted(CANFD_STOCK, key=str))
@pytest.mark.parametrize('alt', (False, True))
def test_actual_two_panda_factory_matches_only_active_native_owner_and_complete_inventory(identity, alt):
  fp = gen_empty_fingerprint()
  fp[4][0x1A3] = 8
  if alt:
    fp[4][0x1BE] = 3
  cp = CarInterface.get_params(identity, fp, [], True, False, False)
  assert len(cp.safetyConfigs) == 2 and str(cp.safetyConfigs[0].safetyModel) == 'noOutput'
  assert cp.pcmCruise and not cp.openpilotLongitudinalControl
  assert qualified(cp)
  cp.alternativeExperience = policy_for(cp).alternative_experience_addition
  word = 16 | int(alt)
  assert qualified(cp, marked_only=True)
  assert native_matches_cp(cp, 20, word)
  assert not native_matches_cp(cp, int(cp.safetyConfigs[0].safetyModel.raw), 0)
  assert CarInterface(cp).CC.CAN.pt == 4
  slots = (PandaSlot(0, 'silent', int(cp.safetyConfigs[0].safetyModel.raw), 0, 32), PandaSlot(1, 'honda', 20, word, 32))
  state = SafetyState(1, True, 10_000_000_000, 10_200_000_000, 20, word, False, False, False, False, 'honda', 'session', 10_000_000_000, slots)
  assert native_inventory_matches_cp(state, cp)
  assert not state.lateralAllowed and not state.longitudinalAllowed
  assert not native_inventory_matches_cp(replace(state, pandaInventory=()), cp)  # Legacy v1 cannot admit two slots.
  for bad_slots in (
    slots[::-1],
    (slots[0], replace(slots[1], hardwareSerial='silent')),
    (replace(slots[0], controlsAllowed=True), slots[1]),
    (slots[0], replace(slots[1], safetyRxChecksInvalid=True)),
    (slots[0], replace(slots[1], faults=1)),
    (slots[0], replace(slots[1], heartbeatLost=True)),
  ):
    assert not native_inventory_matches_cp(replace(state, pandaInventory=bad_slots), cp)
  bad = cp.as_reader().as_builder()
  bad.safetyConfigs[0].safetyParam = 1
  assert not qualified(bad)
  bad = cp.as_reader().as_builder()
  bad.safetyConfigs[1].safetyParam = 18
  assert not qualified(bad)
  assert not native_matches_cp(bad, 20, 18)


@pytest.mark.parametrize('identity', sorted(CANFD_STOCK, key=str))
def test_actual_two_slot_card_publishes_final_cp_and_keeps_initial_intent_unarmed(identity):
  import os
  from unittest.mock import patch
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.common.params import Params
  from openpilot.selfdrive.car.card import Car
  from openpilot.selfdrive.controls.controlsd import Controls

  fp = gen_empty_fingerprint()
  fp[4][0x1A3] = 8
  initial = CarInterface.get_params(identity, fp, [], False, False, False)
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '1'}):
    params = Params()
    params.put_bool('OpenpilotEnabledToggle', True, block=True)
    params.put_bool('AlwaysOnLateral', True, block=True)
    ci = CarInterface(initial)
    card = Car(CI=ci, RI=ci.RadarInterface(initial))
    assert card.aol_qualified and card.CP.alternativeExperience == 32
    assert not card.aol_card_intent.allowed_latch
    with structs.CarParams.from_bytes(params.get('CarParams')) as published:
      assert published.to_dict() == card.CP.to_dict()
    assert Controls().CP.to_dict() == card.CP.to_dict()


@pytest.mark.parametrize('identity', sorted(CANFD_STOCK, key=str))
def test_actual_two_slot_card_sd_controls_consume_exact_v2_inventory(identity):
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import honda_family_case

  fp = gen_empty_fingerprint()
  fp[4][0x1A3] = 8
  cp = CarInterface.get_params(identity, fp, [], False, False, False)
  with OpenpilotPrefix():
    case = honda_family_case('honda', False)
    case.family_cp = cp.as_reader()
    try:
      sd, _, drive, card, receive = case.family_callers(False)
      axis, _, command = drive(2, native_request_long=False, panda_controls_allowed=False)
      assert axis.lateralActive and command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert receive(10_020_000_000, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      denied, _, command = drive(3, native_valid=False, native_request_long=False)
      assert not denied.lateralActive and not command.latActive
    finally:
      case.doCleanups()
