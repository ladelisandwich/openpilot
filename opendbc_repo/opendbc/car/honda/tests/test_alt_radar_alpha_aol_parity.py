"""Exact DEBUG alpha owner retains physical LAT intent without restoring LONG."""
import os
from unittest.mock import patch

import pytest

from opendbc.car import structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.honda.tests.test_alt_radar_stock_aol_parity import IDENTITIES, factory
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix


@pytest.mark.parametrize('identity', IDENTITIES)
@pytest.mark.parametrize('alt', (False, True))
def test_actual_alpha_factory_card_serialization_and_explicit_off(identity, alt):
  from opendbc.car.honda.stock_aol import qualified
  from openpilot.selfdrive.car.card import Car
  from openpilot.starpilot.car.honda.aol import native_accepts_cp, policy_for
  initial = factory(identity, alt=alt, alpha=True)
  assert initial.openpilotLongitudinalControl and not initial.pcmCruise
  assert qualified(initial)
  addition = 1568 if str(identity) == 'ACURA_MDX_4G' else 1056
  assert policy_for(initial).safety_param_addition == addition
  base_word = initial.safetyConfigs[0].safetyParam
  for enabled in (False, True):
    with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '1'}):
      saved = Params()
      saved.put_bool('OpenpilotEnabledToggle', True, block=True)
      saved.put_bool('AlwaysOnLateral', enabled, block=True)
      cp = initial.as_reader().as_builder()
      card = Car(CI=CarInterface(cp), RI=RadarInterface(cp))
      assert card.aol_qualified == enabled
      assert card.CP.safetyConfigs[0].safetyParam == base_word | (addition if enabled else 0)
      assert card.CP.alternativeExperience == (32 if enabled else 0)
      assert card.CP.openpilotLongitudinalControl and not card.CP.pcmCruise
      with structs.CarParams.from_bytes(saved.get('CarParams')) as published:
        assert published.to_dict() == card.CP.to_dict()
      assert native_accepts_cp(card.CP, 20, card.CP.safetyConfigs[0].safetyParam) == enabled


@pytest.mark.parametrize('identity', IDENTITIES)
@pytest.mark.parametrize('alt', (False, True))
def test_actual_native_set_brake_and_host_axes_do_not_auto_resume_long(identity, alt):
  from opendbc.safety.tests.libsafety import libsafety_py
  from opendbc.safety.tests.test_honda_stock_aol import TestHondaStockAol
  from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import honda_family_case, car_state
  with OpenpilotPrefix():
    case = honda_family_case('honda', True)
    case.family_cp = factory(identity, alt=alt, alpha=True).as_reader()
    native = TestHondaStockAol(methodName='runTest')
    try:
      sd, _, drive, card, receive = case.family_callers(True)
      word = card.CP.safetyConfigs[0].safetyParam
      assert word in (1058, 1059, 1570, 1571)
      native.configure(('alt-alpha', 20, word, 1, card.CI.CC.packer.dbc.name, False, 'GEARBOX_AUTO', 4))
      native.stream()
      native.normal_engage()  # Actual SET falling edge from checksummed CAN.
      native.safety.aol_set_host_request(3)
      if not native.debug:
        assert native.safety.aol_get_permission_mask() == 0
        return  # RELEASE native refusal is authoritative, never fabricated grant.
      assert native.safety.aol_get_permission_mask() == 3
      axis, _, command = drive(2, allowed=True, native_request_long=True,
                               panda_controls_allowed=bool(native.safety.get_controls_allowed()))
      assert axis.lateralActive and axis.longitudinalActive and command.latActive and command.longActive
      native.press_brake()
      assert not native.safety.get_controls_allowed()
      native.safety.aol_set_host_request(1)
      permission = native.safety.aol_get_permission_mask()
      assert permission == 1
      state = car_state(brake=True)
      axis, _, command = drive(3, observed_state=state, allowed=bool(permission & 1), native_request_long=False,
                               panda_controls_allowed=bool(native.safety.get_controls_allowed()))
      assert axis.lateralActive and command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert receive(10_030_000_000, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'], state=state)
      native.stream()
      axis, _, command = drive(4, native_request_long=False, allowed=bool(native.safety.aol_get_permission_mask() & 1),
                               panda_controls_allowed=bool(native.safety.get_controls_allowed()))
      assert not native.safety.get_controls_allowed()
      assert axis.lateralActive and command.latActive and not axis.longitudinalActive and not command.longActive
      native.stream(gear=2)
      assert native.safety.aol_get_permission_mask() == 0
      native.stream()
      assert native.safety.aol_get_permission_mask() == 1
      native.stream(eps=7)
      native.stream(eps=0)
      assert native.safety.aol_get_permission_mask() == 0
    finally:
      libsafety_py.libsafety.set_alternative_experience(0)
      case.doCleanups()
