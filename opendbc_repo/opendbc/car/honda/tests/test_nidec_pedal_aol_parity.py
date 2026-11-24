"""Nidec pedal AOL preserves the actual selected sender and current sensor gate."""
import os
from unittest.mock import patch

import pytest

from opendbc.car import structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.radar_interface import RadarInterface
from opendbc.car.honda.tests.test_nidec_interceptor import IDENTITIES, factory
from opendbc.car.honda.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix


@pytest.mark.parametrize('identity', IDENTITIES)
def test_actual_pedal_factory_card_and_reconstructed_controller_keep_the_sender(identity):
  from opendbc.car.honda.stock_aol import qualified
  from opendbc.car.honda.nidec_interceptor import qualified as pedal_qualified
  from openpilot.selfdrive.car.card import Car
  from openpilot.starpilot.car.honda.aol import native_accepts_cp
  original = factory(identity)
  assert pedal_qualified(original) and qualified(original)
  base = original.safetyConfigs[0].safetyParam
  for enabled in (False, True):
    with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION':'1', 'AOL_REPLAY_RUNTIME':'1'}):
      saved = Params()
      saved.put_bool('OpenpilotEnabledToggle', True, block=True)
      saved.put_bool('AlwaysOnLateral', enabled, block=True)
      cp = original.as_reader().as_builder()
      card = Car(CI=CarInterface(cp), RI=RadarInterface(cp))
      expected = base | ((512 | (256 if identity == CAR.HONDA_ODYSSEY_TWN else 0)) if enabled else 0)
      assert card.CP.safetyConfigs[0].safetyParam == expected
      assert card.CP.alternativeExperience == (32 if enabled else 0)
      assert card.aol_qualified == enabled
      assert not card.CP.pcmCruise and card.CP.openpilotLongitudinalControl
      with structs.CarParams.from_bytes(saved.get('CarParams')) as published:
        assert published.to_dict() == card.CP.to_dict()
        restored = published.as_builder()
      # Construct from FINAL published params, not merely the pre-marker object.
      rebuilt = CarInterface(restored)
      assert rebuilt.CC.nidec_interceptor is not None
      assert pedal_qualified(restored)
      assert native_accepts_cp(restored, 1, expected) == enabled
      bad = restored.as_reader().as_builder()
      bad.safetyConfigs[0].safetyParam |= 0x4000
      assert not qualified(bad) and not pedal_qualified(bad)


CASES = (
  ('pedal', 1, 544, 0, 'honda_civic_touring_2016_can_generated', False, 'GEARBOX_CVT', 4),
  ('pedal-alt', 1, 548, 0, 'acura_ilx_2016_can_generated', True, 'GEARBOX_AUTO', 4),
  ('pedal-twn', 1, 804, 0, 'honda_odyssey_twn_2018_generated', True, 'GEARBOX_AUTO', 4),
)


@pytest.mark.parametrize('profile', CASES)
@pytest.mark.parametrize('loss', ('sensor_fault', 'sensor_expired', 'wrong_bus', 'bad_tx_crc'))
def test_actual_pedal_native_brake_sensor_and_transmitted_crc_contract(profile, loss):
  from opendbc.car.honda.nidec_interceptor import create_command
  from opendbc.safety.tests.test_honda_stock_aol import TestHondaStockAol
  from opendbc.safety.tests.libsafety import libsafety_py
  native = TestHondaStockAol(methodName='runTest')
  try:
    native.configure(profile)
    native.stream()
    native.normal_engage()
    native.safety.aol_set_host_request(3)
    assert native.safety.get_controls_allowed()
    assert native.safety.aol_get_permission_mask() == 3
    active = create_command(native.packer, .5, 0)
    # CANPackerSafety inherits the actual packer; use its real message encoding.

    def packet(msg):
      return libsafety_py.make_CANPacket(msg[0], msg[2], msg[1])
    assert native.safety.safety_tx_hook(packet(active))
    native.press_brake()
    assert not native.safety.get_controls_allowed()
    assert native.safety.aol_get_permission_mask() == 1
    assert not native.safety.safety_tx_hook(packet(active))
    neutral = create_command(native.packer, 0., 1)
    assert native.safety.safety_tx_hook(packet(neutral))
    native.stream()
    assert not native.safety.get_controls_allowed()  # No longitudinal auto-resume.
    if loss == 'sensor_fault':
      assert native.safety.safety_rx_hook(native.packet('GAS_SENSOR', {'STATE':1}))
      assert native.safety.aol_get_permission_mask() == 0
    elif loss in ('sensor_expired', 'wrong_bus'):
      native.safety.set_timer(1_200_001)
      native.stream(omit='GAS_SENSOR')
      if loss == 'wrong_bus':
        assert native.safety.safety_rx_hook(native.packet('GAS_SENSOR', {'STATE':0}, 1))
      assert native.safety.aol_get_permission_mask() == 0
    else:
      data = bytearray(neutral[1])
      data[5] ^= 1
      assert not native.safety.safety_tx_hook(libsafety_py.make_CANPacket(neutral[0], neutral[2], data))
    if loss != 'bad_tx_crc':
      native.stream()
      assert native.safety.aol_get_permission_mask() == 0
  finally:
    libsafety_py.libsafety.set_alternative_experience(0)


@pytest.mark.parametrize('identity', (CAR.HONDA_CIVIC, CAR.ACURA_ILX, CAR.HONDA_ODYSSEY_TWN))
def test_actual_pedal_card_sd_controls_lat_only_preserves_long_pause(identity):
  from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import honda_family_case
  with OpenpilotPrefix():
    case = honda_family_case('honda', True)
    case.family_cp = factory(identity).as_reader()
    try:
      sd, _, drive, card, receive = case.family_callers(False)
      assert card.CI.CC.nidec_interceptor is not None
      axis, _, command = drive(2, native_request_long=False, panda_controls_allowed=False)
      assert axis.lateralActive and command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert receive(10_020_000_000, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    finally:
      case.doCleanups()
