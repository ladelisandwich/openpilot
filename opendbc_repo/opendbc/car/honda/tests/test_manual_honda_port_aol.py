"""Real forced-selection factory must not strand established manual-only ports."""

import os
from unittest.mock import patch

import pytest

from opendbc.can.dbc import DBC as CANDBC
from opendbc.car import Bus, car_helpers, gen_empty_fingerprint, structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, DBC
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix

IDS = (CAR.HONDA_CRV_SA, CAR.HONDA_CLARITY, CAR.HONDA_ACCORD_9G, CAR.ACURA_MDX_3G, CAR.ACURA_MDX_3G_MMR, CAR.ACURA_TLX_1G, CAR.HONDA_FIT_4G, CAR.ACURA_INTEGRA)


@pytest.mark.parametrize('identity', IDS)
def test_real_registered_manual_selection_reaches_card_controls_without_native_grant(identity):
  from openpilot.selfdrive.car.card import Car
  from openpilot.selfdrive.controls.controlsd import Controls
  from opendbc.car.honda.stock_aol import qualified
  from opendbc.safety.tests.libsafety import libsafety_py

  fp = gen_empty_fingerprint()
  messages = CANDBC(DBC[identity][Bus.pt]).name_to_msg
  gear = messages.get('GEARBOX_AUTO') or messages.get('GEARBOX_CVT')
  fp[0][gear.address] = gear.size
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '1', 'FINGERPRINT': ''}):
    saved = Params()
    saved.put_bool('OpenpilotEnabledToggle', True, block=True)
    saved.put_bool('AlwaysOnLateral', True, block=True)
    # Patch acquisition only. Actual get_car applies forced selection/source=fixed
    # and runs the real factory/controller. Never relabel fixed as automatic FW.
    observation = (None, fp, '0' * 17, [], structs.CarParams.FingerprintSource.can, False)
    with patch.object(car_helpers, 'fingerprint', return_value=observation):
      ci = car_helpers.get_car(lambda *a: [], lambda *a: None, lambda *a: None, False, False, forced_candidate=str(identity))
    assert ci.CP.fingerprintSource == structs.CarParams.FingerprintSource.fixed
    assert qualified(ci.CP)
    card = Car(CI=ci, RI=CarInterface.RadarInterface(ci.CP))
    assert card.aol_qualified and card.CP.alternativeExperience == 32
    assert not card.aol_card_intent.allowed_latch
    with structs.CarParams.from_bytes(saved.get('CarParams')) as published:
      assert published.fingerprintSource == structs.CarParams.FingerprintSource.fixed
      assert published.to_dict() == card.CP.to_dict()
    controls = Controls()
    assert controls.CP.to_dict() == card.CP.to_dict()
    safety = libsafety_py.libsafety
    try:
      safety.init_tests()
      safety.set_alternative_experience(32)
      cfg = card.CP.safetyConfigs[0]
      assert safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam) == 0
      safety.set_aol_test_heartbeat(True)
      safety.aol_set_host_request(1)
      assert safety.aol_get_permission_mask() == 0  # Selection supplies no physical authority.
    finally:
      safety.set_alternative_experience(0)
