import os
from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car

CARS = (CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN, CAR.KIA_CARNIVAL_4TH_GEN, CAR.GENESIS_GV70_1ST_GEN, CAR.GENESIS_GV80)


def params(identity, hybrid=False, alpha=True):
  fp = gen_empty_fingerprint()
  fp[0].update({0x1CF: 8, 0x130: 32, 0x35: 32, 0x175: 24, 0xA0: 24, 0xEA: 24, 0x1A0: 32})
  if hybrid:
    fp[0][0xFA] = 8
  return CarInterface.get_params(identity, fp, [], alpha, False, False)


@pytest.mark.parametrize("identity", CARS)
@pytest.mark.parametrize("hybrid", [False, True])
def test_radar_hda1_factory_requires_startup_owner_before_transmission(identity, hybrid):
  cp = params(identity, hybrid)
  assert cp.openpilotLongitudinalControl and not cp.pcmCruise and cp.safetyConfigs[0].safetyParam in (4, 6)
  sent = []
  owner = CarInterface.startup_owner(cp, (list, sent.extend), requested=True)
  assert not sent
  assert owner is not None, "HDA1_RADAR_LONG_HAS_NO_PREPUBLICATION_OWNER"
  assert CarInterface.startup_required(cp)


@pytest.mark.parametrize("identity", CARS)
@pytest.mark.parametrize("hybrid", [False, True])
def test_actual_card_rejects_unprepared_radar_hda1_before_publishing(identity, hybrid):
  cp = params(identity, hybrid)
  with OpenpilotPrefix(), patch.dict(os.environ, {"SIMULATION": "0", "AOL_REPLAY_RUNTIME": "1"}):
    saved = Params()
    saved.put_bool("OpenpilotEnabledToggle", True, block=True)
    saved.put_bool("AlwaysOnLateral", False, block=True)
    with pytest.raises(RuntimeError, match="requires a matching prepared startup owner"):
      Car(CI=CarInterface(cp), RI=RadarInterface(cp))
    assert saved.get("CarParams") is None and not saved.get_bool("ControlsReady")
