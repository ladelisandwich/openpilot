import os
from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint, structs

from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.tests.test_radar_hda2_startup import params
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car

CARS = frozenset((
  CAR.GENESIS_G80_2ND_GEN_FL,
  CAR.HYUNDAI_IONIQ_5_N,
  CAR.HYUNDAI_KONA_2ND_GEN,
  CAR.HYUNDAI_KONA_HEV_2ND_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_2025,
  CAR.HYUNDAI_SONATA_2024,
  CAR.HYUNDAI_SONATA_HEV_2024,
  CAR.HYUNDAI_STARIA_4TH_GEN,
  CAR.HYUNDAI_TUCSON_2025,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  CAR.HYUNDAI_TUCSON_HEV_2025,
  CAR.HYUNDAI_TUCSON_PHEV_2025,
  CAR.KIA_CARNIVAL_2025,
  CAR.KIA_CARNIVAL_HEV_4TH_GEN,
  CAR.KIA_K4_2025,
  CAR.KIA_K5_2025,
  CAR.KIA_K8_HEV_1ST_GEN,
  CAR.KIA_NIRO_EV_2ND_GEN,
  CAR.KIA_NIRO_HEV_2ND_GEN,
  CAR.KIA_SORENTO_2024,
  CAR.KIA_SPORTAGE_5TH_GEN,
))
FUEL_CASES = tuple((identity, hybrid) for identity in sorted(CARS) for hybrid in (False, True)
                   if not (identity.config.flags & HyundaiFlags.EV and hybrid))


@pytest.mark.parametrize('identity,hybrid', FUEL_CASES)
@pytest.mark.parametrize('alt', [False, True])
def test_existing_generic_hda2_long_requires_prepublication_owner(identity, hybrid, alt):
  cp = params(identity, hybrid, alt=alt)
  gas = 1 if identity.config.flags & HyundaiFlags.EV else 2 if hybrid or identity == CAR.KIA_CARNIVAL_HEV_4TH_GEN else 0
  calibration = 0x2000 if identity in (CAR.KIA_CARNIVAL_2025, CAR.KIA_CARNIVAL_HEV_4TH_GEN) else 0
  assert cp.openpilotLongitudinalControl and cp.safetyConfigs[0].safetyParam == (0x14 | gas | (128 if alt else 0) | calibration)
  sent = []
  owner = CarInterface.startup_owner(cp, (list, sent.extend), requested=True)
  assert not sent
  assert owner is not None, 'GENERIC_EV_HDA2_LONG_HAS_NO_PREPUBLICATION_OWNER'
  assert CarInterface.startup_required(cp)


@pytest.mark.parametrize('identity,hybrid', FUEL_CASES)
@pytest.mark.parametrize('alt', [False, True])
def test_actual_card_rejects_unprepared_generic_hda2_long(identity, hybrid, alt):
  cp = params(identity, hybrid, alt=alt)
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    saved = Params()
    saved.put_bool('OpenpilotEnabledToggle', True, block=True)
    saved.put_bool('AlwaysOnLateral', False, block=True)
    with pytest.raises(RuntimeError, match='requires a matching prepared startup owner'):
      Car(CI=CarInterface(cp), RI=RadarInterface(cp))
    assert saved.get('CarParams') is None and not saved.get_bool('ControlsReady')


def test_registry_cannot_add_an_unowned_ordinary_ecu_disable_path_silently():
  # This audits advertised source paths, not physical firmware or road acceptance.
  known_contracts = {
    (CAR.KIA_EV6, True), (CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, True),
  }
  missing, observed_contracts = [], set()
  for identity in sorted(identity for identity in CAR if identity.config.flags & HyundaiFlags.CANFD):
    for alt in (False, True, None):
      if alt is None:
        fp = gen_empty_fingerprint()
        fp[0].update({0x1CF: 8, 0x130: 32, 0x35: 32, 0x175: 24, 0xA0: 24, 0xEA: 24, 0x1A0: 32})
        fw = [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.adas, address=0x730, bus=1)]
        cp = CarInterface.get_params(identity, fp, fw, True, False, False)
      else:
        cp = params(identity, alt=alt)
      requires_disable = (cp.openpilotLongitudinalControl and not cp.flags &
                          (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC | HyundaiFlags.NON_SCC))
      if not requires_disable or cp.dashcamOnly:
        continue
      try:
        owner = CarInterface.startup_owner(cp, (list, list), requested=True)
      except ValueError as error:
        missing.append((str(identity), alt, str(error)))
        continue
      if (identity, alt) in known_contracts:
        # Alternate captured-ADRV recipes retain their distinct receiver contract.
        assert owner is None
        observed_contracts.add((identity, alt))
      elif owner is None:
        missing.append((str(identity), alt, 'missing prepublication owner'))
  assert observed_contracts == known_contracts
  assert not missing, missing
