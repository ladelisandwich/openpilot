import json
from pathlib import Path

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.radar_filter import hardware_supported, configured
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.aol import qualified

DATA=Path(__file__).with_name('data')/'rav4_2025_filter_discovery.json'


def observed():
  evidence=json.loads(DATA.read_text())
  fw=[structs.CarParams.CarFw(**{**f,'fwVersion':bytes.fromhex(f['fwVersion'])}) for f in evidence['firmware']]
  fp=gen_empty_fingerprint()
  for f in evidence['advertisements']:
    fp[f['bus']][f['address']]=len(bytes.fromhex(f['data']))
  return fw,fp


@pytest.mark.parametrize('alpha',[False,True])
def test_observed_firmware_and_filter_keep_radar_enabled(alpha):
  fw,fp=observed()
  exact,found=match_fw_to_car(fw,'',log=False)
  assert exact and found=={CAR.TOYOTA_RAV4_TSS2_2023}
  cp=CarInterface.get_params(CAR.TOYOTA_RAV4_TSS2_2023,fp,fw,alpha,False,False)
  old=gen_empty_fingerprint()
  old[0][0x2AA]=8
  canonical=CarInterface.get_params(CAR.TOYOTA_RAV4_TSS2_2023,old,fw,alpha,False,False)
  assert cp.to_dict()==canonical.to_dict()
  assert hardware_supported(CAR.TOYOTA_RAV4_TSS2_2023,fp) and configured(cp) and qualified(cp)
  assert cp.flags & ToyotaFlags.RADAR_CAN_FILTER and not cp.flags & ToyotaFlags.DISABLE_RADAR
  assert cp.safetyConfigs[0].safetyParam==(0x1449 if alpha else 0x1649)


@pytest.mark.parametrize('bad',['missing_bus0','wrong_bus','bad_dlc0','bad_dlc2','pedal_mix','mixed_advertisement','legacy_sdsu','other_model'])
def test_observed_filter_admission_rejects_other_topologies(bad):
  _,fp=observed()
  candidate=CAR.TOYOTA_RAV4_TSS2_2023
  if bad=='missing_bus0':
    fp[0].clear()
  elif bad=='wrong_bus':
    fp[1][0x2FF]=4
  elif bad=='bad_dlc0':
    fp[0][0x2FF]=3
  elif bad=='bad_dlc2':
    fp[2][0x2FF]=8
  elif bad=='pedal_mix':
    fp[0][0x201]=6
  elif bad=='mixed_advertisement':
    fp[0][0x2AA]=8
  elif bad=='legacy_sdsu':
    fp[0][0x2FF]=8
    fp[2].clear()
  elif bad=='other_model':
    candidate=CAR.TOYOTA_RAV4H
  assert not hardware_supported(candidate,fp)
