"""Full acquired firmware uses static catalog keys, never enum-reader hashes."""
import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.toyota import prius_retrofit_advert as owner
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests.test_prius_retrofit_advert_factory import factory


def complete_firmware():
  result=[]
  for (ecu,address,subaddress),versions in FW_VERSIONS[CAR.TOYOTA_PRIUS].items():
    result.append(structs.CarParams.CarFw(ecu=ecu,address=address,subAddress=subaddress or 0,
                                        fwVersion=owner.EPS_FIRMWARE if ecu==structs.CarParams.Ecu.eps else versions[0],brand='toyota',bus=0))
  assert any(entry.ecu==structs.CarParams.Ecu.fwdCamera for entry in result)
  assert any(entry.ecu==structs.CarParams.Ecu.fwdRadar for entry in result)
  return result


@pytest.mark.parametrize('candidate',owner.CARS)
def test_complete_acquired_firmware_is_admitted(candidate):
  fp=gen_empty_fingerprint()
  fp[0][0x2FF]=4
  fw=complete_firmware()
  assert owner.hardware_supported(candidate,fp,fw), 'EPS73_COMPLETE_ACQUIRED_FW_NOT_ADMITTED'
  assert owner.configured(factory(candidate,fp=fp,fw=fw))


@pytest.mark.parametrize('candidate',owner.CARS)
@pytest.mark.parametrize('ecu',(structs.CarParams.Ecu.fwdCamera,structs.CarParams.Ecu.fwdRadar))
@pytest.mark.parametrize('field,value',(('logging',True),('bus',1),('brand','gm'),('fwVersion',b'unknown')))
def test_complete_acquired_firmware_still_rejects_invalid_metadata(candidate,ecu,field,value):
  fw=complete_firmware()
  entry=next(entry for entry in fw if entry.ecu==ecu)
  setattr(entry,field,value)
  fp=gen_empty_fingerprint()
  fp[0][0x2FF]=4
  assert not owner.hardware_supported(candidate,fp,fw)
  assert not owner.configured(factory(candidate,fp=fp,fw=fw))
