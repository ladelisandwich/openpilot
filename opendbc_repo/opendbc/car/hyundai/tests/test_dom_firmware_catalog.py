import importlib

import pytest

from opendbc.car import structs
from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car, match_fw_to_car_exact

CASES = [['hyundai', 'HYUNDAI_SONATA', 'abs', 2001, None, 'f100444e204553432001203130372007032035383931302d4c31333030'],
 ['hyundai', 'HYUNDAI_SONATA', 'eps', 2004, None, 'f100444e38204d445053204320312e303020312e30332035363331302d4c313231302034444e4443313033'],
 ['hyundai',
  'HYUNDAI_SONATA',
  'fwdCamera',
  1988,
  None,
  'f100444e38204d464320204154204b4f52204c484420312e303020312e30362039393231312d4c3130303020323130333235'],
 ['hyundai',
  'KIA_FORTE',
  'fwdCamera',
  1988,
  None,
  'f100424420204c4b415320415420555341204c484420312e303020312e30322039353734302d4d36303030204a3331'],
 ['hyundai',
  'HYUNDAI_KONA_EV_2ND_GEN',
  'fwdCamera',
  1988,
  None,
  'f100535832454d46432020415420555341204c484420312e303020312e30322039393231312d424630303020323330383233'],
 ['hyundai',
  'KIA_K4_2025',
  'fwdCamera',
  1988,
  None,
  'f100434c34204d46432020415420555341204c484420312e303020312e30342039393231302d474731303020323531323035'],
 ['hyundai',
  'HYUNDAI_STARIA_4TH_GEN',
  'fwdCamera',
  1988,
  None,
  'f100555334204d464320204154204155532052484420312e303020312e30342039393231312d434730303020323130383139']]


@pytest.mark.parametrize('brand,identity,ecu_name,address,subaddress,firmware_hex', CASES)
def test_original_firmware_catalog_and_actual_serialized_exact_matcher(brand, identity, ecu_name, address, subaddress, firmware_hex):
  values = importlib.import_module(f'opendbc.car.{brand}.values')
  fingerprints = importlib.import_module(f'opendbc.car.{brand}.fingerprints')
  candidate = getattr(values.CAR, identity)
  ecu = getattr(structs.CarParams.Ecu, ecu_name)
  firmware = bytes.fromhex(firmware_hex)
  expected = fingerprints.FW_VERSIONS[candidate]
  assert firmware in expected[(ecu, address, subaddress)]

  live = {(addr, sub):{versions[0]} for (_, addr, sub), versions in expected.items()}
  live[(address, subaddress)] = {firmware}
  observations = [structs.CarParams.CarFw(ecu=kind, address=addr, subAddress=sub or 0,
    fwVersion=next(iter(live[(addr, sub)])), brand=brand) for kind, addr, sub in expected]
  cp = structs.CarParams()
  cp.carFw = observations
  with structs.CarParams.from_bytes(cp.to_bytes()) as decoded:
    current_live = build_fw_dict(decoded.carFw, filter_brand=brand)
    assert current_live[(address, subaddress)] == {firmware}
    assert candidate in match_fw_to_car_exact(current_live, match_brand=brand, log=False)
    exact, matches = match_fw_to_car(decoded.carFw, '', allow_exact=True, allow_fuzzy=False, log=False)
    assert exact and candidate in matches

  if ecu_name != 'debug':
    live[(address, subaddress)] = {firmware + b' altered'}
    assert candidate not in match_fw_to_car_exact(live, match_brand=brand, log=False)
