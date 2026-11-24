"""Recorded Bolt identity selection, capability evidence and explicit overrides."""
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.car import car_helpers, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.pedal_capability import automatic_bolt_candidate
from opendbc.car.gm.values import CAR, GMFlags

RECORDINGS = json.loads((Path(__file__).parent / 'testdata/bolt_vin_fingerprints.json').read_text())
BOLTS = (CAR.CHEVROLET_BOLT_CC_2017, CAR.CHEVROLET_BOLT_CC_2018_2021,
         CAR.CHEVROLET_BOLT_CC_2022_2023, CAR.CHEVROLET_BOLT_ACC_2022_2023,
         CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL)


def recording(index):
  row = RECORDINGS[index]
  fp = gen_empty_fingerprint()
  fp.update({int(bus): {int(address): size for address, size in messages.items()}
             for bus, messages in row['fingerprint'].items()})
  # Descriptor/year retained; check digit and serial are synthetic.
  vin = row['vinPrefix'] + '0' + row['year'] + '0000000'
  return fp, {bytes.fromhex(raw) for raw in row['cameraPayloads']}, vin


def packets(fp, camera):
  return [CanData(address, bytes(size), bus) for bus, messages in fp.items()
          for address, size in messages.items() if not (bus == 2 and address == 0x370)] + [
            CanData(0x370, raw, 2) for raw in camera]


def receiver(frames: list[CanData]) -> CanRecvCallable:
  def recv(wait_for_one: bool = False) -> list[list[CanData]]:
    return [frames]
  return recv


@pytest.mark.parametrize('index', range(4))
def test_recorded_routes_select_correct_identity_through_can(index):
  fp, camera, vin = recording(index)
  selected, observed = car_helpers.can_fingerprint(receiver(packets(fp, camera)), vin=vin)
  assert selected == RECORDINGS[index]['expected']
  assert observed == fp


@pytest.mark.parametrize('year,expected', [('H', BOLTS[0]), ('J', BOLTS[1]), ('K', BOLTS[1]),
                                          ('L', BOLTS[1]), ('M', BOLTS[1])])
@pytest.mark.parametrize('alias', [None, *BOLTS, CAR.CHEVROLET_BOLT_EUV])
def test_gen1_year_corrects_all_automatic_aliases(year, expected, alias):
  fp, camera, vin = recording(0)
  vin = vin[:9] + year + vin[10:]
  assert automatic_bolt_candidate(alias, fp, camera, vin) == expected


@pytest.mark.parametrize('year', ['N', 'P'])
@pytest.mark.parametrize('pedal', [False, True])
@pytest.mark.parametrize('acc', [False, True])
@pytest.mark.parametrize('alias', [None, *BOLTS, CAR.CHEVROLET_BOLT_EUV])
def test_gen2_capability_selects_all_three_identities(year, pedal, acc, alias):
  fp, camera, vin = recording(3 if acc else 2)
  vin = vin[:9] + year + vin[10:]
  fp[0].pop(0x201, None)
  if pedal:
    fp[0][0x201] = 6
  expected = BOLTS[4 if pedal else 3] if acc else BOLTS[2]
  assert automatic_bolt_candidate(alias, fp, camera, vin) == expected


@pytest.mark.parametrize('camera', [set(), {b'\0\4\0\0\0\0'}, {b'\0\5\0\0\0\0'},
                                  {b'\1\0\0\0\0\0'}, {bytes(6), b'\0\2\0\0\0\0'}])
def test_missing_nonadaptive_or_conflicting_camera_cannot_select_no_acc(camera):
  fp, _, vin = recording(2)
  assert automatic_bolt_candidate(BOLTS[1], fp, camera, vin) is None


@pytest.mark.parametrize('address', [0xD3, 0xBE, 0x130, 0x1CA, 0x236])
def test_missing_generation_evidence_cannot_recover_unknown_candidate(address):
  fp, camera, vin = recording(3)
  del fp[0][address]
  assert automatic_bolt_candidate(None, fp, camera, vin) is None


@pytest.mark.parametrize('vin', ['', '0' * 17, '1G1FX6S00Z0000000', '1G1RA6S00P0000000',
                                '1G1FX6S00P000000I', '1G1FX6S00P'])
def test_invalid_nonbolt_or_unsupported_vin_cannot_recover_unknown_candidate(vin):
  fp, camera, _ = recording(3)
  assert automatic_bolt_candidate(None, fp, camera, vin) is None


def test_wrong_bus_or_dlc_is_not_pedal_evidence():
  fp, camera, vin = recording(3)
  fp[0].pop(0x201)
  assert automatic_bolt_candidate(None, fp, camera, vin) == BOLTS[3]
  fp[0][0x201] = 7
  assert automatic_bolt_candidate(None, fp, camera, vin) is None


@pytest.mark.parametrize('candidate', [CAR.CHEVROLET_VOLT, CAR.CHEVROLET_VOLT_ASCM,
                                     CAR.CHEVROLET_VOLT_2019, CAR.CHEVROLET_MALIBU, 'HYUNDAI IONIQ 6 2023'])
def test_other_identities_are_never_rewritten(candidate):
  fp, camera, vin = recording(3)
  assert automatic_bolt_candidate(candidate, fp, camera, vin) == candidate


def fingerprint_with_firmware(fp, camera, vin, fw_candidate, fixed=''):
  cached = SimpleNamespace(brand='gm', carVin=vin, carFw=[object()])
  with patch.dict(car_helpers.os.environ, {'FINGERPRINT': fixed, 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': ''}), \
       patch.object(car_helpers, 'match_fw_to_car', return_value=(True, {fw_candidate})):
    return car_helpers.fingerprint(receiver(packets(fp, camera)), lambda _: None, lambda _: None, cached)


@pytest.mark.parametrize('index', range(4))
def test_firmware_alias_cannot_undo_vin_refinement(index):
  fp, camera, vin = recording(index)
  result = fingerprint_with_firmware(fp, camera, vin, CAR.CHEVROLET_BOLT_EUV)
  assert result[0] == RECORDINGS[index]['expected']


@pytest.mark.parametrize('manual', BOLTS)
def test_environment_override_wins_after_all_automatic_selection(manual):
  fp, camera, vin = recording(3)
  result = fingerprint_with_firmware(fp, camera, vin, BOLTS[1], fixed=manual)
  assert result[0] == manual
  assert result[4] == structs.CarParams.FingerprintSource.fixed


@pytest.mark.parametrize('manual', BOLTS)
@pytest.mark.parametrize('source', ['forced', 'environment'])
def test_get_car_preserves_exact_manual_identity_and_safety(manual, source):
  fp, _, vin = recording(3)
  # Avoid starting Card/controller: build actual CP, capture its final selection.

  class Factory:
    get_params = CarInterface.get_params

    def __new__(cls, cp):
      return SimpleNamespace(CP=cp)

  fingerprint = (BOLTS[4] if source == 'forced' else manual, fp, vin, [],
                 structs.CarParams.FingerprintSource.can if source == 'forced' else structs.CarParams.FingerprintSource.fixed, True)
  with patch.object(car_helpers, 'fingerprint', return_value=fingerprint), \
       patch.dict(car_helpers.interfaces, {car: Factory for car in BOLTS}), \
       patch.dict(car_helpers.os.environ, {'FINGERPRINT': '' if source == 'forced' else manual}):
    car = car_helpers.get_car(receiver([]), lambda _: None, lambda _: None, False, False,
                             forced_candidate=manual if source == 'forced' else None)
  expected = CarInterface.get_params(manual, fp, [], False, False, False, auto_pedal=False)
  assert car.CP.carFingerprint == manual
  assert [s.to_dict() for s in car.CP.safetyConfigs] == [s.to_dict() for s in expected.safetyConfigs]
  assert bool(car.CP.flags & GMFlags.PEDAL_LONG) == bool(expected.flags & GMFlags.PEDAL_LONG)


@pytest.mark.parametrize('index,pedal,expected', [(0, True, BOLTS[0]), (1, True, BOLTS[1]),
                                                (2, True, BOLTS[2]), (3, False, BOLTS[3]), (3, True, BOLTS[4])])
def test_automatic_selection_reaches_existing_safety_and_torque_profile(index, pedal, expected):
  from opendbc.car.gm.values import CarControllerParams
  fp, camera, vin = recording(index)
  fp[0].pop(0x201, None)
  if pedal:
    fp[0][0x201] = 6
  selected, _ = car_helpers.can_fingerprint(receiver(packets(fp, camera)), vin=vin)
  assert selected == expected
  cp = CarInterface.get_params(selected, fp, [], False, False, False)
  existing = CarInterface.get_params(expected, fp, [], False, False, False)
  assert cp.carFingerprint == expected
  assert [s.to_dict() for s in cp.safetyConfigs] == [s.to_dict() for s in existing.safetyConfigs]
  assert bool(cp.flags & GMFlags.PEDAL_LONG) == pedal
  assert CarControllerParams(cp).STEER_MAX == (450 if index == 0 else 300)


@pytest.mark.parametrize('bus,size', [(0, 6), (128, 6), (130, 6), (2, 5), (2, 7)])
def test_wrong_camera_bus_or_dlc_cannot_select_acc(bus, size):
  fp, camera, vin = recording(3)
  fp[2].pop(0x370, None)
  frames = packets(fp, set()) + [CanData(0x370, next(iter(camera))[:size].ljust(size, b'\0'), bus)]
  selected, _ = car_helpers.can_fingerprint(receiver(frames), vin=vin)
  assert selected is None
