"""Exact PE recognition without expanding Hyundai's custom fuzzy coverage."""
from copy import deepcopy
from itertools import product
import unittest
from unittest.mock import patch

from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.hyundai import values
from opendbc.car.hyundai.values import CAR, IONIQ5_PE_EXACT_CAMERA_FW
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu
PE = CAR.HYUNDAI_IONIQ_5_PE
CAMERA = (Ecu.fwdCamera, 0x7c4, None)
RADAR = (Ecu.fwdRadar, 0x7d0, None)
CAMERA_FW = b'\xf1\x00NE  MFC  AT USA LHD 1.00 1.10 99211-PI110 260204'
RADAR_FW = b'\xf1\x00NE__ RDR -----      1.00 1.01 99110-PI000         '
ADAS_FW = b'\xf1\x00NEaPFBL5 1.00 1.10 260204'


def firmware(key, version, *, bus=1, logging=False):
  ecu, address, sub_address = key
  return CarParams.CarFw(ecu=ecu, address=address, subAddress=sub_address or 0,
                         fwVersion=version, brand='hyundai', bus=bus, logging=logging)


def recording():
  # The same radar response on two buses is one ECU in the production dictionary.
  return [firmware(CAMERA, CAMERA_FW), firmware(RADAR, RADAR_FW, bus=0), firmware(RADAR, RADAR_FW),
          firmware((Ecu.adas, 0x730, None), ADAS_FW, logging=True)]


class TestIoniq5PEFirmware(unittest.TestCase):
  def assert_exact_pe(self, entries):
    for options in ({'allow_fuzzy': False}, {}):
      self.assertEqual(match_fw_to_car(entries, '00000000000000000', log=False, **options), (True, {PE}))

  def assert_no_pe(self, entries):
    for options in ({'allow_fuzzy': False}, {}):
      self.assertNotIn(PE, match_fw_to_car(entries, '00000000000000000', log=False, **options)[1])

  def test_recorded_firmware_uses_global_exact_and_default_matchers(self):
    self.assertEqual(IONIQ5_PE_EXACT_CAMERA_FW, CAMERA_FW)
    for route in ('00000003--257ac116da', '00000004--5172ba470c'):
      with self.subTest(route=route):
        entries = recording()
        self.assertEqual(build_fw_dict(entries), {(0x7c4, None): {CAMERA_FW}, (0x7d0, None): {RADAR_FW}})
        self.assert_exact_pe(entries)

  def test_all_existing_and_new_camera_radar_combinations_remain_exact(self):
    for camera, radar in product(FW_VERSIONS[PE][CAMERA], FW_VERSIONS[PE][RADAR]):
      with self.subTest(camera=camera, radar=radar):
        self.assert_exact_pe([firmware(CAMERA, camera), firmware(RADAR, radar)])

  def test_missing_or_logging_only_required_ecu_is_rejected(self):
    for required in (CAMERA, RADAR):
      for logging in (False, True):
        with self.subTest(required=required, logging=logging):
          entries = [fw for fw in recording() if fw.address != required[1]]
          if logging:
            entries.append(firmware(required, CAMERA_FW if required == CAMERA else RADAR_FW, logging=True))
          self.assert_no_pe(entries)

  def test_sibling_ecu_substitutions_are_rejected(self):
    for sibling, required in product((CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_IONIQ_5_N, CAR.KIA_EV9), (CAMERA, RADAR)):
      for version in FW_VERSIONS[sibling][required]:
        with self.subTest(sibling=sibling, required=required, version=version):
          entries = [fw for fw in recording() if fw.address != required[1]]
          self.assert_no_pe(entries + [firmware(required, version)])

  def test_unrecorded_camera_revisions_dates_and_parts_are_rejected(self):
    unknown = [CAMERA_FW.replace(b'1.10', b'1.09'), CAMERA_FW.replace(b'1.10', b'1.11'),
               CAMERA_FW.replace(b'260204', b'260203'), CAMERA_FW.replace(b'260204', b'260205'),
               CAMERA_FW.replace(b'260204', b'250407'), CAMERA_FW.replace(b'PI110', b'PI120')]
    for version in unknown:
      with self.subTest(version=version):
        self.assert_no_pe([firmware(CAMERA, version), firmware(RADAR, RADAR_FW)])

  def test_old_part_fuzzy_date_envelope_is_not_extended(self):
    for camera, date in product(FW_VERSIONS[PE][CAMERA], (b'250514', b'260204')):
      if camera == CAMERA_FW:
        continue
      version = camera[:-6] + date
      with self.subTest(version=version):
        self.assert_no_pe([firmware(CAMERA, version), firmware(RADAR, RADAR_FW)])

  def test_complete_family_fuzzy_code_and_date_sets_are_unchanged(self):
    baseline = deepcopy(FW_VERSIONS)
    baseline[PE][CAMERA].remove(CAMERA_FW)
    baseline[PE][RADAR].remove(RADAR_FW)

    def expected_codes(database):
      result = {}
      original = values.get_platform_codes
      for candidate, ecus in database.items():
        result[candidate] = {}
        for key, versions in ecus.items():
          if key[0] not in values.PLATFORM_CODE_ECUS:
            continue
          calls = []

          def capture(versions, calls=calls):
            codes = original(versions)
            calls.append(codes)
            return codes

          # Isolate each ECU so an unrelated missing date cannot short circuit
          # inspection of later ECUs in a platform's database.
          live = {key[1:]: set(FW_VERSIONS[candidate][key])}
          with patch.object(values, 'get_platform_codes', side_effect=capture):
            values.match_fw_to_car_fuzzy(live, '', {candidate: {key: versions}})
          self.assertEqual(len(calls), 2, (candidate, key))
          result[candidate][key] = calls[0]
      return result

    before, after = expected_codes(baseline), expected_codes(FW_VERSIONS)
    self.assertEqual(before, after)
    codes = after[PE][CAMERA]
    self.assertEqual({code for code, _ in codes}, {b'NE-PI000', b'NE-PI010', b'NE-GI500', b'NE-GI510'})
    self.assertEqual((min(date for _, date in codes), max(date for _, date in codes)), (b'240809', b'250513'))


if __name__ == '__main__':
  unittest.main()
