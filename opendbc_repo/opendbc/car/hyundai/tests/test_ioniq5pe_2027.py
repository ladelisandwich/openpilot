"""2027 PE route firmware and unchanged angle transport regression."""
import json
from pathlib import Path
import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.ioniq5pe import qualified
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.values import CAR, HyundaiFlags

FIXTURE = json.loads((Path(__file__).parent / 'testdata/ioniq5pe_2027.json').read_text())


def firmware():
  return [structs.CarParams.CarFw(ecu=row['ecu'], address=row['address'], fwVersion=bytes.fromhex(row['fw']), brand='hyundai')
          for row in FIXTURE['firmware']]


def params(*, alpha=False, release=False):
  fingerprint = gen_empty_fingerprint()
  for bus, entries in FIXTURE['fingerprint'].items():
    fingerprint[int(bus)].update({int(address): length for address, length in entries.items()})
  return CarInterface.get_params(CAR.HYUNDAI_IONIQ_5_PE, fingerprint, firmware(), alpha, release, False)


class TestIoniq5PE2027(unittest.TestCase):
  def test_recorded_camera_and_radar_match_exactly_without_manual_selection(self):
    exact, matched = match_fw_to_car(firmware(), '', allow_fuzzy=False, log=False)
    self.assertTrue(exact)
    self.assertEqual(matched, {CAR.HYUNDAI_IONIQ_5_PE})

  def test_camera_only_or_unknown_camera_cannot_select_the_pe(self):
    fw = firmware()
    camera = [item for item in fw if item.ecu == structs.CarParams.Ecu.fwdCamera]
    self.assertNotIn(CAR.HYUNDAI_IONIQ_5_PE, match_fw_to_car(camera, '', allow_fuzzy=False, log=False)[1])
    for item in fw:
      if item.ecu == structs.CarParams.Ecu.fwdCamera:
        item.fwVersion = item.fwVersion.replace(b'99211-PI110', b'99211-PI999')
    self.assertNotIn(CAR.HYUNDAI_IONIQ_5_PE, match_fw_to_car(fw, '', allow_fuzzy=False, log=False)[1])

  def test_recorded_topology_keeps_existing_stock_angle_safety_and_tune(self):
    for alpha in (False, True):
      for release in (False, True):
        with self.subTest(alpha=alpha, release=release):
          cp = params(alpha=alpha, release=release)
          self.assertTrue(qualified(cp))
          self.assertEqual(cp.safetyConfigs[-1].safetyParam, 0x5491)
          self.assertEqual(cp.steerControlType, structs.CarParams.SteerControlType.angle)
          self.assertTrue(cp.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT)
          self.assertFalse(cp.openpilotLongitudinalControl)
          self.assertEqual(cp.alphaLongitudinalAvailable, not release)
          self.assertTrue(cp.pcmCruise)
          self.assertFalse(cp.radarUnavailable)
          self.assertAlmostEqual(cp.wheelbase, 2.97)
          self.assertAlmostEqual(cp.steerRatio, 14.26, places=6)

  def test_frozen_angle_camera_and_mdps_messages_retain_existing_crc_layout(self):
    required = {(2, '0x110', 32), (2, '0x362', 32), (1, '0xea', 24), (1, '0x35', 32), (1, '0x1a0', 32)}
    found = set()
    for message in FIXTURE['frames']:
      key = (message['bus'], message['address'], message['length'])
      if key not in required:
        continue
      found.add(key)
      for sample in message['samples']:
        data = bytes.fromhex(sample['hex'])
        self.assertEqual(len(data), message['length'])
        self.assertEqual(int.from_bytes(data[:2], 'little'), hkg_can_fd_checksum(int(message['address'], 16), None, bytearray(data)))
    self.assertEqual(found, required)
