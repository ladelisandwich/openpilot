from copy import deepcopy
from types import SimpleNamespace
import unittest

from opendbc.car.hyundai.lane_body import CAMERA_FW, RADAR_FW, dimensions


def identity():
  # A generated checksum-valid metadata fixture, not a captured vehicle VIN.
  vin = list('KMHAAAAA0PA000001')
  values = dict(zip('0123456789ABCDEFGHJKLMNPRSTUVWXYZ', '012345678912345678123457923456789', strict=True))
  remainder = sum(int(values[c]) * w for c, w in zip(vin, (8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2), strict=True)) % 11
  vin[8] = 'X' if remainder == 10 else str(remainder)
  firmware = [SimpleNamespace(ecu=kind, fwVersion=version, address=address, subAddress=0, bus=1, brand='hyundai', logging=False)
              for kind, version, address in (('fwdCamera', CAMERA_FW, 0x7c4), ('fwdRadar', RADAR_FW, 0x7d0))]
  return SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_IONIQ_6', fuzzyFingerprint=False, carVin=''.join(vin), carFw=firmware)


class TestLaneBody(unittest.TestCase):
  def test_corroborated_metadata_is_read_only_and_has_explicit_body_height(self):
    cp = identity()
    before = deepcopy(cp)
    self.assertEqual(dimensions(cp), (1.880, 4.855, 1.495))
    self.assertEqual(cp, before)

  def test_absent_or_conflicting_identity_has_no_positive_bounds(self):
    for field, value in (('brand', 'other'), ('carFingerprint', 'HYUNDAI_IONIQ_5'), ('fuzzyFingerprint', True),
                         ('fuzzyFingerprint', None), ('carVin', ''), ('carVin', '0' * 17), ('carFw', [])):
      cp = identity()
      setattr(cp, field, value)
      self.assertIsNone(dimensions(cp))
    cp = identity()
    for index, value in ((9, 'R'), (8, '0'), (6, '1'), (0, 'J')):
      changed = deepcopy(cp)
      changed.carVin = cp.carVin[:index] + value + cp.carVin[index + 1:]
      self.assertIsNone(dimensions(changed))
    self.assertIsNone(dimensions(object()))

  def test_unrecognized_incomplete_logging_or_conflicting_firmware_is_refused(self):
    for index in (0, 1):
      for field, value in (('fwVersion', b'unknown'), ('brand', 'other'), ('address', 0), ('subAddress', 1), ('bus', 4), ('logging', True)):
        cp = identity()
        setattr(cp.carFw[index], field, value)
        self.assertIsNone(dimensions(cp))
      cp = identity()
      cp.carFw.pop(index)
      self.assertIsNone(dimensions(cp))
    cp = identity()
    conflict = deepcopy(cp.carFw[0])
    conflict.fwVersion = b'other'
    cp.carFw.append(conflict)
    self.assertIsNone(dimensions(cp))


if __name__ == '__main__':
  unittest.main()
