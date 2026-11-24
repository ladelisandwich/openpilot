"""Exact known camera2AA hardware; 2AA is never a status or button lease."""
from opendbc.car import structs
from opendbc.car.toyota.values import CAR, ToyotaFlags
from opendbc.car.toyota.advert_filter import AdvertFilterCANParser

CARS = frozenset((
  CAR.TOYOTA_ALPHARD_TSS2,
  CAR.TOYOTA_AVALON_TSS2,
  CAR.TOYOTA_CAMRY_TSS2,
  CAR.TOYOTA_COROLLA_TSS2,
  CAR.TOYOTA_HIGHLANDER_TSS2,
  CAR.TOYOTA_PRIUS_TSS2,
  CAR.TOYOTA_RAV4_TSS2,
  CAR.TOYOTA_MIRAI,
  CAR.LEXUS_ES_TSS2,
  CAR.LEXUS_NX_TSS2,
  CAR.LEXUS_LC_TSS2,
  CAR.LEXUS_RX_TSS2,
  CAR.LEXUS_IS_TSS2,
  CAR.LEXUS_RC_TSS2,
))
LONG_WORD = 0xC049
STOCK_WORD = 0xC249
WORDS = frozenset((LONG_WORD, STOCK_WORD))


def hardware_supported(candidate, fingerprint, car_fw):
  if (candidate not in CARS or fingerprint.get(0, {}).get(0x2AA) != 8 or
      any(address in messages for messages in fingerprint.values() for address in (0x201, 0x2FF)) or
      any(bus not in (0, 2) or messages[0x2AA] != 8 for bus, messages in fingerprint.items() if 0x2AA in messages) or
      any(fingerprint.get(2, {}).get(address, 8) != 8 for address in (0x343, 0x4CB))):
    return False
  from opendbc.car.toyota.fingerprints import FW_VERSIONS
  known = FW_VERSIONS[candidate]
  required = {structs.CarParams.Ecu.eps, structs.CarParams.Ecu.fwdCamera}
  if any(ecu == structs.CarParams.Ecu.fwdRadar for ecu, _, _ in known):
    required.add(structs.CarParams.Ecu.fwdRadar)
  for ecu in required:
    observed = [fw for fw in car_fw if fw.ecu == ecu]
    if not observed or any(fw.logging or fw.bus != 0 or fw.brand != 'toyota' or
                           bytes(fw.fwVersion) not in known.get((ecu, fw.address, fw.subAddress or None), ()) for fw in observed):
      return False
  return True


def apply(cp):
  if cp is None or cp.brand != 'toyota' or cp.carFingerprint not in CARS or len(cp.safetyConfigs) != 1:
    return False
  cp.flags |= int(ToyotaFlags.RADAR_CAN_FILTER)
  cp.safetyConfigs[0].safetyParam = LONG_WORD
  cp.openpilotLongitudinalControl = True
  cp.alphaLongitudinalAvailable = True
  return True


def configured(cp):
  from opendbc.car.toyota.pedal_composite import parent_cp
  cp = parent_cp(cp)
  try:
    if (cp is None or cp.brand != 'toyota' or cp.carFingerprint not in CARS or cp.notCar or cp.passive or cp.dashcamOnly or
        cp.secOcRequired or not cp.pcmCruise or len(cp.safetyConfigs) != 1 or
        cp.steerControlType != structs.CarParams.SteerControlType.torque):
      return False
    required = int(CAR(cp.carFingerprint).config.flags | ToyotaFlags.RADAR_CAN_FILTER | ToyotaFlags.RAISED_ACCEL_LIMIT)
    config = cp.safetyConfigs[0]
    return bool(cp.flags & required == required and not int(cp.flags) & ~int(required | ToyotaFlags.HYBRID | ToyotaFlags.HAS_BSM) and
                config.safetyModel == structs.CarParams.SafetyModel.toyota and
                config.safetyParam == (LONG_WORD if cp.openpilotLongitudinalControl else STOCK_WORD) and
                cp.autoResumeSng == cp.openpilotLongitudinalControl)
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False


def enabled(cp):
  return configured(cp) and cp.openpilotLongitudinalControl


def prepare_stock(cp):
  from opendbc.car.toyota import pedal_composite
  if pedal_composite.marked(cp):
    return pedal_composite.prepare_stock(cp)
  if not configured(cp):
    return False
  cp.openpilotLongitudinalControl = False
  cp.safetyConfigs[0].safetyParam = STOCK_WORD
  cp.autoResumeSng = False
  return True


class CameraTwoAACANParser(AdvertFilterCANParser):
  """Real MAIN/EPS authority; contradictory hardware identity stays refused."""
  def __init__(self, *args):
    super().__init__(*args)
    self.hardware_ambiguous = False

  def update(self, strings, sendcan=False):
    batches = []
    for stamp, frames in strings:
      accepted = []
      for address, data, bus in frames:
        # pandad marks TX confirmations/rejections with bus >= 128; they
        # are not received hardware (see panda.h CAN_*_BUS_OFFSET).
        if bus >= 128:
          continue
        if address in (0x201, 0x2FF):
          self.hardware_ambiguous = True
          continue
        if address == 0x2AA:
          self.hardware_ambiguous |= bus not in (0, 2) or len(data) != 8
          continue
        accepted.append((address, data, bus))
      batches.append((stamp, accepted))
    result = super().update(batches, sendcan)
    if self.hardware_ambiguous:
      self.main_valid = self.eps_valid = False
    return result
