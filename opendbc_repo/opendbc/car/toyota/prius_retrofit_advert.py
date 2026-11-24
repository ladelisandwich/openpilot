"""EPS73 Prius four-byte filter identity, independent of status and physical MAIN."""
from opendbc.car import structs
from opendbc.car.toyota.values import CAR, ToyotaFlags

LONG_WORD = 0xD049
STOCK_WORD = 0xD249
WORDS = frozenset((LONG_WORD, STOCK_WORD))
EPS_FIRMWARE = b'8965B47070' + bytes(6)
CARS = (CAR.TOYOTA_PRIUS, CAR.TOYOTA_PRIUS_RETROFIT)


def hardware_supported(candidate, fingerprint, car_fw):
  if (candidate not in CARS or fingerprint.get(0, {}).get(0x2FF) != 4 or
      any(0x2FF in messages and (bus not in (0, 2) or messages[0x2FF] != 4) for bus, messages in fingerprint.items()) or
      any(address in messages for messages in fingerprint.values() for address in (0x201, 0x2AA)) or
      any(fingerprint.get(0, {}).get(address, 8) != 8 for address in (0x1D3, 0x262)) or
      any(fingerprint.get(2, {}).get(address, 8) != 8 for address in (0x343, 0x4CB))):
    return False
  eps = [fw for fw in car_fw if fw.ecu == structs.CarParams.Ecu.eps]
  if not eps or any(fw.logging or fw.bus != 0 or fw.brand != 'toyota' or fw.address != 0x7A1 or fw.subAddress or
                    bytes(fw.fwVersion) != EPS_FIRMWARE for fw in eps):
    return False
  from opendbc.car.toyota.fingerprints import FW_VERSIONS
  known = FW_VERSIONS[CAR.TOYOTA_PRIUS]
  # Cap'n Proto enum readers compare equal to Ecu constants but have different
  # hashes. Use the catalog's static enum value for each dictionary lookup.
  for ecu in (structs.CarParams.Ecu.fwdCamera, structs.CarParams.Ecu.fwdRadar):
    if any(fw.logging or fw.bus != 0 or fw.brand != 'toyota' or
           bytes(fw.fwVersion) not in known.get((ecu, fw.address, fw.subAddress or None), ()) for fw in car_fw if fw.ecu == ecu):
      return False
  return True


def configured(cp):
  from opendbc.car.toyota.pedal_composite import parent_cp
  cp = parent_cp(cp)
  if (cp is None or cp.brand != 'toyota' or cp.carFingerprint not in CARS or cp.notCar or cp.passive or cp.dashcamOnly or
      cp.secOcRequired or not cp.pcmCruise or cp.steerControlType != structs.CarParams.SteerControlType.torque or
      len(cp.safetyConfigs) != 1 or cp.alphaLongitudinalAvailable or cp.autoResumeSng):
    return False
  flags = int(ToyotaFlags.HYBRID | ToyotaFlags.LONG_FILTER | ToyotaFlags.RAISED_ACCEL_LIMIT)
  config = cp.safetyConfigs[0]
  return (cp.flags & ~int(ToyotaFlags.ZSS) == flags and config.safetyModel == structs.CarParams.SafetyModel.toyota and
          config.safetyParam == (LONG_WORD if cp.openpilotLongitudinalControl else STOCK_WORD))
