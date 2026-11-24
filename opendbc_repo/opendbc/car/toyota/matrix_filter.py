"""Explicit manual Matrix enrollment into existing EPS88 filter protocols.

This identity is manually selected, not a firmware-catalog match. The filter
never supplies physical MAIN or EPS authority. Their current native sources,
checksums, health checks and permissions remain required.
"""
from opendbc.car import structs
from opendbc.car.toyota.values import CAR, ToyotaFlags

PROTOCOLS = {8: (0x9158, 0x9358), 4: (0x9558, 0x9758)}


def hardware_supported(candidate, fingerprint, car_fw, *, length):
  if (candidate != CAR.TOYOTA_MATRIX_RETROFIT or length not in PROTOCOLS or
      fingerprint.get(0, {}).get(0x2FF) != length or
      any(0x2FF in messages and (bus not in (0, 2) or messages[0x2FF] != length)
          for bus, messages in fingerprint.items()) or
      any(address in messages for messages in fingerprint.values() for address in (0x201, 0x2AA)) or
      any(fingerprint.get(0, {}).get(address, size) != size for address, size in ((0x1D3, 8), (0x262, 5))) or
      any(fingerprint.get(2, {}).get(address, 8) != 8 for address in (0x343, 0x4CB))):
    return False
  # A Matrix is not hybrid; contradictory firmware must not reach a composite assertion.
  if any(fw.ecu == structs.CarParams.Ecu.hybrid for fw in car_fw):
    return False
  # Preserve the manual-selection boundary without inventing catalog replies.
  critical = (structs.CarParams.Ecu.eps, structs.CarParams.Ecu.fwdCamera, structs.CarParams.Ecu.fwdRadar)
  return not any(fw.ecu in critical and (fw.logging or fw.bus != 0 or fw.brand != 'toyota' or not bytes(fw.fwVersion))
                 for fw in car_fw)


def configured(cp, *, length):
  if (cp is None or cp.brand != 'toyota' or cp.carFingerprint != CAR.TOYOTA_MATRIX_RETROFIT or
      length not in PROTOCOLS or cp.notCar or cp.passive or cp.dashcamOnly or cp.secOcRequired or
      not cp.pcmCruise or cp.steerControlType != structs.CarParams.SteerControlType.torque or
      len(cp.safetyConfigs) != 1 or cp.autoResumeSng or cp.alphaLongitudinalAvailable):
    return False
  flags = ToyotaFlags.LONG_FILTER if length == 8 else ToyotaFlags.RADAR_CAN_FILTER
  config = cp.safetyConfigs[0]
  return (cp.flags == int(flags) and config.safetyModel == structs.CarParams.SafetyModel.toyota and
          config.safetyParam == PROTOCOLS[length][not cp.openpilotLongitudinalControl])
