"""Exact EPS5 smartDSU profiles with alternate brake and DSU MAIN sources."""

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.toyota.values import CAR, ToyotaFlags

PROFILES = {
  CAR.TOYOTA_COROLLA: (0x9158, 0x9358),
  CAR.TOYOTA_MATRIX_RETROFIT: (0x9158, 0x9358),
  CAR.TOYOTA_RAV4: (0x9149, 0x9349),
  CAR.LEXUS_ES: (0x9149, 0x9349),
  CAR.TOYOTA_PRIUS_V: (0x9164, 0x9364),
  CAR.LEXUS_CTH: (0x9164, 0x9364),
  CAR.LEXUS_GS_F: (0xD149, 0xD349),
  CAR.LEXUS_IS: (0xD04D, 0xD24D),
  CAR.LEXUS_RC: (0xD04D, 0xD24D),
}
WORDS = frozenset(word for pair in PROFILES.values() for word in pair)
ALT_MAIN_CARS = frozenset((CAR.LEXUS_GS_F, CAR.LEXUS_IS, CAR.LEXUS_RC))


def hardware_supported(candidate, fingerprint, car_fw):
  if candidate == CAR.TOYOTA_MATRIX_RETROFIT:
    from opendbc.car.toyota import matrix_filter
    return matrix_filter.hardware_supported(candidate, fingerprint, car_fw, length=8)
  if (
    candidate not in PROFILES
    or fingerprint.get(0, {}).get(0x2FF) != 8
    or any(0x2FF in messages and (bus != 2 or messages[0x2FF] != 8) for bus, messages in fingerprint.items() if bus != 0)
    or any(address in messages for messages in fingerprint.values() for address in (0x201, 0x2AA))
    or any(fingerprint.get(2, {}).get(address, 8) != 8 for address in (0x343, 0x4CB))
  ):
    return False
  from opendbc.car.toyota.fingerprints import FW_VERSIONS

  known = FW_VERSIONS[candidate]
  eps = structs.CarParams.Ecu.eps
  required = {structs.CarParams.Ecu.fwdCamera, structs.CarParams.Ecu.fwdRadar}
  # CT/PriusV do not expose an EPS catalog response. Bind their catalog engine
  # identity instead; never invent an EPS response to pass enrollment.
  required.add(eps if any(ecu == eps for ecu, _, _ in known) else structs.CarParams.Ecu.engine)
  for ecu in required | {eps}:
    observed = [fw for fw in car_fw if fw.ecu == ecu]
    if ecu in required and not observed:
      return False
    if any(
      fw.logging or fw.bus != 0 or fw.brand != "toyota" or bytes(fw.fwVersion) not in known.get((ecu, fw.address, fw.subAddress or None), ()) for fw in observed
    ):
      return False
  return True


def apply(cp):
  cp.flags |= int(ToyotaFlags.LONG_FILTER)
  cp.alphaLongitudinalAvailable = False
  cp.openpilotLongitudinalControl = True
  cp.safetyConfigs[0].safetyParam = PROFILES[cp.carFingerprint][0]


def configured(cp):
  from opendbc.car.toyota.pedal_composite import parent_cp
  cp = parent_cp(cp)
  if cp is not None and cp.carFingerprint == CAR.TOYOTA_MATRIX_RETROFIT:
    from opendbc.car.toyota import matrix_filter
    return matrix_filter.configured(cp, length=8)
  if (
    cp is None
    or cp.brand != "toyota"
    or cp.carFingerprint not in PROFILES
    or cp.notCar
    or cp.passive
    or cp.dashcamOnly
    or cp.secOcRequired
    or not cp.pcmCruise
    or len(cp.safetyConfigs) != 1
    or cp.steerControlType != structs.CarParams.SteerControlType.torque
  ):
    return False
  required = int(CAR(cp.carFingerprint).config.flags | ToyotaFlags.LONG_FILTER)
  if cp.flags & required != required or int(cp.flags) & ~int(required | ToyotaFlags.HYBRID):
    return False
  config = cp.safetyConfigs[0]
  return config.safetyModel == structs.CarParams.SafetyModel.toyota and config.safetyParam == PROFILES[cp.carFingerprint][not cp.openpilotLongitudinalControl]


def prepare_stock(cp):
  from opendbc.car.toyota import pedal_composite
  if pedal_composite.marked(cp):
    return pedal_composite.prepare_stock(cp)
  if not configured(cp):
    return False
  cp.openpilotLongitudinalControl = False
  cp.safetyConfigs[0].safetyParam = PROFILES[cp.carFingerprint][1]
  cp.autoResumeSng = False
  return True


class AlternateMainSDSUCANParser(CANParser):
  """Neither malformed DSU MAIN nor malformed status can renew its source."""

  def __init__(self, *args):
    super().__init__(*args)
    self.main_valid = False

  def update(self, strings, sendcan=False):
    batches = []
    for stamp, frames in strings:
      accepted = []
      for address, data, bus in frames:
        if bus == self.bus:
          if address == 0x365:
            self.main_valid = len(data) == 7
            if not self.main_valid:
              continue
          if address == 0x2FF and len(data) != 8:
            continue
        accepted.append((address, data, bus))
      batches.append((stamp, accepted))
    return super().update(batches, sendcan)
