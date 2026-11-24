"""Source-owned MEB stock ACC option; upstream system-long remains the default."""
from opendbc.car import structs
from opendbc.car.volkswagen.values import CAR, VolkswagenFlags
from opendbc.car.volkswagen.stock_aol import safety_config

STOCK_WORD = 4
AOL_WORD = 8
ALT_GEAR_WORD = 16
MEB_CARS = frozenset(c for c in CAR if c.config.flags & VolkswagenFlags.MEB)
KNOWN_FLAGS = int(VolkswagenFlags.MEB | VolkswagenFlags.MEB_GEN2 | VolkswagenFlags.ALT_GEAR |
                  VolkswagenFlags.STOCK_KLR_PRESENT | VolkswagenFlags.HAS_BSM)
AOL_WORDS = frozenset((8, 10, 12, 14, 24, 26, 28, 30))


def base_word(cp):
  return (2 if cp.flags & VolkswagenFlags.MEB_GEN2 else 0) | (0 if cp.openpilotLongitudinalControl else STOCK_WORD)


def profile_word(cp):
  return base_word(cp) | AOL_WORD | (ALT_GEAR_WORD if cp.flags & VolkswagenFlags.ALT_GEAR else 0)


def supported(cp):
  try:
    return bool(cp.brand == 'volkswagen' and cp.carFingerprint in MEB_CARS and
      cp.flags & VolkswagenFlags.MEB and not cp.flags & ~KNOWN_FLAGS and not cp.notCar and not cp.dashcamOnly and
      cp.networkLocation == structs.CarParams.NetworkLocation.gateway and
      cp.transmissionType == structs.CarParams.TransmissionType.direct and
      cp.steerControlType == structs.CarParams.SteerControlType.curvature and safety_config(cp) is not None and
      safety_config(cp).safetyModel == structs.CarParams.SafetyModel.volkswagenMeb and
      safety_config(cp).safetyParam in (base_word(cp), profile_word(cp)))
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
    return False


def qualified(cp, *, marked_only=False):
  try:
    if not supported(cp) or cp.passive or cp.openpilotLongitudinalControl == cp.pcmCruise:
      return False
    word = int(safety_config(cp).safetyParam)
    return ((cp.alternativeExperience == 32 and word == profile_word(cp)) or
            (not marked_only and cp.alternativeExperience == 0 and word == base_word(cp)))
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
    return False


def prepare_stock(cp, requested):
  if requested and supported(cp):
    cp.openpilotLongitudinalControl = False
    cp.pcmCruise = True
    safety_config(cp).safetyParam |= STOCK_WORD


def settings_capable(cp):
  try:
    return supported(cp) and not cp.passive
  except AttributeError:
    return False
