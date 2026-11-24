"""Exact MQB stock-ACC capability; direct drive uses its own checked gear source."""
from opendbc.car import Bus, structs
from opendbc.car.volkswagen.values import CAR, DBC, VolkswagenFlags

AOL_WORD = 8
MANUAL_WORD = 16
DIRECT_WORD = 32
AOL_WORDS = frozenset((8, 24, 40))
MQB_CARS = frozenset(car for car in CAR if DBC[car].get(Bus.pt) == "vw_mqb" and
                     not car.config.flags & (VolkswagenFlags.PQ | VolkswagenFlags.MLB | VolkswagenFlags.MEB))
KNOWN_FLAGS = int(VolkswagenFlags.STOCK_HCA_PRESENT | VolkswagenFlags.KOMBI_PRESENT | VolkswagenFlags.HAS_BSM)


def safety_config(cp):
  """One owner, optionally preceded by one quiet noOutput board."""
  try:
    configs = cp.safetyConfigs
    if len(configs) == 1:
      return configs[0]
    if (len(configs) == 2 and configs[0].safetyModel == structs.CarParams.SafetyModel.noOutput and
        configs[0].safetyParam == 0):
      return configs[1]
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
    pass
  return None


def _qualified(cp, *, marked_only=False, longitudinal=False):
  if (cp is None or cp.brand != "volkswagen" or cp.carFingerprint not in MQB_CARS or
      cp.passive or cp.notCar or cp.dashcamOnly or cp.flags & ~KNOWN_FLAGS or
      cp.steerControlType != structs.CarParams.SteerControlType.torque or
      cp.transmissionType not in (structs.CarParams.TransmissionType.automatic, structs.CarParams.TransmissionType.manual,
                                 structs.CarParams.TransmissionType.direct) or
      cp.networkLocation not in (structs.CarParams.NetworkLocation.gateway, structs.CarParams.NetworkLocation.fwdCamera) or
      bool(cp.openpilotLongitudinalControl) != longitudinal or bool(cp.pcmCruise) == longitudinal or
      (longitudinal and (cp.networkLocation != structs.CarParams.NetworkLocation.gateway or not cp.alphaLongitudinalAvailable))):
    return False
  word = AOL_WORD | (MANUAL_WORD if cp.transmissionType == structs.CarParams.TransmissionType.manual else 0)
  word |= DIRECT_WORD if cp.transmissionType == structs.CarParams.TransmissionType.direct else 0
  word |= int(longitudinal)
  safety = safety_config(cp)
  return (safety is not None and safety.safetyModel == structs.CarParams.SafetyModel.volkswagen and
          (cp.alternativeExperience == 32 and safety.safetyParam == word or
           not marked_only and cp.alternativeExperience == 0 and safety.safetyParam == int(longitudinal)))


def temporary_restriction(cp, state):
  gears = structs.CarState.GearShifter
  return bool(state is not None and qualified(cp, marked_only=True) and state.canValid and not state.canTimeout and
              not state.steerFaultPermanent and (state.steerFaultTemporary or state.standstill or
              state.gearShifter not in (gears.drive, gears.sport, gears.eco, gears.manumatic)))


def native_observation(native, *, latched, panda_ready, restricted, now_ns, pending_since_ns):
  lost = not panda_ready or native is None
  if not latched or lost or restricted:
    return 0, lost, False
  if native.requestedLateral and native.lateralAllowed:
    return 0, False, False
  pending_since_ns = pending_since_ns or now_ns
  return pending_since_ns, False, now_ns - pending_since_ns >= 200_000_000


def qualified(cp, *, marked_only=False):
  try:
    return bool(_qualified(cp, marked_only=marked_only))
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
    return False
