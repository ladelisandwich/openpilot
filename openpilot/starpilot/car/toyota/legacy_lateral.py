"""Keep the exact smartDSU lateral owner separate from ACC's minimum speed."""
from opendbc.car import structs
from opendbc.car.toyota.values import CAR
from opendbc.car.toyota.rav4h_filter import configured
from opendbc.car.toyota import dsu_bypass, legacy_sdsu, legacy_advert_filter
from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS, INTENT_MAX_AGE_NS, SAFETY_MAX_AGE_NS, native_inventory_matches_cp


def below_engage_lateral_allowed(cp, cs, intent, native, *, source_ns, now_ns, axis_session_id):
  try:
    bypass = dsu_bypass.configured(cp)
    legacy_advert = legacy_advert_filter.configured(cp)
    return bool((bypass or configured(cp) or legacy_advert) and
      (bypass or cp.carFingerprint in (CAR.TOYOTA_HIGHLANDER, CAR.TOYOTA_AVALON) or legacy_sdsu.configured(cp) or legacy_advert) and
      cp.alternativeExperience == 32 and
      cp.steerAtStandstill and cp.minSteerSpeed <= 0 and cp.minEnableSpeed > 0 and 0 <= cs.vEgo < cp.minEnableSpeed and
      intent is not None and intent.settingsQualified and intent.allowedLatch and intent.lateralArmed and not intent.pauseLateral and
      intent.producerSessionId and intent.carStateLogMonoTime == source_ns and
      0 < source_ns <= intent.observedMonoTime <= now_ns <= intent.validUntilMonoTime and
      now_ns-source_ns <= AXIS_MAX_AGE_NS and now_ns-intent.observedMonoTime <= INTENT_MAX_AGE_NS and
      intent.validUntilMonoTime-intent.observedMonoTime <= SAFETY_MAX_AGE_NS and
      cs.canValid and not cs.canTimeout and cs.cruiseState.available and not cs.steerFaultTemporary and not cs.steerFaultPermanent and
      not cs.accFaulted and not cs.stockAeb and not cs.doorOpen and
      cs.gearShifter in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.sport, structs.CarState.GearShifter.low) and
      native is not None and native.compatible and native.protocolVersion == 1 and axis_session_id and native.axisSessionId == axis_session_id and
      native.pandaInventory and native_inventory_matches_cp(native, cp) and
      0 < native.observedMonoTime <= now_ns <= native.validUntilMonoTime and
      now_ns-native.observedMonoTime <= SAFETY_MAX_AGE_NS and native.validUntilMonoTime-native.observedMonoTime <= SAFETY_MAX_AGE_NS and
      # Permit an initial request, but do not hide an actual native rejection.
      (native.lateralAllowed or not native.requestedLateral))
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False
