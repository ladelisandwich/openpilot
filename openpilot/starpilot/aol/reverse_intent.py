"""Retain established LAT intent across a physical Reverse/native withdrawal join."""

from opendbc.car import structs
from openpilot.cereal import log
from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS, current_native
from openpilot.starpilot.aol.transport_pause import cp_identity


def retain_reverse_intent(feedback, sm, cp, cs, axis, now_ns, *, latched, producer, qualified_cp,
                          extra_user_events=(), informational_events=(), allow_withdrawal_join=False):
  """Retain only an existing intent under its original fresh LAT-only proof.

  This never grants output or renews a healthy/source/native/output deadline.
  A subsequent OFF proof must satisfy the unchanged configured-native path.
  """
  try:
    if (
      not qualified_cp
      or not latched
      or feedback.failed
      or feedback.paused
      or feedback.pending_event
      or feedback.awaiting
      or feedback.retention_started_ns
      or feedback.retention_pending
      or feedback.retention_qualified
      or feedback.source_retention_started_ns
      or feedback.cp != cp_identity(cp)
      or cs.gearShifter != structs.CarState.GearShifter.reverse
      or not cs.canValid
      or cs.canTimeout
      or cs.steerFaultTemporary
      or cs.steerFaultPermanent
      or cs.vehicleSensorsInvalid
      or cs.accFaulted
      or cs.gasPressed
      or cs.steeringDisengage
      or not cs.cruiseState.available
      or cs.buttonEvents
    ):
      return False
    previous = feedback.signature
    stamp = int(sm.logMonoTime['aolAxisState'])
    active = bool(str(axis.mode) == 'lateralOnly' and axis.desiredLateral and axis.lateralActive and axis.nativeAcknowledged)
    # Separate producer/native sockets can deliver a fresh OFF receipt before
    # the axis producer acknowledges it. Retain only the original LAT intent.
    withdrawing = bool(allow_withdrawal_join and str(axis.mode) == 'off' and
      not axis.desiredLateral and not axis.lateralActive and not axis.nativeAcknowledged)
    if (
      not feedback.session
      or str(axis.sessionId) != feedback.session
      or axis.faultSessionId != axis.sessionId
      or str(axis.faultReason) != 'none'
      or not (active or withdrawing)
      or not axis.qualified
      or axis.desiredLongitudinal
      or axis.longitudinalActive
      or not sm.seen['aolAxisState']
      or not sm.valid['aolAxisState']
      or not sm.alive['aolAxisState']
      or not 0 < axis.sourceCarStateMonoTime <= stamp == axis.observedMonoTime <= now_ns <= axis.validUntilMonoTime
      or now_ns - axis.sourceCarStateMonoTime > AXIS_MAX_AGE_NS
      or axis.validUntilMonoTime - stamp > AXIS_MAX_AGE_NS
      or not producer
      or producer != feedback.availability_producer
      or str(axis.intentProducerSessionId) != producer
      or int(axis.intentIntegrityEpoch) != feedback.availability_epoch
      or feedback.availability_epoch is None
      or not feedback.lateral_only_baseline
      or not feedback.lateral_inventory
      or previous is None
      or not 0 < feedback.lateral_source_ns <= feedback.lateral_baseline_ns <= now_ns
      or now_ns - feedback.lateral_source_ns > AXIS_MAX_AGE_NS
      or previous[0] != feedback.session
      or previous[7] != feedback.session
      or previous[5] != 'none'
      or not previous[9]
      or not previous[11]
      or not previous[13]
      or previous[10]
      or previous[12]
      or previous[14] != producer
      or previous[15] != feedback.availability_epoch
      or not 0 < previous[8] <= previous[2] <= now_ns <= previous[4]
      or now_ns - previous[8] > AXIS_MAX_AGE_NS
      or axis.sourceCarStateMonoTime < previous[8]
      or stamp < previous[2]
      or not (
        axis.sequence > feedback.sequence
        or axis.sequence == feedback.sequence
        and stamp == previous[2]
        and axis.sourceCarStateMonoTime == previous[8]
        and axis.validUntilMonoTime == previous[4]
        and axis.faultEventMonoTime == previous[6]
      )
    ):
      return False
    event_ns = int(sm.logMonoTime['onroadEvents'])
    allowed = (log.OnroadEvent.EventName.pcmDisable, log.OnroadEvent.EventName.wrongGear,
               log.OnroadEvent.EventName.reverseGear, *extra_user_events, *informational_events)
    if (
      not sm.seen['onroadEvents']
      or not sm.valid['onroadEvents']
      or not sm.alive['onroadEvents']
      or not 0 < event_ns == axis.faultEventMonoTime <= stamp
      or now_ns - event_ns > 1_500_000_000
      or any(
        event.name not in allowed
        or event.immediateDisable
        or event.softDisable
        and event.name != log.OnroadEvent.EventName.wrongGear
        or event.userDisable
        and event.name not in (log.OnroadEvent.EventName.pcmDisable, log.OnroadEvent.EventName.reverseGear, *extra_user_events)
        for event in sm['onroadEvents']
      )
    ):
      return False
    native = current_native(sm, cp, now_ns=now_ns, axis_session_id=feedback.session)
    if (
      native is None
      or not native.pandaInventory
      or not native.sourcePandaStatesMonoTime
      or active and not native.requestedLateral
      or withdrawing and (native.requestedLateral or native.observedMonoTime < stamp)
      or native.lateralAllowed
      or native.requestedLongitudinal
      or native.longitudinalAllowed
      or native.observedMonoTime < feedback.lateral_native_ns
      or (native.pandaSerial, tuple(p.hardwareSerial for p in native.pandaInventory)) != feedback.lateral_inventory
      or any(p.controlsAllowed for p in native.pandaInventory)
    ):
      return False
    # Same raw latest-Panda veto as configured transport; its BOOTTIME source
    # stamp is not interpreted as a monotonic lease. Native v2 owns that lease.
    pandas = sm['pandaStates']
    if (
      not sm.seen['pandaStates']
      or not sm.valid['pandaStates']
      or not sm.alive['pandaStates']
      or int(sm.logMonoTime['pandaStates']) <= 0
      or len(pandas) != len(cp.safetyConfigs)
    ):
      return False
    return all(
      int(p.safetyModel.raw) == int(c.safetyModel.raw)
      and p.safetyParam == c.safetyParam
      and p.alternativeExperience == cp.alternativeExperience
      and not p.controlsAllowed
      and not p.safetyRxChecksInvalid
      and not p.heartbeatLost
      and not p.faults
      for p, c in zip(pandas, cp.safetyConfigs, strict=True)
    )
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False
