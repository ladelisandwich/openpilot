"""Intent only for the first full-engagement Ford gas/brake or temporary-EPS join."""

from opendbc.car import structs
from opendbc.car.ford.aol import qualified
from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS, current_native
from openpilot.starpilot.aol.transport_pause import cp_identity


def retain_physical_pause_intent(feedback, sm, cp, cs, axis, now_ns, *, latched, producer):
  """Use only the original healthy active proof; change no state or lease."""
  try:
    gas = bool(cp.openpilotLongitudinalControl and cs.gasPressed and not cs.steerFaultTemporary and not cs.brakePressed)
    eps = bool(cs.steerFaultTemporary and not cs.gasPressed and not cs.brakePressed)
    brake = bool(cp.openpilotLongitudinalControl and cs.brakePressed and not cs.gasPressed and not cs.steerFaultTemporary)
    ordinary = not brake
    if (
      not qualified(cp, marked_only=True)
      or not (gas or eps or brake)
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
      or not cs.canValid
      or cs.canTimeout
      or cs.steerFaultPermanent
      or cs.vehicleSensorsInvalid
      or cs.accFaulted
      or cs.steeringDisengage
      or cs.regenBraking
      or cs.doorOpen
      or cs.seatbeltUnlatched
      or cs.buttonEvents
      or cs.gearShifter != structs.CarState.GearShifter.drive
      or not cs.cruiseState.available
      or bool(cs.cruiseState.enabled) != ordinary
    ):
      return False
    long_active = bool(cp.openpilotLongitudinalControl)
    previous = feedback.signature
    stamp = int(sm.logMonoTime['aolAxisState'])
    if (
      not feedback.session
      or str(axis.sessionId) != feedback.session
      or str(axis.faultSessionId) != feedback.session
      or str(axis.faultReason) != 'none'
      or str(axis.mode) != ('combined' if long_active else 'lateralOnly')
      or not axis.qualified
      or not axis.desiredLateral
      or not axis.lateralActive
      or not axis.nativeAcknowledged
      or bool(axis.desiredLongitudinal) != long_active
      or bool(axis.longitudinalActive) != long_active
      or not sm.seen['aolAxisState']
      or not sm.valid['aolAxisState']
      or not sm.alive['aolAxisState']
      or not 0 < axis.sourceCarStateMonoTime == stamp == axis.observedMonoTime <= now_ns <= axis.validUntilMonoTime
      or now_ns - stamp > AXIS_MAX_AGE_NS
      or axis.validUntilMonoTime - stamp > AXIS_MAX_AGE_NS
      or not producer
      or producer != feedback.availability_producer
      or str(axis.intentProducerSessionId) != producer
      or int(axis.intentIntegrityEpoch) != feedback.availability_epoch
      or feedback.availability_epoch is None
      or not feedback.healthy_inventory
      or not 0 < feedback.healthy_feedback_ns <= now_ns
      or now_ns - feedback.healthy_feedback_ns > AXIS_MAX_AGE_NS
      or not 0 < feedback.healthy_native_ns <= now_ns
      or now_ns - feedback.healthy_native_ns > AXIS_MAX_AGE_NS
      or previous is None
      or previous[0] != feedback.session
      or previous[7] != feedback.session
      or previous[5] != 'none'
      or not previous[9]
      or not previous[11]
      or not previous[13]
      or previous[10] != long_active
      or previous[12] != long_active
      or previous[14] != producer
      or previous[15] != feedback.availability_epoch
      or not 0 < previous[8] == previous[2] == previous[3] <= now_ns <= previous[4]
      or now_ns - previous[8] > AXIS_MAX_AGE_NS
      or axis.sourceCarStateMonoTime < previous[8]
      or axis.faultEventMonoTime != previous[6]
      or not (axis.sequence > feedback.sequence or axis.sequence == feedback.sequence and stamp == previous[2] and axis.validUntilMonoTime == previous[4])
    ):
      return False
    # Only the original fault-free event publication preceding the physical
    # edge may join here. Subsequent typed withdrawals use existing handlers.
    event_ns = int(sm.logMonoTime['onroadEvents'])
    if (
      not sm.seen['onroadEvents']
      or not sm.valid['onroadEvents']
      or not sm.alive['onroadEvents']
      or not 0 < event_ns == axis.faultEventMonoTime <= stamp
      or now_ns - event_ns > 1_500_000_000
      or len(sm['onroadEvents']) != 0
    ):
      return False
    native = current_native(sm, cp, now_ns=now_ns, axis_session_id=feedback.session)
    if (
      native is None
      or not native.pandaInventory
      or not native.sourcePandaStatesMonoTime
      or not native.requestedLateral
      or bool(native.lateralAllowed) != (gas or brake)
      or bool(native.requestedLongitudinal) != long_active
      or bool(native.longitudinalAllowed) != (eps and long_active)
      or native.observedMonoTime < feedback.healthy_native_ns
      or (native.pandaSerial, tuple(p.hardwareSerial for p in native.pandaInventory)) != feedback.healthy_inventory
      or any(bool(p.controlsAllowed) != (ordinary and p.safetyModel == int(structs.CarParams.SafetyModel.ford))
             for p in native.pandaInventory)
      or not sm.seen['pandaStates']
      or not sm.valid['pandaStates']
      or not sm.alive['pandaStates']
      or int(sm.logMonoTime['pandaStates']) <= 0
      or len(sm['pandaStates']) != len(cp.safetyConfigs)
    ):
      return False
    return all(
      int(p.safetyModel.raw) == int(c.safetyModel.raw)
      and p.safetyParam == c.safetyParam
      and p.alternativeExperience == cp.alternativeExperience
      and bool(p.controlsAllowed) == (ordinary and c.safetyModel == structs.CarParams.SafetyModel.ford)
      and not p.safetyRxChecksInvalid
      and not p.heartbeatLost
      and not p.faults
      for p, c in zip(sm['pandaStates'], cp.safetyConfigs, strict=True)
    )
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False
