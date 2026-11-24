"""Bounded GM AOL withdrawal observations; never an authority input."""

import json

from openpilot.starpilot.aol.runtime import AXIS_MAX_AGE_NS

MAX_EVENTS = 64
MIN_EVENT_INTERVAL_NS = 1_000_000_000
MAX_EVENT_BYTES = 4096
AXIS_SERVICE = 'aolAxisState'


class TransitionDiagnostic:
  def __init__(self):
    self.disabled = False
    self.emitted = 0
    self.suppressed = 0
    self.last_event_ns = 0
    self.pending = None

  def capture(self, sm, now_ns):
    if self.disabled or self.emitted >= MAX_EVENTS:
      return None
    try:
      # Retain the immutable reader. Do not serialize or copy a proof per tick.
      return (
        now_ns,
        int(sm.logMonoTime[AXIS_SERVICE]),
        sm[AXIS_SERVICE],
        bool(sm.seen[AXIS_SERVICE]),
        bool(sm.valid[AXIS_SERVICE]),
        bool(sm.alive[AXIS_SERVICE]),
        int(sm.logMonoTime['carState']),
      )
    except Exception:
      self.disabled = True
      self.pending = None
      return None

  @staticmethod
  def axis_record(snapshot):
    now, header, axis, seen, valid, alive, source = snapshot
    return {
      'atNs': now,
      'headerNs': header,
      'carStateHeaderNs': source,
      'seen': seen,
      'valid': valid,
      'alive': alive,
      'session': str(axis.sessionId)[:64],
      'sequence': int(axis.sequence),
      'sourceNs': int(axis.sourceCarStateMonoTime),
      'observedNs': int(axis.observedMonoTime),
      'deadlineNs': int(axis.validUntilMonoTime),
      'qualified': bool(axis.qualified),
      'desired': bool(axis.desiredLateral),
      'active': bool(axis.lateralActive),
      'desiredLongitudinal': bool(axis.desiredLongitudinal),
      'longitudinalActive': bool(axis.longitudinalActive),
      'nativeAcknowledged': bool(axis.nativeAcknowledged),
      'mode': str(axis.mode)[:24],
      'faultReason': str(axis.faultReason)[:32],
      'faultEventNs': int(axis.faultEventMonoTime),
    }

  @staticmethod
  def axis_rejection(snapshot):
    now, header, axis, seen, valid, alive, _ = snapshot
    if not seen:
      return 'axis_unseen'
    if not valid:
      return 'axis_invalid'
    if not alive:
      return 'axis_not_alive'
    if not axis.qualified:
      return 'axis_unqualified'
    if not axis.sessionId:
      return 'axis_session_missing'
    if header > now:
      return 'axis_header_future'
    if now - header > AXIS_MAX_AGE_NS:
      return 'axis_header_expired'
    if int(axis.observedMonoTime) > now:
      return 'axis_observation_future'
    if now > int(axis.validUntilMonoTime):
      return 'axis_deadline_expired'
    return 'axis_rejected_unclassified'

  def gate_reason(self, snapshot, axis, native, cs, standstill, steer_at_standstill):
    try:
      if axis is None:
        return self.axis_rejection(snapshot)
      if native is None:
        return 'native_rejected'
      if not axis.nativeAcknowledged:
        return 'native_unacknowledged'
      if not axis.lateralActive:
        return 'axis_withdrawn'
      if bool(axis.desiredLateral) != bool(native.requestedLateral):
        return 'native_request_mismatch'
      if not native.lateralAllowed:
        return 'native_denied'
      if cs.steerFaultTemporary:
        return 'steer_temporary_fault'
      if cs.steerFaultPermanent:
        return 'steer_permanent_fault'
      if standstill and not steer_at_standstill:
        return 'below_steering_speed'
      return 'lateral_gate_rejected_unclassified'
    except Exception:
      return 'diagnostic_classification_unavailable'

  def finish(
    self,
    entry,
    gate,
    *,
    now_ns,
    refreshed,
    native,
    native_header_ns,
    native_flags,
    native_raw,
    reason,
    cs,
    enabled,
    long_active,
    axis=None,
    standstill=False,
    steer_at_standstill=False,
  ):
    if self.disabled or self.emitted >= MAX_EVENTS or entry is None or gate is None:
      return
    if self.pending is not None or (self.last_event_ns and now_ns - self.last_event_ns < MIN_EVENT_INTERVAL_NS):
      self.suppressed = min(self.suppressed + 1, 0xFFFFFFFF)
      return
    # Keep only bounded immutable readers/references and scalars before output.
    self.pending = (
      entry,
      gate,
      now_ns,
      refreshed,
      native,
      native_header_ns,
      native_flags,
      native_raw if len(native_raw) <= MAX_EVENT_BYTES else b'',
      reason,
      cs,
      enabled,
      long_active,
      axis,
      standstill,
      steer_at_standstill,
    )
    self.last_event_ns = now_ns

  def _format(self, pending):
    (
      entry,
      gate,
      now_ns,
      refreshed,
      native,
      native_header_ns,
      native_flags,
      native_raw,
      reason,
      cs,
      enabled,
      long_active,
      axis,
      standstill,
      steer_at_standstill,
    ) = pending
    native_accepted = native
    accepted = native is not None
    if native is None and len(native_raw) <= MAX_EVENT_BYTES:
      from openpilot.starpilot.aol.wire import decode_safety

      native = decode_safety(native_raw)
    status = {
      'headerNs': int(native_header_ns),
      'seen': bool(native_flags[0]),
      'valid': bool(native_flags[1]),
      'alive': bool(native_flags[2]),
      'decoded': native is not None,
      'accepted': accepted,
    }
    if native is not None:
      status.update(
        observedNs=int(native.observedMonoTime),
        deadlineNs=int(native.validUntilMonoTime),
        session=str(native.axisSessionId)[:64],
        requested=bool(native.requestedLateral),
        allowed=bool(native.lateralAllowed),
        requestedLongitudinal=bool(native.requestedLongitudinal),
        longitudinalAllowed=bool(native.longitudinalAllowed),
        model=int(native.safetyModel),
        param=int(native.safetyParam),
      )
    if native is not None:
      inventory = getattr(native, 'pandaInventory', ())
      status.update(
        compatible=bool(native.compatible),
        protocolVersion=int(native.protocolVersion),
        inventoryCount=len(inventory),
        inventory=[
          {
            'slot': int(slot.slotIndex),
            'model': int(slot.safetyModel),
            'param': int(slot.safetyParam),
            'ae': int(slot.alternativeExperience),
            'rxInvalid': bool(slot.safetyRxChecksInvalid),
            'heartbeatLost': bool(slot.heartbeatLost),
            'faults': int(slot.faults),
            'selectedOwner': slot.hardwareSerial == native.pandaSerial,
          }
          for slot in inventory[:2]
        ],
      )
    return {
      'version': 1,
      'clock': 'monotonic',
      'entry': self.axis_record(entry),
      'gate': self.axis_record(gate),
      'finishedNs': now_ns,
      'refreshAttempted': bool(refreshed),
      'reason': self.gate_reason(gate, axis, native_accepted, cs, standstill, steer_at_standstill) if reason == 'lateral_gate' else reason,
      'native': status,
      'physical': {
        'canValid': bool(cs.canValid),
        'canTimeout': bool(cs.canTimeout),
        'brake': bool(cs.brakePressed),
        'gas': bool(cs.gasPressed),
        'paddle': bool(cs.regenBraking),
        'steering': bool(cs.steeringPressed),
        'temporaryFault': bool(cs.steerFaultTemporary),
        'permanentFault': bool(cs.steerFaultPermanent),
        'standstill': bool(cs.standstill),
        'speed': float(cs.vEgo),
        'gear': str(cs.gearShifter)[:24],
        'mainAvailable': bool(cs.cruiseState.available),
        'cruiseEnabled': bool(cs.cruiseState.enabled),
      },
      'ordinaryEnabled': bool(enabled),
      'longActive': bool(long_active),
      'suppressed': self.suppressed,
    }

  def emit(self, sink, control_message=None):
    pending, self.pending = self.pending, None
    if pending is None or self.disabled:
      return
    try:
      record = self._format(pending)
      record['controlHeaderNs'] = int(control_message.logMonoTime) if control_message is not None else 0
      # One bounded event per falling edge, rate limited and lifetime capped.
      if len(json.dumps(record, separators=(',', ':'), allow_nan=False).encode()) > MAX_EVENT_BYTES:
        self.disabled = True
        return
      sink('aol_lateral_withdrawal', diagnostic=record)
      self.emitted += 1
      self.suppressed = 0
    except Exception:
      self.disabled = True
