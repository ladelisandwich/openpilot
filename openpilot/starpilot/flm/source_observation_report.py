"""Bounded optional learner observations; never admission or publisher proof."""

_SERVICES = ('deviceMotion', 'extrinsicsCalibration', 'carState', 'driverMonitoringState')


def _snapshot(collector, sm, now_ns, reason):
  ready = collector.ready
  command, output = ready[-1] if ready else (None, None)
  return {
    'reason': reason, 'observedMonoTime': now_ns, 'readyPairs': len(ready),
    'currentNative': bool(getattr(collector, '_learning_current_native_allowed', False)),
    'controlMonoTime': None if command is None else command.control_ns,
    'applyMonoTime': None if output is None else output.apply_ns,
    'originalNativeObservedMonoTime': None if command is None else command.native_observed_ns,
    'originalNativeValidUntilMonoTime': None if command is None else command.native_expires_ns,
    'receiver': {
      service: {
        'seen': bool(sm.seen[service]), 'alive': bool(sm.alive[service]),
        'valid': bool(sm.valid[service]), 'freqOk': bool(sm.freq_ok[service]),
        'sourceEventMonoTime': int(sm.logMonoTime[service]),
        'receivedMonoTime': int(sm.recv_time[service] * 1e9),
      } for service in _SERVICES
    },
  }


def record_refusal(collector, sm, now_ns, reason, *, error=None):
  """Existing rejected branches only; at most two snapshots per collector."""
  try:
    counts = getattr(collector, '_source_observation_refusals', None)
    if counts is None:
      counts = collector._source_observation_refusals = {}
    if reason in counts or len(counts) < 24:
      counts[reason] = counts.get(reason, 0) + 1
    collector._source_observation_current_refusal = (reason, now_ns)
    if error is not None:
      collector._source_observation_exception = {'observedMonoTime': now_ns, 'type': type(error).__name__, 'message': str(error)[:192]}
    if not getattr(collector, '_source_observation_first_attempted', False):
      collector._source_observation_first_attempted = True
      collector._source_observation_first_refusal = _snapshot(collector, sm, now_ns, reason)
    if collector.ready and not getattr(collector, '_source_observation_ready_attempted', False):
      collector._source_observation_ready_attempted = True
      collector._source_observation_ready_refusal = _snapshot(collector, sm, now_ns, reason)
  except Exception:
    # Observability must not alter rejection, reset, clock or driving behavior.
    pass


def refusal_report(collector):
  current = getattr(collector, '_source_observation_current_refusal', None)
  return {
    'publisherRefusal': 'unknown_not_transmitted',
    'scope': 'learner receiver only; not the Controls publisher admission reason',
    'counts': dict(getattr(collector, '_source_observation_refusals', {})),
    'lastRefusal': None if current is None else {'reason': current[0], 'observedMonoTime': current[1]},
    'lastException': getattr(collector, '_source_observation_exception', None),
    'firstRefusal': getattr(collector, '_source_observation_first_refusal', None),
    'firstReadyPairRefusal': getattr(collector, '_source_observation_ready_refusal', None),
  }
