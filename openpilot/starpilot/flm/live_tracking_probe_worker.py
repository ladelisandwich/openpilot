"""Optional bounded controller/native numerical work, never command authority."""
import json
import os
import resource
import sys
import time

MAX_INPUT = 8*1024*1024
MAX_OUTPUT = 65536


def _evaluate_data(data, *, now_ns):
  from openpilot.starpilot.flm.controller_replay import factory_from_snapshot, ControllerReplay, unpack_primitive_snapshot, Snapshot, ReplayFrame
  from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
  from openpilot.starpilot.flm.runtime_adapter_registry import SourceState
  from openpilot.starpilot.flm.live_tracking_response import witness_from_dict
  from openpilot.starpilot.flm.live_tracking_probe import probe_endpoint
  from openpilot.starpilot.flm.effect_trace import trace_from_dict
  from openpilot.starpilot.flm.torque_emission_replay import projection

  binding = LiveTrialBinding(**data['binding']).validate()
  factory, objective = factory_from_snapshot(data['replay'])
  # Restore exact raw records, not the old normalized verifier which correctly
  # refuses new nonzero live history. probe_endpoint performs the authoritative
  # actual per-tick controller/native replay on this genuine bounded prefix.
  raw = unpack_primitive_snapshot(data['replay'])
  if raw.get('version') != 1 or raw.get('raw_mode') is not True or not 1 <= len(raw['frames']) <= 256:
    raise ValueError('bounded literal original recorder required')
  end = data['control']['control_ns']
  checkpoints = [row for row in raw['checkpoints'] if row[0] <= end]
  if not checkpoints:
    raise ValueError('genuine original checkpoint missing')
  start, revision, values = checkpoints[-1]
  replay = ControllerReplay(factory, objective, identity=raw['identity'],
    baseline_revision=raw['baseline_revision'], model_revision=raw['model_revision'])
  replay.checkpoint(start, Snapshot(revision, values))
  for row in raw['frames']:
    if not start <= row['control_ns'] <= end:
      continue
    copied = dict(row)
    if copied.get('effect_trace') is not None:
      copied['effect_trace'] = trace_from_dict(copied['effect_trace'])
    if not replay.record_actual_frame(ReplayFrame(**copied)):
      raise ValueError('incomplete actual raw prefix')
  events = tuple(row for row in raw['emissions']['events'] if start <= row['control_ns'] <= end)
  if not events:
    raise ValueError('actual native prefix missing')
  first, last = events[0]['witness']['frame']+1, events[-1]['witness']['frame']+1
  carriers = tuple(row for row in raw['emissions']['carriers'] if first <= row[0] <= last)
  replay.emissions.load({'events': events, 'carriers': carriers}, ((start, end),))

  def control(row):
    copied = dict(row)
    if copied['effect_trace'] is not None:
      copied['effect_trace'] = trace_from_dict(copied['effect_trace'])
    return ControlEvidence(**copied)

  copied = dict(data['output'])
  copied['projection'] = projection(copied['projection'])
  witness = witness_from_dict(data['witness'], binding=binding, now_ns=data['evidence_ns'])
  result = probe_endpoint(factory=factory, control=control(data['control']), output=AppliedEvidence(**copied),
    accepted_full30=data['accepted_full30'], binding=binding, response_witness=witness,
    source=SourceState(**data['source']), now_ns=now_ns, evidence_ns=data['evidence_ns'], queued_ns=data['queued_ns'],
    current_native=False, archived_clock=data.get('archived_clock', False),
    current_vehicle_model_anchor=data['anchor'], replay=replay, group=data['group'], names=data['names'],
    prefix_controls=tuple(control(row) for row in data['prefix_controls']))
  return result


def evaluate_job(job, *, now_ns):
  from dataclasses import asdict
  from openpilot.starpilot.flm.live_tracking_probe_process import decode_probe_job, VERSION
  from openpilot.starpilot.flm.live_tracking_probe_process import _prepared_context, _context_digest
  if type(job) is not dict:
    raise ValueError('exact numerical envelope required')
  context_sha = job.get('preparedContextSha256')
  if context_sha is not None:
    if set(job) != {'version', 'payload', 'sha256', 'preparedContextSha256'}:
      raise ValueError('prepared numerical envelope shape differs')
    job = {key: value for key, value in job.items() if key != 'preparedContextSha256'}
  data = decode_probe_job(job)
  if context_sha is not None and _context_digest(_prepared_context(data)) != context_sha:
    raise ValueError('actual decoded numerical job differs from parent request')
  batch = data.get('batch')
  if batch is not None:
    rows = [[asdict(row) for row in _evaluate_data(endpoint, now_ns=now_ns).rows] for endpoint in batch]
    report = {'version': VERSION, 'sha256': job['sha256'], 'results': rows}
    if context_sha is not None:
      report['contextSha256'] = context_sha
    return report
  result = _evaluate_data(data, now_ns=now_ns)
  report = {'version': VERSION, 'sha256': job['sha256'], 'rows': [asdict(row) for row in result.rows]}
  if context_sha is not None:
    report['contextSha256'] = context_sha
  return report


def main():
  try:
    if sys.platform.startswith('linux'):
      import ctypes
      import signal
      parent = int(os.environ['STARPILOT_FLM_PARENT_PID'])
      if ctypes.CDLL(None).prctl(1, signal.SIGKILL, 0, 0, 0) != 0 or os.getppid() != parent:
        raise ValueError('probe parent disappeared')
    if hasattr(os, 'sched_setscheduler'):
      os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.nice(10)
    for kind, ceiling in ((resource.RLIMIT_AS, 2*1024*1024*1024), (resource.RLIMIT_CPU, 20)):
      soft, hard = resource.getrlimit(kind)
      limit = min((ceiling, *[value for value in (soft, hard) if value >= 0]))
      resource.setrlimit(kind, (limit, hard))
    raw = sys.stdin.buffer.read(MAX_INPUT+1)
    if len(raw) > MAX_INPUT:
      raise ValueError('probe input bound')
    result = evaluate_job(json.loads(raw), now_ns=time.monotonic_ns())
    encoded = json.dumps(result, separators=(',', ':'), allow_nan=False).encode()
    if len(encoded) > MAX_OUTPUT:
      raise ValueError('probe output bound')
    sys.stdout.buffer.write(encoded)
    sys.stdout.buffer.flush()
    return 0
  except (Exception, MemoryError):
    return 1


if __name__ == '__main__':
  raise SystemExit(main())
