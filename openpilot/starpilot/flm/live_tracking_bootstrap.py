"""One initial genuine response experiment; no knob fit or profile admission.

The live tracking path retains this numerical model across ordinary capture
epochs while independent current residual/source checks remain qualified.
Full candidate replay remains available offline and is not called here.
"""

import base64
import pickle

VERSION = 'live-response-bootstrap-v1'


def bootstrap_snapshot(snapshot):
  from openpilot.starpilot.flm.controller_replay import (
    unpack_primitive_snapshot,
    pack_primitive_snapshot,
    factory_from_snapshot,
    MAX_FIT_RAW_BYTES,
  )
  from openpilot.starpilot.flm.primitive_source_memo import independent_primitive_graph
  from openpilot.starpilot.flm.normalized_trace_archive import reserve_for_packet
  from openpilot.starpilot.flm.feedback_replay import PROTOCOL as FEEDBACK_PROTOCOL, EVENT_VERSION, verify_archive
  from openpilot.starpilot.flm.event_mean_evidence import VERSION as MEAN_VERSION
  from openpilot.starpilot.flm.candidate_feedback import CLOSED_POLICY
  from openpilot.starpilot.flm.torque_adapter import Objective
  from openpilot.starpilot.flm.full_surface_profile import envelope_from_dict
  from openpilot.starpilot.flm.source_plant_pipeline import _template, replay_data
  from openpilot.starpilot.flm.original_context_receipts import verify_original_replay
  from openpilot.starpilot.flm.source_plant_fit import source_binding, prepare, PreparedPlant, SOURCE_COMPLETE_SLOT_POLICY as BOOTSTRAP_SLOT_POLICY
  from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding
  from openpilot.starpilot.flm.live_tracking_response import certificate_from_prepared, certificate_to_dict

  if (
    type(snapshot) is not dict
    or snapshot.get('live_job_version') != VERSION
    or snapshot.get('version') != 1
    or type(snapshot.get('snapshot_b64')) is not str
    or len(snapshot['snapshot_b64']) > 6 * 1024 * 1024
  ):
    raise ValueError('explicit bounded live response bootstrap wrapper')
  payload = unpack_primitive_snapshot(base64.b64decode(snapshot['snapshot_b64'], validate=True))
  if (
    payload.get('live_job_version') != VERSION
    or payload.get('version') != 1
    or payload.get('family') not in ('torque', 'torque_pid')
    or payload.get('family') != snapshot.get('family')
    or payload.get('plant_protocol') != FEEDBACK_PROTOCOL
    or payload.get('event_version') != EVENT_VERSION
    or payload.get('event_mean_version') != MEAN_VERSION
    or payload.get('candidate_feedback_policy') != CLOSED_POLICY
    or payload.get('event_operation') != 'trial'
    or tuple(snapshot.get('objective_key', ())) != tuple(payload.get('objective_key', ()))
  ):
    raise ValueError('current original source/native response bootstrap protocol')
  payload = independent_primitive_graph(payload, reserved_bytes=reserve_for_packet(payload['source_packet']))
  packet = payload['source_packet']
  envelope = envelope_from_dict(payload['effect_envelope'])
  replay = replay_data(packet, objective=Objective(**payload['objective']), identity=payload['identity'], baseline_revision=envelope.selected_baseline_revision)
  if len(pickle.dumps(payload, protocol=5)) + len(pickle.dumps(replay, protocol=5)) > MAX_FIT_RAW_BYTES:
    raise ValueError('bootstrap complete original raw byte bound')
  replay = pack_primitive_snapshot(replay)
  factory, _ = factory_from_snapshot(replay)
  template, _, _ = _template(payload, factory)
  actual = verify_original_replay(packet['context_receipts'], template, replay)
  binding = source_binding(packet, actual)
  verify_archive(packet, actual)
  if payload.get('live_slot_policy') != BOOTSTRAP_SLOT_POLICY:
    raise ValueError('explicit live response calibration selection policy')
  from openpilot.starpilot.flm.live_tracking_bootstrap_cohorts import VERSION as COHORT_VERSION, validate_selected_cohort

  if payload.get('live_cohort_version') != COHORT_VERSION:
    raise ValueError('explicit outcome-blind checkpoint cohort protocol required')
  current = LiveTrialBinding(**payload['live_binding'])
  selection = validate_selected_cohort(payload.get('live_cohort_selection'), packet, actual, current, payload['now_ns'], source_complete=True)
  from openpilot.starpilot.flm.plant_lag import TRAINING_DOMAIN_ONLY

  prepared = prepare(packet, actual, binding, slot_policy=BOOTSTRAP_SLOT_POLICY, support_policy=TRAINING_DOMAIN_ONLY)
  if type(prepared) is not PreparedPlant:
    return {
      'live_job_version': VERSION,
      'status': 'collecting',
      'reason': prepared.get('reason', 'response_unavailable'),
      'certificate': None,
      'profile': None,
      'current_permission': False,
    }
  if (
    tuple(tuple(p.mono_ns for p in prepared.source_poses if w.start_ns <= p.mono_ns <= w.end_ns) for w in prepared.original_windows)
    != selection['source_pose_ns']
  ):
    raise ValueError('independent native-verified source pose eligibility differs')
  certificate = certificate_from_prepared(prepared, current, original_packet=packet)
  # No finite differences, group rotation, trial scoring, replay validation,
  # profile, live lease or current/native permission is manufactured here.
  return {
    'live_job_version': VERSION,
    'status': 'response_identified',
    'reason': 'independent_train_heldout_response_passed',
    'certificate': certificate_to_dict(certificate),
    'profile': None,
    'current_permission': False,
  }
