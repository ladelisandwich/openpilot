"""Optional learner plumbing; never imported by controller realtime loops."""
from dataclasses import asdict
import base64
import math

from openpilot.starpilot.flm.source_plant_fit import PROTOCOL, CONTEXT_VERSION, source_binding, experiment_context


def enable(adapter, clock_contract, *, protocol=PROTOCOL):
  from openpilot.starpilot.flm.original_source_capture import OriginalSourceClockContract
  if (not getattr(adapter,'event_collection',False) or adapter.replay_version != 3 or
      clock_contract is not None and not isinstance(clock_contract,OriginalSourceClockContract)):
    raise ValueError('explicit registered event/source clock plant contract')
  if clock_contract is not None:
    clock_contract.validate()
  from openpilot.starpilot.flm.feedback_replay import PROTOCOL as FEEDBACK_PROTOCOL
  if protocol not in (PROTOCOL,FEEDBACK_PROTOCOL):
    raise ValueError('explicit original plant protocol')
  if protocol == FEEDBACK_PROTOCOL:
    from openpilot.starpilot.flm.feedback_archive import OriginalFeedbackArchive
    adapter.original_feedback_archive = OriginalFeedbackArchive()
  adapter.source_plant_protocol = protocol
  adapter.source_plant_clock = clock_contract
  adapter.source_plant_receipt = None
  adapter.source_plant_binding = None
  adapter.source_plant_submitted_epoch = None
  adapter.source_plant_submitted_contexts = {}
  return adapter


def start_original_capture(collector):
  """Source-pin/clock unavailability denies learning without killing producer."""
  adapter = collector.adapter
  try:
    collector.enable_original_source_capture(clock_contract=adapter.source_plant_clock)
  except (OSError,ValueError) as error:
    adapter.source_plant_capture_error = 'original_source_clock_contract_unavailable'
    from openpilot.starpilot.flm.source_observation_report import record_refusal
    import time
    record_refusal(collector, None, time.monotonic_ns(), adapter.source_plant_capture_error, error=error)
    adapter.current_physical_valid = False
    adapter.last_physical_ns = collector.last_qualified_response = 0
    return False
  adapter.source_plant_capture_error = None
  return True


def observe(sm, cp, collector, now_ns, current_native):
  from openpilot.starpilot.flm.runtime_adapter_registry import original_source_pose_observer
  from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose
  from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current
  from openpilot.starpilot.flm.torque_emission_replay import delivered

  adapter = collector.adapter
  def revoke():
    adapter.current_physical_valid = False
    adapter.last_physical_ns = collector.last_qualified_response = 0
  if not current_native:
    from openpilot.starpilot.flm.source_observation_report import record_refusal
    record_refusal(collector, sm, now_ns, 'current_native_unavailable')
    revoke()
    return
  row = original_source_pose_observer(sm,cp,collector,now_ns,distinguish_pending=True)
  from openpilot.starpilot.flm.physical_observation import PendingPhysical
  if type(row) is PendingPhysical and row.fresh(now_ns) and not row.context_changed:
    return  # Keep the ORIGINAL prior receipt; do not renew its timestamp.
  if not isinstance(row,OriginalSourcePose):
    revoke()
    return
  revoke()
  collector.original_source_pose(row,now_ns)
  live = getattr(collector, 'live_tracking', None)
  live_lag = live.optimizer.lag_ns if live is not None else getattr(collector, 'live_response_lag_ns', None)
  collector.live_response_pair = None
  if type(live_lag) is int and 0 <= live_lag <= 500_000_000:
    if live is not None:
      collector.live_response_pair = None
      pair = live.original_pose_pair(collector.ready, row)
      if pair is not None:
        collector.live_response_pair = (*pair, row)
    else:
      lower = [(c,o) for c,o in collector.ready if o.apply_ns <= row.source_lower_ns-live_lag]
      upper = [(c,o) for c,o in collector.ready if o.apply_ns <= row.source_upper_ns-live_lag]
      if lower and upper and lower[-1][1].apply_ns == upper[-1][1].apply_ns:
        collector.live_response_pair = (*lower[-1], row)
    if live is not None:
      live.observe_original_pose(row, collector, now_ns=now_ns, current_native=current_native)
  model, binding = adapter.model, adapter.source_plant_binding
  if model is None or binding is None or adapter.source_plant_receipt is None:
    return
  from openpilot.starpilot.flm.runtime_adapter_registry import digest
  if (row.identity != binding['identity'] or row.source_epoch != binding['source_epoch'] or
      row.baseline_revision != binding['baseline_revision'] or row.calibration != binding['calibration_generation'] or
      row.native_identity != binding['native_identity'] or digest((row.source_clock_contract,row.context[7])) != binding['source_clock_contract_sha256'] or
      digest(row.controller_revision) != binding['controller_revision'] or
      digest((row.context[2],row.context[10],list(row.context[11:]))) != binding['vehicle_model_generation'] or
      model.context != digest(collector.physical_context) or
      not 0 < row.source_lower_ns <= row.source_upper_ns <= now_ns or now_ns-row.source_lower_ns > 100_000_000):
    return
  lower = tuple((control,output) for control,output in collector.ready if output.apply_ns <= row.source_lower_ns-model.lag_ns)
  upper = tuple((control,output) for control,output in collector.ready if output.apply_ns <= row.source_upper_ns-model.lag_ns)
  if not lower or not upper or lower[-1][1].apply_ns != upper[-1][1].apply_ns:
    return
  control,output = lower[-1]
  if (row.source_upper_ns-model.lag_ns-output.apply_ns > 30_000_000 or
      not consumed_sources_current(control,output) or not adapter.replay.applied_matches(control,output) or
      not output.native_allowed or output.limited or output.override or output.fault or not output.valid or
      control.native_required and not control.native_allowed or control.override or control.fault or not control.valid or control.manual or
      not control.enabled or not control.active or control.saturated or control.model_wobble or control.controller_chatter or
      min(control.control_ns,output.apply_ns,row.mono_ns) <= adapter.model_cutoff_ns):
    return
  command = -delivered(output)
  measured = row.speed_mps*row.yaw_rate-9.81*math.sin(row.roll_rad)
  bounds = adapter.model_speed_bounds
  valid = (bounds is not None and bounds[0] <= row.speed_mps <= bounds[1] and
    model.command_low <= command <= model.command_high and
    abs(model.slope*command+model.intercept-measured) <= max(.01,3*model.heldout_rmse))
  if valid:
    adapter.current_physical_valid = True
    adapter.last_physical_ns = collector.last_qualified_response = row.mono_ns


MIN_NORMAL_JOB_AGE_NS = 60_000_000_000  # Fixed before data/fit outcomes; source epoch is100s.
NORMAL_JOB_ATTEMPT_SPACING_NS = 80_000_000_000


def _current_fit_pair_reason(collector, now_ns, *, physical_binding=None):
  """Cheap latest-pair availability only; no history search or fit authority.

  The child/current validators remain independent. A known-unqualified latest
  endpoint must not consume this fixed experiment's one fitting attempt.
  """
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence, MAX_AGE_NS
  from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current, exact_unclipped_pair
  try:
    control, output = collector.ready[-1]
    if (type(control) is not ControlEvidence or type(output) is not AppliedEvidence or
        type(now_ns) is not int or now_ns <= 0):
      return 'invalid_current_pair'
    if not output.apply_ns <= now_ns <= output.apply_ns+MAX_AGE_NS or not consumed_sources_current(control,output) or output.control_ns != control.control_ns:
      return 'current_pair_not_fresh'
    if (control.identity != collector.adapter.identity or control.baseline_revision != collector.adapter.baseline_revision or
        not 0 < control.source_receipt_ns <= control.control_ns <= now_ns <= control.source_receipt_ns+500_000_000):
      return 'current_pair_source_mismatch'
    observed_clip = False
    if physical_binding is not None and output.limited:
      from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding
      from openpilot.starpilot.flm.live_tracking_response import verified_clipped_input
      if type(physical_binding) is LiveTrialBinding:
        observed_clip = verified_clipped_input(collector.adapter.replay, control, output, physical_binding)
    if (not control.enabled or control.manual or not control.active or not control.valid or control.override or control.fault or
        control.saturated or control.model_wobble or control.controller_chatter or
        control.limited and not exact_unclipped_pair(control,output) and not observed_clip):
      return 'current_control_unqualified'
    if (not output.valid or not output.active or output.override or output.fault or output.limited and not observed_clip or not output.native_allowed):
      return 'current_output_unqualified'
    if control.native_required and (not control.native_allowed or not control.native_identity or
        not 0 < control.native_observed_ns <= control.control_ns <= output.apply_ns <= now_ns <= control.native_expires_ns or
        control.native_expires_ns-control.native_observed_ns > 200_000_000):
      return 'current_native_unqualified'
    if not control.native_required and (control.native_allowed or control.native_identity or
        (control.native_observed_ns,control.native_expires_ns) != (0,0)):
      return 'current_native_unqualified'
    return None
  except (AttributeError,IndexError,KeyError,TypeError,ValueError,OverflowError):
    return 'invalid_current_pair'


def snapshot(collector,now_ns):
  adapter,capture = collector.adapter,collector.original_source_capture
  if capture is None:
    adapter.snapshot_readiness = {'reason':'original_source_capture_unavailable'}
    return None
  if (adapter.envelope is None or not collector.ready or not capture.rows or
      adapter.source_plant_submitted_epoch == capture.epoch or
      collector.last_fit and now_ns-collector.last_fit < NORMAL_JOB_ATTEMPT_SPACING_NS):
    return None
  current_decline = _current_fit_pair_reason(collector,now_ns)
  if current_decline is not None:
    adapter.snapshot_readiness = {'reason':current_decline,'childQualification':'not_yet_evaluated'}
    return None
  checkpoints = tuple(stamp for stamp,_ in adapter.replay.checkpoints if stamp <= capture.rows[0].mono_ns)
  if not checkpoints:
    adapter.snapshot_readiness = {'reason':'plant_original_prefix_checkpoint_unavailable'}
    return None
  start = max(checkpoints)
  if now_ns-start < MIN_NORMAL_JOB_AGE_NS:
    adapter.snapshot_readiness = {'reason':'plant_fixed_cohort_still_collecting',
      'elapsedNs':now_ns-start,'requiredNs':MIN_NORMAL_JOB_AGE_NS}
    return None
  # Full original geometry retains the fixed attempt spacing. Its counts
  # are availability only; the child independently reconstructs every join.
  geometry_context = (capture.epoch,getattr(capture,'epoch_start_ns',0),
    getattr(capture,'epoch_cutoff_ns',0),getattr(collector,'context',None),
    getattr(adapter.envelope,'prior_full_profile_sha256',None))
  if getattr(collector,'source_plant_geometry_context',None) == geometry_context:
    last_check = getattr(collector,'source_plant_geometry_check_ns',0)
    if last_check and now_ns-last_check < 1_000_000_000:
      return None
  collector.source_plant_geometry_context = geometry_context
  collector.source_plant_geometry_check_ns = now_ns
  records = tuple(frame for frame in adapter.replay.records if frame.control_ns >= start)
  if not records or len(records) > 12000:
    adapter.snapshot_readiness = {'reason':'plant_original_prefix_frame_budget'}
    return None
  from openpilot.starpilot.flm.event_storage import source_windows
  from openpilot.starpilot.flm.event_windows import whole_event_windows
  from openpilot.starpilot.flm.source_plant_fit import FIRST_SLOT_NS,BLOCK_NS,PURGE_NS
  from types import SimpleNamespace
  view = SimpleNamespace(records=records,replay_windows=(),_frame_context=adapter.replay._frame_context)
  view.replay_windows = source_windows(view)
  cutoff = start+FIRST_SLOT_NS+7*(BLOCK_NS+PURGE_NS)+BLOCK_NS
  count,pending,last_window = 0,0,None
  complete_goal_windows = 0
  for window,first,last,_ in whole_event_windows(view):
    if first <= cutoff:
      continue
    if window != last_window:
      pending = 0
    last_window = window
    points = sum(first <= row.mono_ns <= last for row in capture.rows)
    if points > 240:
      pending = 0
      continue
    # The child requires nine comparable subtle cycles (or three large
    # maneuvers), independently of eight statistical storage bundles. Wait
    # for enough complete original geometry before consuming this epoch's
    # sole attempt. This is availability only: error/excitation/quality and
    # train/heldout lineage remain fresh child qualifications.
    if points >= 4:
      complete_goal_windows += 1
    if pending+points > 240:
      pending = 0
    pending += points
    if pending >= 40:
      count += 1
      pending = 0
  if count < 8 or complete_goal_windows < 9:
    adapter.snapshot_readiness = {'reason':'plant_later_original_goal_geometry',
      'candidateBundles':count,'requiredBundles':8,'completeGoalWindows':complete_goal_windows,
      'requiredCompleteGoalWindows':9,'childQualification':'not_yet_evaluated'}
    return None
  # Geometry availability is not a submitted/failed experiment. Only a ready
  # export attempt starts the existing fit cooldown; later original geometry
  # in this fixed epoch may complete without waiting beyond its100s cutoff.
  collector.last_fit = now_ns
  collector.snapshot_attempts += 1
  try:
    wire = adapter.original_source_packet(collector,((start,records[-1].control_ns),),now_ns)
    return snapshot_from_packet(collector,wire,now_ns)
  except (AttributeError,KeyError,TypeError,ValueError,OverflowError) as error:
    adapter.snapshot_readiness = {'reason':'plant_snapshot_unavailable','detail':str(error)}
    return None


SCHEDULE_VERSION = 'original-source-experiment-objective-v1'


def fit_schedule(collector):
  """Fixed original experiment ordinal, independent of readiness/fit outcomes."""
  from openpilot.starpilot.flm.learner_factory import collection_keys
  capture = collector.original_source_capture
  if capture is None or not collector.ready or not capture.rows or not capture.epoch_start_ns:
    return None
  keys = tuple(collection_keys(collector.ready[-1][0]))
  if (type(capture.epoch) is not int or capture.epoch < 0 or
      not 0 < capture.epoch_start_ns < capture.epoch_cutoff_ns or
      capture.epoch_cutoff_ns-capture.epoch_start_ns > 100_000_000_000 or
      not 1 <= len(keys) <= 12 or len(set(keys)) != len(keys)):
    raise ValueError('original source experiment objective registry')
  return {'version':SCHEDULE_VERSION,'epoch':capture.epoch,
    'start_ns':capture.epoch_start_ns,'cutoff_ns':capture.epoch_cutoff_ns,
    'registered_keys':keys,'objective_key':keys[capture.epoch % len(keys)]}


def fit_key(collector):
  schedule = fit_schedule(collector)
  return None if schedule is None else schedule['objective_key']


def submission_key(snapshot,collector,capture_key):
  if getattr(collector.adapter,'source_plant_protocol',None) is None:
    return capture_key
  key = tuple(snapshot.get('objective_key',()))
  if key != fit_key(collector):
    raise ValueError('original source experiment submitted objective registry')
  return key


def snapshot_from_packet(collector,packet_wire,now_ns):
  """Select the original experiment's key only around the job transport."""
  schedule = fit_schedule(collector)
  if schedule is None:
    return None
  adapter = collector.adapter
  capture_key = adapter.key
  adapter.select_objective(schedule['objective_key'])
  try:
    return _snapshot_from_packet(collector,packet_wire,now_ns,schedule)
  finally:
    adapter.select_objective(capture_key)


def _snapshot_from_packet(collector,packet_wire,now_ns,schedule,*,response_only=False):
  """Common lossless worker transport for an already exported ORIGINAL packet.

  This does no fit-readiness or admission declaration. The normal parent calls
  it only after its predeclared readiness/cadence gates.
  """
  from openpilot.starpilot.flm.controller_replay import pack_primitive_snapshot,MAX_FIT_RAW_BYTES
  from openpilot.starpilot.flm.source_plant_pipeline import replay_data
  import pickle
  adapter,capture = collector.adapter,collector.original_source_capture
  if type(response_only) is not bool:
    raise ValueError('explicit response-only transport purpose')
  try:
    from openpilot.starpilot.flm.original_source_capture import (
      unpack_source_packet, SOURCE_PACKET_VERSION, SOURCE_PACKET_ENCODING, NORMALIZED_SOURCE_VERSION,
    )
    if response_only:
      # This parent owns a freshly encoded, read-only transport graph. Do not
      # expand every shared container just to intern it again. The response
      # worker still makes its independent copy and verifies ALL original data.
      from openpilot.starpilot.flm.controller_replay import unpack_primitive_snapshot
      from openpilot.starpilot.flm.primitive_source_memo import _logical_limits
      from openpilot.starpilot.flm.normalized_trace_archive import reserve_for_packet
      packet = unpack_primitive_snapshot(packet_wire)
      if (type(packet) is not dict or packet.get('version') not in (SOURCE_PACKET_VERSION, NORMALIZED_SOURCE_VERSION) or
          packet.get('encoding') != SOURCE_PACKET_ENCODING or packet.get('qualified') is not False):
        raise ValueError('unsupported original source packet codec/version')
      _logical_limits((packet, bytes(reserve_for_packet(packet))))
      context = None  # Computed independently by the worker; never admission.
    else:
      packet = unpack_source_packet(packet_wire)
      # Trial/restore retains its exact pending experiment binding unchanged.
      from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
      template = EventControllerReplay(adapter.replay,adapter.event_objective,adapter.envelope.replay_anchor())
      binding = source_binding(packet,template)
      context = experiment_context(packet,binding)
    envelope = adapter.envelope
    from openpilot.starpilot.flm.feedback_replay import PROTOCOL as FEEDBACK_PROTOCOL,EVENT_VERSION
    feedback_job = adapter.source_plant_protocol == FEEDBACK_PROTOCOL
    operation,version = 'trial',EVENT_VERSION if feedback_job else 3
    if adapter.restore_envelope is not None and not response_only:
      if feedback_job and adapter.restore_version != 8 or not feedback_job and adapter.restore_version == 8:
        raise ValueError('feedback restore requires explicit version8')
      envelope = adapter.envelope.for_trial(adapter.event_objective.names).restore_from_history(adapter.optimizer_prior)
      operation,version = 'restore',adapter.restore_version
    from openpilot.starpilot.flm.clipping_quality import identification_for_quality,restore_policy,FIXED_MAP_IDENTIFICATION
    payload = {'version':1,'family':adapter.command_family,'identity':adapter.identity,
      'plant_protocol':adapter.source_plant_protocol,'source_packet':packet,'physical_context':context,
      'event_version':version,'event_operation':operation,'event_objective':asdict(adapter.event_objective),
      'effect_envelope':asdict(envelope),'objective':asdict(adapter.objective),
      'event_clipping_quality':adapter.clipping_quality,
      'event_identification_policy':identification_for_quality(adapter.clipping_quality) if operation=='trial' else FIXED_MAP_IDENTIFICATION,
      'now_ns':now_ns,'sequence':collector.sequence+1,
      'prior':tuple(dict(envelope.prior_coordinates)[name] for name in adapter.event_objective.names),
      'objective_key':adapter.key,'fit_schedule':schedule}
    if feedback_job:
      from openpilot.starpilot.flm.event_mean_evidence import VERSION as MEAN_VERSION
      from openpilot.starpilot.flm.feedback_archive import archive_binding
      feedback_wire = packet['feedback_observations']
      from openpilot.starpilot.flm.candidate_feedback import CLOSED_POLICY
      payload['candidate_feedback_policy'] = CLOSED_POLICY
      if operation == 'trial':
        payload['event_mean_version'] = MEAN_VERSION
        feedback_submission = None if response_only else (archive_binding(packet),feedback_wire['digest'].hex(),MEAN_VERSION,CLOSED_POLICY)
      else:
        from openpilot.starpilot.flm.clipping_quality import RESTORE_FEEDBACK_STATISTICS
        payload['event_restore_statistics'] = RESTORE_FEEDBACK_STATISTICS
        feedback_submission = (archive_binding(packet),feedback_wire['digest'].hex(),RESTORE_FEEDBACK_STATISTICS,CLOSED_POLICY)
    else:
      feedback_submission = None
    if operation == 'restore':
      payload['event_restore_policy'] = restore_policy(version)
    from opendbc.car.steering_projection import projection_to_dict
    if not response_only:
      # Trial/restore jobs require their current endpoints. Response-only
      # bootstrap independently verifies its frozen original experiment and
      # never issues a profile or current permission; its worker uses neither.
      control,output = collector.ready[-1]
      from openpilot.starpilot.flm.normalized_trace_archive import normalized_packet,current_record
      payload['control'] = current_record(control,packet['trace_archive']) if normalized_packet(packet) else asdict(control)
      payload['output'] = {**asdict(output),'projection':projection_to_dict(output.projection)}
    replay = replay_data(packet,objective=adapter.objective,identity=adapter.identity,baseline_revision=adapter.baseline_revision)
    payload_raw_bytes = len(pickle.dumps(payload,protocol=5))
    replay_raw_bytes = len(pickle.dumps(replay,protocol=5))
    if payload_raw_bytes+replay_raw_bytes > MAX_FIT_RAW_BYTES:
      raise ValueError('plant aggregate original raw worker byte bound')
    if feedback_job:
      from openpilot.starpilot.flm.primitive_source_memo import intern_primitive_graph
      from openpilot.starpilot.flm.normalized_trace_archive import reserve_for_packet
      reserve = reserve_for_packet(packet)
      if response_only:
        # Source packet sharing is already canonical. Preserve it and validate
        # the SAME logical expansion/depth/reservation caps without rebuilding.
        _logical_limits((payload,bytes(reserve)))
      else:
        payload = intern_primitive_graph((payload,bytes(reserve)))[0] if reserve else intern_primitive_graph(payload)
    packed = pack_primitive_snapshot(payload)  # Existing raw/IPC caps unchanged.
    adapter.source_plant_export_cost = {'sourceWireBytes':len(packet_wire),'payloadRawBytes':payload_raw_bytes,
      'derivedReplayRawBytes':replay_raw_bytes,'aggregateRawBytes':payload_raw_bytes+replay_raw_bytes,'workerCompressedBytes':len(packed)}
    wrapper = {'version':1,'family':adapter.command_family,'snapshot_b64':base64.b64encode(packed).decode('ascii'),
      'objective_key':tuple(adapter.key)}
    if len(wrapper['snapshot_b64']) > 6*1024*1024:
      raise ValueError('plant existing fit wrapper bound')
  except (AttributeError,KeyError,TypeError,ValueError,OverflowError) as error:
    adapter.snapshot_readiness = {'reason':'plant_snapshot_unavailable','detail':str(error)}
    return None
  # Pending calibration is not the active model context. A failed/new attempt
  # must not switch off an independently fresh previously accepted correction.
  if response_only:
    collector.sequence += 1
    return wrapper
  import weakref
  adapter.source_plant_collector = weakref.ref(collector)
  adapter.source_plant_pending_context = context
  collector.sequence += 1
  adapter.source_plant_pending_epoch = capture.epoch
  adapter.source_plant_pending_feedback = feedback_submission
  adapter.snapshot_readiness = {'reason':'snapshot_ready','plantProtocol':adapter.source_plant_protocol,'childQualification':'not_yet_evaluated'}
  return wrapper


def submitted(collector):
  adapter = collector.adapter
  adapter.source_plant_submitted_epoch = adapter.source_plant_pending_epoch
  contexts = adapter.source_plant_submitted_contexts
  contexts[adapter.source_plant_pending_context] = adapter.source_plant_pending_feedback or True
  while len(contexts) > 14:  # Same bound as the existing cached profile registry.
    contexts.pop(next(iter(contexts)))


def install(adapter, report, physical_context, *, commit=False):
  from openpilot.starpilot.flm.runtime_adapter_registry import digest
  protocol = adapter.source_plant_protocol
  if report.get('plant_protocol') != protocol or tuple(report.get('physical_context',())) != tuple(physical_context):
    return False
  expected_context = ('original-source-feedback-mean-context-v1' if protocol ==
    'chronological-source-feedback-mean-fit-v1' else CONTEXT_VERSION)
  if not physical_context or physical_context[0] != expected_context:
    return False
  if tuple(physical_context) not in adapter.source_plant_submitted_contexts:
    return False
  from openpilot.starpilot.flm.feedback_replay import PROTOCOL as FEEDBACK_PROTOCOL,EVENT_VERSION
  if protocol == FEEDBACK_PROTOCOL:
    from openpilot.starpilot.flm.event_mean_evidence import VERSION as MEAN_VERSION
    offered = report.get('feedback_evidence')
    submitted = adapter.source_plant_submitted_contexts[tuple(physical_context)]
    basis = report.get('event_mean_basis')
    from openpilot.starpilot.flm.clipping_quality import RESTORE_FEEDBACK_STATISTICS
    restoring = report.get('event_version') == 8
    marker = RESTORE_FEEDBACK_STATISTICS if restoring else MEAN_VERSION
    from openpilot.starpilot.flm.candidate_feedback import CLOSED_POLICY,PROOF_VERSION,report_policy_matches
    closed = type(submitted) is tuple and len(submitted) == 4 and submitted[3] == CLOSED_POLICY
    expected_fields = {'protocol','binding_digest','archive_digest','verified_digest','known_rows','context_rows'}
    if closed:
      expected_fields |= {'candidate_feedback_policy','candidate_feedback_proof_version'}
    if (type(submitted) is not tuple or len(submitted) not in (3,4) or len(submitted) == 4 and not closed or
        report.get('event_version') not in (EVENT_VERSION,8) or
        not report_policy_matches(submitted,offered) or type(offered) is not dict or set(offered) != expected_fields or
        ((offered['binding_digest'],offered['archive_digest'],marker,CLOSED_POLICY) if closed else
         (offered['binding_digest'],offered['archive_digest'],marker)) != submitted or
        closed and (offered['candidate_feedback_policy'] != CLOSED_POLICY or
          offered['candidate_feedback_proof_version'] != PROOF_VERSION) or
        offered['protocol'] != FEEDBACK_PROTOCOL or type(offered['verified_digest']) is not str or len(offered['verified_digest']) != 64 or
        type(offered['known_rows']) is not int or not 1 <= offered['known_rows'] <= 12000 or
        type(offered['context_rows']) is not int or not 0 <= offered['context_rows'] <= 1 or
        (restoring and (adapter.restore_version != 8 or report.get('event_restore_statistics') != marker or basis is not None) or
         not restoring and (type(basis) is not dict or basis.get('version') != MEAN_VERSION))):
      return False
  elif 'feedback_evidence' in report or 'event_mean_basis' in report:
    return False
  receipt = report.get('plant_identification')
  binding = report.get('plant_binding')
  if (type(receipt) is not dict or type(binding) is not dict or
      receipt.get('receipt_schema') != 'compact-source-plant-receipt-v1' or
      receipt.get('status') != 'physical_affine_test_passed' or not receipt.get('model_receipt_digest') or
      receipt.get('binding_digest') != physical_context[1] or digest(binding) != physical_context[1] or
      receipt.get('cutoff_ns') != report.get('model_cutoff_ns') or
      receipt['decision']['lag_ns'] != report['model']['lag_ns'] or
      report['model']['context'] != digest(physical_context)):
    return False
  from openpilot.starpilot.flm import plant_lag
  decision = receipt['decision']
  declaration = {key:value for key,value in decision.items() if key != 'compact_decision_sha256'}
  expected_receipt = plant_lag.digest({'protocol':plant_lag.PHYSICAL_PROTOCOL,'experiment':receipt['experiment_digest'],
    'decision':decision['digest'],'heldout':receipt['heldout_blocks'],'cutoff_ns':receipt['cutoff_ns'],
    'binding':receipt['binding_digest']})
  if (decision.get('protocol') != plant_lag.PHYSICAL_PROTOCOL or decision.get('compact_decision_sha256') != plant_lag.digest(declaration) or
      receipt['model_receipt_digest'] != expected_receipt or
      tuple(row[0] for row in decision['training_scores']) != plant_lag.LAGS_NS or
      any(not 0 <= error <= .05 or outside != 0 for _,error,outside in receipt['heldout_blocks']) or
      any(report['model'][name] != decision[name] for name in ('slope','intercept','command_low','command_high'))):
    return False
  if commit:
    collector = adapter.source_plant_collector()
    if collector is None:
      return False
    adapter.source_plant_receipt = receipt
    adapter.source_plant_binding = binding
    collector.physical_context = tuple(physical_context)
  return True


def installed_report_context_current(adapter,report,physical_context):
  """Recognize only the exact report installed while awaiting fresh admission.

  The original submission token stays immutable. Installation switches active
  model context, so that same pending report may survive to the next ORIGINAL
  pose without pretending its submission used the new context.
  """
  from openpilot.starpilot.flm.runtime_adapter_registry import canonical_physical_context
  context = tuple(report.get('physical_context',()))
  return (getattr(adapter,'source_plant_protocol',None) in (PROTOCOL,
      'chronological-source-feedback-mean-fit-v1') and
    report.get('plant_protocol') == adapter.source_plant_protocol and context == physical_context and
    adapter.model is not None and adapter.model.context == canonical_physical_context(context) and
    adapter.source_plant_receipt is not None and adapter.source_plant_receipt == report.get('plant_identification') and
    adapter.source_plant_binding == report.get('plant_binding'))
