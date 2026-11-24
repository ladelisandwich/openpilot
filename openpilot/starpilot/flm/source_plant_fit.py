from openpilot.starpilot.flm.normalized_trace_archive import guarded_archive
from openpilot.starpilot.flm.normalized_trace_archive import SOURCE_VERSION as NORMALIZED_SOURCE_VERSION,archive_of,record_body,stored_record
"""Child-owned independent source-pose plant experiment.

No legacy RawPoint or external delay enters this module. The controller owner
must first replay the complete original inventory, including excluded context.
Only actual healthy native/CAN endpoints become the arithmetic command tape.
"""
from bisect import bisect_right
from dataclasses import asdict, dataclass
import hashlib
import math

from openpilot.starpilot.flm import plant_lag as lag

PROTOCOL = 'chronological-source-physical-observation-fit-v1'
CONTEXT_VERSION = 'original-source-physical-observation-context-v1'
BLOCK_NS = 4_000_000_000
PURGE_NS = 600_000_000
FIRST_SLOT_NS = 2_600_000_000


@dataclass(frozen=True)
class TemporalPlantWindow:
  """Chronological physical validation block, not an independent maneuver."""
  window_id: str
  slot: int
  checkpoint_ns: int
  start_ns: int
  end_ns: int  # Inclusive integer convention for the pure helper.
  crossing_ns: int


def _hash(value):
  from openpilot.starpilot.flm.runtime_adapter_registry import digest
  return digest(value)


@dataclass(frozen=True)
class PreparedPlant:
  binding: lag.SourceBinding
  identification: lag.IdentificationResult
  context: tuple
  model: object
  source_poses: tuple
  command_pairs: tuple
  original_windows: tuple


def compact_receipt(result):
  """Bounded worker report; full original rows stay inside the existing child.

  The original decision/model digests still bind the complete training IDs and
  exact row inventory. This wire schema adds compact custody digests rather
  than exceeding FitProcess's unchanged64KiB result limit.
  """
  value = asdict(result)
  rows = value.pop('rows')
  value.update(receipt_schema='compact-source-plant-receipt-v1',row_count=len(rows),rows_sha256=lag.digest(rows))
  if value['decision'] is not None:
    decision = value['decision']
    ids = decision.pop('train_pose_ids')
    decision.update(train_pose_ids_sha256=lag.digest(ids),train_pose_count=len(ids))
    decision['compact_decision_sha256'] = lag.digest(decision)
  return value


def _pose(value, states, binding, controls,outputs,engine):
  """Recompute calibrated yaw/roll and interpolation from original primitives."""
  from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose, validate_pose
  from openpilot.common.transformations.orientation import rot_from_euler, euler_from_rot
  import numpy as np

  def tuples(item):
    return tuple(tuples(child) for child in item) if type(item) in (list,tuple) else item
  row = dict(value)
  for name in ('context', 'angular_velocity_device', 'orientation_ned', 'calibration_rpy', 'left_state', 'right_state',
               'source_clock_contract', 'clock_equivalence', 'ordinary_safety'):
    row[name] = tuples(row[name])
  original = OriginalSourcePose(**row)
  validate_pose(original, original.completed_ns)
  native = controls.get(original.native_control_ns)
  if (native is None or original.native_required is not binding.native_required or
      (native.native_required,native.native_identity,native.native_observed_ns,native.native_expires_ns) !=
      (original.native_required,original.native_identity,original.native_observed_ns,original.native_expires_ns)):
    raise ValueError('plant original pose native owner receipt')
  if not original.native_required and original.ordinary_safety[1] != binding.cp_sha256:
    raise ValueError('plant ordinary native exact CarParams')
  applied = outputs.get((original.native_control_ns,original.native_apply_ns))
  if (applied is None or applied.apply_ns != original.native_apply_ns or
      not native.enabled or not native.active or not native.valid or native.manual or native.override or native.fault or
      not applied.valid or not applied.active or applied.override or applied.fault or not applied.native_allowed or
      not engine.applied_matches(native,applied)):
    raise ValueError('plant original pose receipt actual native/CAN')
  if ((original.identity, original.source_epoch, original.baseline_revision, _hash(original.controller_revision),
       original.native_identity, original.calibration, _hash((original.source_clock_contract,original.context[7]))) !=
      (binding.identity, binding.source_epoch, binding.baseline_revision, binding.controller_revision,
       binding.native_identity, binding.calibration_generation, binding.source_clock_contract_sha256)):
    raise ValueError('plant original pose source/native/calibration')
  # Both speed brackets must occur in independently reconstructed original
  # ControlEvidence, rather than a second copy of supplied pose floats.
  if (states.get(original.left_state_ns) != original.left_state or
      states.get(original.right_state_ns) != original.right_state):
    raise ValueError('plant original state bracket custody')
  device_from_calib = rot_from_euler(np.asarray(original.calibration_rpy))
  yaw = float((device_from_calib.T @ np.asarray(original.angular_velocity_device))[2])
  roll = float(euler_from_rot(rot_from_euler(np.asarray(original.orientation_ned)) @ device_from_calib)[0])
  if yaw != original.yaw_rate or roll != original.roll_rad or not math.isfinite(yaw + roll):
    raise ValueError('plant original calibrated pose recomputation')
  # The estimate identifies exact per-pose extrinsics; lifecycle remains stable.
  if original.calibration_estimate != _hash(tuple(original.calibration_rpy)):
    raise ValueError('plant original extrinsics estimate')
  return lag.SourcePose(pose_id=_hash(asdict(original)),mono_ns=original.mono_ns,received_ns=original.received_ns,
    source_timestamp_ns=original.source_timestamp_ns,source_clock_domain=original.source_clock_domain,
    clock_offset_ns=original.clock_offset_ns,clock_uncertainty_ns=original.clock_uncertainty_ns,
    source_lower_ns=original.source_lower_ns,source_upper_ns=original.source_upper_ns,
    clock_generation=original.clock_generation,speed_mps=original.speed_mps,
    calibrated_yaw_rate=yaw,calibrated_roll_rad=roll,binding_digest=lag.binding_digest(binding))


def _command(control, output, engine, binding):
  from openpilot.starpilot.flm.torque_eligibility import exact_unclipped_pair, consumed_sources_current
  from openpilot.starpilot.flm.torque_emission_replay import delivered
  from opendbc.car.steering_projection import projection_to_dict

  # Preserve the exact original guard order; label a bounded decline in this
  # child-only phase instead of concealing which original endpoint failed.
  rejected = (
    ('disabled',lambda:not control.enabled),('manual',lambda:control.manual),
    ('inactive',lambda:not control.active),('invalid_control',lambda:not control.valid),
    ('required_native_denied',lambda:control.native_required and not control.native_allowed),
    ('override',lambda:control.override),('fault',lambda:control.fault),
    ('saturated',lambda:control.saturated),('model_wobble',lambda:control.model_wobble),
    ('controller_chatter',lambda:control.controller_chatter),
    ('limited_control',lambda:control.limited and not exact_unclipped_pair(control,output)),
    ('legacy_coordinates',lambda:any(control.applied_coordinates)),
    ('command_clock',lambda:output.control_ns != control.control_ns),
    ('invalid_output',lambda:not output.valid),('inactive_output',lambda:not output.active),
    ('output_override',lambda:output.override),('output_fault',lambda:output.fault),
    ('limited_output',lambda:output.limited),('output_native_denied',lambda:not output.native_allowed),
    ('consumed_original_sources',lambda:not consumed_sources_current(control,output)),
    ('actual_native_CAN',lambda:not engine.applied_matches(control,output)),
  )
  for reason,predicate in rejected:
    if predicate():
      raise ValueError('plant actual endpoint '+reason+' control_ns='+str(control.control_ns)+' apply_ns='+str(output.apply_ns))
  return lag.DeliveredCommand(control.control_ns, control.state_ns, output.apply_ns, -delivered(output),
    control.source_receipt_ns, control.native_observed_ns, control.native_expires_ns,
    output.carrier_frame, output.carrier_observed_ns, _hash(projection_to_dict(output.projection)),
    _hash(record_body(control,engine)), _hash({**asdict(output), 'projection': projection_to_dict(output.projection)}),
    control.identity, control.source_epoch, control.baseline_revision, control.native_identity,control.native_required)


def native_output(control, value):
  """Same evidence-only native reconstruction as FullrateCollector._join_emission.

  Original raw output stays in the context packet and packet hash. A current
  independently verified control lease supplies the native permission used by
  the existing healthy EmissionHistory; a parent output flag alone cannot.
  """
  from openpilot.starpilot.flm.torque_raw_fit import output_from_dict
  return _native_output_from_original(control, output_from_dict(value))



def _native_output_from_original(control, output):
  """Same native lease reconstruction from an already parsed original body.

  Private caller-local reuse only, no cached permission or packet authority.
  """
  from dataclasses import replace
  from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence
  if control.native_required:
    if (not control.native_allowed or not control.native_identity or
        not 0 < control.native_observed_ns <= control.control_ns <= output.apply_ns <= control.native_expires_ns or
        control.native_expires_ns-control.native_observed_ns > 200_000_000):
      return output  # Context only; _command rejects if a pose selects it.
    if type(output) is not AppliedEvidence or output.native_allowed is not True:
      output = replace(output,native_allowed=True)
  return output



def _physical_observation(control, output, engine, binding, original_output):
  """New physical-only type. Legacy _command/stage/current stay strict."""
  from openpilot.starpilot.flm.physical_input_observation import from_original
  return from_original(control, output, engine, binding, original_output=original_output)


def _required_controls(controls, poses, *, stamp_field="control_ns"):
  """Exact closed union of every all-lag lookback, in original control order."""
  merged = []
  for lower, upper in sorted((p.source_lower_ns-lag.LAGS_NS[-1]-2*lag.JOIN_NS,
                              p.source_upper_ns) for p in poses):
    if merged and lower <= merged[-1][1]:
      merged[-1] = (merged[-1][0], max(merged[-1][1], upper))
    else:
      merged.append((lower, upper))
  starts = tuple(lower for lower, _ in merged)
  def included(stamp):
    index = bisect_right(starts, stamp)-1
    return index >= 0 and stamp <= merged[index][1]
  return tuple(control for control in controls.values() if included(getattr(control,stamp_field)))


BOOTSTRAP_SLOT_POLICY = 'interval-contained-bootstrap-slots-v1'
SOURCE_COMPLETE_SLOT_POLICY = 'source-complete-fixed-bootstrap-slots-v2'


def bootstrap_slot_rows(rows, start_ns, end_ns):
  selected = []
  for row in rows:
    if type(row) is not dict or type(row.get('mono_ns')) is not int:
      raise ValueError('exact original bootstrap pose clock')
    if not start_ns <= row['mono_ns'] <= end_ns:
      continue
    low, high = row.get('source_lower_ns'), row.get('source_upper_ns')
    if type(low) is not int or type(high) is not int or not 0 < low <= row['mono_ns'] <= high:
      raise ValueError('known original bootstrap source interval required')
    if start_ns <= low and high <= end_ns:
      selected.append(row)
  return tuple(selected)


@guarded_archive(position=1)
def prepare(packet, engine, binding, *, slot_policy=None, support_policy=lag.STRICT_SUPPORT):
  """Exactly one earliest octet; child verifies boundaries before reading y.

  `engine` is the complete original inventory replay returned by the actual
  registered-controller/native archive verifier. A caller assertion is not a
  substitute for that source/native/controller replay.
  """
  from openpilot.starpilot.flm.original_context_receipts import verify as verify_context, reuse_verified_context
  from openpilot.starpilot.flm.torque_raw_fit import output_from_dict
  from openpilot.starpilot.flm.physical_response import PhysicalModel
  from openpilot.starpilot.flm.physical_input_observation import (
    PROTOCOL as OBSERVATION_PROTOCOL, VerifiedPhysicalTape, physical_tape_digest,
  )
  from openpilot.starpilot.flm.runtime_adapter_registry import canonical_physical_context
  from openpilot.starpilot.flm.physical_input_observation import _physical_asdict

  if (packet.get('version') not in ('unpaired-original-source-v8',NORMALIZED_SOURCE_VERSION) or
      packet.get('encoding') != 'primitive-snapshot-v1' or packet.get('qualified') is not False or
      packet.get('plant_lag_ns') is not None or not isinstance(binding, lag.SourceBinding) or
      len(engine.records) > 12000 or len(packet['source_poses']) > 3116):
    raise ValueError('plant independent original packet/bounds')
  if (packet.get('version') == NORMALIZED_SOURCE_VERSION) != (archive_of(engine) is not None):
    raise ValueError('explicit normalized physical source owner')
  if slot_policy not in (None, BOOTSTRAP_SLOT_POLICY, SOURCE_COMPLETE_SLOT_POLICY):
    raise ValueError('explicit physical calibration slot policy')
  frames = tuple(engine.records)
  controls = reuse_verified_context(packet['context_receipts'],engine,frames)
  if controls is None:
    controls = verify_context(packet['context_receipts'],engine,frames)
  q = frames[0].control_ns
  if q not in dict(engine.checkpoints):
    raise ValueError('plant original first checkpoint unavailable')
  final_end = q+FIRST_SLOT_NS+7*(BLOCK_NS+PURGE_NS)+BLOCK_NS
  crossings = tuple(frame for frame in frames if frame.control_ns >= final_end)
  if not crossings:
    return {'status':'collecting','reason':'chronological_plant_slots_not_sealed','profile':None,'validation':None}
  # Seal each slot by its ORIGINAL crossing tick in the same continuous healthy
  # prefix. No fault, buffer tail or source-window closure is called independence.
  prefix = tuple(frame for frame in frames if frame.control_ns <= crossings[0].control_ns)
  if any(not frame.healthy or frame.manual or
         not controls[frame.control_ns].enabled or not controls[frame.control_ns].active or
         not controls[frame.control_ns].valid or controls[frame.control_ns].manual or
         controls[frame.control_ns].override or controls[frame.control_ns].fault for frame in prefix):
    raise ValueError('plant chronological prefix contains invalid physical source context')
  windows = []
  for slot in range(8):
    start = q+FIRST_SLOT_NS+slot*(BLOCK_NS+PURGE_NS)
    end = start+BLOCK_NS
    crossing = next(frame.control_ns for frame in frames if frame.control_ns >= end)
    original = tuple(_physical_asdict(controls[frame.control_ns],_trace_owner=engine) for frame in prefix if start <= frame.control_ns < end)
    window_id = _hash((PROTOCOL,lag.binding_digest(binding),q,slot,start,end,original))
    if slot_policy is not None:
      window_id = _hash((slot_policy,window_id))
    windows.append(TemporalPlantWindow(window_id,slot,q,start,end-1,crossing))
  outputs, original_outputs = {}, {}
  for row in packet['context_receipts']['outputs']:
    stamp = row['control_ns']
    control = controls[stamp]  # Preserve original lookup-before-parse order.
    original = output_from_dict(row)
    key=(stamp,original.apply_ns)
    outputs[key] = _native_output_from_original(control,original)
    original_outputs[key] = original
  states = {}
  for control in controls.values():
    values = dict(control.replay_inputs)
    state = (float(values['vEgo']), float(values['steeringAngleDeg']))
    if control.state_ns in states and states[control.state_ns] != state:
      raise ValueError('plant conflicting original state bracket')
    states[control.state_ns] = state
  poses, selected = [], []
  # Predeclared exact eight slots; a short/long slot rejects this experiment.
  # Never skip/top-up/slide a slot after command or response outcomes are known.
  for window in windows:
    raw = (bootstrap_slot_rows(packet['source_poses'], window.start_ns, window.end_ns) if slot_policy is not None else
      tuple(row for row in packet['source_poses'] if window.start_ns <= row['mono_ns'] <= window.end_ns))
    if not 40 <= len(raw) <= 240:
      return {'status':'collecting','reason':'chronological_plant_slot_pose_count',
        'profile':None,'validation':None,'slot':window.slot,'poseCount':len(raw)}
    selected.append((window, raw))
  if len(selected) != 8:
    return {'status': 'collecting', 'reason': 'independent_original_source_window_support',
      'profile': None, 'validation': None, 'temporalBlocks': len(selected)}
  if slot_policy == SOURCE_COMPLETE_SLOT_POLICY:
    # Preserve the FULL original packet, native reconstruction and fixed slots.
    # Determine eligibility solely from original input metadata. The independent
    # physical verifier below still has to admit EVERY command of EVERY
    # retained all-lag lookback; parent-provided masks are never trusted here.
    from types import SimpleNamespace
    from openpilot.starpilot.flm.live_tracking_bootstrap_cohorts import source_complete_groups
    from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current
    original_events = tuple(engine.emissions.events.values())
    stamps = tuple(event.apply_ns for event in original_events)
    source_known = []
    for event in original_events:
      control = controls.get(event.control_ns)
      output = outputs.get((event.control_ns,event.apply_ns))
      known = (control is not None and output is not None and event.valid and event.active and
        not event.override and not event.fault and event.native_allowed and
        control.valid and control.active and control.enabled and not control.manual and
        not control.override and not control.fault and consumed_sources_current(control,output))
      source_known.append(0. if known else None)
    # Only source clock intervals are exposed to selection, never response
    # yaw/roll/force, tracking error, residual or a proposed tuning coordinate.
    clocks = tuple(tuple(SimpleNamespace(mono_ns=row['mono_ns'],source_lower_ns=row['source_lower_ns'],
      source_upper_ns=row['source_upper_ns']) for row in raw) for _,raw in selected)
    eligible = source_complete_groups(stamps,source_known,clocks)
    replacement = []
    for (window,raw), group in zip(selected,eligible,strict=True):
      included = {row.mono_ns for row in group}
      retained = tuple(row for row in raw if row['mono_ns'] in included)
      if not 40 <= len(retained) <= 240:
        return {'status':'collecting','reason':'source_complete_fixed_slot_count',
          'profile':None,'validation':None,'slot':window.slot,'poseCount':len(retained)}
      replacement.append((window,retained))
    selected = replacement
  declared = []
  for window, raw in selected:
    block_poses = tuple(_pose(row, states, binding,controls,outputs,engine) for row in raw)
    if any(p.source_lower_ns < window.start_ns or p.source_upper_ns > window.end_ns for p in block_poses):
      raise ValueError('plant source interval crosses immutable temporal slot')
    poses.extend(block_poses)
    declared.append(lag.CalibrationWindow(_hash((PROTOCOL, window.window_id)), window.start_ns,
      window.end_ns, tuple(p.pose_id for p in block_poses), _hash(asdict(window))))
  # Verify every original command in all-candidate lookbacks once. An invalid
  # command is an unavailable experiment; never silently drop it from the tape.
  # Controller ticks and Card applications are different clocks. Preserve
  # every actual emission, including repeated CCs; a superseded CC that was
  # never sent is controller context, not an invented physical plant input.
  required = _required_controls(engine.emissions.events,poses,stamp_field="apply_ns")
  if slot_policy == SOURCE_COMPLETE_SLOT_POLICY:
    events = tuple(engine.emissions.events.values())
    stamps = tuple(event.apply_ns for event in events)
    covered = set()
    for pose in poses:
      first = bisect_right(stamps,pose.source_lower_ns-lag.LAGS_NS[-1])-1
      last = bisect_right(stamps,pose.source_upper_ns)-1
      if first < 0 or last < first:
        return {'status':'collecting','reason':'source_complete_lookback_missing','profile':None,'validation':None}
      covered.update(range(first,last+1))
    required = tuple(event for index,event in enumerate(events) if index in covered)

  if not required or any((event.control_ns,event.apply_ns) not in outputs for event in required):
    return {'status':'collecting','reason':'plant_original_application_receipt_missing','profile':None,'validation':None}
  pairs = tuple((controls[event.control_ns], outputs[event.control_ns,event.apply_ns]) for event in required)
  try:
    commands = tuple(_physical_observation(control, output, engine, binding,
      original_outputs[control.control_ns,output.apply_ns]) for control, output in pairs)
  except ValueError as error:
    return {'status':'collecting','reason':'plant_common_original_command_unavailable',
      'detail':str(error),'profile':None,'validation':None}
  verifier_hash = hashlib.sha256(__loader__.get_data(__file__)).hexdigest()
  tape = VerifiedPhysicalTape(OBSERVATION_PROTOCOL, commands, lag.binding_digest(binding),
    physical_tape_digest(binding, commands), verifier_hash)
  if support_policy != lag.STRICT_SUPPORT and slot_policy not in (BOOTSTRAP_SLOT_POLICY,SOURCE_COMPLETE_SLOT_POLICY):
    raise ValueError('training-domain support is response-bootstrap only')
  result = lag.identify_physical_plant_lag(binding, tape, tuple(poses), tuple(declared), support_policy=support_policy)
  if result.status != 'physical_affine_test_passed':
    return {'status': result.status, 'reason': result.reason, 'profile': None, 'validation': None,
      'plant_identification': compact_receipt(result)}
  # Freeze this experiment context before child fitting. The exact packet binds
  # every decision input, and the model itself carries the chosen lag. A parent
  # cannot promote an old external-seed report into this context/version.
  context = experiment_context(packet, binding)
  canonical = canonical_physical_context(context)
  # The new typed observation protocol passed the SAME affine, direction,
  # excitation and ALL-row per-block .05 arithmetic. Bootstrap may retain
  # an explicitly training-domain-only model; its range never expands to cover
  # unseen inputs and every heldout block still needs in-domain support.
  # Do not manufacture legacy
  # PhysicalRow(healthy=True,saturated=False) from tagged originals merely to
  # get past the legacy unclipped-input policy. A numeric plant model is NOT
  # permission; its context binds this exact packet, tagged tape and decision.
  decision = result.decision
  if (decision.protocol != lag.PHYSICAL_PROTOCOL or decision.slope <= 0 or
      not lag.heldout_supported(result) or
      decision.train_block_ids != tuple(window.block_id for window in declared[:4]) or
      tuple(block for block, _, _ in result.heldout_blocks) != tuple(window.block_id for window in declared[4:])):
    raise ValueError('physical observation original affine receipt mismatch')
  model = PhysicalModel(engine.identity, canonical, 'normalized_torque', decision.lag_ns,
    decision.slope, decision.intercept, decision.command_low, decision.command_high,
    decision.train_block_ids, tuple(window.block_id for window in declared[4:]),
    max(error for _, error, _ in result.heldout_blocks))
  return PreparedPlant(binding, result, context, model, tuple(poses), pairs, tuple(window for window, _ in selected))



def experiment_context(packet, binding):
  version = 'original-source-feedback-mean-context-v1' if 'feedback_observations' in packet else CONTEXT_VERSION
  return (version, lag.binding_digest(binding), _hash(packet))


def source_binding(packet, engine):
  """Binding from exact selected factory/anchor and original observer context."""
  from openpilot.starpilot.flm.runtime_adapter_registry import car_params_digest
  first = packet['source_poses'][0]
  context = tuple(first['context'])
  if len(context) != 15 or any(tuple(row['context']) != context for row in packet['source_poses']):
    raise ValueError('plant stable original source observer context')
  session, identity, generation, epoch, baseline, revision, calibration, clock_context, context_clock, native, regime, *geometry = context
  cp = engine.factory.cp
  if (identity != engine.identity or epoch != engine.objective.context_key or baseline != engine.baseline_revision or
      revision != engine.original_anchor['controller_revision'] or geometry !=
      [float(cp.steerRatio), float(cp.wheelbase), float(cp.centerToFront), float(cp.mass)] or
      first['identity'] != identity or first['source_epoch'] != epoch or first['native_identity'] != native):
    raise ValueError('plant selected factory/source observer binding')
  contract = tuple(first['source_clock_contract'])
  if tuple(clock_context) != ('original_source_clock_generation', first['clock_generation']):
    raise ValueError('plant original interval clock context mismatch')
  if any(tuple(row['source_clock_contract']) != contract for row in packet['source_poses']):
    raise ValueError('plant original source clock contract changed')
  required = packet['context_receipts']['controls'][0]['native_required']
  if type(required) is not bool or any(row['native_required'] is not required for row in packet['context_receipts']['controls']):
    raise ValueError('plant original native-required contract changed')
  return lag.SourceBinding(identity, car_params_digest(cp), _hash(engine.original_anchor), _hash(revision),
    epoch, baseline, native, calibration, _hash((generation, regime, geometry)), session, _hash((contract,clock_context)),required)


STAGE_MEMBERSHIP_PROTOCOL = 'original-completed-whole-source-membership-v1'


def completed_stage_windows(engine, cutoff_ns):
  """Goal/source geometry only; no response, fit, clipping or loss selection.

  Whole original windows come from the actual verified replay. Incomplete
  source prefixes/tails are not completed maneuvers. Their frames, poses and
  native packets remain archived and in every counterfactual rollout.
  """
  from openpilot.starpilot.flm.controller_replay import ControllerReplay
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  from openpilot.starpilot.flm.event_windows import whole_event_windows
  if (type(engine) is not EventControllerReplay or type(engine.engine) is not ControllerReplay or
      engine.failed or not engine.raw_mode or type(cutoff_ns) is not int or cutoff_ns <= 0 or
      not 1 <= len(engine.records) <= 12000):
    raise ValueError('stage membership requires actual original replay')
  return tuple(window for window in whole_event_windows(engine) if window[1] > cutoff_ns)


def _stage_context_endpoint(control, output, engine):
  """Excluded goal prefix/tail stays HARD context, never a fitting input.

  Original performance labels remain unchanged/archived. Manual, source,
  clocks, native permission, override, fault and actual CAN are mandatory even
  for an endpoint outside the completed goal windows.
  """
  from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current
  if (not control.enabled or control.manual or not control.active or not control.valid or
      control.native_required and not control.native_allowed or control.override or control.fault or
      any(control.applied_coordinates) or output.control_ns != control.control_ns or
      not output.valid or not output.active or output.override or output.fault or not output.native_allowed or
      not consumed_sources_current(control,output) or not engine.applied_matches(control,output)):
    raise ValueError('plant excluded whole-source prefix hard endpoint control_ns='+str(control.control_ns))


def stage_control_member(windows, control_ns):
  """Pure membership predicate; it confers no source/native/application proof."""
  return any(start <= control_ns <= end for _,start,end,_ in windows)


def rejoin_stage(poses, commands, prepared, engine):
  """Associate NEW stage poses with original delivered commands at locked lag."""
  from openpilot.starpilot.flm.torque_raw_fit import control_from_dict
  from openpilot.starpilot.flm.torque_emission_replay import delivered
  from opendbc.car.steering_projection import projection_to_dict

  if not isinstance(prepared, PreparedPlant):
    raise ValueError('plant frozen stage plan required')
  cutoff, delay = prepared.identification.cutoff_ns, prepared.model.lag_ns
  def parsed_pair(c,o):
    control = control_from_dict(c,trace_archive=archive_of(engine))
    return control,native_output(control,o)
  ordered = tuple(parsed_pair(c,o) for c, o in commands)
  stamps = tuple(output.apply_ns for _, output in ordered)
  if any(right <= left for left, right in zip(stamps, stamps[1:], strict=False)):
    raise ValueError('plant original stage command order')
  windows = completed_stage_windows(engine,cutoff)
  previous, previous_command, rows = cutoff, cutoff, []
  for pose in poses:
    index = bisect_right(stamps, pose.source_lower_ns-delay)-1
    upper_index = bisect_right(stamps,pose.source_upper_ns-delay)-1
    if index < 0 or index != upper_index or pose.source_upper_ns-delay-stamps[index] > lag.JOIN_NS:
      raise ValueError('plant original stage lag support')
    control, output = ordered[index]
    if min(control.control_ns, output.apply_ns, pose.mono_ns) <= cutoff or pose.mono_ns <= previous:
      raise ValueError('plant stage strictly after physical cutoff')
    previous = pose.mono_ns  # Preserve ordering of EVERY original stage pose.
    if not stage_control_member(windows,control.control_ns):
      _stage_context_endpoint(control,output,engine)
      continue  # Original goal geometry only; no performance/y/native selection.
    if output.apply_ns <= previous_command:
      raise ValueError('plant stage strictly after physical cutoff')
    # Invoke exactly the same source/native/output guards again for NEW joins.
    # The binding used by _command is only carried metadata; actual verification
    # is engine.applied_matches, not the earlier calibration tape boolean.
    _command(control, output, engine, prepared.binding)
    previous_command = output.apply_ns
    rows.append({'control_ns': control.control_ns, 'apply_ns': output.apply_ns, 'mono_ns': pose.mono_ns,
      'control': stored_record(control), 'output': {**asdict(output), 'projection': projection_to_dict(output.projection)},
      'physical_context': prepared.context, 'physical': {'applied_command': delivered(output),
        'lateral_accel': pose.speed_mps*pose.calibrated_yaw_rate-9.81*math.sin(pose.calibrated_roll_rad),
        'speed_mps': pose.speed_mps, 'response_ns': pose.mono_ns}})
  return tuple(rows)
