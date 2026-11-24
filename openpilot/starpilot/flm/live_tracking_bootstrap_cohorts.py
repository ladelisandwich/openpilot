"""Outcome-blind scheduling of a FIRST complete original calibration cohort.

This is source/command availability, never plant identification or permission.
Every candidate is an actual captured checkpoint, ordered by source clock. All
response yaw/roll/error/residual values are deliberately absent from selection.
"""

from bisect import bisect_left, bisect_right
import math
import pickle
from typing import cast

VERSION = 'first-complete-physical-feedback-cohort-v7'
MAX_CHECKPOINTS = 128
CHECK_INTERVAL_NS = 1_000_000_000


def _decline(diagnostics, reason):
  if type(diagnostics) is dict:
    diagnostics['reason'] = reason
  return None


def _literal(value):
  kind = type(value)
  if kind in (type(None), bool, int, str, bytes):
    return value
  if kind is float and math.isfinite(value):
    return value
  if kind is tuple:
    return tuple(_literal(item) for item in value)
  raise ValueError('closed source-only cohort scheduling value required')


def source_quality_keys(frames, poses, emissions, checkpoints, controls, *, diagnostics=None):
  """Fresh exact values invalidate scheduling memo; never reads a response.

  The memo avoids rescanning every sealed cohort, NOT original source/native
  verification. Its keys contain all fields that scheduling reads, with Float64
  bits preserved by pickle5. Unknown leaves are rejected before pickle callbacks.
  """
  from openpilot.starpilot.flm.controller_replay import ReplayFrame, Snapshot
  from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence
  from openpilot.starpilot.flm.torque_emission_replay import Emission
  from opendbc.car.steering_projection import TorqueProjection

  if (
    len(frames) > 12000
    or len(poses) > 5120
    or len(emissions) > 12000
    or any(type(row) is not ReplayFrame for row in frames)
    or any(type(row) is not OriginalSourcePose for row in poses)
    or any(type(row) is not Emission or type(row.witness) is not TorqueProjection for row in emissions)
    or len(controls) > 12000
    or any(type(row) is not ControlEvidence for row in controls)
  ):
    raise ValueError('exact bounded original cohort source')
  if len(checkpoints) > MAX_CHECKPOINTS:
    _decline(diagnostics, 'checkpoint_count_exceeds_128_bound')
    raise ValueError('genuine bounded checkpoint metadata')
  if any(type(row) is not tuple or len(row) != 2 or type(row[1]) is not Snapshot for row in checkpoints):
    _decline(diagnostics, 'checkpoint_metadata_type_unavailable')
    raise ValueError('genuine bounded checkpoint metadata')
  raw = (
    tuple((row.control_ns, row.healthy, row.manual, row.frame_sequence) for row in frames),
    tuple(
      (
        row.mono_ns,
        row.source_lower_ns,
        row.source_upper_ns,
        row.completed_ns,
        row.speed_mps,
        row.valid,
        row.native_allowed,
        row.native_required,
        row.identity,
        row.source_epoch,
        row.baseline_revision,
        row.controller_revision,
        row.calibration,
        row.native_identity,
        row.context,
      )
      for row in poses
    ),
    tuple(
      (
        row.control_ns,
        row.state_ns,
        row.apply_ns,
        tuple(row.witness),
        row.valid,
        row.active,
        row.override,
        row.fault,
        row.limited,
        row.native_allowed,
        row.native_required,
        row.identity,
        row.source_epoch,
        row.baseline_revision,
        row.native_identity,
        row.native_observed_ns,
        row.native_expires_ns,
      )
      for row in emissions
    ),
    tuple((stamp, snapshot.revision, snapshot.values) for stamp, snapshot in checkpoints),
    tuple(
      (
        row.control_ns,
        row.state_ns,
        row.enabled,
        row.active,
        row.valid,
        row.manual,
        row.override,
        row.fault,
        row.identity,
        row.source_epoch,
        row.baseline_revision,
        row.controller_revision,
        row.native_identity,
        row.native_required,
        row.native_allowed,
        row.native_observed_ns,
        row.native_expires_ns,
        row.vehicle_model_anchor,
        row.saturated,
        row.model_wobble,
        row.controller_chatter,
        row.limited,
        row.source_receipt_ns,
        row.domain,
        row.stage_schema_revision,
        row.replay_inputs,
        row.requested,
      )
      for row in controls
    ),
  )
  keys = tuple(tuple(pickle.dumps(_literal(row), protocol=5) for row in group) for group in raw)
  for stamps in (
    tuple(row.control_ns for row in frames),
    tuple(row.mono_ns for row in poses),
    tuple(row.apply_ns for row in emissions),
    tuple(row.control_ns for row in controls),
  ):
    if any(type(value) is not int or value <= 0 for value in stamps) or any(right <= left for left, right in zip(stamps, stamps[1:], strict=False)):
      raise ValueError('ordered original source cohort clocks')
  if any(type(row.control_ns) is not int or row.control_ns <= 0 for row in emissions):
    raise ValueError('original emitted command clock type')
  if any(right.control_ns < left.control_ns for left, right in zip(emissions, emissions[1:], strict=False)):
    raise ValueError('original emitted command clock regression')
  return keys


def new_schedule_cache(old, token, keys):
  """Only fresh append-only exact-value source can retain examined anchors."""
  if (
    type(old) is dict
    and old.get('token') == token
    and type(old.get('keys')) is tuple
    and len(old['keys']) == len(keys)
    and type(old.get('examined')) is dict
    and 'selected' in old
    and all(len(new) >= len(before) and new[: len(before)] == before for new, before in zip(keys, old['keys'], strict=True))
  ):
    return dict(old, keys=keys)
  return {'token': token, 'keys': keys, 'examined': {}, 'selected': None}


def _directions(rows):
  for positive in (True, False):
    selected = [(slot, value) for slot, value in rows if (value > 0 if positive else value < 0)]
    if len(selected) < 80:
      return False
    if sum(sum(slot == index for slot, _ in selected) >= 20 for index in range(8)) < 2:
      return False
  return True


def _command_quality(groups):
  from openpilot.starpilot.flm.plant_lag import MIN_EXCITATION

  train = tuple((slot, value) for slot, values in enumerate(groups[:4]) for value in values)
  heldout = tuple((slot + 4, value) for slot, values in enumerate(groups[4:]) for value in values)
  values = tuple(value for _, value in train)
  mean = sum(values) / len(values)
  variance = sum((value - mean) ** 2 for value in values) / len(values)
  if variance < MIN_EXCITATION**2 or not _directions(train) or not _directions(heldout):
    return None
  low, high = min(values), max(values)
  # Scheduling is availability only. Match the response worker's supported
  # TRAINING domain instead of requiring every future input to lie inside it.
  # Outside inputs neither widen that range nor count as supported evidence;
  # the child still validates ALL original heldout errors and native/source data.
  supported = tuple((slot, value) for slot, value in heldout if low <= value <= high)
  if any(sum(slot == index for slot, _ in supported) < 40 for index in range(4, 8)) or not _directions(supported):
    return None
  return {'training_command_std': math.sqrt(variance), 'command_low': low, 'command_high': high}


def source_complete_groups(applies, commands, groups):
  """Input-only eligibility shared by parent scheduling and independent child.

  Keep the fixed eight slot boundaries. An unavailable command excludes every
  pose whose FULL all-lag lookback includes it. No response value is read; no
  command is interpolated, restamped or made known by omitting it from a tape.
  """
  from openpilot.starpilot.flm.plant_lag import LAGS_NS, JOIN_NS

  gap_prefix, invalid_prefix = [0], [0]
  for index, stamp in enumerate(applies):
    gap_prefix.append(gap_prefix[-1] + int(index > 0 and stamp - applies[index - 1] > JOIN_NS))
    invalid_prefix.append(invalid_prefix[-1] + int(commands[index] is None))
  previous = [0] * len(LAGS_NS)
  accepted = []
  for group in groups:
    rows = []
    for row in group:
      first = bisect_right(applies, row.source_lower_ns - LAGS_NS[-1]) - 1
      last = bisect_right(applies, row.source_upper_ns) - 1
      if (
        first < 0
        or last < first
        or row.source_lower_ns - LAGS_NS[-1] - applies[first] > JOIN_NS
        or gap_prefix[last + 1] - gap_prefix[first + 1]
        or invalid_prefix[last + 1] - invalid_prefix[first]
      ):
        continue
      endpoints = []
      for index, lag_ns in enumerate(LAGS_NS):
        lower, upper = row.source_lower_ns - lag_ns, row.source_upper_ns - lag_ns
        left, right = bisect_right(applies, lower) - 1, bisect_right(applies, upper) - 1
        if left < 0 or left != right or upper - applies[left] > JOIN_NS or applies[left] <= previous[index]:
          break
        endpoints.append(applies[left])
      if len(endpoints) == len(LAGS_NS):
        rows.append(row)
        previous = endpoints
    accepted.append(tuple(rows))
  return tuple(accepted)


def _common_command_source(applies, commands, groups):
  filtered = source_complete_groups(applies, commands, groups)
  return all(len(before) == len(after) for before, after in zip(groups, filtered, strict=True))


def quality_for_start(start_ns, *, frames, poses, emissions, controls, binding, now_ns, diagnostics=None, source_complete=False):
  """Fresh literal quality of eight fixed slots; no response values inspected."""
  from openpilot.starpilot.flm.controller_replay import ReplayFrame
  from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
  from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current
  from openpilot.starpilot.flm.runtime_adapter_registry import digest
  from openpilot.starpilot.flm.torque_emission_replay import Emission
  from openpilot.starpilot.flm.source_plant_fit import FIRST_SLOT_NS, BLOCK_NS, PURGE_NS
  from openpilot.starpilot.flm.plant_lag import LAGS_NS, JOIN_NS
  from openpilot.starpilot.flm.live_tracking_response import physical_calibration_generation
  from opendbc.car.steering_projection import TorqueProjection, normalized_applied_command

  end_ns = start_ns + FIRST_SLOT_NS + 7 * (BLOCK_NS + PURGE_NS) + BLOCK_NS
  if type(start_ns) is not int or start_ns <= 0 or end_ns > now_ns:
    return _decline(diagnostics, 'fixed_cohort_clock_incomplete')
  if (
    type(frames) is not tuple
    or type(poses) is not tuple
    or type(emissions) is not tuple
    or len(frames) > 12000
    or len(poses) > 5120
    or len(emissions) > 12000
    or any(type(row) is not ReplayFrame for row in frames)
    or any(type(row) is not OriginalSourcePose for row in poses)
    or any(type(row) is not Emission for row in emissions)
    or type(controls) is not tuple
    or len(controls) > 12000
    or any(type(row) is not ControlEvidence for row in controls)
  ):
    raise ValueError('exact bounded original cohort source required')
  stamps = tuple(frame.control_ns for frame in frames)
  first, crossing = bisect_left(stamps, start_ns), bisect_left(stamps, end_ns)
  if first == len(frames) or stamps[first] != start_ns or crossing == len(frames):
    return _decline(diagnostics, 'fixed_cohort_frame_coverage_unavailable')
  selected_frames = frames[first : crossing + 1]
  if any(frame.healthy is not True or frame.manual is not False for frame in selected_frames) or any(
    not 0 < right.control_ns - left.control_ns <= 50_000_000 or right.frame_sequence != left.frame_sequence + 1
    for left, right in zip(selected_frames, selected_frames[1:], strict=False)
  ):
    return _decline(diagnostics, 'fixed_cohort_frame_health_or_contiguity_unavailable')
  by_stamp = {row.control_ns: row for row in controls}
  if len(by_stamp) != len(controls) or any(frame.control_ns not in by_stamp for frame in selected_frames):
    return _decline(diagnostics, 'fixed_cohort_original_control_coverage_unavailable')
  selected_controls = tuple(by_stamp[frame.control_ns] for frame in selected_frames)
  if any(
    not row.enabled
    or not row.active
    or not row.valid
    or row.manual
    or row.override
    or row.fault
    or row.native_required is not binding.native_required
    or (row.identity, row.source_epoch, row.baseline_revision, row.controller_revision, row.native_identity)
    != (binding.identity, binding.source_epoch, binding.baseline_revision, binding.controller_revision, binding.native_identity)
    or row.native_required
    and (not row.native_allowed or not 0 < row.native_observed_ns <= row.control_ns <= row.native_expires_ns)
    for row in selected_controls
  ):
    return _decline(diagnostics, 'fixed_cohort_control_source_or_native_unqualified')
  if not source_complete and any(
    row.saturated
    or row.model_wobble
    or row.controller_chatter
    or row.limited
    and not (row.stage_schema_revision == 'torque-selected-stage-snapshot-v1' and dict(row.replay_inputs).get('curvature_limited') == 0.0)
    for row in selected_controls
  ):
    # Preserve the conservative legacy controller-row selector. Only the
    # explicit versioned PHYSICAL input policy below may retain a delivered
    # tagged input; its independent native/CAN verifier remains mandatory.
    return _decline(diagnostics, 'fixed_cohort_controller_row_quality_unqualified')
  anchors = tuple(dict.fromkeys(row.vehicle_model_anchor for row in selected_controls))
  if any(type(anchor) is not tuple or len(anchor) != 3 or digest(anchor) != binding.vehicle_model_generation for anchor in anchors):
    return _decline(diagnostics, 'fixed_cohort_vehicle_model_anchor_unavailable')
  selected = []
  calibration = physical_calibration_generation(binding)
  context = None
  for index in range(8):
    lower = start_ns + FIRST_SLOT_NS + index * (BLOCK_NS + PURGE_NS)
    upper = lower + BLOCK_NS
    raw = tuple(row for row in poses if lower <= row.mono_ns < upper)
    if any(
      any(type(value) is not int for value in (row.source_lower_ns, row.source_upper_ns, row.completed_ns))
      or not 0 < row.source_lower_ns <= row.mono_ns <= row.source_upper_ns <= row.completed_ns
      for row in raw
    ):
      return _decline(diagnostics, 'fixed_slot_pose_source_clock_unavailable')
    group = tuple(row for row in raw if lower <= row.source_lower_ns <= row.source_upper_ns < upper)
    if not 40 <= len(group) <= 240:
      return _decline(diagnostics, 'fixed_slot_pose_count_unavailable')
    for row in group:
      # These are source/context/physical QUALITY conditions, not residuals.
      if (
        row.valid is not True
        or row.native_allowed is not True
        or row.native_required is not binding.native_required
        or (row.identity, row.source_epoch, row.baseline_revision, row.controller_revision, row.calibration, row.native_identity)
        != (binding.identity, binding.source_epoch, binding.baseline_revision, binding.controller_revision, calibration, binding.native_identity)
        or type(row.speed_mps) not in (float, int)
        or not math.isfinite(row.speed_mps)
        or row.speed_mps <= 0
        or not 0 < row.source_lower_ns <= row.mono_ns <= row.source_upper_ns <= row.completed_ns <= now_ns
      ):
        return _decline(diagnostics, 'fixed_slot_pose_source_or_native_unqualified')
      if context is None:
        context = row.context
      if row.context != context:
        return _decline(diagnostics, 'fixed_slot_pose_context_changed')
    selected.append(group)
  events = tuple(event for event in emissions if start_ns <= event.control_ns <= selected_frames[-1].control_ns)
  applies = tuple(event.apply_ns for event in events)
  if not events or any(right <= left for left, right in zip(applies, applies[1:], strict=False)):
    return _decline(diagnostics, 'fixed_cohort_original_emission_order_unavailable')
  commands = []
  for event in events:
    if (
      event.valid is not True
      or event.active is not True
      or event.override is not False
      or event.fault is not False
      or event.native_allowed is not True
      or event.native_required is not binding.native_required
      or (event.identity, event.source_epoch, event.baseline_revision, event.native_identity)
      != (binding.identity, binding.source_epoch, binding.baseline_revision, binding.native_identity)
      or type(event.witness) is not TorqueProjection
      or not 0 < event.state_ns <= event.control_ns <= event.apply_ns <= now_ns
      or event.native_required
      and not 0 < event.native_observed_ns <= event.control_ns <= event.apply_ns <= event.native_expires_ns
    ):
      commands.append(None)
      continue
    control = by_stamp.get(event.control_ns)
    # This immutable view copies the ACTUAL emitted source/codec fields only;
    # it supplies no native permission. The child still decodes actual CAN.
    output = AppliedEvidence(
      event.control_ns,
      event.state_ns,
      event.apply_ns,
      'normalized_torque',
      event.witness.reported_torque,
      event.valid,
      event.active,
      event.override,
      event.fault,
      event.limited,
      event.native_allowed,
      projection=event.witness,
    )
    # This selector schedules PHYSICAL calibration only. Original saturation,
    # chatter and limiting remain in the full immutable packet. They do not
    # erase a known emitted input. The independent physical-input owner still
    # verifies its exact CAN/limiter result, source identity and native leases.
    # Current witnesses, trial eligibility and application remain unchanged.
    if control is None or not consumed_sources_current(control, output):
      commands.append(None)
      continue
    value = normalized_applied_command(event.witness)
    commands.append(-cast(float, value) if type(value) in (float, int) and math.isfinite(cast(float, value)) else None)
  source_support = {}
  if source_complete:
    selected = source_complete_groups(applies, commands, selected)
    if any(not 40 <= len(group) <= 240 for group in selected):
      return _decline(diagnostics, 'fixed_slots_source_complete_pose_count_unavailable')
    source_support = {'source_pose_ns': tuple(tuple(row.mono_ns for row in group) for group in selected)}
  if not _common_command_source(applies, commands, selected):
    return _decline(diagnostics, 'fixed_cohort_common_command_source_unavailable')
  # This is NOT lag identification: only whether ANY declared lag has adequate
  # original command support. No response/correlation/error chooses this lag.
  for lag_ns in LAGS_NS:
    groups = []
    for group in selected:
      values = []
      for row in group:
        lower, upper = row.source_lower_ns - lag_ns, row.source_upper_ns - lag_ns
        left, right = bisect_right(applies, lower) - 1, bisect_right(applies, upper) - 1
        if left < 0 or left != right or upper - applies[left] > JOIN_NS or commands[left] is None:
          break
        values.append(commands[left])
      if len(values) != len(group):
        break
      groups.append(tuple(values))
    if len(groups) != 8:
      continue
    summary = _command_quality(groups)
    if summary is not None:
      return dict(
        summary, **source_support, start_ns=start_ns, end_ns=end_ns, pose_counts=tuple(len(group) for group in selected), command_support_lag_ns=lag_ns
      )
  return _decline(diagnostics, 'fixed_slots_command_support_or_excitation_unavailable')


def select_first_cohort(
  *,
  checkpoints,
  frames,
  poses,
  emissions,
  controls,
  binding,
  epoch,
  epoch_start_ns,
  cutoff_ns,
  now_ns,
  cache=None,
  diagnostics=None,
  unavailable_anchors=(),
  source_complete=False,
):
  """Actual checkpoint anchors are declared independently of response outcomes."""
  from openpilot.starpilot.flm.controller_replay import Snapshot

  if (
    type(checkpoints) is not tuple
    or len(checkpoints) > MAX_CHECKPOINTS
    or not poses
    or type(epoch) is not int
    or epoch < 0
    or not 0 < epoch_start_ns < cutoff_ns
    or cutoff_ns - epoch_start_ns > 100_000_000_000
  ):
    raise ValueError('bounded original source epoch required')
  binding.validate()
  if any(type(row) is not tuple or len(row) != 2 or type(row[0]) is not int or type(row[1]) is not Snapshot for row in checkpoints):
    raise ValueError('genuine captured checkpoint anchors required')
  anchors = tuple(stamp for stamp, _ in checkpoints)
  if any(right <= left for left, right in zip(anchors, anchors[1:], strict=False)):
    raise ValueError('original checkpoint order')
  preceding = tuple(stamp for stamp in anchors if stamp <= poses[0].mono_ns)
  if not preceding:
    return _decline(diagnostics, 'original_checkpoint_before_first_pose_unavailable')
  origin = preceding[-1]
  if cache is None:
    cache = {'examined': {}, 'selected': None}
  if cache['selected'] is not None:
    if type(diagnostics) is dict:
      diagnostics['reason'] = 'cached_source_cohort_selected_not_child_qualified'
    return dict(cache['selected'])
  from openpilot.starpilot.flm.source_plant_fit import FIRST_SLOT_NS, BLOCK_NS, PURGE_NS

  duration_ns = FIRST_SLOT_NS + 7 * (BLOCK_NS + PURGE_NS) + BLOCK_NS
  if type(diagnostics) is dict:
    diagnostics.update(
      {
        'reason': 'fixed_cohort_still_collecting_or_previously_declined',
        'requiredCohortNs': duration_ns,
        'originCheckpointNs': origin,
        'examinedAnchors': len(cast(dict[int, bool], cache['examined'])),
      }
    )
  for start_ns in anchors:
    if start_ns < origin or start_ns + duration_ns >= cutoff_ns:
      continue
    if (
      start_ns + duration_ns > now_ns
      or not frames
      or not poses
      or frames[-1].control_ns < start_ns + duration_ns
      or poses[-1].source_lower_ns < start_ns + duration_ns
      or not emissions
      or emissions[-1].control_ns < start_ns + duration_ns
      or not controls
      or controls[-1].control_ns < start_ns + duration_ns
      or start_ns in cast(dict[int, bool], cache['examined'])
    ):
      continue
    if start_ns in unavailable_anchors:
      _decline(diagnostics, 'fixed_cohort_feedback_source_incomplete')
      continue
    if type(diagnostics) is dict:
      diagnostics['lastExaminedAnchorNs'] = start_ns
    result = quality_for_start(
      start_ns,
      frames=frames,
      poses=poses,
      emissions=emissions,
      controls=controls,
      binding=binding,
      now_ns=now_ns,
      diagnostics=diagnostics,
      source_complete=source_complete,
    )
    cast(dict[int, bool], cache['examined'])[start_ns] = result is not None
    if type(diagnostics) is dict:
      diagnostics['examinedAnchors'] = len(cast(dict[int, bool], cache['examined']))
    if result is not None and result['end_ns'] < cutoff_ns:
      selected = dict(result, version=VERSION, epoch=epoch, epoch_start_ns=epoch_start_ns, cutoff_ns=cutoff_ns, origin_checkpoint_ns=origin)
      cache['selected'] = selected
      if type(diagnostics) is dict:
        diagnostics['reason'] = 'source_cohort_selected_not_child_qualified'
      return dict(selected)
  return None


def validate_selected_cohort(selection, packet, engine, binding, now_ns, *, source_complete=False):
  """Fresh child recomputation of selected command support; no parent trust."""
  from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose

  if (
    type(selection) is not dict
    or selection.get('version') != VERSION
    or not engine.records
    or selection.get('epoch') != packet['capture_epoch']
    or selection.get('epoch_start_ns') != packet['capture_epoch_start_ns']
    or selection.get('cutoff_ns') != packet['capture_epoch_cutoff_ns']
    or selection.get('start_ns') != engine.records[0].control_ns
    or type(selection.get('origin_checkpoint_ns')) is not int
    or not 0 < selection['origin_checkpoint_ns'] <= selection['start_ns'] < selection['end_ns'] < selection['cutoff_ns']
    or not 0 <= selection['epoch_start_ns'] - selection['origin_checkpoint_ns'] <= 1_100_000_000
    or selection['start_ns'] not in dict(engine.checkpoints)
  ):
    raise ValueError('explicit original checkpoint cohort selection provenance')
  poses = tuple(OriginalSourcePose(**row) for row in packet['source_poses'])
  quality = quality_for_start(
    selection['start_ns'],
    frames=tuple(engine.records),
    poses=poses,
    emissions=tuple(engine.emissions.events.values()),
    controls=tuple(engine.original_context_controls.values()),
    binding=binding,
    now_ns=now_ns,
    source_complete=source_complete,
  )
  if (
    quality is None
    or set(selection) != set(quality) | {'version', 'epoch', 'epoch_start_ns', 'cutoff_ns', 'origin_checkpoint_ns'}
    or any(selection[name] != value for name, value in quality.items())
  ):
    raise ValueError('current independently verified cohort command/source quality changed')
  return selection
