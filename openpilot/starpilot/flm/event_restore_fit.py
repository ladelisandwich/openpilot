from openpilot.starpilot.flm.normalized_trace_archive import archive_of
"""Fresh whole-map measured/controller proof for an explicitly retained prior.

Restoration does not estimate coefficients or pass a thirty-dimensional map as
four fitted markers. The exact historical map is a fixed counterfactual; every
active canonical group needs fresh joint effect support and heldout benefit.
"""
from dataclasses import asdict
import math

from openpilot.starpilot.flm.adaptive_profile import Block, Profile, Sample
from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
from openpilot.starpilot.flm.event_evaluate import _lower
from openpilot.starpilot.flm.event_physical import fit_event_physical
from openpilot.starpilot.flm.event_raw_fit import _control_frame_index
from openpilot.starpilot.flm.event_windows import regroup_whole_events
from openpilot.starpilot.flm.full_surface_profile import RESTORE_POLICY, envelope_from_dict
from openpilot.starpilot.flm.physical_response import PhysicalRow
from openpilot.starpilot.flm.runtime_adapter_registry import canonical_physical_context, physical_model_revision
from openpilot.starpilot.flm.torque_eligibility import exact_unclipped_pair, consumed_sources_current
from openpilot.starpilot.flm.torque_emission_replay import delivered
from openpilot.starpilot.flm.torque_raw_fit import control_from_dict, output_from_dict


def unavailable(reason, **details):
  return {'status': 'collecting', 'reason': reason, 'profile': None, 'validation': None, **details}


def _restore_original_evidence(payload, engine, objective):
  envelope = envelope_from_dict(payload['effect_envelope'])
  if envelope.operation != 'restore' or envelope.version != 2 or not envelope.restoration_groups:
    raise ValueError('explicit nonneutral historical restore transaction')
  envelope.validate()
  context = canonical_physical_context(payload['physical_context'])
  packets = payload['raw_blocks']
  if not 8 <= len(packets) <= 24:
    raise ValueError('restore raw whole-event storage bound')
  previous, seen = 0, set()
  for packet in packets:
    if packet['identity'] != engine.identity or not 40 <= len(packet['samples']) <= 240:
      raise ValueError('restore endpoint packet identity')
    for point in packet['samples']:
      command, output = control_from_dict(point['control'],trace_archive=archive_of(engine)), output_from_dict(point['output'])
      key = point['control_ns'], point['apply_ns'], point['mono_ns']
      if (key in seen or point['mono_ns'] <= previous or command.identity != engine.identity or
          command.baseline_revision != engine.baseline_revision or command.source_epoch != envelope.source_epoch or
          not 0 < command.source_receipt_ns <= command.control_ns <= command.source_receipt_ns + 500_000_000 or
          not 0 < command.state_ns <= command.control_ns <= command.state_ns + 30_000_000 or
          not consumed_sources_current(command, output) or
          not command.control_ns <= output.apply_ns <= command.control_ns + 100_000_000 or
          command.native_required and (not command.native_allowed or not command.native_identity or
            not 0 < command.native_observed_ns <= command.control_ns <= output.apply_ns <= command.native_expires_ns or
            command.native_expires_ns - command.native_observed_ns > 200_000_000) or
          not command.enabled or command.manual or not command.active or not command.valid or command.override or command.fault or
          command.saturated or command.model_wobble or command.controller_chatter or any(command.applied_coordinates) or
          command.limited and not exact_unclipped_pair(command, output) or not output.valid or not output.active or
          output.override or output.fault or output.limited or not output.native_allowed or
          output.control_ns != command.control_ns or point['control_ns'] != command.control_ns or point['apply_ns'] != output.apply_ns or
          canonical_physical_context(point['physical_context']) != context or not engine.applied_matches(command, output) or
          delivered(output) != point['physical']['applied_command']):
        raise ValueError('restore exact original source/native/consumed endpoint')
      seen.add(key)
      previous = point['mono_ns']
  events = regroup_whole_events(engine, packets, objective)
  return envelope, context, packets, events


def prepare_fixed_restore_calibration(payload, engine, objective):
  from openpilot.starpilot.flm.clipping_quality import event_clipping_quality
  if type(payload.get('event_version')) is not int or payload['event_version'] not in (4, 5, 6):
    raise ValueError('explicit fixed-map restore version4/5/6')
  event_clipping_quality(payload)
  envelope, context, packets, events = _restore_original_evidence(payload, engine, objective)
  _control_frame_index(engine.records)  # reject ambiguous original activation clocks
  if len(events) < 16:
    return unavailable('restore_whole_physical_event_support', wholeBlocks=len(events), requiredWholeBlocks=16)
  rows = tuple(PhysicalRow(event['block_id'], point['apply_ns'], point['mono_ns'],
    -delivered(output_from_dict(point['output'])), point['physical']['lateral_accel'], context,
    'left' if -delivered(output_from_dict(point['output'])) > 0 else 'right', True, False, False)
    for event in events[:8] for point in event['samples'])
  cutoff = max(row.response_ns for row in rows)
  model = fit_event_physical(engine.identity, context, rows, lag_ns=payload['physical_context'][-1], now_ns=cutoff)
  if model is None:
    return unavailable('restore_fixed_eight_independent_physical_model', wholeBlocks=len(events), modelPoints=len(rows))
  stage = tuple(event for event in events[8:] if
    all(min(point['control_ns'], point['apply_ns'], point['mono_ns']) > cutoff for point in event['samples']))
  if len(stage) < 8:
    return unavailable('restore_disjoint_stage_support', stageBlocks=len(stage))
  return model, cutoff, events, stage


def _fixed_restore_rollout(payload, engine, coordinates, *, restore_target=False, restore_omit_group=None):
  # Only v4 turns expected strict-native refusal into a truthful report. v2's
  # replay contract and propagation stay unchanged. No native bypass/token.
  try:
    return engine._rollout(coordinates, restore_target=restore_target, restore_omit_group=restore_omit_group)
  except ValueError as error:
    if payload.get('event_version') not in (4, 5, 6, 8):
      raise
    return unavailable('restore_actual_full_map_controller_replay', status='rejected', event_version=payload['event_version'],
      replay_phase='group-off' if restore_omit_group is not None else 'full-map' if restore_target else 'baseline',
      replay_reason=str(error), restore_activation_plan=asdict(engine.activation_plan))


def fit_event_restore(payload, engine, objective, *, source_plant=None):
  feedback_restore = payload.get('event_version') == 8
  if feedback_restore:
    from openpilot.starpilot.flm.clipping_quality import event_clipping_quality
    from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
    event_clipping_quality(payload)
    if source_plant is None or getattr(engine,'event_mean_model',None) is not None:
      raise ValueError('feedback restore requires fresh source plant and fixed conservative statistics')
    require_feedback_engine(engine)
  if source_plant is not None:
    from openpilot.starpilot.flm.source_plant_fit import PROTOCOL, PreparedPlant
    from openpilot.starpilot.flm.feedback_replay import PROTOCOL as FEEDBACK_PROTOCOL
    from openpilot.starpilot.flm.event_activation import bind_activation_model
    plant, events = source_plant
    if (not isinstance(plant, PreparedPlant) or payload.get('plant_protocol') != (FEEDBACK_PROTOCOL if feedback_restore else PROTOCOL) or
        tuple(payload['physical_context']) != plant.context or engine.activation_plan is None or
        payload.get('event_version') not in ((8,) if feedback_restore else (4,5,6)) or payload.get('event_operation') != 'restore'):
      raise ValueError('explicit child-owned independent restore plant')
    envelope = envelope_from_dict(payload['effect_envelope'])
    envelope.validate()
    context, packets, frames = plant.model.context, events, _control_frame_index(engine.records)
    cutoff, split, declared = plant.identification.cutoff_ns, 0, True
    model = bind_activation_model(plant.model, engine.activation_plan)
  else:
    envelope, context, packets, events = _restore_original_evidence(payload, engine, objective)
    frames = _control_frame_index(engine.records)
    if len(events) < 16:
      return unavailable('restore_whole_physical_event_support', wholeBlocks=len(events), requiredWholeBlocks=16)
    declared = payload.get('event_version') in (4, 5, 6)
    if declared:
      from openpilot.starpilot.flm.event_activation import bind_activation_model
      from openpilot.starpilot.flm.restore_activation import build_restore_plan
      prepared = prepare_fixed_restore_calibration(payload, engine, objective)
      if type(prepared) is dict:
        return prepared
      original_model, cutoff, events, declared_stage = prepared
      plan = build_restore_plan(engine, model=original_model, cutoff_ns=cutoff, stage_events=declared_stage,
        physical_context=context, envelope=envelope, restore_version=payload['event_version'])
      if engine.activation_plan != plan:
        raise ValueError('restore changed frozen full-map activation declaration')
      model, split = bind_activation_model(original_model, plan), 8
    else:
      model, rows, cutoff, split = None, [], 0, 0
      for index, event in enumerate(events[:-8]):
        for point in event['samples']:
          output = output_from_dict(point['output'])
          command = -delivered(output)
          rows.append(PhysicalRow(event['block_id'], output.apply_ns, point['mono_ns'], command, point['physical']['lateral_accel'],
                                  context, 'left' if command > 0 else 'right', True, False, False))
        if index >= 7:
          model = fit_event_physical(engine.identity, context, tuple(rows), lag_ns=payload['physical_context'][-1],
                                     now_ns=event['samples'][-1]['mono_ns'])
          if model is not None:
            cutoff, split = max(row.response_ns for row in rows), index + 1
            break
      if model is None:
        return unavailable('restore_independent_physical_model', wholeBlocks=len(events), modelPoints=len(rows))
  stage = tuple(event for event in events[split:] if
                all(min(point['control_ns'], point['apply_ns'], point['mono_ns']) > cutoff for point in event['samples']))
  if len(stage) < 8:
    return unavailable('restore_disjoint_stage_support', stageBlocks=len(stage))
  original = _fixed_restore_rollout(payload, engine, engine.reference)
  if type(original) is dict:
    return {**original, 'whole_blocks': len(events), 'stage_blocks': len(stage), 'model_cutoff_ns': cutoff, 'model': asdict(model)}
  baseline_requests, baseline, base_saturated = original
  target = _fixed_restore_rollout(payload, engine,
    tuple(dict(envelope.coordinates)[name] for name in objective.names), restore_target=True)
  if type(target) is dict:
    return {**target, 'whole_blocks': len(events), 'stage_blocks': len(stage), 'model_cutoff_ns': cutoff, 'model': asdict(model)}
  proposed_requests, proposed, proposed_saturated = target
  if (set(baseline) != set(proposed) or any(proposed_saturated[stamp] and not base_saturated[stamp] for stamp in baseline_requests) or
      not engine._slew_ok(baseline_requests, proposed_requests, encoded=False) or not engine._slew_ok(baseline, proposed, encoded=True)):
    return unavailable('restore_full_map_native_or_slew_regression')
  # Qualification concerns the ENTIRE proposed map. Named marker coefficients
  # only identify the typed wire envelope and do not stand in for30 fit bases.
  effects = {}
  for group in envelope.restoration_groups:
    ablation = _fixed_restore_rollout(payload, engine, tuple(dict(envelope.coordinates)[name] for name in objective.names),
      restore_target=True, restore_omit_group=GROUPS[group])
    if type(ablation) is dict:
      return {**ablation, 'whole_blocks': len(events), 'stage_blocks': len(stage), 'model_cutoff_ns': cutoff, 'model': asdict(model), 'group': group}
    _, without_group, _ = ablation
    if set(without_group) != set(proposed):
      return unavailable('restore_group_native_support', group=group)
    effects[group] = without_group
  blocks, losses, candidate_losses, group_gains = [], [], [], {group: [] for group in effects}
  engine.projected = {stamp: (value, (0.,) * len(objective.names)) for stamp, value in baseline.items()}
  for event in stage:
    samples, before_errors, after_errors = [], [], []
    signals = {group: [] for group in effects}
    group_errors = {group: [] for group in effects}
    for point in event['samples']:
      frame = frames.get(point['control_ns'])
      stamp = point['apply_ns']
      if frame is None or frame.desired_physical_accel is None or stamp not in baseline:
        raise ValueError('restore exact physical target source')
      target = model.inverse(frame.desired_physical_accel)
      if target is None or model.inverse(point['physical']['lateral_accel']) is None:
        return unavailable('restore_target_outside_measured_support')
      target = -target
      values = (target, baseline[stamp], proposed[stamp], *(trace[stamp] for trace in effects.values()))
      if not all(model.command_low <= -value <= model.command_high for value in values):
        return unavailable('restore_candidate_outside_measured_support')
      samples.append(Sample(point['mono_ns'], point['control_ns'], stamp, (0.,) * len(objective.names), target - baseline[stamp], 1.))
      before_errors.append((model.slope * (target - baseline[stamp])) ** 2)
      after_errors.append((model.slope * (target - proposed[stamp])) ** 2)
      for group, trace in effects.items():
        signals[group].append(abs(trace[stamp] - proposed[stamp]))
        group_errors[group].append((model.slope * (target - trace[stamp])) ** 2)
    blocks.append(Block(engine.identity, event['block_id'], objective.required_cohorts[0], 'torque-selected-effect-v2',
                        'event-independent-physical-v2', tuple(samples)))
    before_loss, after_loss = sum(before_errors) / len(samples), sum(after_errors) / len(samples)
    losses.append(before_loss)
    candidate_losses.append(after_loss)
    for group in effects:
      # Joint group impact must be present in this actual supported trajectory.
      # No unobservable saved low-speed/window coordinate restores on faith.
      if payload.get('event_version') in (5, 6, 8):
        group_gains[group].append(sum(group_errors[group]) / len(samples) - after_loss)
      elif sum(value > 1e-8 for value in signals[group]) < 40:
        group_gains[group].append(None)
      else:
        group_gains[group].append(sum(group_errors[group]) / len(samples) - after_loss)
  middle = len(blocks) // 2
  if payload.get('event_version') in (5, 6, 8):
    from openpilot.starpilot.flm.restore_support import capped_group_credits, independent_half_supported, all_event_lower
    for group, trace in effects.items():
      evidence = tuple((event['block_id'], tuple((point['control_ns'], point['apply_ns'],
        abs(trace[point['apply_ns']] - proposed[point['apply_ns']])) for point in event['samples'])) for event in stage)
      try:
        support_credits = capped_group_credits(evidence)
      except ValueError as error:
        return unavailable('restore_group_unique_whole_event_support', group=group, support_reason=str(error))
      for half in (support_credits[:middle], support_credits[middle:]):
        if not independent_half_supported(half):
          return unavailable('restore_group_aggregate_encoded_support', group=group,
            support_credits=half, required_credits=80, required_effectful_events=2)
  for group, gains in group_gains.items():
    for cohort in (gains[:middle], gains[middle:]):
      if payload.get('event_version') in (5, 6, 8):
        if all_event_lower(cohort, _lower) is None:
          return unavailable('restore_group_independent_heldout_support', group=group)
      else:
        valid = tuple(value for value in cohort if value is not None)
        if len(valid) < 2 or _lower(valid) <= 0:
          return unavailable('restore_group_independent_heldout_support', group=group)
  lower = _lower(tuple(a - b for a, b in zip(losses[middle:], candidate_losses[middle:], strict=True)))
  baseline_loss = sum(losses[middle:]) / len(losses[middle:])
  candidate_loss = sum(candidate_losses[middle:]) / len(candidate_losses[middle:])
  if not math.isfinite(lower) or lower <= 0 or not 0 <= candidate_loss < baseline_loss:
    return unavailable('restore_whole_map_heldout_regression_or_noise')
  profile = Profile(engine.identity, payload['sequence'], payload['now_ns'], payload['now_ns'] + 500_000_000,
                    objective.parameters, tuple(dict(envelope.coordinates)[name] for name in objective.names),
                    tuple(block.block_id for block in blocks[:middle]), tuple(block.block_id for block in blocks[middle:]),
                    baseline_loss, candidate_loss, ((objective.required_cohorts[0], lower),), RESTORE_POLICY, envelope)
  validation = engine.validate(profile, control_from_dict(payload['control'],trace_archive=archive_of(engine)), output_from_dict(payload['output']),
                               tuple(payload['physical_context']), tuple(blocks), model)
  if validation is None or validation.physical_model_revision != physical_model_revision(model):
    return unavailable('restore_actual_full_map_controller_replay')
  return {'status': 'qualified', 'reason': 'restored_history_fresh_whole_map_pending_current_probe',
          'profile': asdict(profile), 'validation': asdict(validation), 'model': asdict(model),
          **({'event_version': payload['event_version'], 'restore_activation_plan': asdict(engine.activation_plan)} if declared else {}),
          **({'event_restore_statistics': payload['event_restore_statistics']} if feedback_restore else {}),
          'model_cutoff_ns': cutoff, 'whole_blocks': len(events), 'stage_blocks': len(stage),
          'restoration_qualified_groups': tuple(effects), 'objective_key': payload.get('objective_key'),
          'speed_mps_bounds': (min(dict(point['control']['stage_values'])['speed'] for event in events for point in event['samples']),
                               max(dict(point['control']['stage_values'])['speed'] for event in events for point in event['samples']))}
