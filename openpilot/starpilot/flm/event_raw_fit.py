from openpilot.starpilot.flm.normalized_trace_archive import archive_of
"""Actual child path: whole events -> measured plant -> tiny canonical effect trial."""
from openpilot.starpilot.flm.clipping_quality import event_clipping_quality
from dataclasses import asdict
import math

from openpilot.starpilot.flm.adaptive_profile import Block, Sample
from openpilot.starpilot.flm.event_episodes import tracking_episodes
from openpilot.starpilot.flm.event_evaluate import evaluate_event
from openpilot.starpilot.flm.event_physical import fit_event_physical
from openpilot.starpilot.flm.event_windows import regroup_whole_events
from openpilot.starpilot.flm.physical_response import PhysicalRow
from openpilot.starpilot.flm.runtime_adapter_registry import canonical_physical_context, physical_model_revision
from openpilot.starpilot.flm.torque_eligibility import exact_unclipped_pair, consumed_sources_current
from openpilot.starpilot.flm.torque_emission_replay import delivered
from openpilot.starpilot.flm.torque_raw_fit import control_from_dict, output_from_dict


def _unavailable(reason, **details):
  return {'status': 'collecting', 'reason': reason, 'profile': None, 'validation': None, **details}


def objective_excited(event, engine, objective):
  """Predeclared whole-episode support, independent of targets or fit gains.

  An opposite-direction episode with zero actual source effect cannot be
  mislabeled as a left/right objective cohort. Every original frame remains
  in counterfactual controller/native replay; only stage estimation differs.
  """
  count = len(objective.parameters)
  rows = [engine.projected[point['apply_ns']][1] for point in event['samples']]
  if not rows or any(len(row) != count or any(not math.isfinite(value) for value in row) for row in rows):
    raise ValueError('whole episode actual encoded objective basis')
  return all(sum(row[index] ** 2 for row in rows) / len(rows) >= parameter.excitation_floor ** 2
             for index, parameter in enumerate(objective.parameters))


def original_event_evidence(payload, engine, objective):
  from openpilot.starpilot.flm.full_surface_profile import envelope_from_dict

  envelope = envelope_from_dict(payload['effect_envelope'])
  if (envelope.trial_names != objective.names or envelope.trial_anchor != objective.anchor_digest or
      envelope.context_key != objective.context_key or envelope.selected_baseline_revision != engine.baseline_revision):
    raise ValueError('event full original anchor/context')
  context = canonical_physical_context(payload['physical_context'])
  lag = payload['physical_context'][-1]
  packets = payload.get('raw_blocks', payload.get('blocks'))
  if not 8 <= len(packets) <= 24:
    raise ValueError('event storage packet bound')
  retain_chatter = objective.retains_controller_chatter
  previous = 0
  seen = set()
  # No storage ID, saved normalization or parent's event count is accepted as
  # proof. Recompute every original source/native/actual consumed endpoint.
  for packet in packets:
    if packet['identity'] != engine.identity or not 40 <= len(packet['samples']) <= 240:
      raise ValueError('event packet provenance/bound')
    for point in packet['samples']:
      command, output = control_from_dict(point['control'],trace_archive=archive_of(engine)), output_from_dict(point['output'])
      key = point['control_ns'], point['apply_ns'], point['mono_ns']
      if (key in seen or point['mono_ns'] <= previous or command.identity != engine.identity or
          command.baseline_revision != engine.baseline_revision or not command.enabled or command.manual or
          not command.active or not command.valid or command.override or command.fault or command.saturated or
          command.limited and not exact_unclipped_pair(command, output) or command.model_wobble or
          command.controller_chatter and not retain_chatter or any(command.applied_coordinates) or
          not output.valid or not output.active or output.override or output.fault or output.limited or not output.native_allowed or
          not consumed_sources_current(command, output) or
          output.control_ns != command.control_ns or point['control_ns'] != command.control_ns or point['apply_ns'] != output.apply_ns or
          canonical_physical_context(point['physical_context']) != context or
          not engine.applied_matches(command, output) or delivered(output) != point['physical']['applied_command']):
        raise ValueError('event exact consumed physical endpoint')
      seen.add(key)
      previous = point['mono_ns']
  events = regroup_whole_events(engine, packets, objective)
  return envelope, context, lag, packets, events


def prepare_fixed_event_calibration(payload, engine, objective):
  """Version3: exactly eight ORIGINAL events, never retry a later cutoff."""
  from openpilot.starpilot.flm.event_activation import CALIBRATION_EVENTS
  envelope, context, lag, packets, events = original_event_evidence(payload, engine, objective)
  if len(events) < CALIBRATION_EVENTS + 8:
    return _unavailable('whole_physical_event_support', wholeBlocks=len(events), requiredWholeBlocks=16)
  rows = tuple(PhysicalRow(event['block_id'], point['apply_ns'], point['mono_ns'],
    -delivered(output_from_dict(point['output'])), point['physical']['lateral_accel'], context,
    'left' if -delivered(output_from_dict(point['output'])) > 0 else 'right', True, False, False)
    for event in events[:CALIBRATION_EVENTS] for point in event['samples'])
  cutoff = max(row.response_ns for row in rows)
  model = fit_event_physical(engine.identity, context, rows, lag_ns=lag, now_ns=cutoff)
  if model is None:
    return _unavailable('fixed_eight_independent_physical_model', wholeBlocks=len(events), modelPoints=len(rows))
  stage = tuple(event for event in events[CALIBRATION_EVENTS:] if
    min(point['control_ns'] for point in event['samples']) > cutoff and
    all(min(point['control_ns'], point['apply_ns'], point['mono_ns']) > cutoff for point in event['samples']))
  if len(stage) < 8:
    return _unavailable('event_disjoint_stage_support', stageBlocks=len(stage))
  return model, cutoff, events, stage


def _control_frame_index(records):
  """Index already validated original frames without changing their identity."""
  frames = {}
  for frame in records:
    stamp = frame.control_ns
    if type(stamp) is not int or stamp <= 0 or stamp in frames:
      raise ValueError('event duplicate/invalid original control frame')
    frames[stamp] = frame
  return frames


def fit_event_torque(payload, engine, objective, *, source_plant=None):
  frames = _control_frame_index(engine.records)
  if source_plant is not None:
    from openpilot.starpilot.flm.source_plant_fit import PROTOCOL, PreparedPlant
    from openpilot.starpilot.flm.full_surface_profile import envelope_from_dict
    from openpilot.starpilot.flm.event_activation import bind_activation_model
    plant, events = source_plant
    declared_protocol = PROTOCOL
    if payload.get('event_version') == 7:
      from openpilot.starpilot.flm.feedback_replay import PROTOCOL as FEEDBACK_PROTOCOL
      declared_protocol = FEEDBACK_PROTOCOL
    if (not isinstance(plant, PreparedPlant) or payload.get('plant_protocol') != declared_protocol or
        tuple(payload['physical_context']) != plant.context or engine.activation_plan is None or
        payload.get('event_version') not in (3,7) or payload.get('event_operation') != 'trial'):
      raise ValueError('explicit child-owned independent source plant')
    envelope = envelope_from_dict(payload['effect_envelope'])
    context, lag, packets = plant.model.context, plant.model.lag_ns, events
    cutoff, split = plant.identification.cutoff_ns, 0
    model = bind_activation_model(plant.model, engine.activation_plan)
  else:
    if payload.get('event_version') == 7:
      raise ValueError('new event mean requires explicit original feedback source plant')
    envelope, context, lag, packets, events = original_event_evidence(payload, engine, objective)
    if len(events) < 16:
      return _unavailable('whole_physical_event_support', storagePackets=len(packets), wholeBlocks=len(events), requiredWholeBlocks=16)
    if payload.get('event_version') == 3:
      from openpilot.starpilot.flm.event_activation import bind_activation_model, build_activation_plan
      prepared = prepare_fixed_event_calibration(payload, engine, objective)
      if type(prepared) is dict:
        return prepared
      original_model, cutoff, events, declared_stage = prepared
      plan = build_activation_plan(engine, model=original_model, cutoff_ns=cutoff, stage_events=declared_stage, physical_context=context,
        quality_contract=event_clipping_quality(payload))
      if engine.activation_plan != plan:
        raise ValueError('event candidate changed frozen activation experiment')
      model, split = bind_activation_model(original_model, plan), 8
    else:
      # Chronologically earlier measured episodes calibrate the response. At least
      # eight later whole-event bundles remain for disjoint stage train/heldout.
      model = None
      rows = []
      cutoff = split = 0
      for index, event in enumerate(events[:-8]):
        for point in event['samples']:
          output = output_from_dict(point['output'])
          command = -delivered(output)
          rows.append(PhysicalRow(event['block_id'], output.apply_ns, point['mono_ns'], command,
                                  point['physical']['lateral_accel'], context,
                                  'left' if command > 0 else 'right', True, False, False))
        if index >= 7:
          model = fit_event_physical(engine.identity, context, tuple(rows), lag_ns=lag, now_ns=event['samples'][-1]['mono_ns'])
          if model is not None:
            cutoff = max(row.response_ns for row in rows)
            split = index + 1
            break
      if model is None:
        return _unavailable('event_independent_physical_model', wholeBlocks=len(events), modelPoints=len(rows))
  if payload.get('event_version') == 7:
    from openpilot.starpilot.flm.event_mean_evidence import VERSION
    from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
    if payload.get('event_mean_version') != VERSION:
      raise ValueError('explicit new event mean version')
    require_feedback_engine(engine)
  elif 'event_mean_version' in payload:
    raise ValueError('legacy event cannot adopt a new mean-evidence policy')
  engine.ensure_causal_features()
  blocks, stage_events = [], []
  unexcited_stage_blocks = 0
  previous = cutoff
  for event in events[split:]:
    if any(point['control_ns'] <= cutoff or point['apply_ns'] <= cutoff or point['mono_ns'] <= cutoff for point in event['samples']):
      continue
    if not objective_excited(event, engine, objective):
      unexcited_stage_blocks += 1
      continue
    samples = []
    for point in event['samples']:
      command, output = control_from_dict(point['control'],trace_archive=archive_of(engine)), output_from_dict(point['output'])
      if point['mono_ns'] <= previous:
        raise ValueError('event stage chronology')
      previous = point['mono_ns']
      frame = frames.get(command.control_ns)
      trace = engine.projected.get(output.apply_ns)
      if frame is None or frame.desired_physical_accel is None or trace is None or model.inverse(point['physical']['lateral_accel']) is None:
        raise ValueError('event stage supported actual physical response')
      target = model.inverse(frame.desired_physical_accel)
      if target is None:
        return _unavailable('event_target_outside_measured_support')
      reference = tuple(dict(envelope.prior_coordinates)[name] for name in objective.names)
      absolute_target = -target - trace[0] + sum(a * b for a, b in zip(trace[1], reference, strict=True))
      samples.append(Sample(point['mono_ns'], command.control_ns, output.apply_ns, trace[1], absolute_target, 1.0))
    blocks.append(Block(engine.identity, event['block_id'], objective.required_cohorts[0], 'torque-selected-effect-v2',
                        'event-independent-physical-v2', tuple(samples)))
    stage_events.append(event)
  if len(blocks) < 8:
    return _unavailable('event_disjoint_stage_support', wholeBlocks=len(events), stageBlocks=len(blocks), unexcitedStageBlocks=unexcited_stage_blocks)
  event_mean_model = None
  if payload.get('event_version') == 7:
    from openpilot.starpilot.flm.event_mean_evidence import build_model
    event_mean_model = build_model(engine,tuple(stage_events),tuple(blocks))
    engine.event_mean_model = event_mean_model
  episodes = tracking_episodes(engine, tuple(stage_events), model, objective)
  prior = None if payload.get('prior') is None else tuple(payload['prior'])
  from openpilot.starpilot.flm.event_activation import ZERO_REFERENCE_ACTIVATION_VERSION
  native_feasible = (engine.native_feasible if engine.activation_plan is not None and
    engine.activation_plan.version == ZERO_REFERENCE_ACTIVATION_VERSION else None)
  result = evaluate_event(engine.identity, objective.parameters, tuple(blocks), episodes=episodes,
                          now_ns=payload['now_ns'], required_cohorts=objective.required_cohorts,
                          response_revision='event-independent-physical-v2', sequence=payload['sequence'], prior=prior,
                          effect_envelope=envelope, native_feasible=native_feasible,
                          **({'event_mean_model':event_mean_model} if event_mean_model is not None else {}))
  mean_basis = {}
  if event_mean_model is not None:
    from openpilot.starpilot.flm.event_mean_evidence import report_basis
    mean_basis = {'event_mean_basis':report_basis(event_mean_model)}
  validation = None
  if result.profile is not None:
    current, applied = control_from_dict(payload['control'],trace_archive=archive_of(engine)), output_from_dict(payload['output'])
    validation = engine.validate(result.profile, current, applied, tuple(payload['physical_context']), tuple(blocks), model)
    if validation is None or validation.physical_model_revision != physical_model_revision(model):
      return {'status':'rejected','reason':'event_actual_controller_replay','profile':None,'validation':None,
        **mean_basis}
  return {'status': result.status, 'reason': result.reason,
          **mean_basis,
          'profile': None if result.profile is None else asdict(result.profile),
          'validation': None if validation is None else asdict(validation), 'model': asdict(model),
          'model_cutoff_ns': cutoff, 'objective_key': payload.get('objective_key'),
          **({'event_version': payload['event_version'], 'activation_plan': asdict(engine.activation_plan)} if payload.get('event_version') in (3,7) else {}),
          'event_counts': {'large': sum(event['kind'] == 'large' for event in episodes),
                           'subtle': sum(event['kind'] == 'subtle' for event in episodes)},
          'whole_blocks': len(events), 'stage_blocks': len(blocks), 'unexcited_stage_blocks': unexcited_stage_blocks,
          'speed_mps_bounds': (min(dict(point['control']['stage_values'])['speed'] for event in events for point in event['samples']),
                               max(dict(point['control']['stage_values'])['speed'] for event in events for point in event['samples']))}
