"""Tiny original-anchor effect proposals; episode counts trigger, never admit."""
from dataclasses import replace
import math

from openpilot.starpilot.flm.adaptive_profile import Result, evaluate_event_proposal, loss
from openpilot.starpilot.flm.event_controller_replay import POLICY

TRIAL_STEP = .005
NATIVE_FEASIBILITY_SCALES = (1., .5, .25, .125, .0625, .03125, .015625, .0078125)


def _lower(values):
  if len(values) < 2:
    return float('-inf')
  mean = sum(values) / len(values)
  variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
  return mean - 12.707 * math.sqrt(variance / len(values))


def evaluate_event(identity, parameters, blocks, *, episodes, now_ns, required_cohorts, response_revision,
                   sequence=1, ridge=.01, prior=None, effect_envelope=None, native_feasible=None, event_mean_model=None):
  """Actual fit boundary; caller reconstructs episodes from exact child trace.

  An episode crossing train/heldout boundaries is rejected, not counted twice.
  At least three comparable large-error episodes OR nine comparable small-error
  cycles permit an attempt. Independent measured-model, stage and full selected
  controller/CAN replay qualifications are additional mandatory caller gates.
  """
  from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope

  if not isinstance(effect_envelope, FullSurfaceEnvelope):
    return Result('unavailable', 'event_full_original_anchor')
  effect_envelope.validate()
  if (tuple(p.name for p in parameters) != tuple('effect.universal.' + name for name in effect_envelope.trial_names) or
      prior != tuple(dict(effect_envelope.prior_coordinates)[name] for name in effect_envelope.trial_names)):
    return Result('unavailable', 'event_current_full_prior')
  if (not 1 <= len(parameters) <= 4 or any(p.stage_revision != 'torque-selected-effect-v2' for p in parameters) or
      len({p.name for p in parameters}) != len(parameters)):
    return Result('unavailable', 'event_parameter_contract')
  split = len(blocks) // 2
  train_ids = {block.block_id for block in blocks[:split]}
  heldout_ids = {block.block_id for block in blocks[split:]}
  all_ids = train_ids | heldout_ids
  episode_ids = set()
  counts = {'large': 0, 'subtle': 0}
  previous = 0
  for episode in episodes:
    # The child-derived key binds original source/CP/baseline/selected group,
    # context, actual zero-cross/neutral boundaries and first/last source stamp.
    if (type(episode) is not dict or set(episode) != {'id', 'kind', 'start_ns', 'end_ns', 'block_ids'} or
        type(episode['id']) is not str or len(episode['id']) != 64 or episode['id'] in episode_ids or
        episode['kind'] not in counts or type(episode['start_ns']) is not int or type(episode['end_ns']) is not int or
        not 0 < episode['start_ns'] <= episode['end_ns'] <= now_ns or episode['start_ns'] <= previous or
        type(episode['block_ids']) is not tuple or not episode['block_ids'] or
        len(set(episode['block_ids'])) != len(episode['block_ids']) or not set(episode['block_ids']) <= all_ids or
        set(episode['block_ids']) & train_ids and set(episode['block_ids']) & heldout_ids):
      return Result('unavailable', 'event_lineage_or_split')
    episode_ids.add(episode['id'])
    previous = episode['end_ns']
    counts[episode['kind']] += 1
  if counts['large'] < 3 and counts['subtle'] < 9:
    return Result('collecting', 'comparable_tracking_episodes')
  fitted = evaluate_event_proposal(identity, parameters, blocks, trial_step=TRIAL_STEP, now_ns=now_ns,
                                   required_cohorts=required_cohorts,
                    response_revision=response_revision, sequence=sequence, ridge=ridge, prior=prior,
                    **({'event_mean_model':event_mean_model} if event_mean_model is not None else {}))
  if fitted.profile is None:
    return fitted
  reference = (0.0,) * len(parameters) if prior is None else prior
  proposal = fitted.profile.coefficients
  if proposal == reference:
    return Result('collecting', 'no_identifiable_event_trial')
  if native_feasible is not None:
    # The direction was fitted ONCE on train. Existing ridge-heldout acceptance
    # above remains a conservative preliminary gate, never a direction selector.
    # Select ONLY by original whole native/source/quality/slew feasibility; no
    # target, sample loss or heldout gain is supplied to this callback.
    delta = tuple(target - old for old, target in zip(reference, proposal, strict=True))
    for scale in NATIVE_FEASIBILITY_SCALES:
      candidate = tuple(old + scale * step for old, step in zip(reference, delta, strict=True))
      if candidate == reference:
        continue  # Finite grid underflow cannot manufacture a neutral profile.
      feasible = native_feasible(candidate)
      if type(feasible) is not bool:
        return Result('unavailable', 'event_native_feasibility_contract')
      if feasible:
        proposal = candidate
        break
    else:
      return Result('collecting', 'event_no_native_feasible_scale')
    # This selected proposal is now frozen. ALL following tiny-loss/heldout
    # checks run once; failure MUST NOT retry a smaller scale.
  heldout = blocks[split:]
  improvements = []
  for cohort in sorted(required_cohorts):
    selected = [block for block in heldout if block.cohort == cohort]
    # Recompute BOTH before/after and original-baseline heldout evidence for the
    # actual tiny proposal. The bounded training proposal is never admitted without these checks.
    gains = [loss(block.samples, reference) - loss(block.samples, proposal) for block in selected]
    original_gains = [loss(block.samples, (0.0,) * len(parameters)) - loss(block.samples, proposal) for block in selected]
    lower = min(_lower(gains), _lower(original_gains))
    if event_mean_model is not None:
      from openpilot.starpilot.flm.event_mean_evidence import lower as event_lower
      lower = min(event_lower(event_mean_model,tuple(selected),tuple(gains),cohort=cohort),
        event_lower(event_mean_model,tuple(selected),tuple(original_gains),cohort=cohort))
    if not math.isfinite(lower) or lower <= 0:
      return Result('rejected', 'event_tiny_trial_heldout_regression_or_noise')
    improvements.append((cohort, lower))
  rows = tuple(row for block in heldout for row in block.samples)
  baseline = min(loss(rows, reference), loss(rows, (0.0,) * len(parameters)))
  candidate = loss(rows, proposal)
  if not 0 <= candidate < baseline:
    return Result('rejected', 'event_tiny_trial_loss')
  profile = replace(fitted.profile, coefficients=proposal, baseline_loss=baseline, candidate_loss=candidate,
                    cohort_improvements=tuple(improvements), policy=POLICY,
                    effect_envelope=effect_envelope.with_proposal(dict(zip(effect_envelope.trial_names, proposal, strict=True))))
  return Result('qualified', 'event_tiny_trial_pending_actual_replay', profile)
