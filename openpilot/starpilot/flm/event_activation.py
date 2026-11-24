"""Child-derived fixed-calibration activation; no live control/wire authority."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math

from openpilot.starpilot.flm.physical_response import PhysicalModel
from openpilot.starpilot.flm.clipping_quality import (
  RECORDED_DEBT, STATE_ALIGNED_DEBT, STATE_ALIGNED_RELIEF, EXACT_EMISSION_QUALITY, ZERO_REFERENCE_QUALITY,
  RESTORE_ACTIVATION_VERSION, RESTORE_AGGREGATE_ACTIVATION_VERSION, RESTORE_ZERO_REFERENCE_VERSION,
  QUALIFIED_EVENT_CLIPPING_QUALITY, clipping_quality, identification_for_quality,
)

ACTIVATION_VERSION = 'fixed-eight-owner-v1'
STATE_ALIGNED_ACTIVATION_VERSION = 'fixed-eight-owner-state-aligned-v2'
COMBINED_ACTIVATION_VERSION = 'fixed-eight-shadow-relief-v3'
EXACT_EMISSION_ACTIVATION_VERSION = 'fixed-eight-shadow-exact-emission-v4'
ZERO_REFERENCE_ACTIVATION_VERSION = 'fixed-eight-shadow-native-feasible-v6'
ACTIVATION_VERSIONS = (ACTIVATION_VERSION, STATE_ALIGNED_ACTIVATION_VERSION, COMBINED_ACTIVATION_VERSION,
                       EXACT_EMISSION_ACTIVATION_VERSION, ZERO_REFERENCE_ACTIVATION_VERSION)
CALIBRATION_EVENTS = 8


def _hex64(value):
  return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


@dataclass(frozen=True)
class EventActivationPlan:
  version: str
  identity: str
  baseline_revision: str
  source_epoch: str
  physical_context: str
  calibration_train_blocks: tuple[str, ...]
  calibration_heldout_blocks: tuple[str, ...]
  cutoff_ns: int
  stage_blocks: tuple[str, ...]
  # Exact original replay window and first stage frame/owner prepare clock.
  # Zero activation means this window is original calibration/context only.
  windows: tuple[tuple[int, int, int, int], ...]

  def validate(self):
    if (self.version not in ACTIVATION_VERSIONS or
        not all(_hex64(value) for value in (self.identity, self.baseline_revision, self.source_epoch, self.physical_context)) or
        type(self.cutoff_ns) is not int or self.cutoff_ns <= 0 or
        type(self.calibration_train_blocks) is not tuple or type(self.calibration_heldout_blocks) is not tuple or
        len(self.calibration_train_blocks) != 4 or len(self.calibration_heldout_blocks) != 4 or
        type(self.stage_blocks) is not tuple or not 8 <= len(self.stage_blocks) <= 24):
      raise ValueError('fixed event calibration activation binding')
    ids = self.calibration_train_blocks + self.calibration_heldout_blocks + self.stage_blocks
    if (any(type(value) is not str or not 1 <= len(value) <= 128 for value in ids) or len(set(ids)) != len(ids) or
        type(self.windows) is not tuple or not 1 <= len(self.windows) <= 24):
      raise ValueError('event activation whole-event lineage')
    previous = 0
    active = False
    for window in self.windows:
      if type(window) is not tuple or len(window) != 4 or any(type(value) is not int for value in window):
        raise ValueError('event activation exact window')
      start, end, control, prepare = window
      if not 0 < start <= end or start <= previous:
        raise ValueError('event activation window chronology')
      if control == prepare == 0:
        pass
      elif not start <= control <= end or not self.cutoff_ns < control or not 0 < prepare <= control:
        raise ValueError('event activation original owner clock')
      else:
        active = True
      previous = end
    if not active:
      raise ValueError('event activation no stage window')
    return self

  def digest(self):
    self.validate()
    return hashlib.sha256(json.dumps(asdict(self), sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

  @property
  def clipping_quality(self):
    self.validate()
    return {ACTIVATION_VERSION: RECORDED_DEBT, STATE_ALIGNED_ACTIVATION_VERSION: STATE_ALIGNED_DEBT,
            COMBINED_ACTIVATION_VERSION: STATE_ALIGNED_RELIEF, EXACT_EMISSION_ACTIVATION_VERSION: EXACT_EMISSION_QUALITY,
            ZERO_REFERENCE_ACTIVATION_VERSION: ZERO_REFERENCE_QUALITY}[self.version]

  @property
  def identification_policy(self):
    return identification_for_quality(self.clipping_quality)

  def activation_for(self, start, end):
    return next((row[2:] for row in self.windows if row[:2] == (start, end)), None)


def activation_from_dict(value):
  if type(value) is not dict or set(value) != set(EventActivationPlan.__dataclass_fields__):
    raise ValueError('event activation report fields')
  copied = dict(value)
  for name in ('calibration_train_blocks', 'calibration_heldout_blocks', 'stage_blocks'):
    copied[name] = tuple(copied[name])
  copied['windows'] = tuple(tuple(row) for row in copied['windows'])
  return EventActivationPlan(**copied).validate()


def build_activation_plan(engine, *, model, cutoff_ns, stage_events, physical_context, quality_contract=RECORDED_DEBT):
  """No trial coordinates, rollout results, desired gains or cutoff search here."""
  clipping_quality(quality_contract)
  version = {RECORDED_DEBT: ACTIVATION_VERSION, STATE_ALIGNED_DEBT: STATE_ALIGNED_ACTIVATION_VERSION,
             STATE_ALIGNED_RELIEF: COMBINED_ACTIVATION_VERSION, EXACT_EMISSION_QUALITY: EXACT_EMISSION_ACTIVATION_VERSION,
             ZERO_REFERENCE_QUALITY: ZERO_REFERENCE_ACTIVATION_VERSION}[quality_contract]
  frames = {frame.control_ns: frame for frame in engine.records}
  windows = tuple(engine.replay_windows)
  if (not stage_events or any(min(point['control_ns'], point['apply_ns'], point['mono_ns']) <= cutoff_ns
                             for event in stage_events for point in event['samples'])):
    raise ValueError('event activation disjoint physical stage')
  activations = {}
  for event in stage_events:
    if not event['event_windows']:
      raise ValueError('event activation original whole boundary')
    start = event['event_windows'][0][0]
    window = next(((a, b) for a, b in windows if a <= start <= b), None)
    if window is None or start <= cutoff_ns:
      raise ValueError('event activation stage boundary precedes calibration')
    # The first original whole stage boundary is fixed before any candidate.
    # Never scan later frames for a less demanding or better-fitting onset.
    if window in activations:
      continue
    frame = frames.get(start)
    prepare = getattr(frame, 'effect_prepare_ns', 0)
    if (frame is None or not engine._frame_context(frame) or not engine._prior_matches(frame) or
        type(prepare) is not int or not 0 < prepare <= frame.control_ns):
      raise ValueError('event activation missing actual owner source clock')
    activations[window] = (start, prepare)
  return EventActivationPlan(version, engine.identity, engine.baseline_revision,
    engine.objective.context_key, physical_context, tuple(model.train_blocks), tuple(model.heldout_blocks), cutoff_ns,
    tuple(event['block_id'] for event in stage_events),
    tuple((a, b, *activations.get((a, b), (0, 0))) for a, b in windows)).validate()


@dataclass(frozen=True)
class ActivatedPhysicalModel(PhysicalModel):
  """The actually fitted unchanged affine model, bound to its replay experiment."""
  activation_version: str
  activation_plan_sha256: str
  calibration_cutoff_ns: int


def bind_activation_model(model, plan):
  plan.validate()
  if (type(model) is not PhysicalModel or model.identity != plan.identity or model.context != plan.physical_context or
      model.train_blocks != plan.calibration_train_blocks or model.heldout_blocks != plan.calibration_heldout_blocks):
    raise ValueError('event activation actual fitted model binding')
  return ActivatedPhysicalModel(**asdict(model), activation_version=plan.version,
    activation_plan_sha256=plan.digest(), calibration_cutoff_ns=plan.cutoff_ns)


def activation_model_from_dict(value):
  copied = dict(value)
  for field in ('train_blocks', 'heldout_blocks'):
    copied[field] = tuple(copied[field])
  if 'activation_version' not in copied:
    return PhysicalModel(**copied)
  model = ActivatedPhysicalModel(**copied)
  restore_versions = (RESTORE_ACTIVATION_VERSION, RESTORE_AGGREGATE_ACTIVATION_VERSION, RESTORE_ZERO_REFERENCE_VERSION)
  if (model.activation_version not in (*ACTIVATION_VERSIONS, *restore_versions) or
      not _hex64(model.activation_plan_sha256) or
      model.domain != 'normalized_torque' or
      any(type(value) not in (int, float) or not math.isfinite(value) for value in
          (model.slope, model.intercept, model.command_low, model.command_high, model.heldout_rmse)) or
      model.slope <= 0 or not model.command_low < 0 < model.command_high or not 0 <= model.heldout_rmse <= .05 or
      type(model.calibration_cutoff_ns) is not int or model.calibration_cutoff_ns <= 0):
    raise ValueError('event activation model report')
  return model


def activation_matches_model(plan, model):
  return bool(type(model) is ActivatedPhysicalModel and model.activation_version == plan.version and
    model.activation_plan_sha256 == plan.digest() and model.calibration_cutoff_ns == plan.cutoff_ns and
    model.identity == plan.identity and model.context == plan.physical_context and
    model.train_blocks == plan.calibration_train_blocks and model.heldout_blocks == plan.calibration_heldout_blocks)


def model_clipping_quality(model):
  if type(model) is ActivatedPhysicalModel:
    if model.activation_version == RESTORE_ZERO_REFERENCE_VERSION:
      return ZERO_REFERENCE_QUALITY
    if model.activation_version in (RESTORE_ACTIVATION_VERSION, RESTORE_AGGREGATE_ACTIVATION_VERSION):
      return QUALIFIED_EVENT_CLIPPING_QUALITY
    if model.activation_version not in ACTIVATION_VERSIONS:
      raise ValueError('unknown activated physical proof version')
    return {ACTIVATION_VERSION: RECORDED_DEBT, STATE_ALIGNED_ACTIVATION_VERSION: STATE_ALIGNED_DEBT,
            COMBINED_ACTIVATION_VERSION: STATE_ALIGNED_RELIEF, EXACT_EMISSION_ACTIVATION_VERSION: EXACT_EMISSION_QUALITY,
            ZERO_REFERENCE_ACTIVATION_VERSION: ZERO_REFERENCE_QUALITY}[model.activation_version]
  return RECORDED_DEBT
