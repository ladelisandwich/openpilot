"""PRIVATE source-only candidate. Pure arithmetic over caller-verified evidence.

This module DOES NOT verify CAN/native signatures, controller replay, profile
admission or persisted authority. The real child must recompute those before
constructing VerifiedCommandTape and again bind the result to its activation
and current-admission proofs. A qualified result is only an affine plant test.
Clock-budget helper only; no file I/O, controllers, executable callbacks or optimization
against heldout data. Protocol constants cannot be passed as tuning arguments.
"""
from bisect import bisect_right
from openpilot.starpilot.flm.producer_clock_equivalence import clock_uncertainty_limit
from dataclasses import asdict, dataclass
import hashlib
import json
import math

PROTOCOL = 'source-pose-plant-lag-v4-private'
PHYSICAL_PROTOCOL = 'source-pose-physical-observation-lag-v1-private'
LAGS_NS = tuple(range(0, 500_000_001, 10_000_000))
JOIN_NS = 30_000_000
MIN_EXCITATION = .05
MAX_RMSE = .05
MAX_COMMANDS = 12_000
MAX_POSES = 8 * 240
TRAIN_WINDOWS = 4
NUM_WINDOWS = 8
STRICT_SUPPORT = 'strict-heldout-domain-v1'
TRAINING_DOMAIN_ONLY = 'training-domain-only-heldout-v1'
LAG_STABILITY_NS = 20_000_000
TIE_ABS_MSE = 1e-12


def digest(value):
  return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _hash(value):
  return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _integer(value, *, zero=False):
  return type(value) is int and (value >= 0 if zero else value > 0)


def _number(value):
  return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class SourceBinding:
  # All hashes are independently computed by the actual child, never read as
  # authority from parent saved healthy/normalized flags.
  identity: str
  cp_sha256: str
  source_sha256: str
  controller_revision: str
  source_epoch: str
  baseline_revision: str
  native_identity: str
  calibration_generation: str
  vehicle_model_generation: str
  session: str
  source_clock_contract_sha256: str
  native_required: bool
  timestamp_convention: str = 'original-kf-bounded-source-clock-v3'


@dataclass(frozen=True)
class DeliveredCommand:
  control_ns: int
  state_ns: int
  apply_ns: int
  # Oriented coordinate MUST be -delivered(output) from original CAN verifier.
  command: float
  source_receipt_ns: int
  native_observed_ns: int
  native_expires_ns: int
  carrier_frame: int
  carrier_observed_ns: int
  encoded_witness_sha256: str
  control_sha256: str
  applied_sha256: str
  identity: str
  source_epoch: str
  baseline_revision: str
  native_identity: str
  native_required: bool = True
  healthy: bool = True
  neutral_coordinates: bool = True
  native_allowed: bool = True
  overridden: bool = False
  saturated: bool = False
  limited: bool = False


@dataclass(frozen=True)
class VerifiedCommandTape:
  commands: tuple[DeliveredCommand, ...]
  binding_digest: str
  # Must equal command_tape_digest(binding, commands). This is tamper binding,
  # not a cryptographic certificate that caller actually ran a verifier.
  verified_tape_digest: str
  verifier_source_sha256: str
  verified_source_native_can: bool


@dataclass(frozen=True)
class SourcePose:
  pose_id: str
  mono_ns: int
  received_ns: int
  source_timestamp_ns: int
  source_clock_domain: str
  clock_offset_ns: int
  clock_uncertainty_ns: int
  source_lower_ns: int
  source_upper_ns: int
  clock_generation: int
  speed_mps: float
  calibrated_yaw_rate: float
  calibrated_roll_rad: float
  binding_digest: str
  healthy: bool = True
  native_allowed_at_receipt: bool = True
  overridden: bool = False
  fault: bool = False


@dataclass(frozen=True)
class CalibrationWindow:
  # Caller recomputes original whole source-window boundaries independently.
  block_id: str
  original_start_ns: int
  original_end_ns: int
  pose_ids: tuple[str, ...]
  source_window_sha256: str


@dataclass(frozen=True)
class Pair:
  block_id: str
  pose_id: str
  command_index: int
  command_ns: int
  response_ns: int
  command: float
  response: float


@dataclass(frozen=True)
class LagDecision:
  protocol: str
  lag_ns: int
  slope: float
  intercept: float
  command_low: float
  command_high: float
  train_block_ids: tuple[str, ...]
  train_pose_ids: tuple[str, ...]
  train_source_digest: str
  training_pairs_digest: str
  training_scores: tuple[tuple[int, float | None], ...]
  training_leave_one_out_lags: tuple[int, ...]
  # Contains NO heldout response primitives, scores or outcome.
  digest: str


@dataclass(frozen=True)
class IdentificationResult:
  status: str
  reason: str
  binding_digest: str
  experiment_digest: str
  decision: LagDecision | None
  rows: tuple[Pair, ...]
  heldout_blocks: tuple[tuple[str, float, int], ...]
  model_receipt_digest: str | None
  cutoff_ns: int | None
  support_policy: str = STRICT_SUPPORT


def binding_digest(binding):
  return digest(asdict(binding))


def command_tape_digest(binding, commands):
  return digest({'protocol': PROTOCOL, 'binding': asdict(binding), 'commands': [asdict(c) for c in commands]})


def _validate(binding, tape, poses, windows):
  if not isinstance(binding, SourceBinding) or not isinstance(tape, VerifiedCommandTape):
    raise ValueError('explicit source binding and independently verified command tape required')
  fields = asdict(binding)
  if (any(not _hash(value) for key, value in fields.items() if key not in ('session', 'timestamp_convention', 'native_required', 'native_identity'))
      or type(binding.native_required) is not bool
      or (not _hash(binding.native_identity) if binding.native_required else binding.native_identity != '')
      or type(binding.session) is not str or not binding.session
      or binding.timestamp_convention != 'original-kf-bounded-source-clock-v3'):
    raise ValueError('source/native/CP/calibration/model binding')
  bound = binding_digest(binding)
  if (type(tape.commands) is not tuple or not 1 <= len(tape.commands) <= MAX_COMMANDS
      or tape.binding_digest != bound or not _hash(tape.verifier_source_sha256)
      or tape.verified_source_native_can is not True
      or tape.verified_tape_digest != command_tape_digest(binding, tape.commands)):
    raise ValueError('exact independently verified tape binding/bounds')
  previous = 0
  previous_carrier = 0
  for c in tape.commands:
    if not isinstance(c, DeliveredCommand):
      raise ValueError('actual delivered command type')
    integers = (c.control_ns, c.state_ns, c.apply_ns, c.source_receipt_ns,
      c.carrier_frame, c.carrier_observed_ns)
    if (not all(_integer(v) for v in integers) or c.apply_ns <= previous or
        not 0 < c.state_ns <= c.control_ns <= c.state_ns + 30_000_000 or
        not c.control_ns <= c.apply_ns <= c.control_ns + 100_000_000 or
        not 0 < c.source_receipt_ns <= c.control_ns <= c.source_receipt_ns + 500_000_000 or
        c.native_required is not binding.native_required or
        (not all(type(v) is int for v in (c.native_observed_ns,c.native_expires_ns))) or
        (not 0 < c.native_observed_ns <= c.control_ns <= c.apply_ns <= c.native_expires_ns or
         c.native_expires_ns-c.native_observed_ns > 200_000_000 if c.native_required else
         (c.native_observed_ns,c.native_expires_ns) != (0,0)) or
        not c.apply_ns <= c.carrier_observed_ns <= c.apply_ns + 100_000_000 or
        previous_carrier and c.apply_ns - previous <= JOIN_NS and c.carrier_frame not in (previous_carrier, previous_carrier + 1) or
        not _number(c.command) or abs(c.command) > 1 or
        any(not _hash(v) for v in (c.encoded_witness_sha256, c.control_sha256, c.applied_sha256)) or
        (c.identity, c.source_epoch, c.baseline_revision, c.native_identity) !=
        (binding.identity, binding.source_epoch, binding.baseline_revision, binding.native_identity) or
        any(v is not True for v in (c.healthy, c.neutral_coordinates, c.native_allowed)) or
        any(v is not False for v in (c.overridden, c.saturated, c.limited))):
      raise ValueError('actual source/native/receipt/clock/neutral/unclipped command guard')
    previous = c.apply_ns
    previous_carrier = c.carrier_frame
  if type(poses) is not tuple or not 1 <= len(poses) <= MAX_POSES or type(windows) is not tuple or len(windows) != NUM_WINDOWS:
    raise ValueError('exact eight calibration windows / bounded original poses')
  previous = 0
  pose_ids = set()
  for p in poses:
    if not isinstance(p, SourcePose):
      raise ValueError('unpaired independent source pose type')
    if (not _hash(p.pose_id) or p.pose_id in pose_ids or
        not all(_integer(v) for v in (p.mono_ns, p.received_ns, p.source_timestamp_ns)) or
        type(p.clock_offset_ns) is not int or not _integer(p.clock_uncertainty_ns, zero=True) or
        p.clock_uncertainty_ns > clock_uncertainty_limit(p.source_clock_domain) or p.source_timestamp_ns - p.clock_offset_ns != p.mono_ns or
        p.source_clock_domain not in ('monotonic', 'boottime', 'unknown-equivalent-at-receipt', 'mono_boot_equivalent_interval_v1') or
        p.source_clock_domain != 'boottime' and p.clock_offset_ns != 0 or
        not all(_integer(v) for v in (p.source_lower_ns,p.source_upper_ns)) or
        not _integer(p.clock_generation,zero=True) or
        not p.source_lower_ns <= p.mono_ns <= p.source_upper_ns or
        max(p.mono_ns-p.source_lower_ns,p.source_upper_ns-p.mono_ns) > p.clock_uncertainty_ns or
        not p.mono_ns <= p.received_ns <= p.mono_ns + 100_000_000 or p.mono_ns <= previous or
        not all(_number(v) for v in (p.speed_mps, p.calibrated_yaw_rate, p.calibrated_roll_rad)) or
        p.speed_mps <= 0 or p.binding_digest != bound or
        p.clock_generation != poses[0].clock_generation or
        any(v is not True for v in (p.healthy, p.native_allowed_at_receipt)) or
        any(v is not False for v in (p.overridden, p.fault))):
      raise ValueError('immutable source pose/clock/native/calibration guard')
    previous = p.mono_ns
    pose_ids.add(p.pose_id)
  if poses[-1].mono_ns - poses[0].mono_ns + LAGS_NS[-1] > 600_000_000_000:
    raise ValueError('original maximum 600 second physical history including candidate lookback')
  mapped = {p.pose_id: p for p in poses}
  assigned = []
  block_ids = set()
  previous = 0
  for w in windows:
    if (not isinstance(w, CalibrationWindow) or not _hash(w.block_id) or w.block_id in block_ids or
        not _hash(w.source_window_sha256) or not _integer(w.original_start_ns) or not _integer(w.original_end_ns) or
        not previous < w.original_start_ns <= w.original_end_ns or type(w.pose_ids) is not tuple or
        not 40 <= len(w.pose_ids) <= 240 or any(pid not in mapped for pid in w.pose_ids)):
      raise ValueError('original fixed chronological whole-window declaration')
    selected = [mapped[pid] for pid in w.pose_ids]
    if (tuple(sorted(selected, key=lambda p: p.mono_ns)) != tuple(selected) or
        any(not w.original_start_ns <= p.mono_ns <= w.original_end_ns for p in selected)):
      raise ValueError('pose outside original declared source window/order')
    assigned.extend(w.pose_ids)
    block_ids.add(w.block_id)
    previous = w.original_end_ns
  if assigned != [p.pose_id for p in poses] or len(set(assigned)) != len(assigned):
    raise ValueError('all original poses must be assigned exactly once without dropping/reordering')
  return bound


def _validate_physical(binding, tape, poses, windows):
  from openpilot.starpilot.flm.physical_input_observation import (
    PROTOCOL as OBSERVATION_PROTOCOL, VerifiedPhysicalTape, physical_tape_digest, validate_input,
  )
  if type(binding) is not SourceBinding or type(tape) is not VerifiedPhysicalTape:
    raise ValueError('explicit physical-only binding and tape required')
  fields = asdict(binding)
  if (any(not _hash(value) for key, value in fields.items() if key not in ('session', 'timestamp_convention', 'native_required', 'native_identity'))
      or type(binding.native_required) is not bool
      or (not _hash(binding.native_identity) if binding.native_required else binding.native_identity != '')
      or type(binding.session) is not str or not binding.session
      or binding.timestamp_convention != 'original-kf-bounded-source-clock-v3'):
    raise ValueError('source/native/CP/calibration/model binding')
  bound = binding_digest(binding)
  if (tape.protocol != OBSERVATION_PROTOCOL or type(tape.commands) is not tuple or
      not 1 <= len(tape.commands) <= MAX_COMMANDS or tape.binding_digest != bound or
      not _hash(tape.verifier_source_sha256) or
      tape.verified_tape_digest != physical_tape_digest(binding, tape.commands)):
    raise ValueError('physical-only complete tape custody/bounds')
  previous, previous_carrier = 0, 0
  for command in tape.commands:
    validate_input(command, binding, bound)
    if (command.generation != tape.commands[0].generation or command.apply_ns <= previous or
        previous_carrier and command.apply_ns - previous <= JOIN_NS and
        command.carrier_frame not in (previous_carrier, previous_carrier + 1)):
      raise ValueError('physical original FIFO/carrier chronology')
    previous, previous_carrier = command.apply_ns, command.carrier_frame
  if type(poses) is not tuple or not 1 <= len(poses) <= MAX_POSES or type(windows) is not tuple or len(windows) != NUM_WINDOWS:
    raise ValueError('exact eight calibration windows / bounded original poses')
  previous = 0
  pose_ids = set()
  for p in poses:
    if not isinstance(p, SourcePose):
      raise ValueError('unpaired independent source pose type')
    if (not _hash(p.pose_id) or p.pose_id in pose_ids or
        not all(_integer(v) for v in (p.mono_ns, p.received_ns, p.source_timestamp_ns)) or
        type(p.clock_offset_ns) is not int or not _integer(p.clock_uncertainty_ns, zero=True) or
        p.clock_uncertainty_ns > clock_uncertainty_limit(p.source_clock_domain) or p.source_timestamp_ns - p.clock_offset_ns != p.mono_ns or
        p.source_clock_domain not in ('monotonic', 'boottime', 'unknown-equivalent-at-receipt', 'mono_boot_equivalent_interval_v1') or
        p.source_clock_domain != 'boottime' and p.clock_offset_ns != 0 or
        not all(_integer(v) for v in (p.source_lower_ns,p.source_upper_ns)) or
        not _integer(p.clock_generation,zero=True) or
        not p.source_lower_ns <= p.mono_ns <= p.source_upper_ns or
        max(p.mono_ns-p.source_lower_ns,p.source_upper_ns-p.mono_ns) > p.clock_uncertainty_ns or
        not p.mono_ns <= p.received_ns <= p.mono_ns + 100_000_000 or p.mono_ns <= previous or
        not all(_number(v) for v in (p.speed_mps, p.calibrated_yaw_rate, p.calibrated_roll_rad)) or
        p.speed_mps <= 0 or p.binding_digest != bound or
        p.clock_generation != poses[0].clock_generation or
        any(v is not True for v in (p.healthy, p.native_allowed_at_receipt)) or
        any(v is not False for v in (p.overridden, p.fault))):
      raise ValueError('immutable source pose/clock/native/calibration guard')
    previous = p.mono_ns
    pose_ids.add(p.pose_id)
  if poses[-1].mono_ns - poses[0].mono_ns + LAGS_NS[-1] > 600_000_000_000:
    raise ValueError('original maximum 600 second physical history including candidate lookback')
  mapped = {p.pose_id: p for p in poses}
  assigned = []
  block_ids = set()
  previous = 0
  for w in windows:
    if (not isinstance(w, CalibrationWindow) or not _hash(w.block_id) or w.block_id in block_ids or
        not _hash(w.source_window_sha256) or not _integer(w.original_start_ns) or not _integer(w.original_end_ns) or
        not previous < w.original_start_ns <= w.original_end_ns or type(w.pose_ids) is not tuple or
        not 40 <= len(w.pose_ids) <= 240 or any(pid not in mapped for pid in w.pose_ids)):
      raise ValueError('original fixed chronological whole-window declaration')
    selected = [mapped[pid] for pid in w.pose_ids]
    if (tuple(sorted(selected, key=lambda p: p.mono_ns)) != tuple(selected) or
        any(not w.original_start_ns <= p.mono_ns <= w.original_end_ns for p in selected)):
      raise ValueError('pose outside original declared source window/order')
    assigned.extend(w.pose_ids)
    block_ids.add(w.block_id)
    previous = w.original_end_ns
  if assigned != [p.pose_id for p in poses] or len(set(assigned)) != len(assigned):
    raise ValueError('all original poses must be assigned exactly once without dropping/reordering')
  return bound


def _join_all(tape, poses, windows):
  """Common support uses timestamps/command evidence only, never response y."""
  stamps = [c.apply_ns for c in tape.commands]
  blocks = {pid: w.block_id for w in windows for pid in w.pose_ids}
  joined = {}
  invariants = {} if all(type(p) is SourcePose for p in poses) else None
  for lag in LAGS_NS:
    pairs = []
    previous_command = 0
    for pose_index, p in enumerate(poses):
      _target = p.mono_ns - lag
      index = bisect_right(stamps, p.source_lower_ns-lag) - 1
      upper_index = bisect_right(stamps, p.source_upper_ns-lag) - 1
      if index < 0 or index != upper_index or p.source_upper_ns-lag-stamps[index] > JOIN_NS or stamps[index] <= previous_command:
        # Entire experiment rejects rather than drop this original row at a lag.
        raise ValueError(f'common_command_support:lag={lag}:pose={p.pose_id}')
      c = tape.commands[index]
      # Lazy only after the SAME lag0 endpoint guard. Original max-lag
      # continuity and measured force do not vary with candidate lag. Exact
      # frozen SourcePose only; subclasses retain the original repeated path.
      y = None if invariants is None else invariants.get(pose_index)
      if y is None:
        # A candidate may not bridge unknown/invalid history or original windows.
        first = bisect_right(stamps, p.source_lower_ns - LAGS_NS[-1]) - 1
        last = bisect_right(stamps, p.source_upper_ns) - 1
        if (first < 0 or p.source_lower_ns - LAGS_NS[-1] - stamps[first] > JOIN_NS or last < first or
            any(stamps[i + 1] - stamps[i] > JOIN_NS for i in range(first, last))):
          raise ValueError(f'common_command_continuity:pose={p.pose_id}')
        # Source pose conversion is independent of desired/controller signals.
        y = p.speed_mps * p.calibrated_yaw_rate - 9.81 * math.sin(p.calibrated_roll_rad)
        if not math.isfinite(y):
          raise ValueError('finite measured force response')
        if invariants is not None:
          invariants[pose_index] = y
      pairs.append(Pair(blocks[p.pose_id], p.pose_id, index, c.apply_ns, p.mono_ns, c.command, y))
      previous_command = c.apply_ns
    joined[lag] = tuple(pairs)
  return joined


def _ols(rows):
  n = len(rows)
  mx = sum(r.command for r in rows) / n
  my = sum(r.response for r in rows) / n
  variance = sum((r.command - mx)**2 for r in rows) / n
  if variance < MIN_EXCITATION**2:
    return None
  slope = sum((r.command - mx) * (r.response - my) for r in rows) / (n * variance)
  intercept = my - slope * mx
  if not math.isfinite(slope) or slope <= 0 or not math.isfinite(intercept):
    return None
  return slope, intercept, min(r.command for r in rows), max(r.command for r in rows)


def _score(rows, model):
  slope, intercept, _, _ = model
  ids = tuple(dict.fromkeys(r.block_id for r in rows))
  return sum(sum((r.response - (slope * r.command + intercept))**2 for r in rows if r.block_id == bid) /
             sum(r.block_id == bid for r in rows) for bid in ids) / len(ids)


def _direction_supported(rows):
  for left in (True, False):
    selected = [r for r in rows if (r.command > 0 if left else r.command < 0)]
    if len(selected) < 80:
      return False
    counts = {bid: sum(r.block_id == bid for r in selected) for bid in {r.block_id for r in selected}}
    if sum(n >= 20 for n in counts.values()) < 2:
      return False
  return True


def _pick(training, *, directions=True):
  scores = []
  models = {}
  for lag in LAGS_NS:
    rows = training[lag]
    model = _ols(rows)
    if model is None or directions and not _direction_supported(rows):
      scores.append((lag, None))
    else:
      scores.append((lag, _score(rows, model)))
      models[lag] = model
  valid = sorted((score, lag) for lag, score in scores if score is not None)
  if not valid:
    return None, tuple(scores), 'training_excitation_slope_direction'
  best_score, best = valid[0]
  if len(valid) > 1 and valid[1][0] - best_score <= TIE_ABS_MSE:
    return None, tuple(scores), 'training_lag_ambiguous'
  return (best, models[best]), tuple(scores), ''


def heldout_supported(result):
  """Keep every heldout error; scope use to the unchanged TRAINING domain.

  Out-of-domain rows remain in the archive and error score. They neither
  widen the model's command range nor count toward its interpolation support.
  """
  if len(result.heldout_blocks) != NUM_WINDOWS-TRAIN_WINDOWS or any(
      error > MAX_RMSE for _, error, _ in result.heldout_blocks):
    return False
  if result.support_policy == STRICT_SUPPORT:
    return not any(outside for _, _, outside in result.heldout_blocks)
  if result.support_policy != TRAINING_DOMAIN_ONLY or result.decision is None:
    return False
  decision = result.decision
  ids = tuple(bid for bid, _, _ in result.heldout_blocks)
  if len(set(ids)) != len(ids) or set(ids).intersection(decision.train_block_ids):
    return False
  inside = []
  for bid, error, outside in result.heldout_blocks:
    group = tuple(row for row in result.rows if row.block_id == bid)
    supported = tuple(row for row in group if decision.command_low <= row.command <= decision.command_high)
    if len(supported) < 40 or outside != len(group)-len(supported):
      return False
    # Score ALL original rows, including out-of-domain observations. No
    # response-dependent trimming or favorable validation subset is allowed.
    rmse = math.sqrt(sum((row.response-(decision.slope*row.command+decision.intercept))**2
                         for row in group)/len(group))
    if error != rmse:
      return False
    inside.extend(supported)
  return _direction_supported(tuple(inside))


def identify_plant_lag(binding, tape, poses, windows):
  """Legacy strict-input entrypoint; new physical-only types are rejected."""
  bound = _validate(binding, tape, poses, windows)
  return _identify_validated(binding, tape, poses, windows, bound, protocol=PROTOCOL)


def identify_physical_plant_lag(binding, tape, poses, windows, *, support_policy=STRICT_SUPPORT):
  """Physical-only observations; never profile, stage or current authority.

  The child must reconstruct original native/CAN/source proof before building
  this tape. This pure entrypoint verifies custody and unchanged affine gates.
  """
  bound = _validate_physical(binding, tape, poses, windows)
  return _identify_validated(binding, tape, poses, windows, bound, protocol=PHYSICAL_PROTOCOL, support_policy=support_policy)


def _identify_validated(binding, tape, poses, windows, bound, *, protocol, support_policy=STRICT_SUPPORT):
  """Shared original all51, train-only lag/affine, heldout/support arithmetic."""
  if support_policy not in (STRICT_SUPPORT, TRAINING_DOMAIN_ONLY) or (support_policy != STRICT_SUPPORT and protocol != PHYSICAL_PROTOCOL):
    raise ValueError('explicit physical response support policy')
  experiment = digest({'protocol': protocol, 'binding': asdict(binding), 'tape': tape.verified_tape_digest,
    'verifier': tape.verifier_source_sha256, 'poses': [asdict(p) for p in poses],
    'windows': [asdict(w) for w in windows], 'grid_ns': LAGS_NS})
  def unavailable(reason, *, decision=None, rows=(), heldout=(), cutoff=None):
    return IdentificationResult('collecting' if decision is None else 'rejected', reason,
      bound, experiment, decision, rows, heldout, None, cutoff)
  try:
    joined = _join_all(tape, poses, windows)
  except ValueError as exc:
    return unavailable(str(exc))
  train_ids = tuple(w.block_id for w in windows[:TRAIN_WINDOWS])
  training = {lag: tuple(r for r in rows if r.block_id in train_ids) for lag, rows in joined.items()}
  picked, scores, reason = _pick(training)
  if picked is None:
    return unavailable(reason)
  lag, model = picked
  block_out = []
  for omitted in train_ids:
    folds = {key: tuple(r for r in rows if r.block_id != omitted) for key, rows in training.items()}
    fold, _, fold_reason = _pick(folds, directions=False)
    if fold is None:
      return unavailable('training_block_out_' + fold_reason)
    block_out.append(fold[0])
  if any(abs(value - lag) > LAG_STABILITY_NS for value in block_out):
    return unavailable('training_lag_block_out_unstable')
  rows = joined[lag]
  train_rows = training[lag]
  train_pose_ids = tuple(r.pose_id for r in train_rows)
  # Bind ONLY the training command data actually visible at/before training end.
  # Heldout witness/response changes cannot alter the locked decision digest.
  train_end = max(r.response_ns for r in train_rows)
  train_source = digest({'binding': asdict(binding),
    'commands': [asdict(c) for c in tape.commands if c.apply_ns <= train_end],
    'poses': [asdict(p) for p in poses if p.pose_id in set(train_pose_ids)],
    'windows': [asdict(w) for w in windows[:TRAIN_WINDOWS]],
    'verifier_source': tape.verifier_source_sha256})
  slope, intercept, low, high = model
  declaration = {'protocol': protocol, 'lag_ns': lag, 'slope': slope, 'intercept': intercept,
    'command_low': low, 'command_high': high, 'train_block_ids': train_ids, 'train_pose_ids': train_pose_ids,
    'train_source_digest': train_source, 'training_pairs_digest': digest([asdict(r) for r in train_rows]),
    'training_scores': scores, 'training_leave_one_out_lags': tuple(block_out)}
  decision = LagDecision(**declaration, digest=digest(declaration))
  # No heldout response has been used by _pick, _ols, folds or declaration.
  heldout = tuple(r for r in rows if r.block_id not in train_ids)
  cutoff = max(r.response_ns for r in rows)
  if not _direction_supported(heldout):
    return unavailable('heldout_direction_support', decision=decision, rows=rows, cutoff=cutoff)
  results = []
  for window in windows[TRAIN_WINDOWS:]:
    group = tuple(r for r in heldout if r.block_id == window.block_id)
    outside = sum(not low <= r.command <= high for r in group)
    rmse = math.sqrt(sum((r.response - (slope * r.command + intercept))**2 for r in group) / len(group))
    results.append((window.block_id, rmse, outside))
  frozen_results = tuple(results)
  if support_policy == STRICT_SUPPORT and any(outside for _, _, outside in results):
    return unavailable('heldout_command_range', decision=decision, rows=rows, heldout=frozen_results, cutoff=cutoff)
  if any(error > MAX_RMSE for _, error, _ in results):
    return unavailable('heldout_rmse', decision=decision, rows=rows, heldout=frozen_results, cutoff=cutoff)
  if support_policy == TRAINING_DOMAIN_ONLY:
    proposed = IdentificationResult('physical_affine_test_passed', 'NO_PROFILE_NO_CURRENT_AUTHORITY', bound,
      experiment, decision, rows, frozen_results, None, cutoff, support_policy)
    if not heldout_supported(proposed):
      return unavailable('heldout_training_domain_support', decision=decision, rows=rows, heldout=frozen_results, cutoff=cutoff)
  model_digest = digest({'support_policy': support_policy, 'protocol': protocol, 'experiment': experiment, 'decision': decision.digest,
    'heldout': frozen_results, 'cutoff_ns': cutoff, 'binding': bound})
  return IdentificationResult('physical_affine_test_passed', 'NO_PROFILE_NO_CURRENT_AUTHORITY', bound,
    experiment, decision, rows, frozen_results, model_digest, cutoff, support_policy)
