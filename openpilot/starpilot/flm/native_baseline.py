"""Trusted native parameter refinement; no fitting or control authority."""
from dataclasses import dataclass
import math
import struct


def _f32(value):
  # Both lateralTorqueParameters and CarParams.Torque store Float32.
  return struct.unpack('<f', struct.pack('<f', float(value)))[0]


@dataclass(frozen=True)
class NativeRefinement:
  anchor: tuple[float, ...]
  before: tuple[float, ...]
  after: tuple[float, ...]
  static_before: tuple[float, ...]
  source_ns: int
  applied_ns: int
  source_values: tuple[float, float, float]


def static_values(controller):
  values = [float(controller.steer_max), float(controller.torque_params.steeringAngleDeadzoneDeg)]
  values.extend(float(getattr(controller, name)) for name in ('ff_factor', 'kf') if hasattr(controller, name))
  for name in ('_k_p', '_k_i'):
    for row in getattr(controller.pid, name):
      values.extend(float(value) for value in row)
  if not values or not all(math.isfinite(value) for value in values):
    raise ValueError('native refinement fixed configuration')
  return tuple(values)


def effective_native_valid(controller, cp):
  """Same CP-relative envelope as the existing scalar owner, after one transform."""
  try:
    factor = float(cp.lateralTuning.torque.latAccelFactor)
    friction = float(cp.lateralTuning.torque.friction)
    extension = getattr(controller, 'starpilot_extension', None)
    multiplier = float(getattr(extension, 'parameter_factor', None) or 1.0)
    tune = controller.torque_params
    actual = (float(tune.latAccelFactor), float(tune.latAccelOffset), float(tune.friction))
    if (not all(math.isfinite(value) for value in actual + (factor, friction, multiplier)) or factor <= 0 or multiplier <= 0 or
        not _f32(_f32(.7 * factor) * multiplier) <= actual[0] <= _f32(_f32(1.3 * factor) * multiplier) or
        abs(actual[1]) > _f32(min(.5, .5 * factor)) or
        not _f32(.5 * friction) <= actual[2] <= _f32(max(.1, 1.5 * friction))):
      return False
    positive = controller.lateral_accel_from_torque(controller.steer_max, tune)
    negative = controller.lateral_accel_from_torque(-controller.steer_max, tune)
    return controller.pid.pos_limit == positive and controller.pid.neg_limit == negative
  except (AttributeError, TypeError, ValueError, OverflowError):
    return False


def native_source(sm, cp, now_ns):
  try:
    name = 'lateralTorqueParameters'
    if type(now_ns) is not int or now_ns <= 0 or not sm.all_checks([name]):
      return None
    stamp, received = int(sm.logMonoTime[name]), int(round(sm.recv_time[name] * 1e9))
    row = sm[name]
    values = (float(row.latAccelFactorFiltered), float(row.latAccelOffsetFiltered), float(row.frictionCoefficientFiltered))
    base = cp.lateralTuning.torque
    if (not row.useParams or not row.valid or int(row.version) != 1 or
        not 0 < stamp <= now_ns <= stamp + 500_000_000 or
        not 0 < stamp <= received <= now_ns <= received + 500_000_000 or
        not all(math.isfinite(value) for value in values) or
        not _f32(.7 * float(base.latAccelFactor)) <= values[0] <= _f32(1.3 * float(base.latAccelFactor)) or
        abs(values[1]) > _f32(min(.5, .5 * float(base.latAccelFactor))) or
        not _f32(.5 * float(base.friction)) <= values[2] <= _f32(max(.1, 1.5 * float(base.friction)))):
      return None
    return stamp, values
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None


def apply_native_refinement(controller, cp, sm, *, now_ns, apply, host=False):
  """Run the selected writer once; return whether it ran, not receipt validity.

  Only an exact validated mutation gets a NativeRefinement. An unexpected
  writer result remains unreceipted and therefore invalidates optional FLM.
  """
  from openpilot.starpilot.flm.runtime_adapter_registry import baseline_values
  source = native_source(sm, cp, now_ns)
  if source is None:
    return False
  stamp, values = source
  previous_source = getattr(controller, 'native_learning_source', None)
  if previous_source is not None and (stamp < previous_source[0] or
      stamp == previous_source[0] and values != previous_source[1]):
    return False
  controller.native_learning_source = (stamp, values)
  anchor = tuple(controller.native_learning_baseline)
  last_tick = getattr(controller, 'native_learning_last_tick', 0)
  dt = (now_ns - last_tick) / 1e9 if last_tick else .01
  controller.native_learning_last_tick = now_ns
  before, fixed = baseline_values(controller), static_values(controller)
  apply()
  after = baseline_values(controller)
  if (static_values(controller) != fixed or not effective_native_valid(controller, cp) or
      not _valid_transition(controller, cp, before, after, values, host, dt)):
    # The selected writer ran once; an unreceipted change invalidates FLM.
    return True
  if before != after:
    controller.native_learning_refinement = NativeRefinement(anchor, before, after, fixed, stamp, now_ns, values)
    controller.native_learning_provenance = (1., *_words(stamp), *_words(now_ns), float(host), dt, float(len(before)), *values, *before, *after)
  return True


def accepts_refinement(controller, receipt, *, before, after, now_ns):
  return bool(isinstance(receipt, NativeRefinement) and receipt.anchor == tuple(controller.native_learning_baseline) and
              receipt.before == before and receipt.after == after and receipt.static_before == static_values(controller) and
              0 < receipt.applied_ns <= now_ns <= receipt.applied_ns + 50_000_000 and
              0 < receipt.source_ns <= receipt.applied_ns <= receipt.source_ns + 500_000_000)


def _raw_effective(controller, values):
  extension = getattr(controller, 'starpilot_extension', None)
  transformed = values if extension is None else extension.transform_torque_parameters(*values)
  return tuple(_f32(value) for value in transformed)


def _valid_transition(controller, cp, before, after, target, host, dt):
  if len(before) != len(after) or len(before) < 5 or not 0 < dt or host and dt > .05:
    return False
  expected = _raw_effective(controller, target)
  if not host:
    return tuple(after[1:4]) == tuple(expected)
  base = cp.lateralTuning.torque
  multiplier = float(getattr(getattr(controller, 'starpilot_extension', None), 'parameter_factor', None) or 1.)
  caps = (.5 * float(base.latAccelFactor) * multiplier * dt, .1 * dt, max(.05, float(base.friction)) * dt)
  for old, new, wanted, cap in zip(before[1:4], after[1:4], expected, caps, strict=True):
    if not min(old, wanted) <= new <= max(old, wanted) or abs(new - old) > cap + 1e-12:
      return False
  return True


def _words(ns):
  return float(ns >> 32), float(ns & 0xffffffff)


def checkpoint_native_valid(controller, cp, *, now_ns):
  """A changed checkpoint requires its exact source-to-applied receipt."""
  from openpilot.starpilot.flm.runtime_adapter_registry import baseline_values
  try:
    values = tuple(controller.native_learning_provenance)
    if not values:
      return baseline_values(controller) == tuple(controller.native_learning_baseline)
    if len(values) < 14 or values[0] != 1. or any(not math.isfinite(value) for value in values):
      return False
    source_ns = (int(values[1]) << 32) | int(values[2])
    applied_ns = (int(values[3]) << 32) | int(values[4])
    host, dt, count = int(values[5]), values[6], int(values[7])
    if host not in (0, 1) or values[5] != host or count < 5 or count > 32 or values[7] != count or len(values) != 11 + 2 * count:
      return False
    if any(value < 0 or value > 0xffffffff or int(value) != value for value in values[1:5]):
      return False
    target = values[8:11]
    before, after = values[11:11 + count], values[11 + count:]
    base = cp.lateralTuning.torque
    if (not _f32(.7 * float(base.latAccelFactor)) <= target[0] <= _f32(1.3 * float(base.latAccelFactor)) or
        abs(target[1]) > _f32(min(.5, .5 * float(base.latAccelFactor))) or
        not _f32(.5 * float(base.friction)) <= target[2] <= _f32(max(.1, 1.5 * float(base.friction)))):
      return False
    return bool(0 < source_ns <= applied_ns <= source_ns + 500_000_000 and
                0 < applied_ns <= now_ns and baseline_values(controller) == after and
                effective_native_valid(controller, cp) and _valid_transition(controller, cp, before, after, target, bool(host), dt))
  except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
    return False
