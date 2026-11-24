"""Small deterministic box-constrained TRAIN ridge solve, without loss search."""
from itertools import product

from openpilot.starpilot.flm.adaptive_profile import finite, solve


def bounded_ridge(matrix, rhs, parameters, centre, trial_step):
  """Enumerate at most 3**4 active faces of the strictly convex ridge quadratic.

  A face fixes each active coordinate to a bound and re-solves its free
  coordinates. This is not componentwise clamping of an unconstrained fit.
  No heldout values or actuator/native permission enter this calculation.
  """
  count = len(parameters)
  if (not 1 <= count <= 4 or len(centre) != count or len(rhs) != count or len(matrix) != count or
      not finite(trial_step) or trial_step <= 0 or
      any(len(row) != count or not all(finite(value) for value in row) for row in matrix) or
      not all(finite(value) for value in (*rhs, *centre))):
    raise ValueError('bounded_ridge_contract')
  bounds = tuple((max(p.low, -p.max_step, old - trial_step), min(p.high, p.max_step, old + trial_step))
                 for p, old in zip(parameters, centre, strict=True))
  if any(not all(finite(value) for value in pair) or pair[0] > pair[1] for pair in bounds):
    raise ValueError('bounded_ridge_bounds')
  best = None
  for face in product((0, -1, 1), repeat=count):  # free, lower, upper; deterministic ties
    free = tuple(i for i, status in enumerate(face) if status == 0)
    candidate = [bounds[i][0 if status == -1 else 1] if status else 0. for i, status in enumerate(face)]
    if free:
      reduced = [[matrix[i][j] for j in free] for i in free]
      residual = [rhs[i] - sum(matrix[i][j] * candidate[j] for j in range(count) if face[j]) for i in free]
      try:
        fitted = solve(reduced, residual)
      except ValueError:
        continue
      for i, value in zip(free, fitted, strict=True):
        candidate[i] = value
    if any(not finite(value) or not low <= value <= high
           for value, (low, high) in zip(candidate, bounds, strict=True)):
      continue
    objective = sum(candidate[i] * (sum(matrix[i][j] * candidate[j] for j in range(count)) - 2 * rhs[i])
                    for i in range(count))
    if not finite(objective):
      continue
    if best is None or objective < best[0]:
      best = objective, tuple(candidate)
  if best is None:
    raise ValueError('bounded_ridge_unavailable')
  return best[1]
