"""Separate AOL-only lateral control time from ordinary engagement."""

from array import array


class ControlSamples:
  """Compact route evidence: nine bytes/sample, within existing log bounds."""

  def __init__(self):
    self.stamps = array('Q')
    self.flags = bytearray()

  def append(self, stamp, valid, active):
    self.stamps.append(stamp)
    self.flags.append(int(valid) | (int(active) << 1))

  def __len__(self):
    return len(self.stamps)

  def __getitem__(self, index):
    flags = self.flags[index]
    return self.stamps[index], bool(flags & 1), bool(flags & 2)


def aol_statistics(samples, first, last, *, max_pair_seconds=1.0):
  """Integrate valid carControl samples over the existing observed drive span.

  Callers provide (monotonic nanoseconds, valid, latActive and not enabled).
  carControl is published after the runtime/native gates; latch/intent/axis
  availability alone cannot prove lateral control is active. This measures
  active control, not physical steering motion or nonzero actuator torque.
  Missing, invalid, duplicate, or gapped evidence remains unknown, including
  old recordings with no carControl. Hold each sample only to its next sample.
  """
  unknown = {'aolSeconds': None, 'aolPercent': None}
  if len(samples) < 2 or last <= first:
    return unknown
  bound = max_pair_seconds * 1e9
  if samples[0][0] - first > bound or last - samples[-1][0] > bound:
    return unknown
  total = 0
  covered = 0
  previous = None
  for stamp, valid, active in samples:
    if stamp <= 0 or not valid:
      return unknown
    if previous is not None:
      delta = stamp - previous[0]
      if not 0 < delta <= bound:
        return unknown
      overlap = max(0, min(stamp, last) - max(previous[0], first))
      covered += overlap
      if previous[1]:
        total += overlap
    previous = stamp, active
  if not covered:
    return unknown
  duration = last - first
  return {'aolSeconds': round(total / 1e9, 3), 'aolPercent': round(100 * total / duration, 1)}
