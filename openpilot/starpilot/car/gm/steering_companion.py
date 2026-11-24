"""Exact carState companions from the nonconflating steering-authority transport."""
from openpilot.cereal import messaging
from openpilot.starpilot.aol.runtime import INTENT_MAX_AGE_NS

MAX_COMPANIONS = 16
MAX_DRAIN = 32
SERVICE = 'starpilotCarState'


class SteeringCompanion:
  def __init__(self, sock=None):
    self.sock = sock
    self.samples = {}
    self.latest = None

  def _observe(self, stamp, valid, companion):
    sample = (int(stamp), bool(valid), int(companion.sourceCarStateMonoTime),
              bool(companion.lateralAuthorityUnavailable))
    previous = self.samples.get(sample[0])
    if previous is not None:
      sample = (sample[0], previous[1] and sample[1] and previous[2] == sample[2],
                sample[2], previous[3] or sample[3])
    self.samples[sample[0]] = sample
    if self.latest is None or sample[0] >= self.latest[0]:
      self.latest = sample
    while len(self.samples) > MAX_COMPANIONS:
      del self.samples[min(self.samples)]

  def drain(self):
    if self.sock is not None:
      for _ in range(MAX_DRAIN):
        message = messaging.recv_one_or_none(self.sock)
        if message is None:
          break
        self._observe(message.logMonoTime, message.valid, message.starpilotCarState)
      else:
        return False
    return True

  def current(self, sm, *, source_ns, now_ns, drain=True, allow_raw_projection=False):
    if drain and not self.drain():
      return None
    if self.sock is None and sm.seen.get(SERVICE, False):
      self._observe(sm.logMonoTime[SERVICE], sm.valid.get(SERVICE, False), sm[SERVICE])

    for stamp in tuple(self.samples):
      if not 0 < stamp <= now_ns or now_ns - stamp > INTENT_MAX_AGE_NS:
        del self.samples[stamp]
    if not (sm.seen.get(SERVICE, False) and sm.valid.get(SERVICE, False) and sm.alive.get(SERVICE, False)):
      return None
    # Raw receipts prove the exact selected source independently of when the
    # conflating cache was polled. Its own validity, age and fault veto remain.
    sm_stamp = int(sm.logMonoTime[SERVICE])
    sm_companion = sm[SERVICE]
    if not (int(sm_companion.sourceCarStateMonoTime) == sm_stamp and 0 < sm_stamp <= now_ns):
      return None
    # Exact fresh raw input can arrive after SubMaster's nonblocking poll. The
    # older healthy projection is not a second authority source or input lease.
    # Keep its validity, identity, future-clock and explicit fault vetoes.
    if (now_ns - sm_stamp > INTENT_MAX_AGE_NS and not
        (allow_raw_projection and self.sock is not None and sm_stamp < source_ns)):
      return None
    latest = self.latest
    pair = self.samples.get(source_ns)
    if latest is None or pair is None:
      return None
    stamp, valid, source, unavailable = latest
    if not (valid and source == stamp and source_ns <= stamp <= now_ns and
            now_ns - stamp <= INTENT_MAX_AGE_NS and pair[1] and pair[2] == source_ns):
      return None
    return bool(pair[3] or unavailable or sm_companion.lateralAuthorityUnavailable)
