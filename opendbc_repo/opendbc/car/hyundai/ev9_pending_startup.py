"""EV9 pending startup retains source UDS outcomes and waits for replacement controls."""
import time

from copy import deepcopy

from opendbc.car.hyundai.ev9_longitudinal import qualified, copy_cp
from opendbc.car.hyundai.ioniq5pe_startup import PEPendingStartup
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery


class EV9PendingStartup(PEPendingStartup):
  @staticmethod
  def profile_qualified(cp):
    return qualified(cp)

  def __init__(self, cp, callbacks, *, stock_cp, exchange=None, **kwargs):
    self.disable_response = None
    super().__init__(cp, callbacks, stock_cp=stock_cp,
                     exchange=self._ev9_exchange if exchange is None else exchange, **kwargs)

  def finalize_aol_configuration(self, ci):
    # Negotiate the exact marker before publication; all other CP bytes stay sealed.
    if (self.pending_target and not self.owner.published and ci is self.configured_ci and
        self.snapshot is not None and ci.CP.to_dict() != self.snapshot):
      from opendbc.car.hyundai.ev9_long_aol import qualified as aol_qualified
      expected = deepcopy(self.snapshot)
      expected['alternativeExperience'] = 32
      if not aol_qualified(ci.CP, marked_only=True) or ci.CP.to_dict() != expected:
        raise RuntimeError('EV9 pending finalization changed outside the exact AOL marker')
      self.owner.cp = copy_cp(ci.CP)
      self.snapshot = expected
    super().finalize_aol_configuration(ci)

  def _ev9_exchange(self, recv, send, request, response):
    attempts = 10 if request == b"\x10\x03" else 1
    for _ in range(attempts):
      query = IsoTpParallelQuery(send, recv, 1, [0x730], [request], [b""])
      replies = query.get_data(.1, total_timeout=.1)
      if request == b"\x28\x01\x01":
        if not replies:
          # Silence is sent, unconfirmed; the inherited post-request quiet and
          # independent-source proof must still pass before ownership commits.
          self.disable_response = 'no_reply_sent_unconfirmed'
          return True
        valid = all(payload == b"\x68\x01" for payload in replies.values())
        self.disable_response = 'positive_communication_control' if valid else 'negative_or_malformed_reply'
        return valid
      payload = replies.get((0x730, None))
      if request in (b"\x10\x03", b"\x10\x01"):
        valid = payload is not None and payload.startswith(response) and len(payload) in (2, 6)
      else:
        valid = payload == response
      if valid:
        if request == b"\x10\x03":
          time.sleep(.05)
        return True
      if request == b"\x10\x03":
        time.sleep(.1)
    return False
