"""Exact G90 prepublication failure fallback; sent disable is not confirmed ownership."""
from copy import deepcopy
from typing import cast
import time

from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
from opendbc.can.parser import MAX_BAD_COUNTER
from opendbc.car.hyundai.ecu_continuity import ContinuousECUStartup, StartupKeeper
from opendbc.car.hyundai.ecu_startup import Outcome, PreparedParams, stock_copy as stock_copy


def required(cp):
  return (cp.carFingerprint == CAR.GENESIS_G90 and cp.openpilotLongitudinalControl and
          not cp.passive and not cp.dashcamOnly and
          not cp.flags & (HyundaiFlags.CANFD | HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC))


class G90Startup(ContinuousECUStartup):
  def __init__(self, cp, callbacks, *, clock=time.monotonic, keeper_factory=StartupKeeper):
    super().__init__(cp, callbacks, address=0x7d0, bus=0, label='G90', clock=clock, keeper_factory=keeper_factory)

  def prepare_when_controls_ready(self, *, admission):
    """Keep factory SCC transmitting while the host producers initialize."""
    if self.outcome is not None or self.closed:
      raise RuntimeError('Classic startup preparation is single-use')
    if admission() is not True:
      return self.prepare(admission=lambda: False)
    self.admission = admission
    self.floor_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    self.outcome = Outcome.PENDING
    self.prepared_cp = deepcopy(self.cp.to_dict())
    return self.cp

  def commit_pending(self, ci, *, sources_current, control_current):
    if (self.outcome is not Outcome.PENDING or self.closed or not self.published or
        ci is not self.ci or not self.prepared_for(ci.CP) or not sources_current or
        not control_current or self.admission is None or self.admission() is not True):
      raise RuntimeError('Classic takeover requires current diagnostic, source and inactive control admission')
    try:
      success = self._disable(self.callbacks[0], self._send, bus=self.bus, addr=self.address,
                              com_cont_req=self.communication_control_request())
      if not success or not self.disable_attempted:
        raise RuntimeError('Classic pending ECU disable was not sent successfully')
      self.outcome = Outcome.SENT_UNCONFIRMED
      self.floor_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
      self.keeper = self.keeper_factory(
        lambda: self.callbacks[1]([make_tester_present_msg(self.address, self.bus, suppress_response=True)]), clock=self.clock)
      self.keeper.start()
    except BaseException:
      if self.keeper is not None:
        self.keeper.stop()
      try:
        restored = not (self.session_attempted or self.disable_attempted) or self._restore()
      except Exception:
        restored = False
      self.outcome = Outcome.ABORT_UNCERTAIN
      if not restored:
        raise RuntimeError('Classic pending takeover failed; stock restoration unverified') from None
      raise

  def before_control(self, **context):
    return self.outcome is not Outcome.PENDING and super().before_control(**context)

  def continuity_required(self):
    return required(self.cp)

  def _query(self, *args, **kwargs):
    return IsoTpParallelQuery(*args, **kwargs)

  def finalize_aol_configuration(self, ci):
    if self.published or self.closed or ci is not self.ci or not self.ready:
      raise RuntimeError('G90 AOL finalization requires configured startup')
    if self.prepared_for(ci.CP):
      return
    from opendbc.car.hyundai.classic_long_aol import qualified as long_qualified, ordinary_word, aol_word as long_word
    from opendbc.car.hyundai.classic_scc_aol import qualified as stock_qualified, stock_word, aol_word as stock_aol_word
    if self.outcome in (Outcome.PENDING, Outcome.SENT_UNCONFIRMED) and long_qualified(ci.CP, marked_only=True):
      before_word, after_word = ordinary_word(ci.CP), long_word(ci.CP)
    elif self.outcome in (Outcome.STOCK_UNTOUCHED, Outcome.STOCK_RESTORED) and stock_qualified(ci.CP, marked_only=True):
      before_word, after_word = stock_word(ci.CP), stock_aol_word(ci.CP)
    else:
      raise RuntimeError('G90 startup does not admit this AOL configuration')
    if ci.CP.carFingerprint != CAR.GENESIS_G90 or ci.CP.alternativeExperience != 32:
      raise RuntimeError('G90 AOL configuration identity does not match')
    expected = deepcopy(cast(PreparedParams, self.prepared_cp))
    configs = expected.get('safetyConfigs', [])
    if len(configs) != 1 or configs[0]['safetyModel'] != 'hyundai' or configs[0]['safetyParam'] != before_word or expected['alternativeExperience'] != 0:
      raise RuntimeError('G90 prepared safety profile cannot be finalized')
    configs[0]['safetyParam'] = after_word
    expected['alternativeExperience'] = 32
    if ci.CP.to_dict() != expected:
      raise RuntimeError('G90 AOL finalization changed unrelated CarParams')
    self.prepared_cp = expected

  def sources_current(self, ci, now_ns):
    if self.outcome not in (Outcome.PENDING, Outcome.SENT_UNCONFIRMED):
      return True
    if not ci.CS.out.canValid or ci.CS.out.canTimeout:
      return False
    parser = ci.can_parsers[Bus.pt]
    if parser.bus != 0:
      return False
    for name in ('EMS16', 'WHL_SPD11', 'TCS13', 'MDPS12', 'CLU11'):
      source = parser.message_states.get(parser.dbc.name_to_msg[name].address)
      if (source is None or not source.timestamps or source.counter_fail >= MAX_BAD_COUNTER or
          not self.floor_ns < int(source.timestamps[-1]) <= now_ns or now_ns - int(source.timestamps[-1]) > source.timeout_threshold):
        return False
    return True

  def _warm_stock(self, ci):
    deadline = time.monotonic() + 3.
    while time.monotonic() < deadline:
      packets = self.callbacks[0]()
      stamped = [(int(getattr(packet, 'log_mono_time_ns', 0)), list(packet)) for packet in packets]
      if any(stamp <= 0 for stamp, _ in stamped):
        raise RuntimeError(f'{self.label} stock warmup requires actual timestamped CAN')
      state = ci.update(stamped)
      now = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
      parser = ci.can_parsers[Bus.pt]
      ready = state.canValid and not state.canTimeout and parser.bus == 0
      for name in ('SCC11', 'SCC12'):
        source = parser.message_states.get(parser.dbc.name_to_msg[name].address)
        if source is None or not source.timestamps or source.counter_fail >= MAX_BAD_COUNTER:
          ready = False
          continue
        stamp = int(source.timestamps[-1])
        if not self.floor_ns < stamp <= now or now - stamp > source.timeout_threshold:
          ready = False
      if ready:
        self.ready = True
        return
      time.sleep(.005)
    self.outcome = Outcome.ABORT_UNCERTAIN
    raise RuntimeError(f'{self.label} stock SCC sources did not become fresh before publication')
