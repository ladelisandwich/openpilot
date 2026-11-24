from capnp.lib.capnp import _DynamicStructBuilder
from copy import deepcopy
from enum import Enum
import threading
import time

from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.can.parser import MAX_BAD_COUNTER
from opendbc.car.hyundai.ioniq5pe_long_profile import qualified
from opendbc.car.hyundai.ccnc_ev_stock import qualified as stock_qualified
from opendbc.car.hyundai.ev9_longitudinal import copy_cp
from opendbc.car.hyundai.ioniq5pe_ready import PEReadySource
from opendbc.car.hyundai.ioniq6_handoff import query_ioniq6_adas
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.torque_ev_scc import clock_boot_ns


class Phase(Enum):
  NEW = 'new'
  PENDING = 'pending'
  STOCK = 'stock'
  OWNED = 'owned'
  RESTORED = 'restored'
  ABORT = 'abort'


PE_REPLACED_ECAN = frozenset((0xCB, 0x12A, 0x1A0, 0x1E0, 0x1BA, 0x1E5, 0x160, 0x161,
                             0x162, 0x1EA, 0x200, 0x345, 0x38C, 0x1DA))
STOCK_SCC_HZ = 50.0
PARSER_TIMEOUT_PERIODS = 10.0


class Lease:
  def __init__(self, cp):
    self.cp = cp
    self.published = False
    self.phase = Phase.NEW

  def active(self):
    return self.phase is Phase.OWNED


def _plain_configuration(value):
  pending = [(value, 0)]
  while pending:
    container, depth = pending.pop()
    kind = type(container)
    if kind is dict:
      for key in container:
        if type(key) is not str:
          return False
      children = container.values()
    elif kind is list:
      children = container
    else:
      return False
    for child in children:
      kind = type(child)
      if kind is int or kind is float or kind is bool or kind is str or kind is bytes or child is None:
        continue
      if (kind is not dict and kind is not list) or depth >= 15:
        return False
      pending.append((child, depth + 1))
  return True


def _configuration_bytes(cp):
  return cp.as_reader().as_builder().to_bytes()


class PEPendingStartup:
  @staticmethod
  def profile_qualified(cp):
    return qualified(cp)

  def __init__(self, cp, callbacks: tuple[CanRecvCallable, CanSendCallable], *, stock_cp, clock=time.monotonic,
               boot_clock=clock_boot_ns,
               exchange=query_ioniq6_adas, event_factory=threading.Event, thread_factory=threading.Thread):
    if not self.profile_qualified(cp) or not stock_qualified(stock_cp) or stock_cp.carFingerprint != cp.carFingerprint:
      raise ValueError('PE startup requires exact separately prepared PE LONG and stock CP')
    self.owner = Lease(copy_cp(cp))
    self.stock_cp = copy_cp(stock_cp)
    self.callbacks = callbacks
    self.clock, self.boot_clock, self.exchange = clock, boot_clock, exchange
    self.thread_factory = thread_factory
    self.stop_event = event_factory()
    self.thread = None
    self.pending_target = False
    self.handed_off = False
    self.closed = False
    self.close_completed = False
    self.fault = None
    self.configured_ci = None
    self.snapshot = None
    self.diagnostic_admission = None
    self.prepared_floor_ns = self.source_floor_ns = 0
    self.session_attempted = False
    self.takeover_attempted = self.restored = False
    self.started_at = self.last_tester_at = None
    self.native_seen = False
    self.native_context = None
    self.emission_current = False
    self.quiet_failure = None

  def _ready_probe(self):
    observer = PEReadySource()
    start = self.boot_clock()
    deadline = self.clock() + .5
    while self.clock() < deadline:
      packets = self.callbacks[0](wait_for_one=False)
      for packet in packets:
        stamp = int(getattr(packet, 'log_mono_time_ns', 0))
        observer.update([(stamp, list(packet))], self.boot_clock())

        sample = observer.current(self.boot_clock(), start, 100_000_000)
        if sample is True:
          return True
      time.sleep(.005)

    return observer.current(self.boot_clock(), start, 100_000_000)

  def prepare(self, *, admission):
    if self.owner.phase is not Phase.NEW:
      raise RuntimeError('PE preparation is single-use')
    self.diagnostic_admission = admission
    self.prepared_floor_ns = self.boot_clock()
    ready = self._ready_probe() if admission() is True and self.prepared_floor_ns > 0 else None
    if ready is not False:
      self.owner.phase = Phase.STOCK
      self.owner.cp = copy_cp(self.stock_cp)
    else:
      self.owner.phase = Phase.PENDING
      self.pending_target = True

    self.snapshot = deepcopy(self.owner.cp.to_dict())
    return self.owner.cp

  def prepared_for(self, cp):
    if (self.closed or self.fault is not None or self.snapshot is None or
        self.owner.phase not in (Phase.PENDING, Phase.OWNED, Phase.STOCK)):
      return False
    previous = getattr(self, '_prepared_cp_comparison', None)
    try:
      if (previous is not None and previous[0] is self.owner.cp and previous[1] is cp and
          _plain_configuration(self.snapshot) and self.snapshot == previous[2] and
          _configuration_bytes(self.owner.cp) == previous[3] and _configuration_bytes(cp) == previous[4]):
        return True
    except Exception:
      pass
    matches = self.owner.cp.to_dict() == self.snapshot == cp.to_dict()
    self._prepared_cp_comparison = None
    if (matches and type(self.owner.cp) is _DynamicStructBuilder and type(cp) is _DynamicStructBuilder and
        _plain_configuration(self.snapshot)):
      try:
        self._prepared_cp_comparison = (self.owner.cp, cp, deepcopy(self.snapshot),
                                        _configuration_bytes(self.owner.cp), _configuration_bytes(cp))
      except Exception:
        pass
    return matches

  def configure(self, ci):
    if self.owner.published or not self.prepared_for(ci.CP):
      raise RuntimeError('PE configure requires matching unpublished prepared CP')
    self.configured_ci = ci
    if self.pending_target:
      ci.CC.pe_startup = self

  def finalize_aol_configuration(self, ci):

    if self.owner.published or ci is not self.configured_ci or self.snapshot is None:
      raise RuntimeError('PE finalization requires configured unpublished CP')
    if self.owner.phase is Phase.STOCK:
      if ci.CP.to_dict() != self.snapshot:
        from opendbc.car.hyundai.canfd_angle_aol import qualified as stock_aol_qualified
        expected = deepcopy(self.snapshot)
        expected['alternativeExperience'] = 32
        if not stock_aol_qualified(ci.CP, marked_only=True) or ci.CP.to_dict() != expected:
          raise RuntimeError('PE stock fallback changed outside its existing stock AOL marker')
        self.owner.cp = copy_cp(ci.CP)
        self.snapshot = expected
      return
    if (self.pending_target and ci.CP.to_dict() != self.snapshot):
      from opendbc.car.hyundai.pe_long_aol import qualified as pe_aol_qualified
      expected = deepcopy(self.snapshot)
      expected['alternativeExperience'] = 32
      if not pe_aol_qualified(ci.CP, marked_only=True) or ci.CP.to_dict() != expected:
        raise RuntimeError('PE LONG finalization changed outside its exact AOL marker')
      self.owner.cp = copy_cp(ci.CP)
      self.snapshot = expected
    if ci is not self.configured_ci or not self.prepared_for(ci.CP):
      raise RuntimeError('PE LONG finalization changed the frozen prepared configuration')

  def seal_publication(self):
    if self.configured_ci is None or not self.prepared_for(self.configured_ci.CP):
      raise RuntimeError('PE publication requires configured frozen CP')
    self.owner.published = True

  def _query(self, request, response):
    return self.exchange(*self.callbacks, request, response) is True

  def _confirm_quiet(self, ci, ack_floor_ns):

    start = self.clock()
    self.quiet_failure = None
    deadline = start + PARSER_TIMEOUT_PERIODS / STOCK_SCC_HZ
    while self.clock() < deadline:
      if self.clock() < start:
        self.quiet_failure = {'reason': 'clock_reversal'}
        return False
      packets = self.callbacks[0](wait_for_one=False)
      stamped = []
      for packet in packets:
        stamp = int(getattr(packet, 'log_mono_time_ns', 0))
        frames = list(packet)

        for address, data, bus in frames:
          if (bus == 1 and address in PE_REPLACED_ECAN) or (bus == 0 and address == 0x100):
            self.quiet_failure = {'reason': 'claimed_source_still_transmitting', 'address': hex(address),
                                  'bus': bus, 'length': len(data), 'producer_ns': stamp}
            return False
        if not ack_floor_ns < stamp <= self.boot_clock():
          self.quiet_failure = {'reason': 'unstamped_preack_or_future_source', 'producer_ns': stamp,
                                'ack_floor_ns': ack_floor_ns}
          return False
        stamped.append((stamp, frames))
      if stamped:
        ci.update(stamped)
      time.sleep(.001)
    current = self.sources_current(ci, self.boot_clock(), floor_ns=ack_floor_ns)
    if not current:
      self.quiet_failure = {'reason': 'independent_postack_can_not_current'}
    return current

  def _native_context_current(self):
    context = self.native_context
    if context is None:
      return False
    stamp, receipt, panda = context
    now = self.boot_clock()
    return (0 < stamp <= now and now - stamp <= 300_000_000 and
            0 < receipt <= now and now - receipt <= 300_000_000 and
            panda.safetyModel == self.owner.cp.safetyConfigs[0].safetyModel and
            panda.safetyParam == self.owner.cp.safetyConfigs[0].safetyParam and
            panda.alternativeExperience == self.owner.cp.alternativeExperience and (panda.ignitionLine or panda.ignitionCan))

  def _restore(self):
    if not (self.session_attempted or self.takeover_attempted) or self.restored:
      return True

    try:
      if self.diagnostic_admission is None or self.diagnostic_admission() is not True:
        return False
      if self.takeover_attempted:
        self.restored = (self._query(b'\x10\x03', b'\x50\x03') and self._query(b'\x28\x00\x01', b'\x68\x00') and
                         self._query(b'\x10\x01', b'\x50\x01'))
      else:

        self.restored = self._query(b'\x10\x01', b'\x50\x01')
    except Exception:
      self.restored = False
    self.owner.phase = Phase.RESTORED if self.restored else Phase.ABORT
    return self.restored

  def commit_pending(self, ci, *, sources_current, control_current):
    if (not self.pending_target or not self.owner.published or ci is not self.configured_ci or
        not self.prepared_for(ci.CP) or sources_current is not True or control_current is not True or
        self.diagnostic_admission is None or self.diagnostic_admission() is not True):
      raise RuntimeError('PE takeover requires the sealed pending target and current diagnostic/host inputs')
    parser = ci.can_parsers[Bus.pt]
    accelerator = parser.message_states[parser.dbc.name_to_msg['ACCELERATOR'].address]
    ready = ci.CS.pe_ready.current(self.boot_clock(), self.prepared_floor_ns, accelerator.timeout_threshold)
    if ready is not False:
      self.pending_target = False
      self.fault = 'PE Ready/unknown before takeover; abort published target without ECU mutation'
      self.owner.phase = Phase.ABORT
      raise RuntimeError(self.fault)
    try:
      self.session_attempted = True
      if not self._query(b'\x10\x03', b'\x50\x03'):
        raise RuntimeError('PE diagnostic session not confirmed')

      self.started_at = self.clock()
      self.takeover_attempted = True
      if not self._query(b'\x28\x01\x01', b'\x68\x01'):
        raise RuntimeError('PE communication control negative/malformed/silent; no ownership')
      ack_floor_ns = self.boot_clock()
      if not self._confirm_quiet(ci, ack_floor_ns):
        raise RuntimeError(f'PE acknowledged request did not establish quiet/current ownership: {self.quiet_failure}')
      self.owner.phase = Phase.OWNED
      self.source_floor_ns = self.boot_clock()
      self._tester()
      self.check()
      self.thread = self.thread_factory(target=self._loop, name='pe-startup-tester', daemon=True)
      try:
        self.thread.start()
      except BaseException:
        self.stop_event.set()
        if getattr(self.thread, 'ident', None) is None:
          self.thread = None
        raise
    except BaseException:
      self.owner.phase = Phase.ABORT
      self._restore()
      self.fault = 'PE pending takeover failed; published target must stop'
      raise
    finally:
      self.pending_target = False

  def _tester(self):
    if not self.owner.active() or self.fault is not None or self.closed:
      return
    now = self.clock()
    if self.started_at is None or now < self.started_at or not self.handed_off and now - self.started_at >= 1.:
      self.fault = 'PE first-replacement software deadline; no measured ECU tolerance claimed'
      self.stop_event.set()
      return
    if self.last_tester_at is not None:
      if now < self.last_tester_at:
        self.fault = 'PE diagnostic clock reversed'
        self.stop_event.set()
        return
      if now - self.last_tester_at < 1.:
        return
    self.callbacks[1]([make_tester_present_msg(0x730, 1, suppress_response=True)])
    self.last_tester_at = now

  def _loop(self):
    try:
      while not self.stop_event.wait(.5):
        self._tester()
    except BaseException:
      self.fault = 'PE diagnostic keeper send failed'
      self.stop_event.set()

  def sources_current(self, ci, now_ns, *, floor_ns=None):
    if self.owner.phase is Phase.STOCK:
      return True
    parser = ci.can_parsers[Bus.pt]
    if parser.bus != 1 or not parser.can_valid or parser.bus_timeout:
      return False
    floor = floor_ns if floor_ns is not None else self.prepared_floor_ns if self.pending_target else self.source_floor_ns
    if floor <= 0:
      return False
    for name in ('MDPS', 'ACCELERATOR', 'WHEEL_SPEEDS', 'TCS', 'STEERING_SENSORS', 'CRUISE_BUTTONS'):
      source = parser.message_states.get(parser.dbc.name_to_msg[name].address)
      if source is None or not source.timestamps or source.counter_fail >= MAX_BAD_COUNTER:
        return False
      stamp = int(source.timestamps[-1])
      if not floor < stamp <= now_ns or now_ns - stamp > source.timeout_threshold:
        return False
    return True

  def after_state(self, *, ci, state, now_ns, control_enabled, control_current, pandas,
                  panda_log_ns, panda_recv_ns, panda_current):
    self.emission_current = False
    if self.pending_target:
      state.buttonEnable = False
      state.blockPcmEnable = True
      return
    if not self.owner.active():
      return
    matched = False
    if panda_current and len(pandas) == 1:
      panda = pandas[0]
      cfg = self.owner.cp.safetyConfigs[0]
      matched = (panda.safetyModel == cfg.safetyModel and panda.safetyParam == cfg.safetyParam and
                 panda.alternativeExperience == self.owner.cp.alternativeExperience and not panda.faults and not panda.safetyRxChecksInvalid and
                 (panda.ignitionLine or panda.ignitionCan) and
                 self.source_floor_ns < panda_log_ns <= now_ns and now_ns - panda_log_ns <= 300_000_000 and
                 self.source_floor_ns < panda_recv_ns <= now_ns and now_ns - panda_recv_ns <= 300_000_000)
      if matched:
        self.native_context = (panda_log_ns, panda_recv_ns, panda)
    if self.native_seen and not matched:
      self.fault = 'PE native configuration/health continuity lost'
    self.native_seen |= matched
    self.emission_current = (matched and control_current and state.canValid and not state.canTimeout and
                             self.sources_current(ci, now_ns))

  def can_emit(self, controller):
    return (not self.closed and self.fault is None and self.owner.active() and self.emission_current and
            self.started_at is not None and (self.handed_off or 0 <= self.clock() - self.started_at < 1.) and
            self.native_context is not None and self._native_context_current() and
            self.configured_ci is not None and self.configured_ci.CC is controller and
            self.prepared_for(self.configured_ci.CP) and self.sources_current(self.configured_ci, self.boot_clock()))

  def before_control(self, *, configured, sources_current, control_current):
    self.check()
    if self.owner.phase is Phase.STOCK:
      return True
    self.emission_current &= bool(configured and sources_current and control_current)
    return not self.pending_target and self.emission_current

  def sent(self, frames, *, valid):
    if (not valid or self.configured_ci is None or not self.can_emit(self.configured_ci.CC)):
      return

    if not self.handed_off:
      critical = [(address, bus) for address, _, bus in frames
                  if (address, bus) in ((0x1A0, 1), (0x100, 0), (0xCB, 1), (0x730, 1))]
      if len(critical) != len(set(critical)):
        self.fault = 'PE first emitted replacement contains duplicate critical frame'
        return
    by_id = {(address, bus): data for address, data, bus in frames}
    tester = make_tester_present_msg(0x730, 1, suppress_response=True)
    scc, heartbeat, angle = by_id.get((0x1A0, 1)), by_id.get((0x100, 0)), by_id.get((0xCB, 1))
    if not self.handed_off:
      if tester not in frames or scc is None or heartbeat is None or angle is None:
        return
      if any(len(data) != length or int.from_bytes(data[:2], 'little') != hkg_can_fd_checksum(address, None, bytearray(data))
             for address, data, length in ((0x1A0, scc, 32), (0x100, heartbeat, 24), (0xCB, angle, 24))):
        self.fault = 'PE first emitted replacement integrity/length invalid'
        return
      raw = (((scc[17] & 7) << 8) | scc[16]) - 1023
      value = ((scc[18] << 4) | (scc[17] >> 4)) - 1023
      if raw != 0 or value != 0 or ((scc[8] >> 4) & 7) != 0 or ((angle[3] >> 4) & 15) != 1:
        self.fault = 'PE first emitted replacement was not neutral'
        return
      self.handed_off = True
      self.stop_event.set()
      if self.thread is not None:
        self.thread.join()
    if tester in frames:
      self.last_tester_at = self.clock()

  def maintain(self, *, configured):
    self.check()
    if self.owner.active() and self.handed_off and configured and (self.last_tester_at is None or self.clock() - self.last_tester_at >= 1.):
      self._tester()

  def consume_cruise_resume(self):
    return None

  def check(self):
    if self.fault is not None or self.owner.phase in (Phase.NEW, Phase.ABORT):
      raise RuntimeError(self.fault or 'Unresolved PE startup')

  def quiesce(self):
    self.closed = True
    self.emission_current = False
    self.stop_event.set()
    if self.thread is not None:
      self.thread.join()

  def close(self):
    if self.close_completed:
      if (self.session_attempted or self.takeover_attempted) and not self.restored:
        raise RuntimeError('PE stock restoration remains unverified')
      return
    self.quiesce()
    self.close_completed = True
    if not self._restore():
      self.owner.phase = Phase.ABORT
      raise RuntimeError('PE stock restoration remains unverified; never relabel published CP')
