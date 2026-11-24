"""Hyundai prepublication ECU transaction; sent disable is not confirmed ownership."""
from enum import Enum
from collections.abc import Callable
from typing import TypedDict, cast
from copy import deepcopy
import time

from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.hyundai.canfd_owner import config_index
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery


class PreparedSafetyConfig(TypedDict):
  safetyModel: str
  safetyParam: int


class PreparedParams(TypedDict):
  safetyConfigs: list[PreparedSafetyConfig]
  alternativeExperience: int


class Outcome(Enum):
  PENDING = 'pending'
  STOCK_UNTOUCHED = 'stock_untouched'
  SENT_UNCONFIRMED = 'sent_unconfirmed'
  STOCK_RESTORED = 'stock_restored'
  ABORT_UNCERTAIN = 'abort_uncertain'


def stock_copy(cp):
  with structs.CarParams.from_bytes(cp.to_bytes()) as reader:
    stock = reader.as_builder()
  stock.openpilotLongitudinalControl = False
  stock.pcmCruise = True
  if cp.openpilotLongitudinalControl and stock.carFingerprint in (
      CAR.KIA_CARNIVAL_4TH_GEN, CAR.KIA_CARNIVAL_2025,
      CAR.KIA_CARNIVAL_HEV_4TH_GEN, CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN):
    for config in stock.safetyConfigs:
      config.safetyParam &= ~0x2000
      if stock.carFingerprint in (CAR.KIA_CARNIVAL_2025, CAR.KIA_CARNIVAL_HEV_4TH_GEN) and stock.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        config.safetyParam |= 0x2000
  for config in stock.safetyConfigs:
    if stock.carFingerprint == CAR.GENESIS_GV60_EV_1ST_GEN and config.safetyParam in (0x9015, 0x9095):
      config.safetyParam &= ~0x9000
    config.safetyParam &= ~HyundaiSafetyFlags.LONG.value
  return stock


class HyundaiECUStartup:
  def __init__(self, cp, callbacks, *, address, bus, label):
    self.address, self.bus, self.label = address, bus, label
    self.cp = cp
    self.callbacks = callbacks
    self.outcome = None
    self.session_attempted = False
    self.disable_attempted = False
    self.restored = False
    self.admission = None
    self.floor_ns = 0
    self.published = False
    self.ci = None
    self.closed = False
    self.ready = False
    self.prepared_cp = None

  def communication_control_request(self):
    return b'\x28\x83\x01'

  def _disable(self, can_recv, can_send, *, bus, addr, com_cont_req):
    self.disable_response = None
    for _ in range(10):
      try:
        session = self._query(can_send, can_recv, bus, [(addr, None)], [b"\x10\x03"], [b"\x50\x03"])
        if not session.get_data(.1):
          time.sleep(.1)
          continue
        time.sleep(.05)
        command = self._query(can_send, can_recv, bus, [(addr, None)], [com_cont_req], [b""])
        replies = command.get_data(.1)
        if not replies:
          self.disable_response = ('suppressed_reply_sent_unconfirmed' if com_cont_req[1] & 0x80 else 'reply_absent_sent_unconfirmed')
          return True
        for payload in replies.values():
          if payload == bytes((0x68, com_cont_req[1] & 0x7f)):
            self.disable_response = 'positive_communication_control'
          elif len(payload) == 3 and payload[:2] == b"\x7f\x28":
            self.disable_response = 'negative_communication_control'
            return False
          else:
            self.disable_response = 'unexpected_or_malformed_reply'
            return False
        return True
      except Exception:
        if self.disable_attempted:
          return False
        time.sleep(.1)
    return False

  def _query(self, *args, **kwargs):
    return IsoTpParallelQuery(*args, **kwargs)

  def _warm_stock(self, ci):
    raise NotImplementedError

  def _send(self, frames):
    # The exact3-byte UDS request fits an ISO-TP single frame. Mark before
    # invoking transport: a callback exception cannot prove no ECU mutation.
    for address, data, bus in frames:
      if address == self.address and bus == self.bus and len(data) >= 3 and data[:3] == b'\x02\x10\x03':
        self.session_attempted = True
      if address == self.address and bus == self.bus and len(data) >= 4 and data[:4] == b'\x03' + self.communication_control_request():
        self.disable_attempted = True
    self.callbacks[1](frames)

  def _restore(self):
    admission = cast(Callable[[], bool], self.admission)
    if not admission():
      return False
    recv, send = self.callbacks
    if self.disable_attempted:
      session = self._query(send, recv, self.bus, [(self.address, None)], [b'\x10\x03'], [b'\x50\x03'])
      session_payload = session.get_data(.1).get((self.address, None))
      if session_payload is None or len(session_payload) not in (0, 4):
        return False
      time.sleep(.05)
      restore = self._query(send, recv, self.bus, [(self.address, None)], [b'\x28\x00\x01'], [b'\x68\x00'])
      if restore.get_data(.1).get((self.address, None)) != b'':
        return False
    normal = self._query(send, recv, self.bus, [(self.address, None)], [b'\x10\x01'], [b'\x50\x01'])
    normal_payload = normal.get_data(.1).get((self.address, None))
    if normal_payload is None or len(normal_payload) not in (0, 4):
      return False
    self.restored = True
    self.floor_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    return True

  def prepare(self, *, admission):
    self.admission = admission
    self.floor_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    if not admission():
      self.outcome = Outcome.STOCK_UNTOUCHED
      self.cp = stock_copy(self.cp)
      self.prepared_cp = deepcopy(self.cp.to_dict())
      return self.cp
    try:
      success = self._disable(self.callbacks[0], self._send, bus=self.bus, addr=self.address, com_cont_req=self.communication_control_request())
    except Exception:
      success = False
    if success and self.disable_attempted:
      self.outcome = Outcome.SENT_UNCONFIRMED
      self.prepared_cp = deepcopy(self.cp.to_dict())
      return self.cp
    if not (self.session_attempted or self.disable_attempted):
      self.outcome = Outcome.STOCK_UNTOUCHED
    else:
      try:
        restored = self._restore()
      except Exception:
        restored = False
      if not restored:
        self.outcome = Outcome.ABORT_UNCERTAIN
        raise RuntimeError(f'{self.label} disable failed; stock ECU restoration unverified')
      self.outcome = Outcome.STOCK_RESTORED
    self.cp = stock_copy(self.cp)
    self.prepared_cp = deepcopy(self.cp.to_dict())
    return self.cp

  def prepared_for(self, cp):
    return (self.outcome is not None and self.outcome is not Outcome.ABORT_UNCERTAIN and
            self.prepared_cp is not None and cp.to_dict() == self.prepared_cp)

  def configure(self, ci):
    if not self.prepared_for(ci.CP):
      raise RuntimeError(f'{self.label} prepared startup does not match constructed CP')
    self.ci = ci
    if self.outcome in (Outcome.PENDING, Outcome.SENT_UNCONFIRMED):
      self.ready = True
      return  # Preserve ordinary controller commands and1Hz tester cadence.
    self._warm_stock(ci)

  def finalize_aol_configuration(self, ci):
    """Finalize only the supported stock AOL marker before publication."""
    if self.published or self.closed or ci is not self.ci or not self.ready:
      raise RuntimeError('Hyundai AOL finalization requires configured startup')
    if self.prepared_for(ci.CP):
      return
    from opendbc.car.hyundai.ev6_aol import qualified as ev6_aol_qualified
    from opendbc.car.hyundai.canfd_stock_aol import qualified_long
    from opendbc.car.hyundai.gv70_startup import eligible as gv70_eligible
    captured_long = ev6_aol_qualified(ci.CP, marked_only=True) or (
      gv70_eligible(ci.CP) and qualified_long(ci.CP, marked_only=True))
    if self.outcome is Outcome.SENT_UNCONFIRMED and captured_long:
      expected = deepcopy(cast(PreparedParams, self.prepared_cp))
      configs = expected.get('safetyConfigs', [])
      owner = config_index(ci.CP)
      prepared_word = 0x2015 if gv70_eligible(ci.CP) else 0x15
      if owner is None or len(configs) != owner + 1 or configs[owner]['safetyParam'] != prepared_word or expected['alternativeExperience'] != 0:
        raise RuntimeError('Captured ADRV prepared LONG profile cannot be finalized')
      configs[owner]['safetyParam'] |= 0x0800
      expected['alternativeExperience'] = 32
      if ci.CP.to_dict() != expected:
        raise RuntimeError('Captured ADRV AOL finalization changed unrelated CarParams')
      self.prepared_cp = expected
      return
    from opendbc.car.hyundai.canfd_stock_aol import qualified as stock_aol_qualified
    if (self.outcome not in (Outcome.STOCK_UNTOUCHED, Outcome.STOCK_RESTORED) or
        not stock_aol_qualified(ci.CP, marked_only=True)):
      raise RuntimeError('Hyundai startup does not admit this AOL configuration')
    expected = deepcopy(cast(PreparedParams, self.prepared_cp))
    configs = expected.get('safetyConfigs', [])
    owner = config_index(ci.CP)
    if owner is None or len(configs) != owner + 1 or configs[owner]['safetyParam'] not in (0x11, 0x91):
      raise RuntimeError('Hyundai startup stock profile cannot be finalized')
    configs[owner]['safetyParam'] |= 0x0800
    if ci.CP.to_dict() != expected:
      raise RuntimeError('Hyundai AOL finalization changed unrelated CarParams')
    self.prepared_cp = expected

  def seal_publication(self):
    self.check()
    if self.ci is None or not self.ready:
      raise RuntimeError(f'{self.label} startup requires configure before publication')
    if not self.prepared_for(self.ci.CP):
      raise RuntimeError(f'{self.label} CP differs from its prepared startup decision')
    self.published = True

  def check(self):
    if self.outcome is None or self.outcome is Outcome.ABORT_UNCERTAIN:
      raise RuntimeError('Unresolved Hyundai startup')

  def close(self):
    if self.closed:
      if self.outcome is Outcome.ABORT_UNCERTAIN:
        raise RuntimeError(f'{self.label} cleanup remains unverified')
      return
    self.closed = True
    if (self.session_attempted or self.disable_attempted) and not self.published and not self.restored:
      try:
        restored = self._restore()
      except Exception:
        restored = False
      if not restored:
        self.outcome = Outcome.ABORT_UNCERTAIN
        raise RuntimeError(f'{self.label} unpublished startup cleanup could not confirm stock restoration')

  def after_state(self, **context):
    pass

  def consume_cruise_resume(self):
    return None

  def before_control(self, **context):
    self.check()
    return True

  def sources_current(self, ci, now_ns):
    return True

  def maintain(self, *, configured):
    pass

  def sent(self, frames, *, valid):
    pass
