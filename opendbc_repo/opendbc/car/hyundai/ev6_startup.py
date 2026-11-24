"""Exact first-generation EV6 startup compatibility wrapper."""
from copy import deepcopy
import time

from opendbc.car import make_tester_present_msg
from opendbc.car.hyundai.ecu_startup import HyundaiECUStartup, Outcome
from opendbc.car.hyundai.canfd_owner import config_index
from opendbc.car.hyundai.captured_adrv_startup import CapturedADRVStartup, first_generation
from opendbc.car.hyundai.ev6_template import EV6Template
from opendbc.car.hyundai.values import CAR


def eligible(cp):
  return cp.carFingerprint == CAR.KIA_EV6 and first_generation(cp)


def required(cp):
  return eligible(cp) and cp.openpilotLongitudinalControl


class EV6Startup(CapturedADRVStartup):
  def __init__(self, cp, callbacks):
    super().__init__(cp, callbacks, label='EV6', template_type=EV6Template)

  def prepare_when_controls_ready(self, *, admission):
    """Keep factory ADRV output until the inactive control producer is ready."""
    if self.outcome is not None or self.closed:
      raise RuntimeError('EV6 startup preparation is single-use')
    # The existing captured sender is qualified on physical A-CAN zero.
    if self.template_bus != 0:
      return self.prepare(admission=admission)
    if admission() is not True:
      return self.prepare(admission=lambda: False)
    self.template = self._capture()
    if self.template is None:
      return HyundaiECUStartup.prepare(self, admission=lambda: False)
    self.admission = admission
    self.floor_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    self.outcome = Outcome.PENDING
    self.prepared_cp = deepcopy(self.cp.to_dict())
    return self.cp

  def configure(self, ci):
    super().configure(ci)
    if self.outcome is Outcome.PENDING:
      if self.template is None:
        raise RuntimeError('EV6 pending startup requires its captured template')
      ci.CC.adrv_template = self.template

  def finalize_aol_configuration(self, ci):
    if self.outcome is not Outcome.PENDING:
      return super().finalize_aol_configuration(ci)
    if self.published or self.closed or ci is not self.ci or not self.ready:
      raise RuntimeError('EV6 pending AOL finalization requires configured startup')
    if self.prepared_for(ci.CP):
      return
    from opendbc.car.hyundai.ev6_aol import qualified
    expected = deepcopy(self.prepared_cp)
    index = config_index(ci.CP)
    if (not qualified(ci.CP, marked_only=True) or expected is None or index is None or
        len(expected['safetyConfigs']) != index + 1 or
        expected['safetyConfigs'][index]['safetyParam'] != 0x15 or expected['alternativeExperience'] != 0):
      raise RuntimeError('EV6 pending startup cannot relabel another safety profile')
    expected['safetyConfigs'][index]['safetyParam'] |= 0x0800
    expected['alternativeExperience'] = 32
    if ci.CP.to_dict() != expected:
      raise RuntimeError('EV6 pending AOL configuration changed unrelated CarParams')
    self.prepared_cp = expected

  def sources_current(self, ci, now_ns):
    if self.outcome is Outcome.PENDING:
      return self._sources(ci, now_ns, floor=self.floor_ns)
    return super().sources_current(ci, now_ns)

  def before_control(self, **context):
    return self.outcome is not Outcome.PENDING and super().before_control(**context)

  def commit_pending(self, ci, *, sources_current, control_current):
    if (self.outcome is not Outcome.PENDING or self.closed or not self.published or
        ci is not self.ci or not self.prepared_for(ci.CP) or not sources_current or
        not control_current or self.admission is None or self.admission() is not True):
      raise RuntimeError('EV6 takeover requires current diagnostic, source and inactive control admission')
    template = self._capture()
    if template is None or not self.sources_current(ci, time.clock_gettime_ns(time.CLOCK_BOOTTIME)) or self.admission() is not True:
      raise RuntimeError('EV6 takeover lost its current factory capture or source admission')
    self.template = ci.CC.adrv_template = template
    try:
      success = self._disable(self.callbacks[0], self._send, bus=self.bus, addr=self.address,
                              com_cont_req=self.communication_control_request())
      if not success or not self.disable_attempted:
        raise RuntimeError('EV6 pending ECU disable was not sent successfully')
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
        raise RuntimeError('EV6 pending takeover failed; stock restoration unverified') from None
      raise
