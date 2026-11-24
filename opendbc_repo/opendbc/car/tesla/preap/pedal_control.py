"""Dom VirtualDAS pedal control with frozen calibration and current axis authority."""
import math
from opendbc.car.tesla.preap.pedal_contract import PedalCodec, pedal_max
from opendbc.car.tesla.preap.pedal_dynamics import PedalZeroTorque
from opendbc.car.tesla.preap.virtual_das import VirtualDAS


class PreAPPedalControl:
  def __init__(self, calibration):
    self.codec = PedalCodec(calibration)
    self.zero_torque = PedalZeroTorque()
    self.vdas = VirtualDAS(zero_torque=self.zero_torque)
    self.previous_di = 0.
    self.was_requested = False
    self.engage_frame = -1_000_000

  def update(self, CC, CS, frame, now_ns):
    if frame % 2:
      return []
    feedback = CS.pedal_feedback
    healthy = feedback is not None and feedback.ready(now_ns)
    requested = bool(healthy and CS.cruiseEnabled and CS.enableLongControl)
    active = bool(requested and CC.enabled and CC.longActive and CS.out.canValid and not CS.out.canTimeout and
                  not CS.out.gasPressed and not CS.out.brakePressed and not CS.out.accFaulted and
                  not CS.out.doorOpen and not CS.out.steerFaultPermanent and not CS.out.steeringDisengage)
    if requested != self.was_requested:
      CS.preap_cc_cancel_needed = True
    if requested and not self.was_requested:
      self.engage_frame = frame
      self.previous_di = min(pedal_max(CS.out.vEgo), max(-5., feedback.value))
      self.vdas.reset(a_init=0., pedal_di_init=self.previous_di)
    self.was_requested = requested
    if not active:
      self.previous_di = 0.
      self.vdas.reset()
      return [self.codec.command(enabled=False)]
    values = (CC.actuators.accel, CS.out.vEgo, CS.out.aEgo, *CC.orientationNED)
    if not all(math.isfinite(float(value)) for value in values):
      self.previous_di = 0.
      self.vdas.reset()
      return [self.codec.command(enabled=False)]
    self.zero_torque.update(feedback.torque_level, self.previous_di, CS.out.vEgo)
    elapsed = frame - self.engage_frame
    grace = elapsed < 50
    demand = float(CC.actuators.accel)
    if grace:
      # Original50-control-frame positive engagement grace; no old generic tune rollback.
      demand = max(0., min(demand, max(0., CS.preap_accel_max) * elapsed / 50.))
    proposed = self.vdas.update(demand, CS.out.vEgo, self.previous_di, a_ego=CS.out.aEgo,
                                freeze_integrator=grace, orientation_ned=list(CC.orientationNED))
    # Apply hard domain last: original rate clamp could otherwise escape it
    # when an initial driver observation lay above the speed envelope.
    proposed = max(-5., min(pedal_max(CS.out.vEgo), proposed))
    self.previous_di = proposed
    if proposed <= -4.75 and not grace:
      CS.pccEvent = 'pedalMaxRegen'
    return [self.codec.command(proposed, enabled=True)]
