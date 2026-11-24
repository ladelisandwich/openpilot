from dataclasses import dataclass
import numpy as np

from opendbc.car.hyundai.ev9_longitudinal import (
  Ioniq6LongitudinalTuningState, EV9LongitudinalTuningState, LongCtrlState, BlindspotWarningState,
  update_ioniq_6_longitudinal_tuning, update_ev9_longitudinal_tuning, reset_egmp_longitudinal_tuning,
)
from opendbc.car import structs


START_ACCEL = 1.0
START_SPEED = 0.4
LONGITUDINAL_DELAY = 0.35
STOPPING_RATE = 0.4
ACCEL_STEP = (6.0 / 50.0) * 1.2
DECEL_STEP = (15.0 / 50.0) * 1.2
DIRECT_STOP_SPEED = 1.2


@dataclass(frozen=True)
class Request:
  accel: float
  stopping: bool
  jerk_upper: float
  jerk_lower: float


class PELongitudinalPolicy:
  def __init__(self):
    self.tuning = Ioniq6LongitudinalTuningState()

    self.stop = EV9LongitudinalTuningState()
    self.left_warning = BlindspotWarningState()
    self.right_warning = BlindspotWarningState()

  def update(self, frame, *, active, override, accel, speed, measured_accel, control_state, last_sent_accel):
    if not active or override or control_state not in (LongCtrlState.starting, LongCtrlState.pid, LongCtrlState.stopping):
      self.tuning = Ioniq6LongitudinalTuningState(long_control_state_last=control_state)
      return Request(0.0, False, 1.0, 1.0)
    if self.stop.stop_request:
      reset_egmp_longitudinal_tuning(self.tuning)
    state = self.tuning
    if frame % 5 == 0:
      update_ioniq_6_longitudinal_tuning(state, accel, speed, measured_accel, control_state, True,
                                       low_speed_stop_brake_cap=True, ev9=False)
    smoothed = accel >= state.actual_accel or state.launch_active or state.stopping
    if state.stopping and speed > DIRECT_STOP_SPEED and accel < state.actual_accel:
      smoothed = False
    stopping = control_state == LongCtrlState.stopping
    if smoothed:
      output, stopping = state.actual_accel, state.stopping
    else:
      output = float(np.clip(accel, last_sent_accel - DECEL_STEP, last_sent_accel + ACCEL_STEP))
      state.desired_accel = accel
      state.actual_accel = state.accel_last = output
      state.jerk_upper, state.jerk_lower = 3.0, 5.0
      state.launch_active = False
      state.stopping = stopping
      state.long_control_state_last = control_state

    return Request(float(np.clip(output, -3.5, 2.0)), stopping, state.jerk_upper, state.jerk_lower)

  def update_stop(self, enabled, override, stopping, speed):
    self.stop = update_ev9_longitudinal_tuning(self.stop, enabled and not override, stopping, speed)
    if self.stop.stop_request or not enabled or override:
      reset_egmp_longitudinal_tuning(self.tuning)
      return 0.0
    return None

  def finish_scc(self, request, *, enabled, override, stopping, speed):
    self.stop = update_ev9_longitudinal_tuning(self.stop, enabled and not override, stopping, speed)
    if self.stop.stop_request or not enabled or override:
      reset_egmp_longitudinal_tuning(self.tuning)
      return Request(0.0, request.stopping, request.jerk_upper, request.jerk_lower)
    return request


def lateral_request_allowed(control, state, angle_fault):

  return (control.enabled and state.canValid and not state.canTimeout and not state.standstill and
          state.gearShifter == structs.CarState.GearShifter.drive and
          not (state.brakePressed or state.gasPressed or state.steerFaultTemporary or state.steerFaultPermanent or angle_fault))
