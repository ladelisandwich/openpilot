from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_ti_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.carlog import carlog
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.hybrid import HybridArbiter, HybridRadarManager, MrccResync, RadarMaster, ce_status_is_experimental
from opendbc.car.mazda.longitudinal import CAM_BUS, LONG_COMMAND_STEP, NEAR_STOP_ENTRY_SPEED, RADAR_BUS, \
                                           RADAR_HEARTBEAT_STEP, TESTER_PRESENT_STEP, accel_cmd_to_accel, \
                                           create_longitudinal_messages, create_radar_heartbeat_messages, \
                                           create_radar_tester_present, hold_brake_accel, hold_latched_accel, \
                                           near_stop_brake_accel, update_captured_radar_frames
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaSafetyFlags
from openpilot.common.realtime import DT_CTRL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# radar emulation stop-and-go phase timings
CRZ_CTRL_LATCH_FRAMES = int(round(2.0 / DT_CTRL))
CRZ_CTRL_PASSIVE_FRAMES = int(round(9.6 / DT_CTRL))
CRZ_CTRL_RESUME_REACTIVATE_FRAMES = int(round(0.08 / DT_CTRL))
CRZ_INFO_RESUME_PHASE_FRAMES = int(round(0.20 / DT_CTRL))
HOLD_REQUEST_FRAMES = int(round(6.0 / DT_CTRL))
RESUME_RELEASE_FRAMES = int(round(0.5 / DT_CTRL))

# hybrid long (see hybrid.py)
HYBRID_MODE_READ_STEP = 10                          # experimental-mode state is read at 10 Hz
HYBRID_HANDOVER_FRAMES = int(round(1.5 / DT_CTRL))  # blend from MRCC's last command after a takeover
HYBRID_HANDOVER_RC = 0.3                            # s, time constant of that blend
HYBRID_HANDOVER_ACCEL = (-3.5, 2.0)                 # clip on the blend's starting point

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.ti_apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.ccp = CarControllerParams(CP)
    self.hold_timer_frame = 0
    self.hold_delay_frame = 0
    self.resume_timer_frame = 0
    self.acc_filter = FirstOrderFilter(0.0, .1, DT_CTRL, initialized=False)
    self.filtered_acc_last = 0
    self.long_active_last = False
    # radar emulation state
    self.long_counter = 0
    self.radar_counter = 0
    self.standstill_hold_frames = 0
    self.stop_intent_latched = False
    self.resume_release_frames = 0
    self.resume_crz_latched_frames = 0
    self.resume_phase_frames = 0
    self.resume_ctrl_active_prev = False
    self.virtual_resume_sent_latched = False
    self.resume_button_prev = False
    self.radar_suppress_failed = None
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    # Send phases for the replacement radar frames. Always 0 except in hybrid long, where a
    # takeover restarts them so the first frames go out the cycle the stock radar falls silent.
    self.long_phase = 0
    self.heartbeat_phase = 0
    # hybrid long state (see hybrid.py)
    self.hybrid = bool(CP.flags & MazdaSafetyFlags.HYBRID_LONG) and bool(CP.flags & MazdaSafetyFlags.RADAR_EMULATION)
    self.hybrid_arbiter = HybridArbiter()
    self.hybrid_radar = HybridRadarManager()
    self.mrcc_resync = MrccResync()
    self.hybrid_experimental = False
    self.hybrid_engaged_prev = False
    self.handover_filter = FirstOrderFilter(0.0, HYBRID_HANDOVER_RC, DT_CTRL)
    self.handover_frames = 0

  def _reset_emulation_state(self):
    self.standstill_hold_frames = 0
    self.stop_intent_latched = False
    self.resume_release_frames = 0
    self.resume_crz_latched_frames = 0
    self.resume_phase_frames = 0
    self.resume_ctrl_active_prev = False
    self.virtual_resume_sent_latched = False
    self.resume_button_prev = False

  def _hybrid_experimental(self, starpilot_toggles) -> bool:
    if getattr(starpilot_toggles, "conditional_experimental_mode", False):
      return ce_status_is_experimental(self.params_memory.get_int("CEStatus"))
    return self.params.get_bool("ExperimentalMode")

  def _update_hybrid(self, CC, CS, starpilot_toggles, can_sends) -> bool:
    """Pick the ACC master for this cycle. Returns True while openpilot impersonates the radar."""
    witness = CS.radar_witness
    if witness is None:
      return False

    if self.frame % HYBRID_MODE_READ_STEP == 0:
      self.hybrid_experimental = self._hybrid_experimental(starpilot_toggles)

    # openpilot's engagement, not just the car's cruise: with openpilot disengaged and the car's
    # cruise still on, openpilot is busy cancelling that cruise, and nothing here should fight it.
    engaged = CC.enabled and CS.out.cruiseState.enabled
    if self.hybrid_engaged_prev and not engaged:
      carlog.warning({"event": "mazdaHybridDisengaged", "master": str(self.hybrid_radar.state),
                      "masterFrames": self.hybrid_radar.state_frames, "brake": CS.out.brakePressed,
                      "gas": CS.out.gasPressed, "vEgo": round(CS.out.vEgo, 2)})
    elif engaged and not self.hybrid_engaged_prev and self.hybrid_radar.state == RadarMaster.MRCC:
      # The driver set or resumed cruise themselves, so the stock radar is properly engaged again.
      self.mrcc_resync.note_engaged_by_driver()
    self.hybrid_engaged_prev = engaged

    want = self.hybrid_arbiter.update(self.hybrid_radar.emulating, engaged, self.hybrid_experimental, CS.out.standstill,
                                      CS.out.brakePressed, CS.out.gasPressed, CS.out.vEgo, CC.actuators.accel)
    fsc_ok = CS.fsc_settled or bool(self.CP.flags & MazdaSafetyFlags.NO_FSC)
    self.hybrid_radar.update(want, witness, CS.out.canValid, fsc_ok)
    if self.hybrid_radar.diagnostic is not None:
      can_sends.append(self.hybrid_radar.diagnostic)

    if self.hybrid_radar.just_took_over:
      # Pick up exactly where the stock radar stopped: its own latest heartbeat frames and
      # counters, the first replacement frames this cycle, and its last command as the start
      # of the blend into openpilot's.
      update_captured_radar_frames(witness.heartbeat_frames())
      if witness.crz_info_counter is not None:
        self.long_counter = (witness.crz_info_counter + 1) % 16
      if witness.heartbeat_counter is not None:
        self.radar_counter = (witness.heartbeat_counter + 1) % 16
      self.long_phase = self.frame % LONG_COMMAND_STEP
      self.heartbeat_phase = self.frame % RADAR_HEARTBEAT_STEP
      stock_cmd = witness.stock_accel_cmd
      start = accel_cmd_to_accel(stock_cmd, CS.out.vEgo) if stock_cmd is not None else CC.actuators.accel
      self.handover_filter.x = min(max(start, HYBRID_HANDOVER_ACCEL[0]), HYBRID_HANDOVER_ACCEL[1])
      self.handover_filter.initialized = True
      self.handover_frames = HYBRID_HANDOVER_FRAMES
      self._reset_emulation_state()
      self.hybrid_arbiter.note_takeover()
      carlog.warning({"event": "mazdaHybridTakeover", "engaged": engaged, "stockAccelCmd": stock_cmd,
                      "startAccel": round(self.handover_filter.x, 3), "vEgo": round(CS.out.vEgo, 2)})
    if self.hybrid_radar.just_handed_back:
      self.handover_frames = 0
      carlog.warning({"event": "mazdaHybridHandedBack", "engaged": engaged,
                      "mrccDriving": witness.mrcc_driving, "crzActive": witness.stock_cruise_active,
                      "stockAccelCmd": witness.stock_accel_cmd, "vEgo": round(CS.out.vEgo, 2)})
      self.mrcc_resync.note_handback()

    # The radar sleeps through emulation, so after a hand-back it comes back in standby: cruise
    # still reads engaged but nothing brakes. Nudge it with RES; if it will not take over, keep
    # the radar instead of leaving the car with throttle and no brakes.
    self.mrcc_resync.update(self.hybrid_radar.state == RadarMaster.MRCC, engaged, witness,
                            CS.out.brakePressed, CS.out.gasPressed, CS.out.vEgo)
    self.hybrid_arbiter.mrcc_unavailable = self.mrcc_resync.failed

    return self.hybrid_radar.transmitting

  def update(self, CC, CS, now_nanos, starpilot_toggles):
    can_sends = []

    apply_torque = 0
    ti_apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * self.ccp.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.ccp)
      if self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR:
        if CS.ti_lkas_allowed:
          ti_new_torque = int(round(CC.actuators.torque * self.ccp.TI_STEER_MAX))
          ti_apply_torque = apply_ti_steer_torque_limits(ti_new_torque, self.ti_apply_torque_last,
                                                    CS.out.steeringTorque, self.ccp)

    self.apply_torque_last = apply_torque
    self.ti_apply_torque_last = ti_apply_torque

    if self.CP.flags & MazdaSafetyFlags.GEN1:
      if self.hybrid:
        # Hybrid long: stock MRCC or radar emulation, re-decided every cycle. While MRCC is the
        # master this runs the plain stock-long path below (cancel spam, RES spam from a stop).
        radar_emulation = self._update_hybrid(CC, CS, starpilot_toggles, can_sends)
      else:
        # Read once, after CarInterface.init() has run. If the radar refused the programming
        # session it is still transmitting, so never add our frames on top of it -- fall back
        # to stock MRCC for gas and brake.
        if self.radar_suppress_failed is None:
          self.radar_suppress_failed = self.params.get_bool("EcuDisableFailed")
        radar_emulation = bool(self.CP.flags & MazdaSafetyFlags.RADAR_EMULATION) and not self.radar_suppress_failed
      virtual_resume_sent = False

      if radar_emulation:
        # openpilot owns CRZ_CTRL here, so a cancel is done by dropping the synthetic
        # ACC-active state rather than spamming CRZ_BTNS. A RES press is still needed
        # to ask the chassis to release the HOLD latch after a full stop.
        self.brake_counter = 0
        if CS.out.standstill and CC.cruiseControl.resume and self.frame % 5 == 0:
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
          virtual_resume_sent = True
      elif CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      elif not radar_emulation:
        self.brake_counter = 0
        # Two reasons to press RES while the stock radar is the master: Mazda Stop and Go needs one
        # (or the gas) after a stop of more than 3 seconds, and a radar that came back from a
        # hand-back in standby needs one to start driving again (see hybrid.MrccResync).
        if (CC.cruiseControl.resume or self.mrcc_resync.press_resume) and self.frame % 5 == 0:
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

      if radar_emulation:
        stopping = CC.actuators.longControlState == LongCtrlState.stopping
        starting = CC.actuators.longControlState == LongCtrlState.starting
        # Do not treat tiny positive low-speed PID noise as a real restart request.
        # Only the explicit upstream starting phase releases the synthetic HOLD clamp.
        restart_requested = starting
        # Physical wheel RES survives radar suppression on CRZ_BTNS. For virtual RES,
        # do not start the synthetic unlatch path until the first RES frame has
        # actually been transmitted on the bus.
        if not CC.cruiseControl.resume or not CS.out.standstill:
          self.virtual_resume_sent_latched = False
        elif virtual_resume_sent:
          self.virtual_resume_sent_latched = True
        physical_resume_requested = bool(CS.accel_button)
        virtual_resume_requested = CC.cruiseControl.resume and self.virtual_resume_sent_latched
        effective_resume_requested = False
        release_hold_requested = False
        release_brake = False
        stop_go_release_requested = False
        if not CC.longActive:
          self.standstill_hold_frames = 0
          self.stop_intent_latched = False
          self.resume_release_frames = 0
          self.resume_crz_latched_frames = 0
          self.resume_phase_frames = 0
          self.resume_ctrl_active_prev = False
          self.virtual_resume_sent_latched = False
        else:
          if stopping:
            self.stop_intent_latched = True

          hold_latched_ready = CS.out.standstill and self.standstill_hold_frames > HOLD_REQUEST_FRAMES
          # A physical wheel RES is an explicit driver request, so honour it immediately.
          # With the radar silenced nothing else acts on the CRZ_BTNS press, and gating it
          # behind the 6s latch phase made the button dead for exactly the short stops
          # where it is wanted. Virtual RES keeps the latch requirement so a transient
          # shouldStop flicker cannot release the hold early.
          physical_resume_unlatch_requested = CS.out.standstill and physical_resume_requested
          virtual_resume_unlatch_requested = CS.out.standstill and virtual_resume_requested and hold_latched_ready
          resume_unlatch_requested = physical_resume_unlatch_requested or virtual_resume_unlatch_requested
          effective_resume_requested = resume_unlatch_requested
          resume_rising_edge = effective_resume_requested and not self.resume_button_prev
          release_brake = self.resume_release_frames > 0
          # Mirrors the stock-radar release condition: the planner asking to move is
          # sufficient on its own. HOLD_REQUEST_FRAMES governs how long HOLD is applied,
          # not whether a release is permitted -- conflating the two is what left the car
          # latched until the driver touched the gas.
          planner_release_requested = (CC.cruiseControl.override or CS.out.gasPressed or restart_requested or
                                       (CC.cruiseControl.resume and not stopping))
          base_release_hold_requested = planner_release_requested or release_brake

          if CS.out.standstill and self.stop_intent_latched and not base_release_hold_requested:
            self.standstill_hold_frames += 1
          else:
            self.standstill_hold_frames = 0

          if CS.out.standstill and not base_release_hold_requested and resume_rising_edge and self.standstill_hold_frames >= CRZ_CTRL_PASSIVE_FRAMES:
            # Stock briefly re-enables ACC in the latched-hold profile when RES is first
            # pressed, then drops back into the active stop-go profile.
            self.resume_crz_latched_frames = CRZ_CTRL_RESUME_REACTIVATE_FRAMES
          elif self.resume_crz_latched_frames > 0:
            self.resume_crz_latched_frames -= 1

          # Drive the synthetic unlatch sequence from the planner too, not just RES.
          if resume_unlatch_requested or (planner_release_requested and self.stop_intent_latched and CS.out.standstill):
            self.resume_release_frames = RESUME_RELEASE_FRAMES
          elif self.resume_release_frames > 0:
            self.resume_release_frames -= 1

          release_brake = self.resume_release_frames > 0
          release_hold_requested = base_release_hold_requested or resume_unlatch_requested or release_brake
          stop_go_release_requested = self.stop_intent_latched and release_hold_requested

          if release_hold_requested or (not CS.out.standstill and not stopping and CS.out.vEgo > NEAR_STOP_ENTRY_SPEED):
            self.stop_intent_latched = False

        # Only enter the synthetic stop-go/HOLD path once upstream has actually
        # committed to a stop; keep it latched through standstill until a real
        # restart or a driver override releases it.
        stop_go_request = CC.longActive and self.stop_intent_latched and not release_hold_requested
        standstill_hold_request = stop_go_request and CS.out.standstill
        hold_latched = standstill_hold_request and self.standstill_hold_frames > HOLD_REQUEST_FRAMES
        brake_release_requested = release_hold_requested or effective_resume_requested

        crz_hold_latched = standstill_hold_request and self.standstill_hold_frames >= CRZ_CTRL_LATCH_FRAMES and \
                           (not effective_resume_requested or self.resume_crz_latched_frames > 0)
        # Stock resumes from passive hold by re-enabling ACC while RES is held, instead
        # of sitting indefinitely in the passive-hold substate.
        crz_hold_passive = standstill_hold_request and self.standstill_hold_frames >= CRZ_CTRL_PASSIVE_FRAMES and not effective_resume_requested
        release_brake = self.resume_release_frames > 0
        crz_ctrl_resume_active = release_brake and CS.out.vEgo < self.CP.vEgoStarting and not crz_hold_latched and not crz_hold_passive
        if crz_ctrl_resume_active:
          if not self.resume_ctrl_active_prev:
            self.resume_phase_frames = CRZ_INFO_RESUME_PHASE_FRAMES
          elif self.resume_phase_frames > 0:
            self.resume_phase_frames -= 1
        else:
          self.resume_phase_frames = 0
        crz_info_resume_unlatching = crz_ctrl_resume_active and self.resume_phase_frames > 0
        self.resume_ctrl_active_prev = crz_ctrl_resume_active
        # Keep CRZ_INFO stop bits cleared through the whole synthetic brake-release
        # window, or Mazda sees positive accel while we still advertise an active stop.
        crz_info_hold_request = stop_go_request and not (brake_release_requested or release_brake)

        accel = 0.0
        # hybrid takeover: blend from the stock radar's last command into openpilot's (never set
        # outside hybrid long)
        handover_accel = None
        if self.handover_frames > 0:
          self.handover_frames -= 1
          handover_accel = self.handover_filter.update(CC.actuators.accel)
        if CC.longActive:
          accel = CC.actuators.accel
          if handover_accel is not None and not CS.out.standstill:
            accel = handover_accel
          if release_brake:
            accel = max(accel, 0.0)
          elif CS.out.standstill:
            accel = hold_latched_accel() if hold_latched else hold_brake_accel()
          elif self.stop_intent_latched and not release_hold_requested and (stopping or CS.out.vEgo < NEAR_STOP_ENTRY_SPEED):
            accel = min(accel, near_stop_brake_accel(CS.out.vEgo))

        # hold the radar in its programming session so it stays silent (hybrid: the radar
        # manager owns all UDS traffic, including this)
        if not self.hybrid and self.frame % TESTER_PRESENT_STEP == 0:
          can_sends.append(create_radar_tester_present(RADAR_BUS))

        lead_visible = CC.hudControl.leadVisible
        synthetic_radar_lead = CC.longActive and (lead_visible or stop_go_request or standstill_hold_request or hold_latched or
                                                  crz_hold_latched or crz_hold_passive or crz_ctrl_resume_active or
                                                  stop_go_release_requested or release_brake or starting)
        if (self.frame - self.heartbeat_phase) % RADAR_HEARTBEAT_STEP == 0:
          for bus in (RADAR_BUS, CAM_BUS):
            can_sends.extend(create_radar_heartbeat_messages(bus, self.radar_counter, synthetic_lead=synthetic_radar_lead))
          self.radar_counter = (self.radar_counter + 1) % 16

        if (self.frame - self.long_phase) % LONG_COMMAND_STEP == 0:
          for bus in (RADAR_BUS, CAM_BUS):
            can_sends.extend(create_longitudinal_messages(bus, accel, self.long_counter,
                                                          CC.longActive, lead_visible,
                                                          hold_request=crz_info_hold_request,
                                                          crz_ctrl_hold_request=stop_go_request,
                                                          hold_latched=hold_latched,
                                                          crz_hold_latched=crz_hold_latched,
                                                          crz_hold_passive=crz_hold_passive,
                                                          crz_resume_active=crz_ctrl_resume_active,
                                                          crz_info_resume_unlatching=crz_info_resume_unlatching,
                                                          crz_available=CS.out.cruiseState.available,
                                                          v_ego=CS.out.vEgo))
          self.long_counter = (self.long_counter + 1) % 16
        self.resume_button_prev = effective_resume_requested
      elif self.CP.openpilotLongitudinalControl and self.CP.flags & MazdaSafetyFlags.RADAR_INTERCEPTOR:
        hold = False
        if CS.out.standstill:
          hold = (self.frame - self.hold_timer_frame) < 600
        else:
          self.hold_timer_frame = self.frame

          stock_acc = CS.crz_info["ACCEL_CMD"]
          op_acc = CC.actuators.accel * 1150
          op_acc = max(-1000, min(op_acc, 1000))

          if self.params.get_bool("BlendedACC"):
            if CC.longActive:
              if not self.long_active_last:
                self.acc_filter.initialized = False
              target_acc = op_acc if self.params_memory.get_int("CEStatus") != 0 else stock_acc
              acc_output = self.acc_filter.update(target_acc)
            else:
              acc_output = stock_acc
          else:
            acc_output = op_acc

          acc_output = max(-1000, min(acc_output, 1000))
          CS.crz_info["ACCEL_CMD"] = acc_output

        if self.frame % 2 == 0:
          can_sends.extend(mazdacan.create_radar_command(self.packer, self.frame, CC.longActive, CS, hold))

    elif self.CP.flags & MazdaSafetyFlags.GEN2:
      if self.CP.openpilotLongitudinalControl:
        stock_acc = CS.acc["ACCEL_CMD"]
        op_acc = (CC.actuators.accel * 200) + 2000

        if self.params.get_bool("BlendedACC"):
          if CC.longActive:
            if not self.long_active_last:
              self.acc_filter.initialized = False
            target_acc = op_acc if self.params_memory.get_int("CEStatus") != 0 else stock_acc
            acc_output = self.acc_filter.update(target_acc)
          else:
            acc_output = stock_acc
        else:
          acc_output = op_acc if CC.longActive else stock_acc

        CS.acc["ACCEL_CMD"] = acc_output

      resume = False
      hold = False
      if self.frame % 2 == 0: # send ACC command at 50hz
        """
        Without this hold/resum logic, the car will only stop momentarily.
        It will then start creeping forward again. This logic allows the car to
        apply the electric brake to hold the car. The hold delay also fixes a
        bug with the stock ACC where it sometimes will apply the brakes too early
        when coming to a stop.
        """
        if CS.out.standstill: # if we're stopped
          if not ((self.frame - self.hold_delay_frame) < 50): # and we have been stopped for more than hold_delay duration. This prevents a hard brake if we aren't fully stopped.
            if ((CC.cruiseControl.resume and CC.actuators.longControlState != LongCtrlState.stopping) or
                CC.cruiseControl.override or CS.out.gasPressed or
                (CC.actuators.longControlState == LongCtrlState.starting) or CS.acc["RESUME"]): # if we are resuming or overriding, we want to release the brake
              self.resume_timer_frame = self.frame # reset the resume timer so its active
            else: # otherwise we're holding
              hold = (self.frame - self.hold_timer_frame) < 600 # hold for 6s. This allows the electric brake to hold the car.

        else: # if we're moving
          self.hold_timer_frame = self.frame # reset the hold timer so its active when we stop
          self.hold_delay_frame = self.frame # reset the hold delay

        resume = (self.frame - self.resume_timer_frame) < 50 # stay on for 0.5s to release the brake. This allows the car to move.
        can_sends.append(mazdacan.create_acc_cmd(self.packer, CS.acc, hold, resume))


    # send steering command
    can_sends.extend(mazdacan.create_steering_control(
      self.packer, self.CP, self.frame, apply_torque, CS.cam_lkas,
      ti_apply_torque if self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR else None))

    new_actuators = CC.actuators.as_builder()
    # Report the torque of whichever actuator is actually steering the car. controlsd
    # flags steer_limited_by_safety from |requested - applied|, so reporting the stock
    # EPS channel while the TI does the work makes openpilot think it is being limited
    # the moment the EPS clamps -- which on GEN1 is exactly when the TI is needed. That
    # raised "Take Control, Turn Exceeds Steering Limit" on sharp low-speed turns even
    # though the TI was tracking the request fine.
    if self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR and CS.ti_lkas_allowed:
      new_actuators.torque = ti_apply_torque / self.ccp.TI_STEER_MAX
      new_actuators.torqueOutputCan = ti_apply_torque
    else:
      new_actuators.torque = apply_torque / self.ccp.STEER_MAX
      new_actuators.torqueOutputCan = apply_torque

    self.long_active_last = CC.longActive
    self.frame += 1
    return new_actuators, can_sends