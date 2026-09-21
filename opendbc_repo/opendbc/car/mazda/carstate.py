import copy
from cereal import custom
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs, DT_CTRL
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, MazdaSafetyFlags, TI_STATE, CarControllerParams

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]
    if CP.flags & MazdaSafetyFlags.MANUAL_TRANSMISSION:
      self.shifter_values = can_define.dv["MANUAL_GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    self.cam_lkas = 0
    self.params = CarControllerParams(CP)

    self.distance_button = 0
    # physical RES / SET+ press on the wheel; used by the radar emulation HOLD release
    self.accel_button = 0
    # When openpilot owns the set speed (pcmCruiseSpeed=False) the wheel buttons have to
    # reach VCruiseHelper as button events. Only emit them for that mode -- adding button
    # events unconditionally would change engagement behaviour for everyone else.
    # Hybrid long shares one set speed between MRCC and emulation (the dash value), so
    # openpilot must not own the set speed there.
    self.low_min_set_speed = Params().get_bool("LowerMinSetSpeed") and not (CP.flags & MazdaSafetyFlags.HYBRID_LONG)
    self.speed_up_button = 0
    self.speed_down_button = 0
    # The FSC checks for the radar at cold boot; tearing it down inside that window latches
    # "Smart City Brake Support Malfunction". Gate on settled, error-free CAM_LANEINFO.
    self.fsc_settled_frames = 0
    self.cam_laneinfo_stale_frames = CarControllerParams.CAM_LANEINFO_FRESH_FRAMES
    self.cam_laneinfo_ts_last = 0
    # Raw-frame stock radar witness, set by CarInterface (see hybrid.RadarWitness). Radar
    # liveness must NOT be read through a CANParser: reading cp.vl["CRZ_INFO"] subscribes the
    # parser to it, and once the radar is silenced the parser marks the whole bus invalid
    # (canError, shown as "Unknown Vehicle Variant").
    self.radar_witness = None
    self.ti_ramp_down = False
    self.ti_version = 1
    self.ti_state = TI_STATE.RUN
    self.ti_violation = 0
    self.ti_error = 0
    self.ti_lkas_allowed = False

    self._prev_steering_angle = 0

  @property
  def fsc_settled(self) -> bool:
    return self.fsc_settled_frames >= CarControllerParams.FSC_SETTLE_FRAMES

  def update(self, can_parsers, starpilot_toggles) -> tuple[structs.CarState, custom.StarPilotCarState]:
    if self.CP.flags & (MazdaSafetyFlags.GEN2 | MazdaSafetyFlags.GEN3):
      ret = self.update_gen2(can_parsers)
      fp_ret = custom.StarPilotCarState.new_message()
      return ret, fp_ret
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_body = can_parsers[Bus.body]

    ret = structs.CarState()

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]

    # CX-9 has a dedicated RES button; some Mazdas emit SET_P for the wheel "+" instead
    self.accel_button = int(cp.vl["CRZ_BTNS"]["RES"] == 1 or cp.vl["CRZ_BTNS"]["SET_P"] == 1)
    prev_speed_up_button = self.speed_up_button
    prev_speed_down_button = self.speed_down_button
    self.speed_up_button = int(cp.vl["CRZ_BTNS"]["SET_P"] == 1)
    self.speed_down_button = int(cp.vl["CRZ_BTNS"]["SET_M"] == 1)

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    if self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR:
      ret.steeringTorque = cp_body.vl["TI_FEEDBACK"]["TI_TORQUE_SENSOR"]

      self.ti_version = cp_body.vl["TI_FEEDBACK"]["VERSION_NUMBER"]
      self.ti_state = cp_body.vl["TI_FEEDBACK"]["STATE"] # DISCOVER = 0, OFF = 1, DRIVER_OVER = 2, RUN=3
      self.ti_violation = cp_body.vl["TI_FEEDBACK"]["VIOL"] # 0 = no violation
      self.ti_error = cp_body.vl["TI_FEEDBACK"]["ERROR"] # 0 = no error
      if self.ti_version > 1:
        self.ti_ramp_down = (cp_body.vl["TI_FEEDBACK"]["RAMP_DOWN"] == 1)

      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.TI_STEER_THRESHOLD
      self.ti_lkas_allowed = not self.ti_ramp_down and self.ti_state == TI_STATE.RUN
    else:
      ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
      ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    # TODO: this should be from 0 - 1.
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    #       it should be used for carState.cruiseState.nonAdaptive instead
    ret.cruiseState.standstill = ret.standstill
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    if self.CP.flags & MazdaSafetyFlags.RADAR_EMULATION:
      # The stock radar is suppressed, so CRZ_CTRL is gone from the bus. Derive MRCC
      # state from PEDALS instead: ACC_OFF is asserted while MRCC is armed but not
      # controlling, ACC_ACTIVE once stock ACC takes over. Treating either as
      # "available" stops MADS reading a stock ACC engage as the main switch going off.
      acc_armed = cp.vl["PEDALS"]["ACC_OFF"] == 1
      acc_active = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      ret.cruiseState.available = acc_armed or acc_active
      ret.cruiseState.enabled = acc_active
    elif self.CP.flags & MazdaSafetyFlags.RADAR_INTERCEPTOR:
      self.crz_info = copy.copy(cp_cam.vl["CRZ_INFO"])
      self.crz_cntr = copy.copy(cp_cam.vl["CRZ_CTRL"])
      self.cp_cam = cp_cam
      ret.cruiseState.enabled = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      ret.cruiseState.available = cp.vl["PEDALS"]["CRZ_AVAILABLE"] == 1
    elif self.CP.flags & MazdaSafetyFlags.NO_MRCC:
      ret.cruiseState.enabled = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      ret.cruiseState.available = cp.vl["PEDALS"]["CRZ_AVAILABLE"] == 1
    else:
      ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1

    # stock lkas should be on
    # TODO: is this needed?
    if not self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR:
      if ret.cruiseState.enabled:
        if not self.lkas_allowed_speed and self.acc_active_last:
          self.low_speed_alert = True
        else:
          self.low_speed_alert = False
      ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked and not self.ti_lkas_allowed

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    if not self.CP.flags & MazdaSafetyFlags.NO_FSC:
      ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0
      self.lkas_disabled = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0 if not self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR else False
      self.cam_lkas = cp_cam.vl["CAM_LKAS"]
      self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]

      # FSC settle gate. BIT2 is deliberately excluded: it can stay high for a whole ignition
      # cycle without indicating an incomplete boot. A camera dropout restarts the timer, and
      # the gate starts closed because an unpopulated parser reads all-zero and would
      # otherwise look settled.
      laneinfo = cp_cam.vl["CAM_LANEINFO"]
      # ts_nanos is 0 until a real frame arrives, so this also keeps the gate closed before
      # the first camera frame -- an unpopulated parser reads all-zero and would look settled.
      cam_ts = cp_cam.ts_nanos["CAM_LANEINFO"]["LANE_LINES"]
      if cam_ts != self.cam_laneinfo_ts_last:
        self.cam_laneinfo_ts_last = cam_ts
        self.cam_laneinfo_stale_frames = 0
      else:
        self.cam_laneinfo_stale_frames = min(self.cam_laneinfo_stale_frames + 1,
                                             CarControllerParams.CAM_LANEINFO_FRESH_FRAMES)
      cam_fresh = cam_ts != 0 and self.cam_laneinfo_stale_frames < CarControllerParams.CAM_LANEINFO_FRESH_FRAMES
      settled = cam_fresh and not (laneinfo["NO_ERR_BIT"] or laneinfo["ERR_BIT"])
      self.fsc_settled_frames = self.fsc_settled_frames + 1 if settled else 0
      ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1 if not self.CP.flags & MazdaSafetyFlags.TORQUE_INTERCEPTOR else False
    self.cp_cam = cp_cam
    self.cp = cp

    # TODO: add button types for inc and dec
    # Build the full list in Python before assigning: reading ret.buttonEvents back gives
    # a capnp list builder, which cannot be concatenated with a Python list.
    button_events = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})
    if self.low_min_set_speed:
      button_events = button_events + \
        create_button_events(self.speed_up_button, prev_speed_up_button, {1: ButtonType.accelCruise}) + \
        create_button_events(self.speed_down_button, prev_speed_down_button, {1: ButtonType.decelCruise})
    ret.buttonEvents = button_events

    fp_ret = custom.StarPilotCarState.new_message()

    fp_ret.dashboardStopSign = 1 if cp_cam.vl["CAM_TRAFFIC_SIGNS"]["STOP_SIGN"] == 9 else 0

    return ret, fp_ret

  def update_gen2(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_body = can_parsers[Bus.body]
    ret = structs.CarState()

    self.parse_wheel_speeds(ret,
        cp_cam.vl["WHEEL_SPEEDS"]["FL"],
        cp_cam.vl["WHEEL_SPEEDS"]["FR"],
        cp_cam.vl["WHEEL_SPEEDS"]["RL"],
        cp_cam.vl["WHEEL_SPEEDS"]["RR"],
    )

    ret.steeringTorque = cp_body.vl["TI_FEEDBACK"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_DRIVER_ALLOWANCE

    if self.CP.flags & MazdaSafetyFlags.MANUAL_TRANSMISSION:
      can_gear = int(cp_cam.vl["MANUAL_GEAR"]["GEAR"])
    else:
      can_gear = int(cp_cam.vl["GEAR"]["GEAR"])

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(100, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.gasPressed = cp_cam.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0
    ret.seatbeltUnlatched = False # Cruise will not engage if seatbelt is unlatched (handled by car)
    ret.doorOpen = False # Cruise will not engage if door is open (handled by car)
    ret.brakePressed = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSED"] == 1
    ret.brake = .1
    ret.steerFaultPermanent = False # TODO locate signal. Car shows light on dash if there is a fault
    ret.steerFaultTemporary = False # TODO locate signal. Car shows light on dash if there is a fault

    unit_conversion = CV.MPH_TO_MS if cp.vl["SYSTEM_SETTINGS"]["IMPERIAL_UNIT"] else CV.KPH_TO_MS
    ret.standstill = cp_cam.vl["SPEED"]["SPEED"] * unit_conversion < 0.1
    if self.CP.flags & MazdaSafetyFlags.GEN2:
      ret.cruiseState.speed = cp.vl["CRUZE_STATE"]["CRZ_SPEED"] * unit_conversion
      ret.cruiseState.enabled = (cp.vl["CRUZE_STATE"]["CRZ_STATE"] >= 2)
      ret.cruiseState.available = (cp.vl["CRUZE_STATE"]["CRZ_STATE"] != 0)
    else:
      ret.cruiseState.speed = cp_body.vl["CRUZE_STATE"]["CRZ_SPEED"] * unit_conversion
      ret.cruiseState.enabled = (cp_body.vl["CRUZE_STATE"]["CRZ_STATE"] >= 3)
      ret.cruiseState.available = (cp_body.vl["CRUZE_STATE"]["CRZ_STATE"] >= 2)
    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringRateDeg = (ret.steeringAngleDeg - self._prev_steering_angle) / DT_CTRL
    self._prev_steering_angle = ret.steeringAngleDeg
    ret.cruiseState.standstill = ret.standstill if not self.CP.openpilotLongitudinalControl else False

    self.cp = cp
    self.cp_cam = cp_cam
    self.acc = copy.copy(cp.vl["ACC"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    cam_signals = []
    if not (CP.flags & (MazdaSafetyFlags.GEN2 | MazdaSafetyFlags.GEN3)):
      cam_signals.append(("CAM_TRAFFIC_SIGNS", 0))

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_signals, 2),
    }
