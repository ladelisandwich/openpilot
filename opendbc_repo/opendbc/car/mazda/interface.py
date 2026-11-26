#!/usr/bin/env python3
from math import exp, fabs
import numpy as np

from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, LateralAccelFromTorqueCallbackType
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.hybrid import RadarWitness
from opendbc.car.mazda.longitudinal import capture_stock_radar_frames, enter_radar_programming_session, request_radar_default_session
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, MazdaSafetyFlags, GEN1, GEN2, GEN3
from openpilot.common.params import Params

NON_LINEAR_TORQUE_PARAMS = {
  CAR.MAZDA_3_2019: (4.6, 0.6, 0.134, 0.3605),
  CAR.MAZDA_CX_30: (4.68689, 0.79999, 0.18244, 0.38763),
  CAR.MAZDA_CX_30_2023: (4.68689, 0.79999, 0.18244, 0.38763),
  CAR.MAZDA_CX_50: (4.68689, 0.79999, 0.18244, 0.38763)
}

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP):
    super().__init__(CP)
    # Hybrid long switches ACC master at runtime, so it needs to see the stock radar's own frames.
    # They are read raw here rather than through the CANParser (see hybrid.RadarWitness).
    self.radar_witness = RadarWitness() if CP.flags & MazdaSafetyFlags.HYBRID_LONG else None
    self.CS.radar_witness = self.radar_witness

  def update(self, can_packets):
    if self.radar_witness is not None:
      self.radar_witness.update(can_packets)
    return super().update(can_packets)

  def get_lataccel_torque_siglin(self) -> float:

    def torque_from_lateral_accel_siglin_func(lateral_acceleration: float) -> float:
      # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
      # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
      # This has big effect on the stability about 0 (noise when going straight)
      non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
      assert non_linear_torque_params, "The params are not defined"
      a, b, c, _ = non_linear_torque_params
      sig_input = a * lateral_acceleration
      sig = np.sign(sig_input) * (1 / (1 + exp(-fabs(sig_input))) - 0.5)
      steer_torque = (sig * b) + (lateral_acceleration * c)
      return float(steer_torque)

    lataccel_values = np.arange(-8.0, 8.0, 0.01)
    torque_values = [torque_from_lateral_accel_siglin_func(x) for x in lataccel_values]
    print(torque_values)
    assert min(torque_values) < -1 and max(torque_values) > 1, "The torque values should cover the range [-1, 1]"
    return torque_values, lataccel_values

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def torque_from_lateral_accel_siglin(lateral_acceleration: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(lateral_acceleration, lataccel_values, torque_values)
      return torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  def lateral_accel_from_torque(self) -> LateralAccelFromTorqueCallbackType:
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def lateral_accel_from_torque_siglin(torque: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(torque, torque_values, lataccel_values)
      return lateral_accel_from_torque_siglin
    else:
      return self.lateral_accel_from_torque_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    p = Params()

    ret.radarUnavailable = True
    ret.deprecated.enableBsm = True

    # Upstream keeps every Mazda but the CX-5 2022 and CX-9 2021 dashcam-only (the 45 km/h steering floor), and the fork's
    # GEN2/GEN3 platforms steer only through the Torque Interceptor. With the interceptor installed every platform drives.
    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021) and not p.get_bool("TorqueInterceptorEnabled")

    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_3_2019, CAR.MAZDA_CX_30, CAR.MAZDA_CX_50, CAR.MAZDA_3_2023, CAR.MAZDA_CX_30_2023):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    if p.get_bool("ManualTransmission"):
      ret.flags |= MazdaSafetyFlags.MANUAL_TRANSMISSION.value
      ret.transmissionType = structs.CarParams.TransmissionType.manual
    else:
      ret.transmissionType = structs.CarParams.TransmissionType.automatic

    if candidate in GEN1:
      # Torque Interceptor: lagd starts from steerActuatorDelay + 0.2 until it has its own estimate, and on openpilot 0.11
      # it learns only above 50 mph, so this start is what the model and the controller run on for a long while.
      # 0.115 starts them at 0.315 s, the total the StarPilot 6 fork drove on (its learned 0.216 s on this car plus its
      # 0.1 s lateral smoothing). The fork's 0.335 started them at 0.535 s, and the first StarPilot 7 drives wove at
      # highway speed with the model's own request in the loop. A stock GEN1 keeps upstream's 0.1.
      ret.steerActuatorDelay = 0.115 if p.get_bool("TorqueInterceptorEnabled") else 0.1
      # Upstream runs a stock GEN1 with safety param 0 (and StarPilot 7's stock always-on-lateral only admits that), and the
      # panda treats 0 as GEN1. The GEN1 bit goes in with the first fork flag, where the firmware's TI bus switch looks for it.
      stock_safety_param = ret.safetyConfigs[0].safetyParam
      if p.get_bool("TorqueInterceptorEnabled"): # Torque Interceptor Installed
        ret.flags |= MazdaSafetyFlags.TORQUE_INTERCEPTOR.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.TORQUE_INTERCEPTOR.value
        ret.minSteerSpeed = 0.0
        ret.steerAtStandstill = True
      if p.get_bool("RadarInterceptorEnabled"): # Radar Interceptor Installed
        ret.flags |= MazdaSafetyFlags.RADAR_INTERCEPTOR.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.RADAR_INTERCEPTOR.value
        ret.alphaLongitudinalAvailable = alpha_long
        ret.openpilotLongitudinalControl = True
        ret.radarUnavailable = False
        ret.deprecated.startingState = True
        ret.longitudinalTuning.kiBP = [0., 5., 20., 30.]
        ret.longitudinalTuning.kiV = [0.36, 0.23, 0.17, 0.1]
      # Software radar emulation. Only when no hardware Radar Interceptor is installed:
      # the two paths both own CRZ_INFO/CRZ_CTRL and must never run together.
      if p.get_bool("RadarEmulationEnabled") and not p.get_bool("RadarInterceptorEnabled"):
        ret.flags |= MazdaSafetyFlags.RADAR_EMULATION.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.RADAR_EMULATION.value
        ret.alphaLongitudinalAvailable = alpha_long
        ret.openpilotLongitudinalControl = True
        # vision-only lead, so E2E / Experimental Mode drives gas and brake
        ret.radarUnavailable = True
        # engagement still follows the stock MRCC set/cancel edge
        ret.pcmCruise = True
        ret.deprecated.startingState = True
        ret.deprecated.startAccel = 1.2
        ret.deprecated.vEgoStarting = 0.15
        ret.deprecated.vEgoStopping = 0.5
        ret.longitudinalActuatorDelay = 0.36
        ret.longitudinalTuning.kiBP = [0., 5., 20.]
        ret.longitudinalTuning.kiV = [0.18, 0.12, 0.08]
        # Hybrid: stock MRCC in standard mode, emulation in experimental mode (see hybrid.py).
        # CarParams flag only; the panda runs the radar emulation safety mode for both.
        if p.get_bool("MazdaHybridLong"):
          ret.flags |= MazdaSafetyFlags.HYBRID_LONG.value
      if p.get_bool("NoMRCC"): # No Mazda Radar Cruise Control; Missing CRZ_CTRL signal
        ret.flags |= MazdaSafetyFlags.NO_MRCC.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.NO_MRCC.value
      if p.get_bool("NoFSC"):  # No Front Sensing Camera
        ret.flags |= MazdaSafetyFlags.NO_FSC.value
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.NO_FSC.value
      if ret.safetyConfigs[0].safetyParam != stock_safety_param:
        ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.GEN1.value

    if candidate in GEN2:
      ret.steerActuatorDelay = 0.5
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.GEN2.value
      ret.alphaLongitudinalAvailable = alpha_long
      ret.openpilotLongitudinalControl = True
      ret.stopAccel = -.5
      ret.deprecated.vEgoStarting = .2
      ret.longitudinalActuatorDelay = 0.35 # gas is 0.25s and brake looks like 0.5
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.1, 0.1]
      ret.deprecated.startingState = True
      ret.steerAtStandstill = True
      ret.deprecated.enableBsm = False

    if candidate in GEN3:
      ret.steerActuatorDelay = 0.5
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.GEN3.value
      ret.alphaLongitudinalAvailable = False
      ret.openpilotLongitudinalControl = False
      if p.get_bool("ManualTransmission"):
        ret.flags |= MazdaSafetyFlags.MANUAL_TRANSMISSION.value
      ret.deprecated.enableBsm = False

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def init(CP, can_recv, can_send):
    if CP.flags & MazdaSafetyFlags.RADAR_EMULATION:
      Params().put_bool("EcuDisableFailed", False)
      if CP.flags & MazdaSafetyFlags.HYBRID_LONG:
        # Hybrid starts every drive on the stock radar. The control loop silences it only when
        # openpilot takes over, and captures its frames at that moment (see hybrid.py).
        return
      # Record the radar's own frames while it is still talking, then silence it. Replaying
      # this car's real frames is far more faithful than hardcoded templates from another
      # model -- a CX-9 capture showed 0x362, 0x365 and 0x366 differed materially from the
      # CX-5 templates that shipped, which is the likeliest reason its modules noticed the
      # radar had gone.
      capture_stock_radar_frames(can_recv)
      suppressed = enter_radar_programming_session(can_recv, can_send)
      Params().put_bool("EcuDisableFailed", not suppressed)

  @staticmethod
  def deinit(CP, can_recv, can_send):
    if CP.flags & MazdaSafetyFlags.RADAR_EMULATION:
      request_radar_default_session(can_recv, can_send)
