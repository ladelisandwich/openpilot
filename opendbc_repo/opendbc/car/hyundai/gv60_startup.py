import time

from opendbc.car import structs
from opendbc.car.hyundai.canfd_stock_aol import qualified as stock_qualified
from opendbc.car.hyundai.ecu_continuity import ContinuousECUStartup, StartupKeeper
from opendbc.car.hyundai.ecu_startup import stock_copy
from opendbc.car.hyundai.generic_hda2_startup import GenericHDA2Startup
from opendbc.car.hyundai.values import CAR, HyundaiFlags

PROFILE_MARKER = 0x9000
LONG_WORDS = (0x9015, 0x9095)


def required(cp):
  return (
    cp.carFingerprint == CAR.GENESIS_GV60_EV_1ST_GEN
    and len(cp.safetyConfigs) == 1
    and cp.openpilotLongitudinalControl
    and bool(cp.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)
    and not cp.flags & (HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CANFD_ANGLE_STEERING)
  )


def qualified(cp):
  allowed = (
    HyundaiFlags.CANFD
    | HyundaiFlags.EV
    | HyundaiFlags.CANFD_LKA_STEER_MSG
    | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT
    | HyundaiFlags.CANFD_ALT_GEARS
    | HyundaiFlags.CANFD_ALT_GEARS_2
  )
  if (
    not required(cp)
    or cp.brand != 'hyundai'
    or len(cp.safetyConfigs) != 1
    or cp.passive
    or cp.dashcamOnly
    or cp.notCar
    or cp.steerControlType != structs.CarParams.SteerControlType.torque
    or cp.pcmCruise
    or cp.alternativeExperience != 0
    or not cp.alphaLongitudinalAvailable
    or int(cp.flags) & ~int(allowed)
    or cp.flags & (HyundaiFlags.CANFD | HyundaiFlags.EV | HyundaiFlags.CANFD_LKA_STEER_MSG)
    != (HyundaiFlags.CANFD | HyundaiFlags.EV | HyundaiFlags.CANFD_LKA_STEER_MSG)
    or cp.safetyConfigs[0].safetyModel != structs.CarParams.SafetyModel.hyundaiCanfd
    or cp.safetyConfigs[0].safetyParam not in LONG_WORDS
  ):
    return False
  return stock_qualified(stock_copy(cp))


def select_profile(cp, *, is_release):
  if not is_release and required(cp) and len(cp.safetyConfigs) == 1 and cp.safetyConfigs[0].safetyParam in (0x15, 0x95):
    cp.safetyConfigs[0].safetyParam |= PROFILE_MARKER
    if not qualified(cp):
      cp.safetyConfigs[0].safetyParam &= ~PROFILE_MARKER


class GV60Startup(GenericHDA2Startup):
  def __init__(self, cp, callbacks, *, clock=time.monotonic, keeper_factory=StartupKeeper):
    if not qualified(cp):
      raise ValueError('Exact single-Panda GV60 torque LONG profile required before publication')
    ContinuousECUStartup.__init__(self, cp, callbacks, address=0x730, bus=1, label='GV60 LONG', clock=clock, keeper_factory=keeper_factory)
    self.stock_cp = stock_copy(cp)

  def communication_control_request(self):
    return b'\x28\x01\x01'

  def continuity_required(self):
    return required(self.cp)
