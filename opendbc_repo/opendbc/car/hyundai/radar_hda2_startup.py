from copy import deepcopy
from typing import Any
import time

from opendbc.can.parser import MAX_BAD_COUNTER
from opendbc.car import Bus
from opendbc.car.hyundai.canfd_owner import config_index
from opendbc.car.hyundai.canfd_stock_aol import qualified as stock_qualified
from opendbc.car.hyundai.ecu_continuity import ContinuousECUStartup, StartupKeeper
from opendbc.car.hyundai.ecu_startup import stock_copy
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_hda1_startup import CARS, RadarHDA1Startup
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def required(cp):
  return (cp.carFingerprint in CARS and len(cp.safetyConfigs) == 1 and cp.openpilotLongitudinalControl and
          bool(cp.flags & HyundaiFlags.CANFD_LKA_STEER_MSG) and not cp.flags & HyundaiFlags.CANFD_CAMERA_SCC)


def qualified(cp):
  allowed = (HyundaiFlags.CANFD | HyundaiFlags.CANFD_RADAR_SCC | HyundaiFlags.HYBRID |
             HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT |
             HyundaiFlags.CANFD_ALT_GEARS | HyundaiFlags.CANFD_ALT_GEARS_2)
  owner = config_index(cp)
  stock = stock_copy(cp)
  return (required(cp) and cp.alphaLongitudinalAvailable and cp.alternativeExperience == 0 and not cp.pcmCruise and
          not int(cp.flags) & ~int(allowed) and owner is not None and
          stock.safetyConfigs[owner].safetyParam in (0x10, 0x12, 0x90, 0x92) and stock_qualified(stock) and
          cp.safetyConfigs[owner].safetyParam == (stock.safetyConfigs[owner].safetyParam | 4 |
            (0x2000 if cp.carFingerprint == CAR.KIA_CARNIVAL_4TH_GEN else 0)))


class RadarHDA2Startup(RadarHDA1Startup):
  def __init__(self, cp, callbacks, *, clock=time.monotonic, keeper_factory=StartupKeeper):
    if not qualified(cp):
      raise ValueError('Exact unmarked radar HDA-II LONG configuration required before publication')
    ContinuousECUStartup.__init__(self, cp, callbacks, address=0x730, bus=CanBus(cp).ECAN,
                                  label='Radar HDA-II LONG', clock=clock, keeper_factory=keeper_factory)
    self.stock_cp = stock_copy(cp)

  def continuity_required(self):
    return required(self.cp)

  def finalize_aol_configuration(self, ci):
    if self.published or self.closed or ci is not self.ci or not self.ready:
      raise RuntimeError('Radar HDA-II finalization requires configured startup')
    if self.prepared_for(ci.CP):
      return
    from opendbc.car.hyundai.ecu_startup import Outcome
    if self.outcome not in (Outcome.STOCK_UNTOUCHED, Outcome.STOCK_RESTORED) or not stock_qualified(ci.CP, marked_only=True):
      raise RuntimeError('Radar HDA-II startup does not admit this AOL configuration')
    expected: Any = deepcopy(self.prepared_cp)
    configs = expected.get('safetyConfigs', [])
    if (len(configs) != 1 or configs[0]['safetyModel'] != 'hyundaiCanfd' or
        configs[0]['safetyParam'] not in (0x10, 0x12, 0x90, 0x92) or expected['alternativeExperience'] != 0):
      raise RuntimeError('Radar HDA-II prepared stock profile cannot be finalized')
    configs[0]['safetyParam'] |= 0x800
    if ci.CP.to_dict() != expected:
      raise RuntimeError('Radar HDA-II finalization changed unrelated CarParams')
    self.prepared_cp = expected

  def sources_current(self, ci, now_ns):
    if not super().sources_current(ci, now_ns):
      return False
    parser = ci.can_parsers[Bus.cam]
    if parser.bus != CanBus(ci.CP).CAM or not parser.can_valid or parser.bus_timeout:
      return False
    name = 'CAM_0x362' if ci.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else 'CAM_0x2a4'
    source = parser.message_states.get(parser.dbc.name_to_msg[name].address)
    return bool(source is not None and source.timestamps and source.counter_fail < MAX_BAD_COUNTER and
                self.floor_ns < int(source.timestamps[-1]) <= now_ns and
                now_ns - int(source.timestamps[-1]) <= source.timeout_threshold)
