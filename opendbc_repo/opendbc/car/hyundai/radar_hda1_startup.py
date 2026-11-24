from copy import deepcopy
from typing import Any
import time

from opendbc.can.parser import MAX_BAD_COUNTER
from opendbc.car import Bus
from opendbc.car.hyundai.canfd_owner import config_index
from opendbc.car.hyundai.canfd_stock_aol import qualified as stock_qualified
from opendbc.car.hyundai.ecu_continuity import ContinuousECUStartup, StartupKeeper
from opendbc.car.hyundai.ecu_startup import Outcome, stock_copy
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, HyundaiFlags

CARS = frozenset((CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN, CAR.KIA_CARNIVAL_4TH_GEN, CAR.GENESIS_GV70_1ST_GEN, CAR.GENESIS_GV80))


def required(cp):
  return (
    cp.carFingerprint in CARS
    and len(cp.safetyConfigs) == 1
    and cp.openpilotLongitudinalControl
    and not cp.flags & (HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_CAMERA_SCC)
  )


def qualified(cp):
  allowed = (
    HyundaiFlags.CANFD
    | HyundaiFlags.CANFD_RADAR_SCC
    | HyundaiFlags.HYBRID
    | HyundaiFlags.CANFD_ALT_BUTTONS
    | HyundaiFlags.CANFD_ALT_GEARS
    | HyundaiFlags.CANFD_ALT_GEARS_2
  )
  owner = config_index(cp)
  stock = stock_copy(cp)
  return (
    required(cp)
    and cp.alphaLongitudinalAvailable
    and cp.alternativeExperience == 0
    and not cp.pcmCruise
    and not int(cp.flags) & ~int(allowed)
    and owner is not None
    and stock.safetyConfigs[owner].safetyParam in (0, 2, 0x20, 0x22)
    and stock_qualified(stock)
    and cp.safetyConfigs[owner].safetyParam == stock.safetyConfigs[owner].safetyParam | 4
  )


class RadarHDA1Startup(ContinuousECUStartup):
  def __init__(self, cp, callbacks, *, clock=time.monotonic, keeper_factory=StartupKeeper):
    if not qualified(cp):
      raise ValueError("Exact unmarked radar HDA-I LONG configuration required before publication")
    super().__init__(cp, callbacks, address=0x7D0, bus=CanBus(cp).ECAN, label="Radar HDA-I LONG", clock=clock, keeper_factory=keeper_factory)
    self.stock_cp = stock_copy(cp)

  def continuity_required(self):
    return required(self.cp)

  def prepare(self, *, admission):
    super().prepare(admission=admission)
    if self.outcome in (Outcome.STOCK_UNTOUCHED, Outcome.STOCK_RESTORED):
      self.cp = self.stock_cp.as_reader().as_builder()
      self.prepared_cp = deepcopy(self.cp.to_dict())
    return self.cp

  def finalize_aol_configuration(self, ci):
    if self.published or self.closed or ci is not self.ci or not self.ready:
      raise RuntimeError("Radar HDA-I finalization requires configured startup")
    if self.prepared_for(ci.CP):
      return
    if self.outcome not in (Outcome.STOCK_UNTOUCHED, Outcome.STOCK_RESTORED) or not stock_qualified(ci.CP, marked_only=True):
      raise RuntimeError("Radar HDA-I startup does not admit this AOL configuration")
    expected: Any = deepcopy(self.prepared_cp)
    configs = expected.get("safetyConfigs", [])
    if (
      len(configs) != 1
      or configs[0]["safetyModel"] != "hyundaiCanfd"
      or configs[0]["safetyParam"] not in (0, 2, 0x20, 0x22)
      or expected["alternativeExperience"] != 0
    ):
      raise RuntimeError("Radar HDA-I prepared stock profile cannot be finalized")
    configs[0]["safetyParam"] |= 0x800
    if ci.CP.to_dict() != expected:
      raise RuntimeError("Radar HDA-I finalization changed unrelated CarParams")
    self.prepared_cp = expected

  def sources_current(self, ci, now_ns):
    state = ci.CS.out
    if not state.canValid or state.canTimeout:
      return False
    parser = ci.can_parsers[Bus.pt]
    if parser.bus != self.bus:
      return False
    names = {ci.CS.accelerator_msg_canfd, ci.CS.gear_msg_canfd, ci.CS.cruise_btns_msg_canfd, "WHEEL_SPEEDS", "MDPS", "TCS"}
    if not ci.CP.openpilotLongitudinalControl:
      names.add("SCC_CONTROL")
    for name in names:
      source = parser.message_states.get(parser.dbc.name_to_msg[name].address)
      if (
        source is None
        or not source.timestamps
        or source.counter_fail >= MAX_BAD_COUNTER
        or not self.floor_ns < int(source.timestamps[-1]) <= now_ns
        or now_ns - int(source.timestamps[-1]) > source.timeout_threshold
      ):
        return False
    return True

  def before_control(self, **context):
    self.check()
    return self.outcome is not Outcome.SENT_UNCONFIRMED or (context["configured"] and context["sources_current"] and context["control_current"])

  def _warm_stock(self, ci):
    deadline = self.clock() + 3.0
    while self.clock() < deadline:
      packets = self.callbacks[0]()
      stamped = [(int(getattr(packet, "log_mono_time_ns", 0)), list(packet)) for packet in packets]
      if any(stamp <= 0 for stamp, _ in stamped):
        raise RuntimeError("Radar HDA-I stock warmup requires timestamped CAN")
      ci.update(stamped)
      if self.sources_current(ci, time.clock_gettime_ns(time.CLOCK_BOOTTIME)):
        self.ready = True
        return
      time.sleep(0.005)
    self.outcome = Outcome.ABORT_UNCERTAIN
    raise RuntimeError("Radar HDA-I stock SCC sources did not resume before publication")
