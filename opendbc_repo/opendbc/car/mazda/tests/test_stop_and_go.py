"""Radar emulation stop-and-go through the Mazda CarController: what openpilot sends the car (CRZ_INFO.ACCEL_CMD)
while it stands, then asks to go."""
import pytest

from cereal import car, custom
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.longitudinal import RADAR_BUS
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags

DT = 0.01
CRZ_INFO = 0x21B
GO_FRAME = 800                    # stopped from 2 s to 8 s (well into the hold), then openpilot asks to go
LANEINFO = ("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM")


class Toggles:
  def __getattr__(self, name):
    return False


def accel_cmd(dat: bytes) -> int:
  return (((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096


def emulation_car():
  CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX9_2021)
  CP.flags = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.RADAR_EMULATION)
  CP.openpilotLongitudinalControl = True
  CP.pcmCruise = True
  CP.vEgoStarting = 0.15
  CP.vEgoStopping = 0.5
  CI = CarInterface(CP, custom.StarPilotCarParams.new_message())
  CS = CI.CS
  CS.cam_lkas = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0}
  CS.cam_laneinfo = dict.fromkeys(LANEINFO, 0)
  CS.lkas_allowed_speed = True
  CS.accel_button = 0
  CS.ti_lkas_allowed = True
  CS.crz_btns_counter = 0
  CS.fsc_settled_frames = 10**6
  CI.CC.radar_suppress_failed = False
  return CI


def stop_and_go(hold_lets_go_s: float, start_accel: float = 1.2, stopping_again_frame: int | None = None):
  """Commands sent while the car still stands after openpilot asked to go, and when it rolled (None: never)."""
  CI = emulation_car()
  v, moved, standing_cmds = 4.0, None, []
  for f in range(GO_FRAME + 400):
    if f < 200:
      v = max(0.0, v - 0.02)
      state, accel, resume = ("stopping" if v < 1.0 else "pid"), -1.0, False
    elif f < GO_FRAME:
      v, state, accel, resume = 0.0, "stopping", -2.0, False
    else:
      state, accel, resume = "starting", start_accel, True
      if stopping_again_frame is not None and f >= stopping_again_frame:
        state, accel, resume = "stopping", -2.0, False
      lets_go = (f - GO_FRAME) * DT >= hold_lets_go_s and state != "stopping"
      v = min(3.0, v + 0.01) if lets_go or v > 0.0 else 0.0
      if v > 0.0 and moved is None:
        moved = f
    cs = car.CarState.new_message(vEgo=v, standstill=v < 0.001, canValid=True)
    cs.cruiseState.enabled = cs.cruiseState.available = True
    cs.cruiseState.standstill = v < 0.001
    CI.CS.out = cs
    cc = car.CarControl.new_message(enabled=True, latActive=True, longActive=True)
    cc.actuators.accel = accel
    cc.actuators.longControlState = state
    cc.cruiseControl.resume = resume
    cc.hudControl.leadVisible = True
    _, sends = CI.apply(cc.as_reader(), int(f * DT * 1e9), Toggles())
    if f >= GO_FRAME and (moved is None or f < moved):
      standing_cmds += [accel_cmd(dat) for addr, dat, bus in sends if addr == CRZ_INFO and bus == RADAR_BUS]
  return standing_cmds, moved


@pytest.mark.parametrize("hold_lets_go_s,start_accel", [(0.3, 1.2), (1.5, 1.2), (1.5, 0.2), (3.0, 0.6)])
def test_no_brake_against_openpilots_own_start(hold_lets_go_s, start_accel):
  # The car's hold can take longer than openpilot's 0.5 s release window to let go (routes 0000002b at 426 s and
  # 0000002e at 880 s). While openpilot still asks to go, what it sends must not go back to the hold brake.
  cmds, moved = stop_and_go(hold_lets_go_s, start_accel)
  assert moved is not None and cmds
  assert min(cmds) >= 0, cmds


def test_hold_returns_when_openpilot_decides_to_stay_stopped():
  cmds, moved = stop_and_go(9.0, stopping_again_frame=GO_FRAME + 30)
  assert moved is None
  assert cmds[-1] < 0 and sum(c < 0 for c in cmds) > 0.8 * len(cmds)   # back on the brake once the window ends
