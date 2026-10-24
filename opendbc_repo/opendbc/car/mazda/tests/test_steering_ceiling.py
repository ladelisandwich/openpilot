"""The Torque Interceptor ceiling through the Mazda CarController: a request past TI_CEILING is clamped on both
steering channels, reported as sent, and a hold at the ceiling is still dipped (opendbc/car/mazda/torque_interceptor.py)."""
from cereal import car, custom
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaSafetyFlags

DT = 0.01
STOCK_LKAS, TI_LKAS = 0x243, 0x249
LANEINFO = ("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM")


class Toggles:
  def __getattr__(self, name):
    return False


def lkas_request(dat: bytes) -> int:
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def mazda(torque_interceptor: bool = True):
  CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX9_2021)
  flags = int(MazdaSafetyFlags.GEN1)
  if torque_interceptor:
    flags |= int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)
  CP.flags = flags
  CI = CarInterface(CP, custom.StarPilotCarParams.new_message())
  CS = CI.CS
  CS.cam_lkas = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0}
  CS.cam_laneinfo = dict.fromkeys(LANEINFO, 0)
  CS.lkas_allowed_speed = True
  CS.accel_button = 0
  CS.ti_lkas_allowed = True
  CS.crz_btns_counter = 0
  CS.fsc_settled_frames = 10**6
  CI.CC.radar_suppress_failed = True   # stock longitudinal: the steering path alone
  return CI, CarControllerParams(CP)


def steer(CI, torque: float, frames: int, v_ego: float = 20.0):
  """Drives the controller `frames` frames with a constant request; returns (stock commands, TI commands, reported torques)."""
  stock, ti, reported = [], [], []
  for f in range(frames):
    cs = car.CarState.new_message(vEgo=v_ego, canValid=True)
    cs.cruiseState.enabled = cs.cruiseState.available = True
    CI.CS.out = cs
    cc = car.CarControl.new_message(enabled=True, latActive=True)
    cc.actuators.torque = torque
    act, sends = CI.apply(cc.as_reader(), int(f * DT * 1e9), Toggles())
    for addr, dat, bus in sends:
      if addr == STOCK_LKAS:
        stock.append(lkas_request(dat))
      elif addr == TI_LKAS and bus == 1:
        ti.append(lkas_request(dat))
    reported.append(act.torque)
  return stock, ti, reported


def longest_run(seq, pred):
  best = run = 0
  for x in seq:
    run = run + 1 if pred(x) else 0
    best = max(best, run)
  return best


def test_a_full_request_is_clamped_to_the_ti_ceiling_on_both_channels():
  CI, ccp = mazda()
  for sign in (1, -1):
    CI, ccp = mazda()
    stock, ti, reported = steer(CI, sign * 1.0, 120)
    assert stock[-1] == ti[-1] == sign * ccp.TI_CEILING
    assert max(abs(x) for x in stock + ti) == ccp.TI_CEILING
    assert reported[-1] == sign * ccp.TI_CEILING / ccp.STEER_MAX    # what the torque controller's steer_max is set to
    assert all(abs(x) <= ccp.TI_CEILING for x in stock + ti)


def test_requests_inside_the_ceiling_pass_untouched():
  CI, ccp = mazda()
  stock, ti, reported = steer(CI, 0.5, 100)
  assert stock[-1] == ti[-1] == 400
  assert reported[-1] == 0.5


def test_a_hold_at_the_ceiling_is_still_dipped_as_a_backstop():
  CI, ccp = mazda()
  stock, ti, _ = steer(CI, 1.0, 600)
  assert longest_run(ti, lambda x: x == ccp.TI_CEILING) < 184          # 1.84 s of an unbroken value is what faulted the TI
  assert longest_run(ti, lambda x: x > ccp.TI_CEILING_DIP) < ccp.TI_CEILING_HOLD_FRAMES
  assert ccp.TI_CEILING_DIP in ti
  assert ti == stock                                                   # one command on both channels throughout


def test_without_a_torque_interceptor_the_full_scale_stands():
  CI, ccp = mazda(torque_interceptor=False)
  stock, ti, reported = steer(CI, 1.0, 120)
  assert ti == []
  assert stock[-1] == ccp.STEER_MAX == 800
  assert reported[-1] == 1.0
