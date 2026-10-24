"""The TI and the EPS, fed by the build's own message builders (opendbc's mazdacan)."""
import types

import pytest

from mazda_sim.car.ecus import Eps
from mazda_sim.car.ti import STATE_DRIVER_OVER, STATE_OFF, STATE_RUN, TorqueInterceptor, VIOL_CHECKSUM, VIOL_TIMEOUT
from mazda_sim.car.vehicle import Vehicle
from mazda_sim.common.config import PlantOptions
from mazda_sim.common.wire import NET_AUX, NET_CAR, Frame


@pytest.fixture
def build_msgs():
  from opendbc.can.packer import CANPacker
  from opendbc.car.mazda.mazdacan import create_steering_control
  from opendbc.car.mazda.values import MazdaSafetyFlags
  packer = CANPacker("mazda_2017")
  cp = types.SimpleNamespace(flags=MazdaSafetyFlags.GEN1)
  lkas = {"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 0}

  def make(frame: int, torque: int, ti_torque: int | None):
    out = {}
    for m in create_steering_control(packer, cp, frame, torque, lkas, ti_torque):
      addr, dat, bus = (m.address, m.dat, m.src) if hasattr(m, "address") else m
      out[bus] = (addr, bytes(dat))
    return out
  return make


def run_ti(ti: TorqueInterceptor, make, seconds: float, torque: int, driver_nm: float = 0.0, frame0: int = 0) -> int:
  frame = frame0
  for _ in range(int(seconds * 100)):
    ti.set_driver_torque(driver_nm)
    if frame % 2 == 0:   # openpilot sends steering at 50 Hz
      addr, dat = make(frame, torque, torque)[1]
      ti.on_frame(Frame(NET_AUX, addr, dat))
    ti.step(0.01)
    frame += 1
  return frame


def test_ti_arms_and_injects(build_msgs):
  ti = TorqueInterceptor(PlantOptions(), version=1)
  ti.power(True)
  f = run_ti(ti, build_msgs, 1.0, 400)
  assert ti.state != STATE_RUN, "still discovering sensors right after power-on"
  run_ti(ti, build_msgs, 1.0, 400, frame0=f)
  assert ti.state == STATE_RUN and ti.viol == 0
  assert ti.injected_nm == pytest.approx(400 / 800 * PlantOptions().ti_inject_full_nm)


def test_ti_driver_override_and_timeout(build_msgs):
  ti = TorqueInterceptor(PlantOptions(), version=1)
  ti.power(True)
  f = run_ti(ti, build_msgs, 2.0, 200)
  f = run_ti(ti, build_msgs, 0.2, 200, driver_nm=4.0, frame0=f)
  assert ti.state == STATE_DRIVER_OVER and ti.injected_nm == 0.0
  f = run_ti(ti, build_msgs, 0.5, 200, driver_nm=0.0, frame0=f)
  assert ti.state == STATE_RUN
  for _ in range(20):   # commands stop
    ti.step(0.01)
  assert ti.state == STATE_OFF and ti.viol & VIOL_TIMEOUT


def test_ti_rejects_bad_checksum(build_msgs):
  ti = TorqueInterceptor(PlantOptions())
  ti.power(True)
  run_ti(ti, build_msgs, 2.0, 100)
  addr, dat = build_msgs(0, 100, 100)[1]
  bad = bytearray(dat)
  bad[2] ^= 0x01   # CHKSUM no longer equals LKAS_REQUEST
  ti.on_frame(Frame(NET_AUX, addr, bytes(bad)))
  assert ti.viol & VIOL_CHECKSUM


def test_eps_accepts_the_builds_cam_lkas(build_msgs):
  veh = Vehicle(PlantOptions())
  veh.reset(0.0, 0.0, 0.0, 80 / 3.6)
  eps = Eps(veh, PlantOptions())
  for frame in range(200):
    addr, dat = build_msgs(frame, 300, None)[0]
    eps.on_frame(Frame(NET_CAR, addr, dat))
    eps.step(0.01)
  t = eps.telemetry()
  assert t["badFrames"] == 0 and t["lkasReq"] == 300
