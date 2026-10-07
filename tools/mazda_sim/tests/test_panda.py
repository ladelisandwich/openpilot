"""The virtual panda running the build's real safety code (compiled from its opendbc)."""
import shutil
from pathlib import Path

import pytest

from mazda_sim.common.wire import NET_AUX, NET_CAM, NET_CAR

REPO = Path(__file__).resolve().parents[3]

pytest.importorskip("cffi")
if shutil.which("gcc") is None:
  pytest.skip("needs gcc to build the safety library", allow_module_level=True)

from mazda_sim.device.libsafety import Safety, build
from mazda_sim.device.virtual_panda import (MAZDA_FLAG_GEN1, MAZDA_FLAG_TORQUE_INTERCEPTOR, SAFETY_ELM327,
                                            SAFETY_MAZDA, SAFETY_NOOUTPUT, VirtualPanda)


@pytest.fixture(scope="module")
def panda(tmp_path_factory):
  lib = build(str(REPO / "opendbc_repo"), str(tmp_path_factory.mktemp("safety")))
  return VirtualPanda(Safety(lib))


def test_modes_and_wiring(panda):
  panda.set_mode(SAFETY_NOOUTPUT, 0)
  assert not panda.relay_intercept and panda.buses_for_net(NET_CAR) == (0, 2) and panda.buses_for_net(NET_AUX) == ()
  panda.set_mode(SAFETY_ELM327, 0)
  assert panda.obd_mode and panda.buses_for_net(NET_AUX) == (1,)
  panda.set_mode(SAFETY_MAZDA, MAZDA_FLAG_GEN1 | MAZDA_FLAG_TORQUE_INTERCEPTOR)
  assert panda.mode == SAFETY_MAZDA and panda.relay_intercept and panda.obd_mode
  assert panda.buses_for_net(NET_CAR) == (0,) and panda.buses_for_net(NET_CAM) == (2,)
  panda.set_mode(SAFETY_MAZDA, MAZDA_FLAG_GEN1)
  assert not panda.obd_mode and panda.buses_for_net(NET_AUX) == ()


def _ti_cmd(torque: int):
  from opendbc.can.packer import CANPacker
  m = CANPacker("mazda_2017").make_can_msg("CAM_LKAS2", 1, {"LKAS_REQUEST": torque, "CHKSUM": torque, "KEY": 3294744160})
  addr, dat, bus = (m.address, m.dat, m.src) if hasattr(m, "address") else m
  return addr, bytes(dat), bus


def test_lkas_torque_needs_controls_allowed(panda):
  from mazda_sim.car.ecus import make_cam_lkas
  panda.set_mode(SAFETY_MAZDA, MAZDA_FLAG_GEN1 | MAZDA_FLAG_TORQUE_INTERCEPTOR)
  assert not panda.safety.controls_allowed
  out = panda.send([(0x243, make_cam_lkas(0, 0), 0)])
  assert len(out) == 1 and out[0].net == NET_CAR            # zero torque passes
  assert panda.send([(0x243, make_cam_lkas(300, 1), 0)]) == [] and panda.counters.tx_blocked == 1
  echoes = [src for _addr, _dat, src in panda.drain_host()]
  assert 0 + 128 in echoes and 0 + 192 in echoes            # returned and rejected, as pandad reports them
  out = panda.send([_ti_cmd(0)])
  assert len(out) == 1 and out[0].net == NET_AUX            # the TI command goes out on the harness's AUX lines


GEN1_TI_GAP = ("the build's mazda.h torque-checks CAM_LKAS2 (0x249) on bus 1 for GEN2/GEN3 only: " +
               "a GEN1 TI command passes the panda whatever its torque")


@pytest.mark.xfail(reason=GEN1_TI_GAP, strict=False)
def test_gen1_ti_torque_needs_controls_allowed(panda):
  panda.set_mode(SAFETY_MAZDA, MAZDA_FLAG_GEN1 | MAZDA_FLAG_TORQUE_INTERCEPTOR)
  assert not panda.safety.controls_allowed
  assert panda.send([_ti_cmd(300)]) == []
