"""Forward Sensing Camera (fwdCamera, 0x706) on the camera side of the harness (panda bus 2).

Sends the stock LKAS frame (CAM_LKAS, 0x243, idle), lane info (CAM_LANEINFO, 0x440) and traffic signs.
It also reproduces the cold-boot check the radar teardown has to respect: for the first CHECK_END seconds after
power-on the camera expects the radar on its network; a gap longer than GAP_T latches "Smart City Brake Support
Malfunction" (ERR_BIT in CAM_LANEINFO and ERR_BIT_1 in CAM_LKAS) for the rest of the drive.
"""
from __future__ import annotations

from collections.abc import Callable

from ..common.wire import Frame, NET_CAM
from .canbus import Ecu, Periodic, addr_of, encode
from .ecus import CX9_FW, make_cam_lkas
from .isotp import DiagEndpoint, UdsServer

CRZ_INFO_ADDR = addr_of("CRZ_INFO")


class ForwardCamera(Ecu):
  name = "fsc"
  net = NET_CAM
  BOOT_T = 6.0       # NO_ERR_BIT high while booting
  CHECK_START = 1.0  # radar presence watched from here ...
  CHECK_END = 10.0   # ... to here
  GAP_T = 1.0

  def __init__(self, speed_limit_mph: Callable[[], int], lanes_visible: Callable[[], int]):
    super().__init__()
    self.speed_limit_mph = speed_limit_mph
    self.lanes_visible = lanes_visible
    self.t_on = 0.0
    self.since_radar = 0.0
    self.malfunction = False
    self.malfunction_reason = ""
    self.ctr = 0
    self.diag = DiagEndpoint(NET_CAM, CX9_FW["fwdCamera"][0], UdsServer(CX9_FW["fwdCamera"][1]).handle)
    self.periodics = [
      Periodic("CAM_LKAS", 100.0, self._cam_lkas, 0.35),
      Periodic("CAM_LANEINFO", 1.0, self._laneinfo, 0.5),
      Periodic("CAM_TRAFFIC_SIGNS", 5.0, self._signs, 0.25),
    ]

  def power(self, on: bool) -> None:
    if on and not self.powered:
      self.t_on, self.since_radar, self.malfunction, self.malfunction_reason = 0.0, 0.0, False, ""
    self.powered = on

  def on_frame(self, f: Frame) -> None:
    if self.diag.on_frame(f):
      return
    if f.addr == CRZ_INFO_ADDR:
      self.since_radar = 0.0

  def step(self, dt: float) -> None:
    self.t_on += dt
    self.since_radar += dt
    in_window = self.CHECK_START < self.t_on < self.CHECK_END
    if in_window and self.since_radar > self.GAP_T and not self.malfunction:
      self.malfunction = True
      self.malfunction_reason = f"radar silent at {self.t_on:.1f}s after camera boot"

  def emit(self, dt: float) -> list[Frame]:
    return super().emit(dt) + self.diag.drain()

  def _cam_lkas(self) -> bytes:
    self.ctr = (self.ctr + 1) % 16
    return make_cam_lkas(0, self.ctr, er1=int(self.malfunction), b1=0, lnv=int(self.lanes_visible() == 0))

  def _laneinfo(self) -> bytes:
    lines = self.lanes_visible()
    booting = self.t_on < self.BOOT_T
    return encode("CAM_LANEINFO", {
      "LINE_VISIBLE": int(lines > 0), "LINE_NOT_VISIBLE": int(lines == 0), "LANE_LINES": 2 if lines >= 2 else (1 if lines == 0 else 3),
      "BIT1": 1, "BIT2": 0, "BIT3": 1, "NO_ERR_BIT": int(booting), "ERR_BIT": int(self.malfunction),
      "S1": 0, "S1_HBEAM": 0, "TJA": 2,
    })

  def _signs(self) -> bytes:
    return encode("CAM_TRAFFIC_SIGNS", {"SPEED_SIGN": self.speed_limit_mph(), "SPEED_SIGN_ON": 1, "SPEED_SIGN_CAM": 1,
                                        "STOP_SIGN": 0})

  def telemetry(self) -> dict:
    return {"booting": self.t_on < self.BOOT_T, "scbsMalfunction": self.malfunction, "reason": self.malfunction_reason,
            "sinceRadar": round(min(self.since_radar, 99.0), 2)}
