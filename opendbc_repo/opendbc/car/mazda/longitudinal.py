"""Mazda GEN1 radar emulation (openpilot longitudinal without a hardware radar interceptor).

Ported to StarPilot from yummydirtx/opendbc-sunnypilot @ df79f7d
(branch work: 436efc4..df79f7d "mazda: add longitudinal control").

Mechanism: the stock forward radar (0x764) is put into a UDS programming session,
which silences it. openpilot then impersonates the radar module, synthesizing
CRZ_INFO (0x21B) -- which carries the ACCEL_CMD the PCM/brake ECU acts on --
CRZ_CTRL (0x21C), and the radar heartbeat frames (0x499 static + 0x361..0x366
tracks) so the rest of the bus does not notice the radar is gone.

A tester-present at 2 Hz holds the session open. If openpilot stops transmitting,
the session times out and the stock radar comes back on its own within a few seconds.
"""
from __future__ import annotations

from enum import Enum

from opendbc.can.dbc import DBC
from opendbc.can.packer import set_value
from opendbc.car import make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery


MAZDA_LONG_DBC = DBC("mazda_2017")

RADAR_ADDR = 0x764
RADAR_BUS = 0
CAM_BUS = 2

CRZ_INFO_ADDR = 0x21B
CRZ_CTRL_ADDR = 0x21C
RADAR_STATIC_ADDR = 0x499
RADAR_TRACK_ADDRS = (0x361, 0x362, 0x363, 0x364, 0x365, 0x366)
RADAR_SYNTHETIC_LEAD_TRACK_ADDR = 0x364

CRZ_INFO_STANDBY_TEMPLATE = bytes.fromhex("01ffe3ffc0000000")
CRZ_INFO_TEMPLATE = bytes.fromhex("01ffe20006800000")
# Fallbacks only. The live capture in capture_stock_radar_frames() is preferred -- these
# are used only if that fails. Values below are from a 2023 CX-9 bus capture; the CX-5
# templates that shipped originally were wrong for this car on 0x362, 0x365 and 0x366,
# which is the most likely reason its modules noticed the radar had gone.
RADAR_STATIC_TEMPLATE = bytes.fromhex("0008c00000000000")
RADAR_TRACK_EMPTY_TEMPLATES = (
  bytes.fromhex("fff7fefe1fc00080"),   # 0x361 - matched the CX-9 capture
  bytes.fromhex("fff7fefe1fc97080"),   # 0x362 - CX-9 capture (was fff7fefe1fc78c80)
  bytes.fromhex("fff7fefe1fc00000"),   # 0x363 - matched
  bytes.fromhex("fff7fefe1fc00000"),   # 0x364 - matched
  bytes.fromhex("04808e08a0461d40"),   # 0x365 - CX-9 capture (was fff7fe7ffbff3fc0)
  bytes.fromhex("1911607ffbff03c0"),   # 0x366 - CX-9 capture (was fff7fe7ffbff3fc0)
)

# Radar frames observed live before the session is taken, keyed by address. Preferred over
# the templates above because they come from this car in its current state rather than from
# a capture of a different model.
_captured_radar_frames: dict[int, bytes] = {}


def capture_stock_radar_frames(can_recv, timeout: float = 1.0) -> dict[int, bytes]:
  """Record the stock radar's own frames before silencing it, so we replay what this
  particular car actually broadcasts.

  Hardcoded templates are a guess about another vehicle's radar. The real frames are on the
  bus for the taking right up until we silence it, and replaying them is strictly more
  faithful. Falls back to the templates if nothing arrives in time.
  """
  import time
  wanted = set(RADAR_TRACK_ADDRS) | {RADAR_STATIC_ADDR}
  seen: dict[int, bytes] = {}
  deadline = time.monotonic() + timeout
  # wait_for_one=True would block forever on a silent bus and never reach the deadline,
  # stalling car startup. Poll instead so the timeout is always honoured.
  while time.monotonic() < deadline and len(seen) < len(wanted):
    packets = can_recv(wait_for_one=False)
    if not packets:
      time.sleep(0.01)
      continue
    for packet in packets:
      for msg in packet:
        if msg.src == RADAR_BUS and msg.address in wanted:
          seen[msg.address] = bytes(msg.dat)
  _captured_radar_frames.clear()
  _captured_radar_frames.update(seen)
  missing = sorted(hex(a) for a in wanted - set(seen))
  if missing:
    carlog.warning(f"mazda radar capture incomplete, using templates for {missing}")
  else:
    carlog.warning(f"mazda radar capture complete: {sorted(hex(a) for a in seen)}")
  return seen


def update_captured_radar_frames(frames: dict[int, bytes]) -> None:
  """Hybrid long: refresh the replayed heartbeat frames from the radar's own latest traffic,
  taken the moment it goes silent, instead of a capture made at startup."""
  wanted = set(RADAR_TRACK_ADDRS) | {RADAR_STATIC_ADDR}
  fresh = {a: bytes(d) for a, d in frames.items() if a in wanted and len(d) == 8}
  _captured_radar_frames.update(fresh)
  carlog.warning(f"mazda radar frames refreshed at takeover: {sorted(hex(a) for a in fresh)}")


RADAR_SYNTHETIC_LEAD_TRACK_TEMPLATE = bytes.fromhex("0a4000001dc00000")

LONG_COMMAND_STEP = 2
RADAR_HEARTBEAT_STEP = 10
TESTER_PRESENT_STEP = 50

ACCEL_CMD_MAX = 2000.0
ACCEL_CMD_MIN = -2000.0
HOLD_BRAKE_CMD_TARGET = -1024.0
HOLD_LATCHED_CMD_TARGET = -1.0
NEAR_STOP_BRAKE_CMD_TARGET = -750.0
NEAR_STOP_ENTRY_SPEED = 1.0

# Stock Mazda longitudinal is not using one global raw-command scale across all
# speeds. Keep more authority at low/mid speed, and soften the map at highway
# speed where the single-scale version feels jerky.
ACCEL_SCALE_UP_BP = (0.0, 4.2, 11.1, 22.2)
ACCEL_SCALE_UP_V = (1000.0, 1000.0, 950.0, 800.0)
ACCEL_SCALE_DOWN_BP = (0.0, 1.4, 5.6, 22.2)
ACCEL_SCALE_DOWN_V = (1200.0, 1000.0, 925.0, 950.0)


class MazdaLongitudinalProfile(str, Enum):
  STANDBY = "standby"
  ENGAGED_CRUISE = "engaged_cruise"
  ENGAGED_FOLLOW = "engaged_follow"
  STOP_GO_HOLD = "stop_go_hold"
  STOP_GO_HOLD_LATCHED = "stop_go_hold_latched"


CRZ_CTRL_TEMPLATES: dict[MazdaLongitudinalProfile, bytes] = {
  MazdaLongitudinalProfile.STANDBY: bytes.fromhex("0201010000000000"),
  MazdaLongitudinalProfile.ENGAGED_CRUISE: bytes.fromhex("0a018b2000001000"),
  MazdaLongitudinalProfile.ENGAGED_FOLLOW: bytes.fromhex("0a018b4000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD: bytes.fromhex("0a018b6000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED: bytes.fromhex("0a018b8000001000"),
}


def _get_signal(message_name: str, signal_name: str):
  return MAZDA_LONG_DBC.name_to_msg[message_name].sigs[signal_name]


def _patch_signal(message_name: str, raw: bytes, signal_name: str, value: float) -> bytes:
  sig = _get_signal(message_name, signal_name)
  encoded = int(round((value - sig.offset) / sig.factor))
  if encoded < 0:
    encoded = (1 << sig.size) + encoded

  dat = bytearray(raw)
  set_value(dat, sig, encoded)
  return bytes(dat)


def _crz_info_checksum(dat: bytes) -> int:
  """Invert the sum of the first seven bytes, EXCLUDING two bits from the sum.

  STOPPING (d[5] & 0x04) and RESUME_UNLATCHING (d[6] & 0x40) do not contribute. Verified
  against 52,442 stock CRZ_INFO frames captured from the car with the radar alive: every
  one of the 960 frames carrying either bit is reproduced exactly by this, and none by a
  plain sum.

  The previous implementation applied a +4 bias for STOPPING only -- arithmetically the
  same as excluding d[5] & 0x04 -- and handled RESUME_UNLATCHING not at all, so every frame
  in the resume unlatch sequence carried a checksum wrong by 0x40 and was rejected by the
  car. That is why the chassis never released HOLD.
  """
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04) - (dat[6] & 0x40)) & 0xFF)) & 0xFF


def _update_crz_info_checksum(raw: bytes) -> bytes:
  dat = bytearray(raw)
  dat[7] = _crz_info_checksum(dat)
  return bytes(dat)


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def _interp_scale(v_ego: float, bp: tuple[float, ...], values: tuple[float, ...]) -> float:
  if v_ego <= bp[0]:
    return values[0]
  if v_ego >= bp[-1]:
    return values[-1]

  for i in range(1, len(bp)):
    if v_ego <= bp[i]:
      x0, x1 = bp[i - 1], bp[i]
      y0, y1 = values[i - 1], values[i]
      ratio = (v_ego - x0) / (x1 - x0)
      return y0 + (y1 - y0) * ratio

  return values[-1]


def accel_to_accel_cmd(accel: float, v_ego: float) -> int:
  scale = _interp_scale(v_ego, ACCEL_SCALE_UP_BP, ACCEL_SCALE_UP_V) if accel >= 0.0 else _interp_scale(v_ego, ACCEL_SCALE_DOWN_BP, ACCEL_SCALE_DOWN_V)
  return int(round(clip(accel * scale, ACCEL_CMD_MIN, ACCEL_CMD_MAX)))


def accel_cmd_to_accel(accel_cmd: float, v_ego: float) -> float:
  """Inverse of accel_to_accel_cmd: a raw CRZ_INFO.ACCEL_CMD (e.g. the stock radar's last
  command) in m/s^2 on the same map, so a takeover can start from what MRCC was asking for."""
  accel_cmd = clip(accel_cmd, ACCEL_CMD_MIN, ACCEL_CMD_MAX)
  scale = _interp_scale(v_ego, ACCEL_SCALE_UP_BP, ACCEL_SCALE_UP_V) if accel_cmd >= 0.0 else _interp_scale(v_ego, ACCEL_SCALE_DOWN_BP, ACCEL_SCALE_DOWN_V)
  return accel_cmd / scale


def hold_brake_accel() -> float:
  # Stock HOLD keeps a strong negative CRZ_INFO command alive through the
  # active stop/hold phase until the chassis hold latch takes over.
  # Keep the raw target approximately constant as scales change.
  return HOLD_BRAKE_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def hold_latched_accel() -> float:
  # Once the chassis hold latch takes over, stock CRZ_INFO.ACCEL_CMD relaxes
  # back near zero and the stop bits clear.
  return HOLD_LATCHED_CMD_TARGET / ACCEL_SCALE_DOWN_V[0]


def near_stop_brake_accel(v_ego: float) -> float:
  # Stock stop-to-hold ramps into the final HOLD brake command before true
  # standstill, rather than waiting until the speed bit drops to zero.
  ratio = clip(v_ego / NEAR_STOP_ENTRY_SPEED, 0.0, 1.0)
  target = HOLD_BRAKE_CMD_TARGET + (NEAR_STOP_BRAKE_CMD_TARGET - HOLD_BRAKE_CMD_TARGET) * ratio
  return target / ACCEL_SCALE_DOWN_V[0]


def build_crz_info(accel: float, counter: int, long_active: bool, hold_request: bool, v_ego: float,
                   hold_latched: bool = False, acc_set_allowed: bool = False,
                   resume_unlatching: bool = False) -> bytes:
  if not long_active and not acc_set_allowed:
    raw = _patch_signal("CRZ_INFO", CRZ_INFO_STANDBY_TEMPLATE, "CTR1", counter % 16)
    return _update_crz_info_checksum(raw)

  stopping_active = hold_request and not hold_latched
  raw = _patch_signal("CRZ_INFO", CRZ_INFO_TEMPLATE, "ACCEL_CMD", accel_to_accel_cmd(accel, v_ego))
  raw = _patch_signal("CRZ_INFO", raw, "ACC_ACTIVE", int(long_active))
  raw = _patch_signal("CRZ_INFO", raw, "ACC_SET_ALLOWED", int(acc_set_allowed))
  raw = _patch_signal("CRZ_INFO", raw, "CRZ_ENDED", 0)
  raw = _patch_signal("CRZ_INFO", raw, "STOPPING_MAYBE", int(stopping_active))
  raw = _patch_signal("CRZ_INFO", raw, "STOPPING_MAYBE2", int(stopping_active))
  raw = _patch_signal("CRZ_INFO", raw, "RESUME_UNLATCHING_MAYBE", int(resume_unlatching))
  raw = _patch_signal("CRZ_INFO", raw, "CTR1", counter % 16)
  return _update_crz_info_checksum(raw)


def select_profile(long_active: bool, lead_visible: bool, hold_request: bool,
                   crz_hold_latched: bool) -> MazdaLongitudinalProfile:
  if not long_active:
    return MazdaLongitudinalProfile.STANDBY
  if hold_request and crz_hold_latched:
    return MazdaLongitudinalProfile.STOP_GO_HOLD_LATCHED
  if hold_request:
    return MazdaLongitudinalProfile.STOP_GO_HOLD
  if lead_visible:
    return MazdaLongitudinalProfile.ENGAGED_FOLLOW
  return MazdaLongitudinalProfile.ENGAGED_CRUISE


def build_crz_ctrl(long_active: bool, lead_visible: bool, hold_request: bool, hold_latched: bool,
                   crz_hold_latched: bool = False, crz_hold_passive: bool = False,
                   crz_resume_active: bool = False, crz_available: bool = False) -> bytes:
  # Stock stop-and-go progresses through multiple CRZ_CTRL stop phases. Mirror
  # that sequence so the synthetic path keeps the same latch states as stock.
  lead_visible = lead_visible or hold_request or hold_latched or crz_hold_latched or crz_hold_passive
  raw = CRZ_CTRL_TEMPLATES[select_profile(long_active, lead_visible, hold_request, crz_hold_latched)]
  raw = _patch_signal("CRZ_CTRL", raw, "CRZ_ACTIVE", int(long_active))
  raw = _patch_signal("CRZ_CTRL", raw, "CRZ_AVAILABLE", int(long_active or crz_available))
  raw = _patch_signal("CRZ_CTRL", raw, "ACC_ACTIVE_2", int(long_active and not crz_hold_passive))
  raw = _patch_signal("CRZ_CTRL", raw, "DISABLE_TIMER_1", 0)
  raw = _patch_signal("CRZ_CTRL", raw, "DISABLE_TIMER_2", 0)
  raw = _patch_signal("CRZ_CTRL", raw, "RADAR_HAS_LEAD", int(lead_visible))
  # Stock resume transitions rely on more than the coarse 0x21c templates. The
  # live radar path preserves these fields automatically, but the synthetic path
  # has to set them explicitly to match passive hold (distance 4), active
  # stop-go / resume (distance 3 + ACC_GAS_MAYBE2), and follow (distance 2).
  if crz_hold_passive or crz_hold_latched:
    raw = _patch_signal("CRZ_CTRL", raw, "RADAR_LEAD_RELATIVE_DISTANCE", 4)
    raw = _patch_signal("CRZ_CTRL", raw, "ACC_GAS_MAYBE2", 0)
  elif hold_request or crz_resume_active:
    raw = _patch_signal("CRZ_CTRL", raw, "RADAR_LEAD_RELATIVE_DISTANCE", 3)
    raw = _patch_signal("CRZ_CTRL", raw, "ACC_GAS_MAYBE2", 1)
  elif not long_active and crz_available:
    # In the synthetic pre-engage state, keep CRZ_CTRL's availability and
    # distance setting paired with CRZ_INFO.ACC_SET_ALLOWED.
    raw = _patch_signal("CRZ_CTRL", raw, "DISTANCE_SETTING", 2)
  return raw


def create_longitudinal_messages(bus: int, accel: float, counter: int, long_active: bool,
                                 lead_visible: bool, *, hold_request: bool = False,
                                 crz_ctrl_hold_request: bool | None = None,
                                 hold_latched: bool = False, crz_hold_latched: bool = False,
                                 crz_hold_passive: bool = False,
                                 crz_resume_active: bool = False,
                                 crz_info_resume_unlatching: bool = False,
                                 crz_available: bool = False,
                                 v_ego: float = 0.0) -> list[CanData]:
  if crz_ctrl_hold_request is None:
    crz_ctrl_hold_request = hold_request

  return [
    CanData(CRZ_INFO_ADDR, build_crz_info(accel, counter, long_active, hold_request, v_ego,
                                          hold_latched=hold_latched,
                                          acc_set_allowed=long_active or crz_available,
                                          resume_unlatching=crz_info_resume_unlatching), bus),
    CanData(CRZ_CTRL_ADDR, build_crz_ctrl(long_active, lead_visible, crz_ctrl_hold_request, hold_latched,
                                          crz_hold_latched=crz_hold_latched,
                                          crz_hold_passive=crz_hold_passive,
                                          crz_resume_active=crz_resume_active,
                                          crz_available=crz_available), bus),
  ]


def build_radar_track(raw: bytes, counter: int) -> bytes:
  dat = bytearray(raw)
  dat[7] = (dat[7] & 0xf0) | (counter % 16)
  return bytes(dat)


def create_radar_heartbeat_messages(bus: int, counter: int, synthetic_lead: bool = False) -> list[CanData]:
  static = _captured_radar_frames.get(RADAR_STATIC_ADDR, RADAR_STATIC_TEMPLATE)
  can_sends = [CanData(RADAR_STATIC_ADDR, static, bus)]
  for addr, template in zip(RADAR_TRACK_ADDRS, RADAR_TRACK_EMPTY_TEMPLATES, strict=True):
    raw = _captured_radar_frames.get(addr, template)
    # The synthetic-lead payload is a CX-5 capture. Injecting it into a car whose own radar
    # frames we are otherwise replaying would put a pattern on the bus that this car's radar
    # never produces -- the exact problem the capture exists to avoid. Only use it when we
    # have no capture for this address. A verified lead-present capture from this vehicle
    # would be needed to signal a lead faithfully.
    if synthetic_lead and addr == RADAR_SYNTHETIC_LEAD_TRACK_ADDR and addr not in _captured_radar_frames:
      raw = RADAR_SYNTHETIC_LEAD_TRACK_TEMPLATE
    can_sends.append(CanData(addr, build_radar_track(raw, counter), bus))
  return can_sends


def create_radar_tester_present(bus: int = RADAR_BUS) -> CanData:
  return make_tester_present_msg(RADAR_ADDR, bus, suppress_response=True)


def _uds_request(can_recv, can_send, bus: int, addr: int, request: bytes, response: bytes,
                 *, timeout: float = 0.1) -> bool:
  query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr, None)], [request], [response])
  return len(query.get_data(timeout)) > 0


def enter_radar_programming_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR,
                                    retry: int = 10, timeout: float = 0.2) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.PROGRAMMING])

  for attempt in range(retry):
    try:
      if _uds_request(can_recv, can_send, bus, addr, request, response, timeout=timeout):
        carlog.warning(f"mazda radar programming session enabled on {hex(addr)}")
        return True
    except Exception:
      carlog.exception("mazda radar programming session exception")
    carlog.error(f"mazda radar programming session retry ({attempt + 1})")

  carlog.error("mazda radar programming session failed")
  return False


def request_radar_default_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.DEFAULT])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.DEFAULT])

  try:
    return _uds_request(can_recv, can_send, bus, addr, request, response)
  except Exception:
    carlog.exception("mazda radar default session exception")
    return False
