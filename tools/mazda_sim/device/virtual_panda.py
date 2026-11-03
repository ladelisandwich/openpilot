"""A panda that lives in software: pandad + the panda firmware, in front of the simulated car's wiring.

Mirrors, for the parts openpilot can observe:
  firmware (panda/board/main.c, drivers/can_common.h, drivers/fdcan.h)
    - safety mode -> harness relay (intercept for car modes; pass-through for SILENT/NOOUTPUT/ELM327)
    - bus 1 -> harness OBD lines for ELM327 param 0 and for MAZDA GEN1 + Torque Interceptor (CAN_MODE_OBD_CAN2)
    - every received frame: forwarding decision (safety_fwd_hook), then safety_rx_hook, then to the host
    - every host frame: safety_tx_hook; sent frames come back with src + 128, rejected ones with src + 192
    - safety_tick once a second: rx checks that lag drop controls_allowed
  pandad (selfdrive/pandad/panda_safety.cc, pandad.cc)
    - ELM327 for fingerprinting, ObdMultiplexingEnabled -> ELM327 param, ObdMultiplexingChanged ack
    - FirmwareQueryDone + ControlsReady -> safety mode from CarParams (+ StarPilotCarParams param bits)
    - offroad or ignition off -> NOOUTPUT
The safety code itself is the build's own (device/libsafety.py), so its decisions are the real ones.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from ..common.wire import Frame, NET_AUX, NET_CAM, NET_CAR
from .libsafety import Safety

SAFETY_SILENT = 0
SAFETY_ELM327 = 3
SAFETY_MAZDA = 13
SAFETY_NOOUTPUT = 19
PASS_THROUGH_MODES = (SAFETY_SILENT, SAFETY_NOOUTPUT, SAFETY_ELM327)
MAZDA_FLAG_GEN1 = 1
MAZDA_FLAG_TORQUE_INTERCEPTOR = 8

RETURNED = 0x80
REJECTED = 0xC0


@dataclass
class PandaCounters:
  tx_blocked: int = 0
  rx_invalid: int = 0
  tx_sent: int = 0
  rx_frames: int = 0
  blocked_by_addr: dict = field(default_factory=dict)


class VirtualPanda:
  def __init__(self, safety: Safety):
    self.safety = safety
    self.mode = SAFETY_SILENT
    self.param = 0
    self.alternative_experience = 0
    self.relay_intercept = False
    self.obd_mode = False
    self.relay_stuck = False         # fault injection: the relay never opens (stuck in pass-through)
    self.ignition = False
    self.counters = PandaCounters()
    self.host_rx: list[tuple[int, bytes, int]] = []   # (addr, dat, src) waiting for the next `can` message
    self.t_us = 0
    self.next_tick_us = 0
    self.set_mode(SAFETY_NOOUTPUT, 0)

  # ---- mode ----
  def set_mode(self, mode: int, param: int) -> None:
    if not self.safety.set_mode(mode, param):
      mode, param = SAFETY_SILENT, 0
      self.safety.set_mode(mode, param)
    self.safety.lib.set_alternative_experience(self.alternative_experience)
    self.mode, self.param = mode, param
    self.counters.tx_blocked = 0
    self.counters.rx_invalid = 0
    self.counters.blocked_by_addr = {}
    self.relay_intercept = mode not in PASS_THROUGH_MODES and not self.relay_stuck
    if mode == SAFETY_ELM327:
      self.obd_mode = param == 0
    elif mode == SAFETY_MAZDA:
      self.obd_mode = bool(param & MAZDA_FLAG_GEN1) and bool(param & MAZDA_FLAG_TORQUE_INTERCEPTOR)
    else:
      self.obd_mode = False

  def set_alternative_experience(self, alt: int) -> None:
    self.alternative_experience = alt
    self.safety.lib.set_alternative_experience(alt)

  # ---- wiring ----
  @property
  def joined(self) -> bool:
    return not self.relay_intercept

  def buses_for_net(self, net: int) -> tuple[int, ...]:
    if net == NET_CAR:
      return (0, 2) if self.joined else (0,)
    if net == NET_CAM:
      return (2, 0) if self.joined else (2,)
    if net == NET_AUX:
      return (1,) if self.obd_mode else ()
    return ()

  @staticmethod
  def net_for_bus(bus: int, obd_mode: bool) -> int | None:
    if bus == 0:
      return NET_CAR
    if bus == 2:
      return NET_CAM
    if bus == 1 and obd_mode:
      return NET_AUX
    return None

  # ---- time ----
  def advance(self, t_us: int) -> None:
    self.t_us = t_us
    self.safety.set_timer(t_us)
    if t_us >= self.next_tick_us:
      self.safety.tick()
      self.next_tick_us = t_us + 1_000_000

  # ---- car -> panda ----
  def receive(self, frames: list[Frame]) -> list[Frame]:
    """Frames the car put on its networks. Returns the frames the panda forwards back onto the car's wiring."""
    out: list[Frame] = []
    for f in frames:
      for bus in self.buses_for_net(f.net):
        self._rx(bus, f.addr, f.dat, out)
    return out

  def _rx(self, bus: int, addr: int, dat: bytes, out: list[Frame]) -> None:
    self.counters.rx_frames += 1
    if self.relay_intercept:
      fwd = self.safety.fwd(bus, addr)
      if fwd >= 0:
        net = self.net_for_bus(fwd, self.obd_mode)
        if net is not None:
          out.append(Frame(net, addr, dat))
          self.host_rx.append((addr, dat, fwd + RETURNED))
    if not self.safety.rx(bus, addr, dat):
      self.counters.rx_invalid += 1
    self.host_rx.append((addr, dat, bus))

  # ---- openpilot -> panda ----
  def send(self, msgs: list[tuple[int, bytes, int]]) -> list[Frame]:
    """sendcan from openpilot. Returns frames that made it onto the car's wiring."""
    out: list[Frame] = []
    for addr, dat, bus in msgs:
      if bus > 2:
        continue
      allowed = self.mode != SAFETY_SILENT and self.safety.tx(bus, addr, dat)
      net = self.net_for_bus(bus, self.obd_mode)
      if not allowed:
        self.counters.tx_blocked += 1
        self.counters.blocked_by_addr[f"{bus}:{addr:#x}"] = self.counters.blocked_by_addr.get(f"{bus}:{addr:#x}", 0) + 1
        self.host_rx.append((addr, dat, bus + REJECTED))
        continue
      self.counters.tx_sent += 1
      self.host_rx.append((addr, dat, bus + RETURNED))
      if net is None:
        continue  # bus 1 not switched to the OBD lines: nothing is wired there
      out.append(Frame(net, addr, dat))
      if self.joined and bus in (0, 2):
        # one physical bus: the panda's other transceiver hears it like any other frame
        other = 2 if bus == 0 else 0
        dummy: list[Frame] = []
        self._rx(other, addr, dat, dummy)
    return out

  def drain_host(self) -> list[tuple[int, bytes, int]]:
    out, self.host_rx = self.host_rx, []
    return out

  def state(self) -> dict:
    s = self.safety
    return {
      "safetyModel": self.mode, "safetyParam": self.param, "alternativeExperience": self.alternative_experience,
      "controlsAllowed": s.controls_allowed, "relayIntercept": self.relay_intercept, "obdMode": self.obd_mode,
      "relayMalfunction": s.relay_malfunction, "rxChecksInvalid": s.rx_checks_invalid, "ignition": self.ignition,
      "txBlocked": self.counters.tx_blocked, "rxInvalid": self.counters.rx_invalid, "txSent": self.counters.tx_sent,
      "blockedByAddr": dict(sorted(self.counters.blocked_by_addr.items(), key=lambda kv: -kv[1])[:6]),
    }
