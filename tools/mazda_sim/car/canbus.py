"""CAN plumbing for the simulated car: networks, ECUs and their periodic messages."""
from __future__ import annotations

from collections.abc import Callable

from opendbc.can.dbc import DBC
from opendbc.can.packer import set_value

from ..common.wire import Frame, NET_CAM, NET_CAR

DBC_NAME = "mazda_2017"
_dbc = DBC(DBC_NAME)


def encode(name: str, values: dict[str, float], template: bytes | None = None) -> bytes:
  """Pack signals into a message, optionally on top of a captured template. No checksum is added."""
  msg = _dbc.name_to_msg[name]
  dat = bytearray(template) if template is not None else bytearray(msg.size)
  for sig_name, value in values.items():
    sig = msg.sigs[sig_name]
    ival = int(round((value - sig.offset) / sig.factor))
    if ival < 0:
      ival += 1 << sig.size
    ival &= (1 << sig.size) - 1
    set_value(dat, sig, ival)
  return bytes(dat)


def decode(name: str, dat: bytes) -> dict[str, float]:
  msg = _dbc.name_to_msg[name]
  out = {}
  for sig in msg.sigs.values():
    raw = 0
    # same bit walk as opendbc's parser
    i = sig.msb // 8
    bits = sig.size
    while 0 <= i < len(dat) and bits > 0:
      lsb = sig.lsb if (sig.lsb // 8) == i else i * 8
      msb = sig.msb if (sig.msb // 8) == i else (i + 1) * 8 - 1
      size = msb - lsb + 1
      d = (dat[i] >> (lsb - (i * 8))) & ((1 << size) - 1)
      raw |= d << (bits - size)
      bits -= size
      i = i - 1 if sig.is_little_endian else i + 1
    if sig.is_signed and raw & (1 << (sig.size - 1)):
      raw -= 1 << sig.size
    out[sig.name] = raw * sig.factor + sig.offset
  return out


def addr_of(name: str) -> int:
  return _dbc.name_to_msg[name].address


def mazda_sum_checksum(dat: bytes | bytearray) -> int:
  """The inverted byte sum most GEN1 chassis frames carry in their last byte."""
  return (~sum(dat[:7])) & 0xFF


def with_checksum(dat: bytes) -> bytes:
  d = bytearray(dat)
  d[7] = mazda_sum_checksum(d)
  return bytes(d)


class Periodic:
  """A message sent every 1/hz seconds. Non-integer tick ratios (83 Hz on a 100 Hz tick) are spread evenly."""

  def __init__(self, name_or_addr: str | int, hz: float, build: Callable[[], bytes | None], phase: float = 0.0):
    self.addr = name_or_addr if isinstance(name_or_addr, int) else addr_of(name_or_addr)
    self.hz = hz
    self.build = build
    self.acc = phase

  def due(self, dt: float) -> bool:
    self.acc += self.hz * dt
    if self.acc >= 1.0:
      self.acc -= 1.0
      if self.acc >= 1.0:  # never burst after a stall
        self.acc = 0.0
      return True
    return False


class Ecu:
  """One module on a network. Subclasses fill self.periodics and override on_frame()/step()."""
  name = "ecu"
  net = NET_CAR

  def __init__(self):
    self.periodics: list[Periodic] = []
    self.powered = True
    self.silent = False  # e.g. a radar held in its programming session

  def step(self, dt: float) -> None:
    """Advance internal state by dt."""

  def on_frame(self, frame: Frame) -> None:
    """A frame seen on this ECU's network (from another ECU or from the comma)."""

  def emit(self, dt: float) -> list[Frame]:
    out = []
    for p in self.periodics:
      if p.due(dt) and self.powered and not self.silent:
        dat = p.build()
        if dat is not None:
          out.append(Frame(self.net, p.addr, dat))
    return out


class Networks:
  """The car's CAN wiring as seen from the harness.

  With the harness relay passing through (comma unplugged, or the panda in a no-output mode) the Forward
  Sensing Camera is wired straight to the car: CAR and CAM are one network. With the relay intercepting they
  are two, joined only by whatever the panda forwards.
  """

  def __init__(self):
    self.ecus: list[Ecu] = []
    self.relay_intercept = False

  def add(self, ecu: Ecu) -> Ecu:
    self.ecus.append(ecu)
    return ecu

  def reaches(self, src_net: int, ecu_net: int) -> bool:
    if src_net == ecu_net:
      return True
    joined = not self.relay_intercept
    return joined and {src_net, ecu_net} == {NET_CAR, NET_CAM}

  def deliver(self, frames: list[Frame], exclude: Ecu | None = None) -> None:
    for f in frames:
      for ecu in self.ecus:
        if ecu is not exclude and ecu.powered and self.reaches(f.net, ecu.net):
          ecu.on_frame(f)

  def tick(self, dt: float) -> list[Frame]:
    """Step every ECU and return the frames they put on the wire this tick."""
    produced: list[tuple[Ecu, list[Frame]]] = []
    for ecu in self.ecus:
      ecu.step(dt)
      produced.append((ecu, ecu.emit(dt)))
    out = []
    for ecu, frames in produced:
      self.deliver(frames, exclude=ecu)
      out.extend(frames)
    return out
