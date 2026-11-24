"""Bounded receive-only vehicle observations while normal driving services are off.

This creates only existing CarState decoders and CAN parsers. It never creates a
CarInterface/CarController, fingerprints, initializes ECUs, or publishes CAN.
The caller must independently verify physical Panda noOutput and process state.
"""

from dataclasses import dataclass
import importlib
import math
import time

from openpilot.cereal import messaging
from opendbc.car.can_definitions import CanData
from opendbc.car.values import PLATFORMS

MAX_AGE_NS = 250_000_000
MAX_PACKETS = 32
MAX_FRAMES = 512
DWELL_NS = 1_000_000_000


@dataclass(frozen=True)
class ParkedSample:
  source_boot_ns: int
  received_mono_ns: int
  buttons: tuple[tuple[str, bool], ...]


def stationary(state) -> bool:
  values = (state.vEgo, state.vEgoRaw, state.wheelSpeeds.fl, state.wheelSpeeds.fr,
            state.wheelSpeeds.rl, state.wheelSpeeds.rr)
  return bool(state.canValid and not state.canTimeout and state.standstill and str(state.gearShifter) == 'park' and
              not state.gasPressed and not state.cruiseState.enabled and
              all(math.isfinite(v) and abs(v) <= .01 for v in values))


class ParkedVehicleReader:
  """Independent raw CAN subscriber with original parser checksum/counter checks."""
  def __init__(self, cp):
    platform = PLATFORMS.get(cp.carFingerprint)
    if cp.notCar or platform is None or platform.__module__.split('.')[-2] != cp.brand:
      raise ValueError('A recognized current vehicle identity is required')
    # Detached copy prevents a UI/manager transition changing parser identity.
    self.cp = cp.as_builder().as_reader()
    state_type = importlib.import_module(f'opendbc.car.{cp.brand}.carstate').CarState
    self.state = state_type(self.cp)
    self.parsers = self.state.get_can_parsers(self.cp)
    if not self.parsers or any(parser is None for parser in self.parsers.values()):
      raise ValueError('Vehicle does not expose a complete read-only parser')
    self.socket = messaging.sub_sock('can', conflate=False)
    self.after_boot_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    self.last_source_ns = 0
    self.sample: ParkedSample | None = None
    self.since_ns: int | None = None
    self.closed = False

  def poll(self) -> tuple[ParkedSample, ...]:
    if self.closed:
      return ()
    samples = []
    for _ in range(MAX_PACKETS):
      message = messaging.recv_one_or_none(self.socket)
      if message is None:
        return tuple(samples)
      # Sample AFTER receiving: a publisher may commit a new packet during recv.
      received = time.monotonic_ns()
      boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
      stamp = int(message.logMonoTime)
      if stamp <= self.after_boot_ns:
        continue
      if (not message.valid or stamp <= self.last_source_ns or not 0 <= boot - stamp <= MAX_AGE_NS or
          len(message.can) > MAX_FRAMES):
        raise ValueError('CAN observation is stale, invalid or out of order')
      self.last_source_ns = stamp
      packets = [(stamp, [CanData(int(frame.address), bytes(frame.dat), int(frame.src)) for frame in message.can])]
      for parser in self.parsers.values():
        parser.update(packets)
      state = self.state.update(self.parsers)
      state.canValid = all(parser.can_valid for parser in self.parsers.values())
      state.canTimeout = any(parser.bus_timeout for parser in self.parsers.values())
      self.state.out = state
      # Parser can_valid intentionally tolerates a few failures and learns some
      # frequencies. The game additionally requires fresh actual required frames;
      # receiving unrelated CAN cannot keep a remembered Park/zero speed alive.
      required = [message for parser in self.parsers.values() for message in parser.message_states.values() if not message.ignore_alive]
      fresh = bool(required) and all(message.timestamps and
                                    0 <= stamp - message.timestamps[-1] <= MAX_AGE_NS for message in required)
      if not stationary(state) or not fresh:
        self.sample = None
        self.since_ns = None
        continue
      if self.sample is None or not 0 <= received - self.sample.received_mono_ns <= MAX_AGE_NS:
        self.since_ns = received
      self.sample = ParkedSample(stamp, received, tuple((str(e.type), bool(e.pressed)) for e in state.buttonEvents))
      samples.append(self.sample)
    raise ValueError('CAN observation exceeded the bounded drain')

  def ready(self, now_mono_ns: int, now_boot_ns: int) -> bool:
    sample = self.sample
    return bool(not self.closed and sample is not None and self.since_ns is not None and
                now_mono_ns - self.since_ns >= DWELL_NS and
                0 <= now_mono_ns - sample.received_mono_ns <= MAX_AGE_NS and
                0 <= now_boot_ns - sample.source_boot_ns <= MAX_AGE_NS)

  def close(self) -> None:
    self.closed = True
    self.socket = None
    self.sample = None
    self.since_ns = None
    self.parsers.clear()
