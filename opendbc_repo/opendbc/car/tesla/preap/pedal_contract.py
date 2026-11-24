"""Frozen working Dom pedal calibration and0x551/0x552 wire contract."""
from dataclasses import dataclass
import math
import struct

M1 = 0.050796813
M2 = 0.101593626
D = -22.85856576
FEEDBACK_TIMEOUT_NS = 500_000_000


def float32(value):
  return struct.unpack('<f', struct.pack('<f', value))[0]


@dataclass(frozen=True)
class PedalCalibration:
  factor: float
  zero: float
  bus: int

  def valid(self):
    try:
      if (type(self.bus) is not int or self.bus not in (0, 2) or
          not math.isfinite(self.factor) or abs(self.factor) <= 1e-6 or not math.isfinite(self.zero) or
          self.factor != float32(self.factor) or self.zero != float32(self.zero)):
        return False
      # Preserve Dom's clipping at the electrical floor. A saturated endpoint
      # may not create incompatible values for the two physical channels.
      for di in (-5., 90.):
        one = max(0, min(65534, int((self.physical(di) - D) / M1)))
        two = max(0, min(65534, int((self.physical(di) - D) / M2)))
        if not 0 <= one - 2 * two <= 1:
          return False
      return True
    except (ValueError, TypeError, OverflowError, struct.error):
      return False

  def physical(self, di):
    return self.zero + di / self.factor

  def di(self, physical):
    return (physical - self.zero) * self.factor


def checksum(address, first_five):
  return ((address & 255) + (address >> 8) + sum(first_five)) & 255


def pedal_max(speed):
  bp = (0., 5., 12., 20., 30., 40.)
  values = (50., 58., 66., 74., 82., 90.)
  if speed <= 0:
    return values[0]
  for index in range(1, len(bp)):
    if speed <= bp[index]:
      ratio = (speed - bp[index - 1]) / (bp[index] - bp[index - 1])
      return values[index - 1] + ratio * (values[index] - values[index - 1])
  return values[-1]


class PedalCodec:
  def __init__(self, calibration):
    if type(calibration) is not PedalCalibration or not calibration.valid():
      raise ValueError('invalid frozen Tesla pedal calibration')
    self.calibration = calibration
    self.counter = 0

  def command(self, di=0., *, enabled=False):
    if type(enabled) is not bool or not math.isfinite(di):
      raise ValueError('invalid Tesla pedal demand')
    one = two = 0
    if enabled:
      if not -5. <= di <= 90.:
        raise ValueError('Tesla pedal DI outside original envelope')
      physical = self.calibration.physical(di)
      one = max(0, min(65534, int((physical - D) / M1)))
      two = max(0, min(65534, int((physical - D) / M2)))
      if not 0 <= one <= 65534 or not 0 <= two <= 65534 or abs(one - 2 * two) > 1:
        raise ValueError('invalid dual-channel Tesla pedal command')
    prefix = struct.pack('>HHB', one, two, (int(enabled) << 7) | self.counter)
    self.counter = (self.counter + 1) & 15
    return 0x551, prefix + bytes([checksum(0x551, prefix)]), self.calibration.bus


class PedalFeedback:
  def __init__(self, calibration):
    if type(calibration) is not PedalCalibration or not calibration.valid():
      raise ValueError('invalid frozen Tesla pedal calibration')
    self.calibration = calibration
    self.last_counter = None
    self.source_ns = 0
    self.last_observed_ns = 0
    self.healthy = False
    self.value = self.value2 = 0.
    self.torque_level = 0.

  def clear(self):
    self.healthy = False
    self.source_ns = 0
    self.last_counter = None

  def observe(self, payload, bus, source_ns):
    if (type(payload) is not bytes or len(payload) != 6 or bus != self.calibration.bus or
        type(source_ns) is not int or source_ns < self.last_observed_ns):
      self.clear()
      return False
    self.last_observed_ns = source_ns
    one, two, flags = struct.unpack('>HHB', payload[:5])
    state, counter = flags >> 4, flags & 15
    # Dom did not define a feedback checksum validator. Do not substitute the
    # unrelated upstream pedal CRC. Keep state/counter/dual-channel provenance.
    physical_one, physical_two = one * M1 + D, two * M2 + D
    if (state != 0 or one == 65535 or two == 65535 or
        abs(physical_one - physical_two) > 2 * M2):
      self.clear()
      return False
    previous = self.last_counter
    self.last_counter = counter
    self.healthy = previous is not None and counter == ((previous + 1) & 15)
    self.source_ns = source_ns if self.healthy else 0
    self.value = self.calibration.di(physical_one)
    self.value2 = self.calibration.di(physical_two)
    return self.healthy

  def ready(self, now_ns):
    return bool(self.healthy and self.source_ns > 0 and self.source_ns <= now_ns and
                now_ns - self.source_ns <= FEEDBACK_TIMEOUT_NS)

  def gas_pressed(self):
    return self.value > 2. or self.value2 > 2.
