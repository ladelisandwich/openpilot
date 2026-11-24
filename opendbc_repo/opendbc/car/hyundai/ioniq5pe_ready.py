from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.torque_ev_scc import clock_boot_ns


class PEReadySource:
  def __init__(self):
    self.observation = None
    self.last_stamp = 0
    self.last_counter = None

  def observe(self, packets):
    now = clock_boot_ns()
    if now <= 0:
      self.observation = None
      return
    self.update(packets, now)

  def update(self, packets, now_boot_ns):
    for stamp, frames in packets:
      # One CAN event can contain consecutive frames with the same producer
      # time. Qualify its timestamp once; replaying that event still fails.
      packet_current = type(stamp) is int and self.last_stamp < stamp <= now_boot_ns
      for address, data, bus in frames:
        if address != 0x35 or bus != 1:
          continue
        if not packet_current or len(data) != 32:
          self.observation = None
          continue
        self.last_stamp = stamp
        if int.from_bytes(data[:2], 'little') != hkg_can_fd_checksum(address, None, bytearray(data)):
          self.observation = None
          continue
        counter = data[2]
        advancing = self.last_counter is not None and counter == (self.last_counter + 1) % 256
        self.last_counter = counter
        self.observation = (stamp, bool(data[3] & 0x40)) if advancing else None

  def current(self, now_boot_ns, floor_ns, max_age_ns):
    if self.observation is None:
      return None
    stamp, ready = self.observation
    if not floor_ns < stamp <= now_boot_ns or now_boot_ns - stamp > max_age_ns:
      return None
    return ready
