"""Exact Lexus IS DSU MAIN source, paired with the native 303031 us lease."""
from opendbc.can import CANParser


class LexusIsCANParser(CANParser):
  def __init__(self, *args):
    super().__init__(*args)
    self.main_valid = False

  def update(self, strings, sendcan=False):
    batches = []
    for stamp, frames in strings:
      accepted = []
      for address, data, bus in frames:
        if address == 0x365 and bus == self.bus:
          self.main_valid = len(data) == 7
          if not self.main_valid:
            continue
        accepted.append((address, data, bus))
      batches.append((stamp, accepted))
    return super().update(batches, sendcan)
