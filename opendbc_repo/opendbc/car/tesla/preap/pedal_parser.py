"""Actual selected-bus CANParser plus original-source pedal feedback provenance."""
from opendbc.can import CANParser


class PedalCANParser(CANParser):
  def __init__(self, feedback):
    # Dom's 0x552 format is validated by PedalFeedback, independently of
    # generic interceptor DBC messages and their unrelated checksum rules.
    super().__init__('tesla_can', [], feedback.calibration.bus)
    self.feedback = feedback

  def update(self, strings, sendcan=False):
    if strings and not isinstance(strings[0], list | tuple):
      strings = [strings]
    for stamp, frames in strings:
      for address, payload, bus in frames:
        if address == 0x552 and bus == self.feedback.calibration.bus:
          self.feedback.observe(bytes(payload), bus, int(stamp))
    return super().update(strings, sendcan)
