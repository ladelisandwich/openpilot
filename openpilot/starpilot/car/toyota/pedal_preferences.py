from opendbc.car.toyota import pedal
from openpilot.starpilot.car.toyota.prius_preferences import filter_preference_enabled


class PedalPreference:
  def __init__(self, cp, params):
    self.cp, self.params = cp, params
    self.read_ns = 0
    self.enabled = False

  def update(self, now):
    if not pedal.configured(self.cp) or not self.cp.openpilotLongitudinalControl or now <= 0 or now < self.read_ns:
      self.enabled = False
      self.read_ns = 0
      return False
    if self.read_ns == 0 or now - self.read_ns >= 250_000_000:
      self.enabled = filter_preference_enabled(self.params)
      self.read_ns = now
    return self.enabled
