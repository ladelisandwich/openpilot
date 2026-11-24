from opendbc.car.toyota.rav4h_filter import enabled
from openpilot.starpilot.car.toyota.prius_preferences import filter_preference_enabled


class Rav4hFilterPreference:
  def __init__(self, cp, params):
    self.cp, self.params = cp, params
    self.read_ns = 0
    self.enabled = False

  def update(self, now_ns):
    if type(now_ns) is not int or now_ns <= 0 or now_ns < self.read_ns or not enabled(self.cp):
      self.read_ns = 0
      self.enabled = False
      return False
    if self.read_ns == 0 or now_ns - self.read_ns >= 250_000_000:
      self.read_ns = now_ns
      self.enabled = filter_preference_enabled(self.params)
    return self.enabled
