from opendbc.car import structs
from opendbc.car.hyundai.ev9_longitudinal import qualified as ev9_qualified
from opendbc.car.hyundai.ioniq5pe_long_profile import qualified as pe_qualified
from opendbc.car.hyundai.values import CAR, HyundaiFlags, Buttons


def qualified(cp):
  if ev9_qualified(cp) or pe_qualified(cp):
    return True
  dynamic = int(HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.USE_FCA | HyundaiFlags.SEND_LFA)
  declared = int(CAR.HYUNDAI_ELANTRA_HEV_2024.config.flags)
  return bool(cp.brand == 'hyundai' and cp.carFingerprint == CAR.HYUNDAI_ELANTRA_HEV_2024 and
    int(cp.flags) & ~dynamic == declared & ~dynamic and not cp.passive and not cp.notCar and not cp.dashcamOnly and
    cp.openpilotLongitudinalControl and not cp.pcmCruise and cp.alphaLongitudinalAvailable and
    cp.steerControlType == structs.CarParams.SteerControlType.torque and cp.alternativeExperience == 0 and
    len(cp.safetyConfigs) == 1 and cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.hyundai and
    cp.safetyConfigs[0].safetyParam == 0x800E)


class LongCruiseButtons:
  def __init__(self, *, track_main):
    self.track_main = track_main
    self.main_on = False
    self.adaptive_held = False
    self.normal_held = False
    self.button = Buttons.NONE
    self.blocked = False
    self.button_enable = False
    self.cruise_transitions = ()
    self.main_transitions = ()
    self.main_seen = False
    self.normal_seen = False

  def observe(self, samples):
    cruise, main = [], []
    self.button_enable = self.main_seen = self.normal_seen = False
    cancel_seen = False
    for adaptive, normal, button in samples:
      adaptive, normal, button = bool(adaptive), bool(normal), int(button)
      any_main = adaptive or normal
      self.main_seen |= any_main
      self.normal_seen |= normal
      cancel_seen |= button == Buttons.CANCEL
      if adaptive != self.adaptive_held:
        main.append((int(adaptive), int(self.adaptive_held)))
        if adaptive:
          self.main_on = not self.main_on
      if button != self.button:
        cruise.append((button, self.button))
      if any_main or button == Buttons.CANCEL or self.blocked or (self.track_main and not self.main_on):
        self.blocked = any_main or button != Buttons.NONE
      elif button != self.button and self.button in (Buttons.SET_DECEL, Buttons.RES_ACCEL):
        self.button_enable = True
      self.button = button
      self.adaptive_held, self.normal_held = adaptive, normal
    if self.main_seen or cancel_seen:
      self.button_enable = False
    self.cruise_transitions, self.main_transitions = tuple(cruise), tuple(main)

  def available(self, source_available):
    if self.track_main:
      return bool(source_available and self.main_on and not self.normal_held and not self.normal_seen)
    return bool(source_available and not self.adaptive_held and not self.normal_held and not self.main_seen)
