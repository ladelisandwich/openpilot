"""The body switches the Mazda GEN1 car state reads off the decoded signals: parking brake, liftgate, auto hold,
TCS off, the stalks and hazards, the lane-keep button, the gear gap, and the fields handed to StarPilot."""
from cereal import custom
from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


class Toggles:
  def __getattr__(self, name):
    return False


class Car:
  """A GEN1 TI car fed one bus-0 frame set per 10 ms step; messages not mentioned keep their last frame."""
  def __init__(self):
    CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX9_2021)
    CP.flags = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR)
    self.CS = CarInterface(CP, custom.StarPilotCarParams.new_message()).CS
    self.parsers = CarState.get_can_parsers(CP)
    self.packer = CANPacker("mazda_2017")
    self.t = 0
    raw_by_name = {name: raw for raw, name in self.CS.shifter_values.items()}
    self.park, self.reverse = raw_by_name["P"], raw_by_name["R"]
    # the parser subscribes a message the first time the car state reads it; one empty update does that for all
    self.CS.update(self.parsers, Toggles())

  def step(self, **msgs):
    self.t += 10_000_000
    frames = [self.packer.make_can_msg(name, 0, values) for name, values in msgs.items()]
    for cp in self.parsers.values():
      cp.update([(self.t, frames)])
    return self.CS.update(self.parsers, Toggles())


def test_parking_brake_and_auto_hold():
  car = Car()
  ret, fp = car.step(EPB={"EPB_ACTIVE": 1, "HOLD_STATE": 3})
  assert ret.parkingBrake and car.CS.auto_hold_active and fp.autoHoldActive
  assert not ret.brakeHoldActive   # deliberately not the standard field: it would refuse engagement while holding
  ret, fp = car.step(EPB={"EPB_ACTIVE": 0, "HOLD_STATE": 2})
  assert not ret.parkingBrake and not car.CS.auto_hold_active and not fp.autoHoldActive


def test_liftgate_counts_as_a_door():
  car = Car()
  assert car.step(DOORS={"LIFTGATE_OPEN": 1})[0].doorOpen
  assert not car.step(DOORS={"LIFTGATE_OPEN": 0})[0].doorOpen
  assert car.step(DOORS={"FL": 1})[0].doorOpen


def test_tcs_off_is_esp_disabled():
  car = Car()
  assert car.step(TRACTION={"TCS_OFF": 1})[0].espDisabled
  assert not car.step(TRACTION={"TCS_OFF": 0})[0].espDisabled


def test_the_stalk_is_the_blinker_and_hazards_are_not():
  car = Car()
  ret, _ = car.step(TURN_SWITCH={"TURN_LEFT_SWITCH": 1})
  assert ret.leftBlinker and not ret.rightBlinker          # at once, before any lamp has flashed
  ret, _ = car.step(TURN_SWITCH={"TURN_LEFT_SWITCH": 0})
  assert not ret.leftBlinker                                # and off with the stalk, no hold
  ret, fp = car.step(TURN_SWITCH={"HAZARD": 1}, BLINK_INFO={"LEFT_BLINK": 1, "RIGHT_BLINK": 1})
  assert not ret.leftBlinker and not ret.rightBlinker
  assert fp.hazardLights and car.CS.hazard_lights
  ret, fp = car.step(TURN_SWITCH={"HAZARD": 0}, BLINK_INFO={"LEFT_BLINK": 0, "RIGHT_BLINK": 0})
  assert not fp.hazardLights and not car.CS.hazard_lights


def test_a_one_touch_signal_holds_across_the_flash_gap():
  car = Car()
  assert car.step(BLINK_INFO={"LEFT_BLINK": 1})[0].leftBlinker
  for _ in range(30):
    ret, _ = car.step(BLINK_INFO={"LEFT_BLINK": 0})
  assert ret.leftBlinker                                    # the lamp's off phase is bridged
  for _ in range(20):
    ret, _ = car.step(BLINK_INFO={"LEFT_BLINK": 0})
  assert not ret.leftBlinker


def test_the_lane_keep_button_is_the_lkas_button():
  car = Car()
  ret, _ = car.step(MSG_05={"LAS_BUTTON": 1})
  assert [(be.type, be.pressed) for be in ret.buttonEvents] == [(ButtonType.lkas, True)]
  ret, _ = car.step(MSG_05={"LAS_BUTTON": 1})
  assert not ret.buttonEvents
  ret, _ = car.step(MSG_05={"LAS_BUTTON": 0})
  assert [(be.type, be.pressed) for be in ret.buttonEvents] == [(ButtonType.lkas, False)]


def test_the_gear_gap_keeps_the_last_position_and_trusts_the_reverse_lamp():
  car = Car()
  assert car.step(GEAR={"GEAR": car.park})[0].gearShifter == GearShifter.park
  assert car.step(GEAR={"GEAR": 0})[0].gearShifter == GearShifter.park            # the lever on its way
  assert car.step(GEAR={"GEAR": 0}, MSG_11={"REVERSE_LAMP": 1})[0].gearShifter == GearShifter.reverse
  assert car.step(GEAR={"GEAR": car.reverse})[0].gearShifter == GearShifter.reverse
  fresh = Car()
  assert fresh.step(GEAR={"GEAR": 0})[0].gearShifter == GearShifter.unknown      # nothing to hold yet


def test_the_fields_handed_to_starpilot():
  car = Car()
  _, fp = car.step(BLINK_INFO={"WIPER_LO": 1}, TURN_SWITCH={"HORN": 15}, MSG_11={"LAMPS_ON": 1})
  assert fp.wipersActive and fp.hornPressed and fp.lampsOn and not fp.washerActive
  _, fp = car.step(BLINK_INFO={"WIPER_LO": 0, "WIPER_HI": 1, "FRONT_WASHER": 1}, TURN_SWITCH={"HORN": 0}, MSG_11={"LAMPS_ON": 0})
  assert fp.wipersActive and fp.washerActive and not fp.hornPressed and not fp.lampsOn
  _, fp = car.step(BLINK_INFO={"WIPER_HI": 0, "FRONT_WASHER": 0})
  assert not fp.wipersActive and not fp.washerActive
