"""Exact startup selection and Dom button-owner behavior."""

import pytest
from opendbc.car import structs, gen_empty_fingerprint
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR
from opendbc.car.subaru.software_cruise import prepare, selected
from openpilot.starpilot.car.subaru.software_cruise import SoftwareCruise
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences
from openpilot.common.params import Params


@pytest.mark.parametrize("candidate", list(CAR))
def test_exact_optional_factory_configuration_keeps_alpha_long_disabled(candidate, tmp_path):
  params = Params(str(tmp_path))
  params.put_bool("SubaruRedneckCruise", True, block=True)
  params.put_bool("SafeMode", False, block=True)
  prefs = VehicleStartupPreferences.read(params, enabled=True)
  for alpha in (False, True):
    cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, False, False)
    original = cp.to_dict()
    cp = prefs.prepare(cp)
    assert not cp.openpilotLongitudinalControl and not cp.alphaLongitudinalAvailable
    assert selected(cp) == (candidate == CAR.SUBARU_IMPREZA_2020)
    if candidate != CAR.SUBARU_IMPREZA_2020:
      assert cp.to_dict() == original


def state():
  cs = structs.CarState(canValid=True, vEgo=15, gearShifter="drive")
  cs.cruiseState.enabled = cs.cruiseState.available = True
  cs.cruiseState.speedCluster = 60 / 3.6
  cc = structs.CarControl(enabled=True)
  cp = prepare(
    CarInterface.get_params(CAR.SUBARU_IMPREZA_2020, gen_empty_fingerprint(), [], False, False, False),
    True,
  )
  return cp, cs, cc


def test_dom_resume_set_timing_stock_off_and_held_driver_button():
  cp, cs, cc = state()
  owner = SoftwareCruise(cp)
  assert any(owner.run(cs, cc, 65 / 3.6, True)[0] == 1 for _ in range(30))
  assert any(owner.run(cs, cc, 55 / 3.6, True)[0] == 2 for _ in range(30))
  cs.buttonEvents = [structs.CarState.ButtonEvent(type="accelCruise", pressed=True)]
  assert owner.run(cs, cc, 65 / 3.6, True)[0] == 0
  cs.buttonEvents = []
  assert all(owner.run(cs, cc, 65 / 3.6, True)[0] == 0 for _ in range(100))
  cs.buttonEvents = [structs.CarState.ButtonEvent(type="accelCruise", pressed=False)]
  owner.run(cs, cc, 65 / 3.6, True)
  cs.buttonEvents = []
  assert any(owner.run(cs, cc, 65 / 3.6, True)[0] == 1 for _ in range(30))
  cs.cruiseState.enabled = False
  assert all(owner.run(cs, cc, 65 / 3.6, True)[0] == 0 for _ in range(100))


@pytest.mark.parametrize(
  "field,value",
  [
    ("brakePressed", True),
    ("gasPressed", True),
    ("accFaulted", True),
    ("steerFaultPermanent", True),
    ("canValid", False),
    ("canTimeout", True),
    ("gearShifter", "reverse"),
    ("standstill", True),
  ],
)
def test_software_owner_preserves_physical_and_fault_vetoes(field, value):
  cp, cs, cc = state()
  owner = SoftwareCruise(cp)
  setattr(cs, field, value)
  assert all(owner.run(cs, cc, 65 / 3.6, True)[0] == 0 for _ in range(30))
