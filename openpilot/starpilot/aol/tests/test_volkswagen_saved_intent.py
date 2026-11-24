"""Real intent owner regressions; run with current production dependencies under root's runtime."""
import pytest
from opendbc.car import structs
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.car.volkswagen.aol import VolkswagenStockCardIntent


@pytest.mark.parametrize('main_action', (0, 3, 4, 9))
@pytest.mark.parametrize('enabled', (False, True))
def test_saved_main_action_and_off_are_preserved(main_action, enabled):
  fp = {bus: {} for bus in range(8)}
  fp[0][0xAD] = 8
  cp = CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK7, fp, [], False, False, False)
  settings = AolSettings(enabled=enabled, pause_brake_mps=5., lkas_action=0, main_action=main_action,
                         distance_actions=(0, 0, 0), cancel_actions=(0, 0, 0))
  owner = VolkswagenStockCardIntent(cp, settings)
  assert owner.settings == settings
  state = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive, vEgo=20.)
  for tick, available in enumerate((True, False, True, True)):
    state.cruiseState.available = available
    owner.update(state, now_ns=1_000_000_000 + tick * 10_000_000)
  assert owner.allowed_latch == enabled


@pytest.mark.parametrize('enabled', (False, True))
def test_full_engagement_without_main_cycle_preserves_only_lateral_intent(enabled):
  fp = {bus: {} for bus in range(8)}
  fp[0][0xAD] = 8
  cp = CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK7, fp, [], False, False, False)
  settings = AolSettings(enabled, 5., 0, 0, (0, 0, 0), (0, 0, 0))
  owner = VolkswagenStockCardIntent(cp, settings)
  state = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive, vEgo=20.)
  state.cruiseState.available = True
  for tick, engaged in enumerate((False, True)):
    now = 1_000_000_000 + tick * 10_000_000
    owner.update(state, now_ns=now, standard_enabled=engaged)
    owner.observe_full_engagement(state, enabled=engaged, active=engaged, source_ns=now,
                                  observation_valid=True, now_ns=now, fault_active=False)
  assert owner.allowed_latch == enabled
  state.brakePressed = True
  owner.update(state, now_ns=1_020_000_000)
  assert owner.allowed_latch == enabled
  assert not owner.pause_longitudinal
  for gear in (structs.CarState.GearShifter.reverse, structs.CarState.GearShifter.drive):
    state.gearShifter = gear
    owner.update(state, now_ns=1_030_000_000)
    assert owner.allowed_latch == enabled
  state.steerFaultPermanent = True
  owner.update(state, now_ns=1_040_000_000)
  assert not owner.allowed_latch
