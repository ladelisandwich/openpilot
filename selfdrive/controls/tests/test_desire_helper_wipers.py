"""Lane changes in the rain (Mazda "Wipers Mean Rain"): no automated change while the wipers sweep, none starting
for a few seconds after the washer, a nudged change still allowed."""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.desire_helper import DT_MDL, WASHER_BLOCK_S, DesireHelper, LaneChangeState
from openpilot.selfdrive.controls.tests.test_navigation_desires import make_car_state, make_plan, make_toggles


def toggles(**overrides):
  values = {"mazda_wiper_weather_mode": True, "nav_desires_allowed": False, "nav_lane_positioning_allowed": False,
            "lane_change_delay": 0.0}
  values.update(overrides)
  return make_toggles(**values)


def body(wipers=False, washer=False):
  return SimpleNamespace(wipersActive=wipers, washerActive=washer)


def run(helper, car, tog, plan, body_state, frames=1):
  for _ in range(frames):
    helper.update(car, True, 0.0, plan, tog, True, starpilot_car_state=body_state)
  return helper.lane_change_state


def test_nudgeless_change_goes_ahead_in_the_dry():
  helper = DesireHelper()
  car = make_car_state(leftBlinker=True)
  assert run(helper, car, toggles(), make_plan(), body(), frames=3) == LaneChangeState.laneChangeStarting


def test_no_automated_change_while_the_wipers_sweep_but_a_nudge_still_goes():
  helper = DesireHelper()
  car = make_car_state(leftBlinker=True)
  assert run(helper, car, toggles(), make_plan(), body(wipers=True), frames=10) == LaneChangeState.preLaneChange
  nudged = make_car_state(leftBlinker=True, steeringPressed=True, steeringTorque=1.0)
  assert run(helper, nudged, toggles(), make_plan(), body(wipers=True)) == LaneChangeState.laneChangeStarting


def test_nothing_starts_for_a_few_seconds_after_the_washer():
  helper = DesireHelper()
  idle = make_car_state()
  run(helper, idle, toggles(), make_plan(), body(washer=True))
  signal = make_car_state(leftBlinker=True)
  assert run(helper, signal, toggles(), make_plan(), body(), frames=int(WASHER_BLOCK_S / DT_MDL) - 5) == LaneChangeState.off
  # the blinker is still on when the block lifts: a fresh edge is needed, as with any signal, so drop and raise it
  run(helper, idle, toggles(), make_plan(), body(), frames=10)
  assert run(helper, signal, toggles(), make_plan(), body(), frames=3) == LaneChangeState.laneChangeStarting


def test_with_the_toggle_off_the_body_state_is_ignored():
  helper = DesireHelper()
  car = make_car_state(leftBlinker=True)
  tog = toggles(mazda_wiper_weather_mode=False)
  assert run(helper, car, tog, make_plan(), body(wipers=True, washer=True), frames=3) == LaneChangeState.laneChangeStarting
