"""The wipers as a rain report (Mazda "Wipers Mean Rain"): what the weather checker applies while they sweep."""
from types import SimpleNamespace

from openpilot.starpilot.controls.lib.weather_checker import WIPER_RAIN_DEFAULTS, WIPER_RAIN_ID, WeatherChecker

OFFSETS = ("increase_following_distance", "increase_stopped_distance", "reduce_acceleration", "reduce_lateral_acceleration")


def toggles(**rain):
  values = {f"{k}_{s}": 0.0 for k in OFFSETS for s in ("low_visibility", "rain", "rain_storm", "snow")}
  values.update({f"{k}_rain": v for k, v in rain.items()})
  return SimpleNamespace(**values)


def checker():
  return WeatherChecker(SimpleNamespace(gps_position={}))


def offsets(w):
  return {k: getattr(w, k) for k in OFFSETS}


def test_sweeping_wipers_with_a_clear_forecast_apply_the_built_in_rain_offsets():
  w = checker()
  w.apply_wipers(True, toggles())
  assert w.weather_id == WIPER_RAIN_ID and w.wiper_rain
  assert offsets(w) == WIPER_RAIN_DEFAULTS


def test_the_drivers_rain_presets_win_over_the_built_in_values():
  w = checker()
  w.apply_wipers(True, toggles(increase_following_distance=0.5, reduce_acceleration=0.3))
  assert w.increase_following_distance == 0.5 and w.reduce_acceleration == 0.3
  assert w.reduce_lateral_acceleration == 0.0        # the driver set some rain values: those, and only those


def test_lifted_the_moment_the_wipers_stop():
  w = checker()
  w.apply_wipers(True, toggles())
  w.apply_wipers(False, toggles())
  assert w.weather_id == 0 and not w.wiper_rain
  assert not any(offsets(w).values())


def test_a_forecast_that_already_says_rain_is_left_alone():
  w = checker()
  w.weather_id = 501
  w.update_offsets(toggles(increase_following_distance=0.4))
  w.apply_wipers(True, toggles(increase_following_distance=0.4))
  assert w.weather_id == 501 and not w.wiper_rain and w.increase_following_distance == 0.4
  w.apply_wipers(False, toggles(increase_following_distance=0.4))
  assert w.weather_id == 501 and w.increase_following_distance == 0.4


def test_off_when_the_wipers_are_off():
  w = checker()
  w.apply_wipers(False, toggles())
  assert w.weather_id == 0 and not any(offsets(w).values())
