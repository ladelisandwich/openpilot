"""The steer-limit classifier (selfdrive/controls/lib/steer_limit.py, ported from zoompilot) and its controlsd wiring
for the Mazda torque cars. The unit table is zoompilot's, run at the Mazda GEN1's slew (10 up, 25 down, of 800) with
the Torque Interceptor's ceiling as the rail, and at the 2022 CX-5 EPS's (12 of 1200, rail 648)."""
import pytest

from cereal import car, custom
from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.values import CAR as MAZDA, MazdaSafetyFlags
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.selfdrive.controls.lib.steer_limit import CLEAN, SteerLimit, SteerLimitReclassifier, classify, mazda_steer_limit_schedule

MAZDA_TI = {'slew_up': 10.0 / 800.0, 'slew_down': 25.0 / 800.0, 'rail_scale': 700.0 / 800.0}
CX5_2022 = {'slew_up': 12.0 / 1200.0, 'slew_down': 12.0 / 1200.0, 'rail_scale': 648.0 / 1200.0}
SIDES = [pytest.param(MAZDA_TI, id='mazda_ti'), pytest.param(CX5_2022, id='cx5_2022')]

DEEPENING = {'error_prev': 0.2, 'integrator': 0.3}   # same sign: integrating would grow |i|
DECAYING = {'error_prev': -0.2, 'integrator': 0.3}   # opposite signs: integrating shrinks |i|


def cl(cmd, applied, applied_prev, side, upstream_flag=True, **kw):
  args = {**side, **DEEPENING, **kw}
  return classify(cmd, applied, applied_prev, upstream_flag=upstream_flag, **args)


class TestClassifyTable:
  @pytest.mark.parametrize('side', SIDES)
  def test_clean_when_command_and_applied_agree(self, side):
    assert cl(0.30, 0.30, 0.29, side) == CLEAN
    assert cl(0.30, 0.305, 0.29, side) == CLEAN  # inside the 0.01 threshold
    assert cl(0.30, 0.30, 0.29, side, **DECAYING) == CLEAN

  @pytest.mark.parametrize('side', SIDES)
  def test_full_slew_step_toward_the_command_is_rate_limited(self, side):
    step = side['slew_up']
    lim = cl(0.40, 0.30 + step, 0.30, side)
    assert lim == SteerLimit(limited=True, driver_limited=False, rate_limited=True, at_rail=False)

  def test_command_walking_under_the_rate_limit_is_lag_not_a_driver(self):
    # on the 800 scale a step up is 0.0125: a command moving 0.012 per frame is tracked exactly, one
    # carOutput frame late, and the 0.012 gap clears the 0.01 threshold every frame
    lim = cl(0.312, 0.300, 0.288, MAZDA_TI)
    assert (lim.rate_limited, lim.driver_limited) == (True, False)
    # the same gap with the actuator barely moving is a driver holding it
    lim = cl(0.312, 0.300, 0.299, MAZDA_TI)
    assert (lim.rate_limited, lim.driver_limited) == (False, True)

  @pytest.mark.parametrize('side', SIDES)
  def test_short_step_is_driver_limited(self, side):
    step = side['slew_up']
    lim = cl(0.40, 0.30 + 0.5 * step, 0.30, side)
    assert lim == SteerLimit(limited=True, driver_limited=True, rate_limited=False, at_rail=False)
    assert cl(0.40, 0.30 + 0.92 * step, 0.30, side).rate_limited   # the 0.9 tolerance

  @pytest.mark.parametrize('side', SIDES)
  def test_walking_down_toward_zero_uses_the_down_step(self, side):
    step = side['slew_down']
    assert cl(0.0, -0.30 + step, -0.30, side).rate_limited
    assert cl(0.0, 0.30 - step, 0.30, side).rate_limited

  @pytest.mark.parametrize('side', SIDES)
  def test_pinned_applied_is_driver_limited(self, side):
    lim = cl(0.40, 0.30, 0.30, side)
    assert lim == SteerLimit(limited=True, driver_limited=True, rate_limited=False, at_rail=False)

  @pytest.mark.parametrize('side', SIDES)
  def test_step_away_from_the_command_is_not_rate_limited(self, side):
    step = side['slew_up']
    lim = cl(0.20, 0.30 + step, 0.30, side)
    assert lim.driver_limited and not lim.rate_limited

  @pytest.mark.parametrize('side', SIDES)
  @pytest.mark.parametrize('kind', ['rate', 'driver'])
  def test_deepening_integrator_freezes_decaying_one_bleeds(self, side, kind):
    applied = 0.30 + (side['slew_up'] if kind == 'rate' else 0.0)
    assert cl(0.40, applied, 0.30, side, error_prev=0.2, integrator=0.3).limited
    assert cl(0.40, applied, 0.30, side, error_prev=-0.2, integrator=-0.3).limited
    assert not cl(0.40, applied, 0.30, side, error_prev=-0.2, integrator=0.3).limited
    assert not cl(0.40, applied, 0.30, side, error_prev=0.2, integrator=-0.3).limited

  @pytest.mark.parametrize('side', SIDES)
  def test_empty_integrator_counts_as_deepening(self, side):
    assert cl(0.40, 0.30, 0.30, side, error_prev=0.2, integrator=0.0).limited
    assert cl(0.40, 0.30, 0.30, side, error_prev=0.0, integrator=0.3).limited

  @pytest.mark.parametrize('side', SIDES)
  def test_applied_on_the_rail_is_never_limited(self, side):
    rail = side['rail_scale']
    for kw in (DEEPENING, DECAYING):
      lim = cl(rail + 0.20, rail, rail, side, **kw)
      assert lim == SteerLimit(limited=False, driver_limited=False, rate_limited=False, at_rail=True)
      lim = cl(-rail - 0.20, -rail, -rail, side, **kw)
      assert lim == SteerLimit(limited=False, driver_limited=False, rate_limited=False, at_rail=True)

  @pytest.mark.parametrize('side', SIDES)
  def test_rail_reported_even_without_a_mismatch(self, side):
    rail = side['rail_scale']
    assert cl(rail, rail, rail, side, upstream_flag=False) == SteerLimit(False, False, False, True)

  def test_full_scale_is_a_rail_on_platforms_without_a_ceiling(self):
    flat = {'slew_up': 0.01, 'slew_down': 0.02, 'rail_scale': 1.0}
    assert cl(1.0, 1.0, 1.0, flat).at_rail
    assert not cl(1.0, 0.99, 0.98, flat).at_rail

  @pytest.mark.parametrize('side', SIDES)
  @pytest.mark.parametrize('flag', [True, False])
  def test_first_frame_falls_back_to_the_upstream_flag(self, side, flag):
    lim = cl(0.40, 0.30, None, side, upstream_flag=flag, **DECAYING)
    assert lim == SteerLimit(limited=flag, driver_limited=flag, rate_limited=False, at_rail=False)
    assert cl(0.30, 0.30, None, side, upstream_flag=flag) == CLEAN


def cx9_params(torque_interceptor=True):
  CP = interfaces[MAZDA.MAZDA_CX9_2021].get_non_essential_params(MAZDA.MAZDA_CX9_2021)
  CP.flags = (int(CP.flags) & ~int(MazdaSafetyFlags.TORQUE_INTERCEPTOR)) | (int(MazdaSafetyFlags.TORQUE_INTERCEPTOR) if torque_interceptor else 0)
  return CP.as_reader()


class TestMazdaSchedule:
  def test_ti_car_slews_at_the_gen1_rates_and_rails_at_the_ti_ceiling(self):
    assert mazda_steer_limit_schedule(cx9_params(True)) == pytest.approx((10 / 800, 25 / 800, 700 / 800))

  def test_stock_eps_car_rails_at_full_scale(self):
    assert mazda_steer_limit_schedule(cx9_params(False)) == pytest.approx((10 / 800, 25 / 800, 1.0))

  def test_other_brands_keep_controlsds_flag(self):
    CP = interfaces[TOYOTA.TOYOTA_RAV4].get_non_essential_params(TOYOTA.TOYOTA_RAV4)
    assert mazda_steer_limit_schedule(CP.as_reader()) is None
    CP = car.CarParams.new_message(brand="tesla", steerControlType="angle")
    assert mazda_steer_limit_schedule(CP.as_reader()) is None


class TestReclassifier:
  """What controlsd.publish hands the next frame: its own mismatch flag, re-decided while the toggle is on and lateral active."""
  TI = (10 / 800, 25 / 800, 700 / 800)

  def test_deepening_mismatch_keeps_the_flag(self):
    r = SteerLimitReclassifier(self.TI)
    r.applied_prev = 0.29
    assert r.update(True, True, cmd=0.40, applied=0.30, upstream_flag=True, error_prev=0.2, integrator=0.3) is True
    assert r.applied_prev == 0.30

  def test_decaying_integrator_releases_the_flag(self):
    r = SteerLimitReclassifier(self.TI)
    r.applied_prev = 0.29
    assert r.update(True, True, 0.40, 0.30, True, error_prev=-0.2, integrator=0.3) is False

  def test_the_ti_ceiling_clears_the_flag(self):
    r = SteerLimitReclassifier(self.TI)
    r.applied_prev = 0.875
    assert r.update(True, True, 1.0, 0.875, True, error_prev=0.2, integrator=0.3) is False

  def test_first_active_frame_keeps_the_upstream_flag(self):
    for flag in (True, False):
      r = SteerLimitReclassifier(self.TI)
      assert r.update(True, True, 0.40, 0.30, flag, error_prev=-0.2, integrator=0.3) is flag
      assert r.applied_prev == 0.30

  def test_toggle_off_other_brands_and_inactive_lateral_pass_the_flag_through_and_drop_history(self):
    for schedule, enabled, lat_active in ((self.TI, False, True), (None, True, True), (self.TI, True, False)):
      r = SteerLimitReclassifier(schedule)
      r.applied_prev = 0.29
      assert r.update(enabled, lat_active, 0.40, 0.30, True, error_prev=-0.2, integrator=0.3) is True
      assert r.applied_prev is None
    r = SteerLimitReclassifier(self.TI)
    r.applied_prev = 0.29
    r.reset()
    assert r.applied_prev is None

  def test_runs_frame_by_frame_like_the_car(self):
    # the actuator walks toward a command one step a frame: the flag is on while the error would wind the integrator
    # up, off once the error has turned around, and off at the ceiling
    r = SteerLimitReclassifier(self.TI)
    applied, flags = 0.0, []
    for _ in range(80):
      cmd = 1.0
      flags.append(r.update(True, True, cmd, applied, abs(cmd - applied) > 1e-2, error_prev=0.5, integrator=0.1))
      applied = min(0.875, applied + 10 / 800)
    assert all(flags[1:70]) and not any(flags[72:])
    assert r.update(True, True, 1.0, 0.875, True, error_prev=-0.5, integrator=0.1) is False


def test_toggle_is_declared():
  from openpilot.common.params import _HEADER_ONLY_KEYS
  key_info = _HEADER_ONLY_KEYS.get("MazdaSteerLimitClassifier")
  assert key_info is not None and key_info[0] == "BOOL"
  assert custom.StarPilotCarParams  # the vehicle settings page offers it next to Hybrid Longitudinal (vehicle.py)
