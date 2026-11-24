"""Actual selected pedal factory reaches current LongControl and adaptive MPC."""
import pytest
from opendbc.car import structs
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.car.tesla.tests.pedal_fixture import actual_pedal_startup
from openpilot.starpilot.car.tesla.tests.test_preap_adaptive import frame
from openpilot.starpilot.longitudinal.tests.test_lead_approach_planner import DRIVE_NS, SPEED
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import message_bytes


@pytest.mark.parametrize('bus', [0, 2])
@pytest.mark.parametrize('radar_mode', ['off', 'front', 'nosecone'])
def test_actual_selected_startup_uses_original_integral_tune_in_current_long_control(bus, radar_mode, monkeypatch):
  with actual_pedal_startup(bus, radar_mode, False, monkeypatch) as (card, controls, calibration, cp_bytes):
    loc = controls.LoC
    assert loc.CP.to_dict() == card.CP.to_dict()
    assert loc.extension is None
    assert loc.stopping_decel_rate == 1.0
    for speed, gain in ((0., .05), (3., .08), (6., .1), (35., .15)):
      loc.pid.speed = speed
      assert loc.pid.k_p == 0.0
      assert loc.pid.k_i == pytest.approx(gain)
    state = structs.CarState(vEgo=35., aEgo=0., gearShifter='drive', canValid=True)
    output = loc.update(True, state, .2, False, (-1.5, 1.))
    assert loc.long_control_state == structs.CarControl.Actuators.LongControlState.pid
    assert loc.pid.i == pytest.approx(.0003)
    assert output == pytest.approx(.2003)
    assert loc.update(False, state, .2, False, (-1.5, 1.)) == 0.0
    assert loc.pid.i == 0.0


@pytest.mark.parametrize('bus', [0, 2])
@pytest.mark.parametrize('radar_mode', ['off', 'front', 'nosecone'])
def test_actual_selected_startup_reaches_adaptive_planner_and_current_mpc(bus, radar_mode, monkeypatch):
  with actual_pedal_startup(bus, radar_mode, False, monkeypatch) as (card, controls, calibration, cp_bytes):
    params = Params()
    params.put_bool('NAPAdaptiveAccel', False, block=True)
    original = LongitudinalPlanner(card.CP, init_v=SPEED)
    params.put_bool('NAPAdaptiveAccel', True, block=True)
    selected = LongitudinalPlanner(card.CP, init_v=SPEED)
    assert selected.preap_adaptive.enabled and not original.preap_adaptive.enabled
    params.put_bool('NAPAdaptiveAccel', False, block=True)
    for index in range(20):
      sm = frame(index)
      before = message_bytes(sm.envelopes)
      original.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
      selected.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
      assert before == message_bytes(sm.envelopes)
      assert selected.mpc.solution_status == original.mpc.solution_status == 0
      assert all(float(value) < .8 for value in selected.mpc.params[:, 1])
      assert all(float(value) == pytest.approx(ACCEL_MAX) for value in original.mpc.params[:, 1])
      assert selected.output_a_target <= max(float(value) for value in selected.mpc.params[:, 1])
      assert selected.output_a_target <= original.output_a_target + 1e-5
