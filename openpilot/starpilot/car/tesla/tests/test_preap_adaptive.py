"""Real planner/MPC coverage of typed pedal-profile protocol; not pedal startup proof."""
import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.interfaces import ACCEL_MAX
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.car.tesla.preap_adaptive import PreapAdaptiveAccel, profile_key
from openpilot.starpilot.longitudinal.tests.test_lead_approach_planner import Frame, DRIVE_NS, SPEED
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import snapshot, message_bytes


def protocol_params():
  # Existing factory remains stock; native pedal admission is a separate unfinished packet.
  cp = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  assert not cp.openpilotLongitudinalControl and cp.pcmCruise and cp.safetyConfigs[0].safetyParam == 0
  cp.openpilotLongitudinalControl = True
  cp.pcmCruise = False
  cp.safetyConfigs[0].safetyParam = 1
  return cp


def frame(index):
  sm = Frame(index).with_drive()
  sm['carState'].gearShifter = 'drive'
  return sm


def test_real_planner_mpc_and_cruise_receive_cap_without_mutating_inputs():
  with OpenpilotPrefix():
    saved = Params()
    cp = protocol_params()
    saved.put_bool('NAPAdaptiveAccel', False, block=True)
    original = LongitudinalPlanner(cp, init_v=SPEED)
    saved.put_bool('NAPAdaptiveAccel', True, block=True)
    selected = LongitudinalPlanner(cp, init_v=SPEED)
    assert original.preap_adaptive is not None and not original.preap_adaptive.enabled
    assert selected.preap_adaptive is not None and selected.preap_adaptive.enabled
    # The original complete saved choice is frozen for each planner instance.
    saved.put_bool('NAPAdaptiveAccel', False, block=True)
    for index in range(20):
      sm = frame(index)
      before = message_bytes(sm.envelopes)
      original.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
      selected.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
      assert message_bytes(sm.envelopes) == before
      assert original.mpc.solution_status == selected.mpc.solution_status == 0
      assert all(float(v) < 0.8 for v in selected.mpc.params[:, 1])
      assert all(float(v) == pytest.approx(ACCEL_MAX) for v in original.mpc.params[:, 1])
      assert selected.output_a_target <= max(float(v) for v in selected.mpc.params[:, 1])
      assert selected.output_a_target <= original.output_a_target + 1e-5


@pytest.mark.parametrize('defect', ['stale', 'future', 'old_drive', 'can', 'gas', 'brake', 'off', 'absent', 'changed_config'])
def test_actual_planner_rejected_optional_input_matches_unmodified_path(defect):
  with OpenpilotPrefix():
    saved = Params()
    cp = protocol_params()
    saved.put_bool('NAPAdaptiveAccel', False, block=True)
    original = LongitudinalPlanner(cp, init_v=SPEED)
    saved.put_bool('NAPAdaptiveAccel', True, block=True)
    selected = LongitudinalPlanner(cp, init_v=SPEED)
    for index in range(4):
      sm = frame(index)
      if defect == 'stale':
        sm.logMonoTime['radarState'] = sm.now_ns - 250_000_001
      elif defect == 'future':
        sm.logMonoTime['radarState'] = sm.now_ns + 1
      elif defect == 'old_drive':
        sm.logMonoTime['carState'] = DRIVE_NS - 1
      elif defect == 'can':
        sm['carState'].canValid = False
      elif defect in ('gas', 'brake'):
        setattr(sm['carState'], defect + 'Pressed', True)
      elif defect == 'off':
        sm['carControl'].longActive = False
      elif defect == 'absent':
        sm['radarState'].leadOne.present = False
      else:
        # Retire only the optional owner key; solver CP stays unchanged.
        selected.preap_adaptive.key = None
      original.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
      selected.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
      assert snapshot(original) == snapshot(selected)


@pytest.mark.parametrize('identity', [CAR.TESLA_MODEL_S_PREAP, CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_3,
                                      CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_X])
def test_real_current_factory_profiles_remain_outside_unfinished_pedal_path(identity):
  cp = CarInterface.get_params(identity, gen_empty_fingerprint(), [], False, False, False)
  assert profile_key(cp) is None
  assert not PreapAdaptiveAccel(cp, True).enabled
