"""Numerical/shape tests only; fixtures never establish real source/native authority.

Root must additionally call probe_endpoint on actual authenticated checkpoint,
controller and emitted-CAN records. No fitted response or permission is mocked
as though these unit tests were that qualification.
"""
from dataclasses import replace
import math

import pytest
from openpilot.starpilot.flm import live_tracking_probe as probe
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, GROUPS, canonical_names
from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding, MAX_STEP
from openpilot.starpilot.flm.pid_stages import SUPPORTED


def binding():
  return LiveTrialBinding('1'*64, '2'*64, 'controller', '3'*64, '4'*64,
    'reset', 'calibration', 'vehicle', '', '5'*64, '', False).validate()


def numerical_fixture(domain='torque', group='ff.left'):
  current = binding()
  names = canonical_names(tuple(name for name in GROUPS[group] if domain == 'torque' or name in SUPPORTED))
  rows = tuple(probe._physical_row(name, 0., 0., MAX_STEP, .1, .1, .11, 2., 'separated') for name in names)
  value = probe.ProbeResult(current, 100, 110, '6'*64, current.physical_model_revision, rows, domain, group, None)
  object.__setattr__(value, '_seal', (probe._OWNER, probe._probe_key(value)))
  return value


def test_oriented_delivered_torque_and_measured_slope_have_correct_units_and_sign():
  row = probe._physical_row(NAMES[0], 0., -.005, .005, .105, .1, .11, 2., 'separated')
  assert row.physical_derivative == pytest.approx(-2.)
  assert row.max_step_physical_delta == pytest.approx(2. * MAX_STEP)
  assert row.max_step_physical_delta == abs(row.physical_derivative)*MAX_STEP
  # A derivative is not the physical delta at the current bounded trial cap.
  assert row.max_step_physical_delta != abs(row.physical_derivative)
  reversed_delivery = probe._physical_row(NAMES[0], 0., -.005, .005, .105, .11, .1, 2., 'separated')
  assert reversed_delivery.physical_derivative == pytest.approx(2.)


def test_one_sided_original_bound_uses_actual_secant_width():
  row = probe._physical_row(NAMES[0], .25, .245, .25, .1, .1, .11, 2., 'separated')
  assert row.physical_derivative == pytest.approx(-4.)
  assert row.max_step_physical_delta == pytest.approx(4. * MAX_STEP)


@pytest.mark.parametrize('status', ['quantized', 'native_unavailable'])
def test_can_quantization_or_unavailable_native_support_cannot_fabricate_feature(status):
  row = probe._physical_row(NAMES[0], 0., -.005, .005, .1, .1, .1, 2., status)
  assert row.physical_derivative == row.max_step_physical_delta == 0.
  assert row.native_separated is False


@pytest.mark.parametrize('domain,count', [('torque', 30), ('torque_pid', 26)])
def test_full_domain_coverage_is_over_honest_independent_small_groups(domain, count):
  covered = set()
  for group, names in GROUPS.items():
    if domain == 'torque_pid' and not any(name in SUPPORTED for name in names):
      continue
    value = numerical_fixture(domain, group)
    assert probe.validate_probe_result(value, value.binding) is value
    assert 1 <= len(value.rows) <= 4
    covered.update(row.name for row in value.rows)
  assert len(covered) == count


def test_partial_duplicate_or_forged_wire_result_never_validates():
  value = numerical_fixture()
  for changed in [replace(value, _seal=None), replace(value, rows=value.rows[:-1]),
                  replace(value, rows=(value.rows[0],)*4), replace(value, model_revision='7'*64)]:
    with pytest.raises(ValueError):
      probe.validate_probe_result(changed)


def test_mutated_row_and_binding_fail_fresh_content_check():
  value = numerical_fixture()
  object.__setattr__(value.rows[0], 'physical_derivative', value.rows[0].physical_derivative*2)
  object.__setattr__(value.rows[0], 'max_step_physical_delta', value.rows[0].max_step_physical_delta*2)
  with pytest.raises(ValueError, match='contents changed'):
    probe.validate_probe_result(value)
  value = numerical_fixture()
  object.__setattr__(value.binding, 'vehicle_model_generation', 'changed')
  with pytest.raises(ValueError, match='contents changed'):
    probe.validate_probe_result(value)


@pytest.mark.parametrize('field,value', [('physical_derivative', math.nan), ('baseline_coordinate', True),
  ('plus_coordinate', MAX_STEP + .001), ('native_separated', 1), ('status', 'unknown')])
def test_invalid_per_coordinate_domain_rejected(field, value):
  result = numerical_fixture()
  row = replace(result.rows[0], **{field: value})
  altered = replace(result, rows=(row, *result.rows[1:]))
  with pytest.raises(ValueError):
    probe.validate_probe_result(altered)


def test_unknown_rows_and_source_objects_do_not_call_hooks_or_reducers():
  class Trap:
    def __getattr__(self, name):
      raise AssertionError('callback')
    def __reduce__(self):
      raise AssertionError('reducer')
  result = numerical_fixture()
  with pytest.raises(ValueError):
    probe.validate_probe_result(replace(result, rows=(Trap(),)))
  with pytest.raises(ValueError):
    probe._primitive(Trap())


def test_foreign_factory_rejected_before_constructor_or_snapshot_work():
  class Factory:
    def create(self):
      raise AssertionError('foreign constructor')
  with pytest.raises(ValueError, match='actual selected'):
    probe.probe_endpoint(factory=Factory(), control=None, output=None, accepted_full30=(), binding=None,
      response_witness=None, source=None, now_ns=1, current_native=True,
      current_vehicle_model_anchor=(1., 1., 1.), replay=None, group='ff.left', names=GROUPS['ff.left'], prefix_controls=())


@pytest.mark.parametrize('broken', ['no_checkpoint', 'missing_tick', 'missing_endpoint', 'over100'])
def test_prefix_holds_instead_of_inventing_missing_controller_history(broken):
  from collections import deque
  from types import SimpleNamespace
  from openpilot.starpilot.flm.controller_replay import ControllerReplay, ReplayFrame, Snapshot
  from openpilot.starpilot.flm.torque_adapter import Objective
  from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
  # Invalid-only shells: any attempted actual construction/decoding is a test
  # failure. These cannot satisfy the successful probe authority path.
  current = binding()
  factory = object()
  replay = object.__new__(ControllerReplay)
  replay.factory, replay.failed, replay.raw_mode = factory, False, True
  replay.identity, replay.baseline_revision = current.identity, current.baseline_revision
  replay.objective = Objective('feedforward', 'low', 'steady')
  replay.emissions = object.__new__(EmissionHistory)
  replay.checkpoints = deque(((100, Snapshot('invalid-test-only', (('value', 1.),))),))
  replay.records = deque(ReplayFrame(100+i*10, (), 0., (), frame_sequence=i+1) for i in range(3))
  endpoint = 120
  if broken == 'no_checkpoint':
    replay.checkpoints.clear()
  elif broken == 'missing_tick':
    replay.records = deque((replay.records[0], replay.records[-1]))
  elif broken == 'missing_endpoint':
    replay.records.pop()
  else:
    replay.records = deque(ReplayFrame(100+i, (), 0., (), frame_sequence=i+1) for i in range(101))
    endpoint = 200
  with pytest.raises(ValueError, match='checkpoint|prefix'):
    probe._prefix_before(replay, SimpleNamespace(control_ns=endpoint), SimpleNamespace(apply_ns=endpoint), current, factory)



def test_actual_consumed_map_is_replayed_without_forging_legacy_trace():
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence
  current = binding()
  # Exact production dataclass shape; other fields are irrelevant to this
  # numerical transport helper and do not impersonate successful probe authority.
  control = object.__new__(ControlEvidence)
  object.__setattr__(control, "effect_trace", None)
  live = tuple(.001 if index == 0 else 0. for index in range(len(NAMES)))
  object.__setattr__(control, "live_applied_coordinates", live)
  rows = tuple(zip(NAMES, live, strict=True))
  actual, has_source = probe._consumed_coordinates(control, current)
  assert actual == rows and has_source
  assert control.effect_trace is None
  changed = tuple(.002 if index == 0 else value for index, value in enumerate(live))
  object.__setattr__(control, "live_applied_coordinates", changed)
  assert probe._consumed_coordinates(control, current) == (tuple(zip(NAMES, changed, strict=True)), True)
  for malformed in (rows, live[:-1], tuple(True for _ in NAMES), tuple(math.nan for _ in NAMES), list(live)):
    object.__setattr__(control, "live_applied_coordinates", malformed)
    with pytest.raises(ValueError):
      probe._consumed_coordinates(control, current)
  object.__setattr__(control, "live_applied_coordinates", ())
  assert probe._consumed_coordinates(control, current)[1] is False


@pytest.mark.parametrize("pid", [False, True])
def test_actual_frame_transport_preserves_live_map_and_owner_anchor(pid):
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence
  from openpilot.starpilot.flm.learner_factory import _frame
  from openpilot.starpilot.flm.original_context_receipts import _same_original_frame
  control = object.__new__(ControlEvidence)
  values = {'control_ns': 100, 'replay_inputs': (("control_frame", 1), ("desired_curvature", .001),
    ("vEgo", 20.), ("roll", 0.)), 'requested': .1, 'applied_coordinates': (), 'valid': True, 'fault': False,
    'override': False, 'manual': False, 'limited': False, 'applied_parameter_names': (), 'effect_trace': None, 'effect_prepare_ns': 0,
    'vehicle_model_anchor': (15., 1., 1.), 'live_applied_coordinates': tuple(.001 if i == 0 else 0. for i in range(30)),
    'stage_schema_revision': "pid-selected-stage-snapshot-v2" if pid else "torque-raw",
    'stage_values': tuple((name, 0.) for name in ("selected_command", "effective_factor", "offset",
      "friction", "pid_pos_limit", "pid_neg_limit", "actual_delta_ff", "actual_delta_threshold", "setpoint"))}
  for name, value in values.items():
    object.__setattr__(control, name, value)
  frame = _frame(control)
  assert frame.live_applied_coordinates == control.live_applied_coordinates
  assert frame.vehicle_model_anchor == control.vehicle_model_anchor
  assert not _same_original_frame(frame, replace(frame, live_applied_coordinates=(0.,)*30))
  assert not _same_original_frame(frame, replace(frame, vehicle_model_anchor=(16., 1., 1.)))
