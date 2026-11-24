"""Declared synthetic arithmetic/type contracts, NOT native or route evidence.

Root executes in the matching composed project. Author did not import/run.
Actual registered producer/native/CAN positive remains a separate root fixture.
"""
from dataclasses import asdict, replace
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from openpilot.starpilot.flm import plant_lag as p
from openpilot.starpilot.flm import physical_input_observation as o


def h(value):
  return p.digest(value)


def reseal(row, **changed):
  row = replace(row, **changed)
  return replace(row, observation_digest=o.observation_digest(row))


def retape(binding, tape, commands):
  commands = tuple(commands)
  return replace(tape, commands=commands, verified_tape_digest=o.physical_tape_digest(binding, commands))


def physical_fixture(*, tags=None):
  """All51 complete varying OBSERVED inputs, all performance tags originally True.

  Evidence hashes are explicitly synthetic placeholders; this does not test the
  real from_original constructor or grant native permission/profile authority.
  """
  binding = p.SourceBinding(*(h(name) for name in ('identity', 'cp', 'source', 'controller', 'epoch',
    'baseline', 'native', 'calibration', 'vm')), session='SYNTHETIC_ARITHMETIC_NOT_NATIVE_PROOF',
    source_clock_contract_sha256=h('synthetic-clock-proof'), native_required=True)
  bound = p.binding_digest(binding)
  tags = tags if tags is not None else o.OriginalPerformanceTags(True, True, True, True, True)
  commands, poses, windows = [], [], []
  for block in range(8):
    start = 20_000_000_000 + block * 6_000_000_000
    sign = 1 if block % 2 == 0 else -1

    def value(stamp, *, start=start, sign=sign):
      t = (stamp - start) * 1e-9
      return sign * (.25 + .13 * math.sin(1.9 * t) + .02 * math.sin(7.3 * t))

    for index in range(501):
      apply = start + index * 10_000_000
      control = apply - 1_000_000
      row = o.PhysicalDeliveredInput(o.PROTOCOL, control, control, control, apply, value(apply),
        control - 10_000_000, control - 1_000_000, apply + 100_000_000, len(commands) + 1,
        apply + 1_000_000, h(('encoded', block, index)), h(('original-control', block, index)),
        h(('original-applied', block, index)), h(('effective-applied', block, index)),
        h(('declared-synthetic-emission', block, index)), binding.identity, 1, binding.source_epoch,
        binding.baseline_revision, binding.controller_revision, binding.native_identity, True,
        'measured', tags, 'observed_input_performance_tagged' if any(asdict(tags).values()) else
        'observed_input_clean', bound, '')
      commands.append(reseal(row))
    ids = []
    for index in range(80):
      source = start + 605_000_000 + index * 50_000_000
      pid = h(('pose', block, index))
      ids.append(pid)
      selected = start + ((source - 170_000_000 - start) // 10_000_000) * 10_000_000
      y = 1.98 * value(selected) + .01
      poses.append(p.SourcePose(pid, source, source + 40_000_000, source,
        'mono_boot_equivalent_interval_v1', 0, 1_000, source - 1_000, source + 1_000,
        0, 25., y / 25., 0., bound))
    windows.append(p.CalibrationWindow(h(('time-block', block)), start, start + 5_000_000_000,
      tuple(ids), h(('declared-original-slot', block))))
  commands = tuple(commands)
  tape = o.VerifiedPhysicalTape(o.PROTOCOL, commands, bound,
    o.physical_tape_digest(binding, commands), h('synthetic-not-native-verifier'))
  return binding, tape, tuple(poses), tuple(windows)


def test_known_limited_physical_inputs_preserve_all51_original_rows_and_affine_gates():
  evidence = physical_fixture()
  result = p.identify_physical_plant_lag(*evidence)
  assert result.status == 'physical_affine_test_passed'
  assert result.reason == 'NO_PROFILE_NO_CURRENT_AUTHORITY'
  assert result.decision.protocol == p.PHYSICAL_PROTOCOL != p.PROTOCOL
  assert result.decision.lag_ns == 170_000_000
  assert result.decision.slope == pytest.approx(1.98)
  assert result.decision.intercept == pytest.approx(.01)
  assert len(result.decision.training_scores) == 51
  assert all(score is not None for _, score in result.decision.training_scores)
  assert len(result.rows) == 640
  assert tuple(row.pose_id for row in result.rows) == tuple(pose.pose_id for pose in evidence[2])
  assert all(error < 1e-10 and outside == 0 for _, error, outside in result.heldout_blocks)
  assert all(all(asdict(row.tags).values()) for row in evidence[1].commands)
  assert not isinstance(evidence[1].commands[0], p.DeliveredCommand)
  assert not hasattr(result, 'profile') and not hasattr(result, 'admission')


def test_performance_tag_disposition_is_bound_but_not_a_numeric_physics_parameter():
  tagged = p.identify_physical_plant_lag(*physical_fixture())
  clean = p.identify_physical_plant_lag(*physical_fixture(tags=o.OriginalPerformanceTags(False, False, False, False, False)))
  assert (tagged.decision.lag_ns, tagged.decision.slope, tagged.decision.intercept) == (
    clean.decision.lag_ns, clean.decision.slope, clean.decision.intercept)
  assert tagged.rows == clean.rows
  assert tagged.experiment_digest != clean.experiment_digest
  assert tagged.decision.train_source_digest != clean.decision.train_source_digest


def test_legacy_strict_types_cannot_receive_or_coerce_physical_observations():
  binding, tape, poses, windows = physical_fixture()
  with pytest.raises(ValueError, match='independently verified command tape'):
    p.identify_plant_lag(binding, tape, poses, windows)
  disguised = p.VerifiedCommandTape(tape.commands, p.binding_digest(binding),
    p.command_tape_digest(binding, tape.commands), h('fake-strict-wrapper'), True)
  with pytest.raises(ValueError, match='actual delivered command type'):
    p.identify_plant_lag(binding, disguised, poses, windows)


def test_tag_or_original_body_tamper_cannot_keep_an_old_observation_digest():
  binding, tape, poses, windows = physical_fixture()
  for changed in (
    replace(tape.commands[0], tags=replace(tape.commands[0].tags, output_limited=False)),
    replace(tape.commands[0], original_applied_sha256=h('forged-original-body')),
    replace(tape.commands[0], actual_emission_sha256=h('different-native-emission')),
  ):
    attacked = retape(binding, tape, (changed, *tape.commands[1:]))
    with pytest.raises(ValueError, match='custody'):
      p.identify_physical_plant_lag(binding, attacked, poses, windows)


@pytest.mark.parametrize('changed', [
  {'state_ns': 1}, {'output_state_ns': 1}, {'source_receipt_ns': 1},
  {'native_expires_ns': 1}, {'native_observed_ns': 0}, {'native_required': False},
  {'native_identity': h('different-native')}, {'identity': h('different-car')},
  {'source_epoch': h('different-epoch')}, {'baseline_revision': h('different-baseline')},
  {'controller_revision_sha256': h('different-controller')}, {'binding_digest': h('different-binding')},
  {'limiter_kind': 'unknown'}, {'disposition': 'live_permission'}, {'command': 1.01}, {'generation': -1},
])
def test_hard_clock_native_identity_support_guards_survive_resealed_attack(changed):
  binding, tape, poses, windows = physical_fixture()
  changed_row = reseal(tape.commands[0], **changed)
  attacked = retape(binding, tape, (changed_row, *tape.commands[1:]))
  with pytest.raises(ValueError):
    p.identify_physical_plant_lag(binding, attacked, poses, windows)


def test_boolean_flags_are_not_truthy_integer_permissions():
  binding, tape, poses, windows = physical_fixture()
  changed = reseal(tape.commands[0], tags=replace(tape.commands[0].tags, control_limited=1))
  with pytest.raises(ValueError, match='boolean tags'):
    p.identify_physical_plant_lag(binding, retape(binding, tape, (changed, *tape.commands[1:])), poses, windows)


def test_fifo_previous_carrier_and_stale_tape_hash_reject():
  binding, tape, poses, windows = physical_fixture()
  for commands in (
    (tape.commands[0], reseal(tape.commands[1], carrier_frame=tape.commands[1].carrier_frame + 4), *tape.commands[2:]),
    (tape.commands[1], tape.commands[0], *tape.commands[2:]),
  ):
    with pytest.raises(ValueError, match='FIFO/carrier'):
      p.identify_physical_plant_lag(binding, retape(binding, tape, commands), poses, windows)
  with pytest.raises(ValueError, match='tape custody'):
    p.identify_physical_plant_lag(binding, replace(tape, commands=tape.commands[1:]), poses, windows)


def test_missing_one_maximum_lookback_input_rejects_entire_common51_experiment():
  binding, tape, poses, windows = physical_fixture()
  commands = tuple(row for row in tape.commands if row.apply_ns >= poses[0].mono_ns - 100_000_000)
  result = p.identify_physical_plant_lag(binding, retape(binding, tape, commands), poses, windows)
  assert result.status == 'collecting' and result.reason.startswith('common_command_')
  assert result.decision is None and not result.rows


def test_heldout_failure_cannot_choose_another_lag_or_mutate_training_receipt():
  binding, tape, poses, windows = physical_fixture()
  before = p.identify_physical_plant_lag(binding, tape, poses, windows)
  attacked = tuple(replace(pose, calibrated_yaw_rate=pose.calibrated_yaw_rate + .5)
                   if index >= 320 else pose for index, pose in enumerate(poses))
  result = p.identify_physical_plant_lag(binding, tape, attacked, windows)
  assert result.status == 'rejected' and result.reason == 'heldout_rmse'
  assert result.decision == before.decision
  assert result.model_receipt_digest is None and len(result.heldout_blocks) == 4


@pytest.mark.parametrize('changed', [{'healthy': False}, {'native_allowed_at_receipt': False},
                                   {'overridden': True}, {'fault': True}, {'clock_generation': 7}])
def test_performance_tags_cannot_cover_invalid_original_pose(changed):
  binding, tape, poses, windows = physical_fixture()
  attacked = (replace(poses[0], **changed), *poses[1:])
  with pytest.raises(ValueError, match='source pose'):
    p.identify_physical_plant_lag(binding, tape, attacked, windows)


def test_heldout_range_and_training_excitation_remain_original_gates():
  binding, tape, poses, windows = physical_fixture()
  original = p.identify_physical_plant_lag(binding, tape, poses, windows)
  edge = windows[4].original_start_ns
  changed = tuple(reseal(row, command=row.command * 1.5) if row.apply_ns >= edge else row
                  for row in tape.commands)
  result = p.identify_physical_plant_lag(binding, retape(binding, tape, changed), poses, windows)
  assert result.reason == 'heldout_command_range' and result.decision == original.decision
  constant = tuple(reseal(row, command=.1) for row in tape.commands)
  quiet = p.identify_physical_plant_lag(binding, retape(binding, tape, constant), poses, windows)
  assert quiet.status == 'collecting' and quiet.reason == 'training_excitation_slope_direction'


def test_observation_packet_does_not_expand_original_resource_or_row_bounds():
  binding, tape, poses, windows = physical_fixture()
  enlarged = retape(binding, tape, tape.commands * 4)
  with pytest.raises(ValueError, match='tape custody/bounds'):
    p.identify_physical_plant_lag(binding, enlarged, poses, windows)
  with pytest.raises(ValueError, match='40|declaration'):
    p.identify_physical_plant_lag(binding, tape, poses, (replace(windows[0], pose_ids=windows[0].pose_ids[:39]), *windows[1:]))


def _boundary_objects():
  """Only negative trust-boundary tests use an uninitialized real engine type."""
  from openpilot.starpilot.flm.controller_replay import ControllerReplay
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
  binding = physical_fixture()[0]
  stamp = 30_000_000_000
  control = ControlEvidence(binding.identity, 1, stamp, stamp, 'normalized_torque', 'stage', 'response',
    .1, (.1,), .1, True, False, True, True, False, False, False, False, False, False,
    source_epoch=binding.source_epoch, source_receipt_ns=stamp - 10_000_000,
    baseline_revision=binding.baseline_revision, controller_revision='declared-boundary-negative',
    native_required=True, native_allowed=True, native_identity=binding.native_identity,
    native_observed_ns=stamp - 1_000_000, native_expires_ns=stamp + 100_000_000)
  output = AppliedEvidence(stamp, stamp, stamp + 1_000_000, 'normalized_torque', .1,
    True, True, False, False, True, True, carrier_frame=1, carrier_observed_ns=stamp + 1_000_000)
  engine = object.__new__(ControllerReplay)
  engine.failed, engine.raw_mode = False, True
  engine.applied_matches = Mock(side_effect=AssertionError('hard rejection must precede any proof call'))
  return control, output, engine, binding


@pytest.mark.parametrize('field,value', [
  ('enabled', False), ('manual', True), ('active', False), ('valid', False),
  ('override', True), ('fault', True), ('applied_coordinates', (.01,)),
])
def test_constructor_keeps_original_hard_control_guard_even_when_performance_tagged(field, value):
  control, output, engine, binding = _boundary_objects()
  control = replace(control, saturated=True, limited=True, controller_chatter=True, model_wobble=True,
                    **{field: value})
  with pytest.raises(ValueError, match='hard validity'):
    o.from_original(control, output, engine, binding, original_output=output)
  engine.applied_matches.assert_not_called()


@pytest.mark.parametrize('field,value', [('valid', False), ('active', False), ('override', True), ('fault', True), ('native_allowed', False)])
def test_constructor_keeps_original_hard_output_guard(field, value):
  control, output, engine, binding = _boundary_objects()
  output = replace(output, **{field: value})
  with pytest.raises(ValueError, match='hard validity'):
    o.from_original(control, output, engine, binding, original_output=output)
  engine.applied_matches.assert_not_called()


def test_caller_callback_or_claimed_qualified_engine_cannot_create_an_observation():
  control, output, _, binding = _boundary_objects()
  forged_engine = SimpleNamespace(applied_matches=lambda *_: True, verified_source_native_can=True)
  with pytest.raises(ValueError, match='actual replay types'):
    o.from_original(control, output, forged_engine, binding, original_output=output)


def test_real_class_without_actual_original_context_cannot_claim_a_receipt():
  control, output, engine, binding = _boundary_objects()
  with pytest.raises(ValueError, match='original context owner body'):
    o.from_original(control, output, engine, binding, original_output=output)
  engine.applied_matches.assert_not_called()


def test_changed_original_control_body_is_not_blessed_by_a_matching_epoch():
  control, output, engine, binding = _boundary_objects()
  engine.original_context_controls = {control.control_ns: replace(control, controller_chatter=True)}
  with pytest.raises(ValueError, match='original context owner body'):
    o.from_original(control, output, engine, binding, original_output=output)
  engine.applied_matches.assert_not_called()


@pytest.mark.parametrize('changed', [{'native_allowed': False}, {'native_observed_ns': 0},
                                   {'native_expires_ns': 1}, {'source_receipt_ns': 1}, {'generation': -1}])
def test_constructor_rejects_original_expired_native_source_or_generation_before_proof(changed):
  control, output, engine, binding = _boundary_objects()
  with pytest.raises(ValueError, match='lease/carrier'):
    o.from_original(replace(control, **changed), output, engine, binding, original_output=output)
  engine.applied_matches.assert_not_called()


@pytest.mark.parametrize('control_field,output_field', [
  ('saturated', None), ('model_wobble', None), ('controller_chatter', None),
  ('limited', None), (None, 'limited'),
])
def test_old_stage_command_verifier_retains_performance_veto(control_field, output_field):
  from openpilot.starpilot.flm.source_plant_fit import _command
  control, output, _, binding = _boundary_objects()
  control = replace(control, **({control_field: True} if control_field else {}))
  output = replace(output, limited=False, **({output_field: True} if output_field else {})) if output_field != 'limited' else replace(output, limited=True)
  proof = Mock(return_value=True)
  with pytest.raises(ValueError, match='plant actual endpoint'):
    _command(control, output, SimpleNamespace(applied_matches=proof), binding)
  proof.assert_not_called()
