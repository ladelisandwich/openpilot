"""Adapter geometry/join units; synthetic witnesses are NOT native authority.

The public control path uses real typed evidence. The finalization units inject
numerical independent endpoints explicitly; real source pose/native admission
and collector integration require root's separate actual caller qualification.
"""

from dataclasses import replace
import math
from types import SimpleNamespace

import pytest
from collections.abc import Callable
from typing import cast

from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256
from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding
from openpilot.starpilot.flm.live_tracking_response import ResponseCertificate, VERSION, _revision, physical_calibration_generation
from openpilot.starpilot.flm.live_tracking_episodes import TrackingEpisodeBuilder, _Endpoint

BASE = 1_000_000_000
DT = 10_000_000


def certificate():
  binding = LiveTrialBinding('a' * 64, 'b' * 64, 'controller-current', 'd' * 64, 'e' * 64, '1' * 64, 'f' * 32, 'a' * 64, '', '0' * 64, 'b' * 64, False)
  value = ResponseCertificate(
    VERSION, binding, '0' * 64, 170_000_000, 3.0, 0.0, -0.3, 0.3, 0.005, ('a0', 'a1', 'a2', 'a3'), ('b0', 'b1', 'b2', 'b3'), 'c' * 64, 'd' * 64, 1
  )
  revision = _revision(value)
  return replace(value, model_revision=revision, binding=replace(binding, physical_model_revision=revision))


def owner():
  cert = certificate()
  return TrackingEpisodeBuilder(cert.binding, cert)


def pair(builder, index, geometry=0.1, *, coordinates=None):
  stamp = BASE + index * DT
  values = tuple(0.0 for _ in NAMES) if coordinates is None else coordinates
  b = builder.binding
  control = ControlEvidence(
    b.identity,
    1,
    stamp,
    stamp,
    'normalized_torque',
    'stage',
    'response',
    0.0,
    (),
    0.0,
    True,
    False,
    True,
    True,
    False,
    False,
    False,
    False,
    False,
    False,
    stage_schema_revision='torque-selected-stage-snapshot-v1',
    stage_values=(('setpoint', geometry),),
    applied_coordinates=(0.0, 0.0),
    live_applied_coordinates=() if coordinates is None else values,
    source_epoch=b.source_epoch,
    source_receipt_ns=stamp,
    baseline_revision=b.baseline_revision,
    controller_revision=b.controller_revision,
    replay_inputs=(('desired_curvature', geometry / 400.0), ('vEgo', 20.0), ('roll', 0.0), ('curvature_limited', 0.0), ('control_frame', float(index))),
  )
  output = AppliedEvidence(stamp, stamp, stamp, 'normalized_torque', 0.0, True, True, False, False, False, False)
  return control, output, full_profile_sha256(b.baseline_revision, dict(zip(NAMES, values, strict=True)))


def tick(builder, index, geometry, **changes):
  c, o, profile = pair(builder, index, geometry)
  c = replace(c, **changes)
  return builder.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)


def test_subtle_requires_two_reversals_after_entry_not_a_storage_chunk():
  b = owner()
  for index in range(100):
    assert tick(b, index, 0.1 if index < 25 or index >= 75 else -0.1)
  assert not b.pending  # Only one later reversal, not a complete cycle.
  for index in range(100, 126):
    assert tick(b, index, -0.1)
  assert len(b.pending) == 1
  start, end, kind, direction, _ = b.pending[0]
  assert (start, end, kind, direction) == (BASE + 25 * DT, BASE + 100 * DT - 1, 'subtle', -1)
  assert not b.drain()  # Neither time nor geometry invents physical endpoints.


def test_large_neutral_requires_half_second_and_retains_true_direction():
  b = owner()
  for index in range(80):
    assert tick(b, index, 0.5 if index < 20 else 0.0)
  assert len(b.pending) == 1
  assert b.pending[0][:4] == (BASE, BASE + 70 * DT, 'large', 1)


@pytest.mark.parametrize('field', ['manual', 'override', 'fault', 'saturated', 'model_wobble'])
def test_interruption_cannot_complete_an_open_episode(field):
  b = owner()
  for index in range(40):
    assert tick(b, index, 0.5)
  assert not tick(b, 40, 0.5, **{field: True})
  assert not b.pending and not b.ticks and b.start == 0
  for index in range(41, 100):
    assert tick(b, index, 0.0)
  assert not b.pending and not b.drain()


def test_actual_clipping_is_excluded_but_async_hint_requires_original_unclipped_proof(monkeypatch):
  b = owner()
  assert not tick(b, 0, 0.1, limited=True)
  monkeypatch.setattr('openpilot.starpilot.flm.live_tracking_episodes.exact_unclipped_pair', lambda c, o: True)
  assert tick(b, 1, 0.1, limited=True)  # Unit branch only, not a forged native proof.
  c, o, profile = pair(b, 2)
  assert not b.observe_control(c, replace(o, limited=True), profile_sha256=profile, now_ns=c.control_ns, current_native=True)


def test_control_gap_and_profile_change_discard_partial_lineage():
  b = owner()
  assert tick(b, 0, 0.5)
  assert tick(b, 20, 0.5)
  assert b.start == BASE + 20 * DT and not b.pending
  c, o, profile = pair(b, 21, 0.5, coordinates=(0.001,) + (0.0,) * 29)
  assert b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  assert b.start == c.control_ns and not b.pending


def completed_builder(*, count=8, speed=20.0, error=0.0):
  b = owner()
  for index in range(80):
    assert tick(b, index, 0.5 if index < 20 else 0.0)
  # Numerical finalization fixtures only; no pose-validation/native assertion.
  for index in range(count):
    stamp = BASE + index * 5 * DT
    b.endpoints.append(_Endpoint(stamp, stamp, stamp + b.certificate.lag_ns, speed if index else 20.0, 0.5, error, 0.5 - error))
  b.watermark = BASE + 79 * DT
  return b


def test_four_independent_endpoints_minimum_and_good_future_zero_error_is_kept():
  b = completed_builder(count=3)
  b._finish()
  assert not b.drain()
  b = completed_builder(count=4, error=0.0)
  b._finish()
  (episode,) = b.drain()
  assert episode.endpoints == 4 and episode.error_rms == 0.0
  assert episode.control_max_gap_ns == DT and episode.sensitivity == ()
  assert episode.kind == 'large' and episode.direction == 1


def test_speed_spread_and_pose_watermark_cannot_be_faked_by_wall_time():
  b = completed_builder(speed=24.0)
  b._finish()
  assert not b.drain()
  b = completed_builder()
  b.watermark = 0
  b._finish()
  assert not b.drain() and len(b.pending) == 1


def test_physical_probe_aggregates_have_signed_derivative_and_independent_native_support(monkeypatch):
  monkeypatch.setattr('openpilot.starpilot.flm.live_tracking_probe.validate_probe_result', lambda value, binding: value)
  b = completed_builder(count=4, error=0.2)
  name = NAMES[0]
  for index, point in enumerate(b.endpoints):
    row = SimpleNamespace(
      name=name, physical_derivative=-2.0, native_separated=index % 2 == 0, max_step_physical_delta=0.01, status='separated',
      baseline_coordinate=0.0, minus_coordinate=-0.05, plus_coordinate=0.05,
      baseline_delivered=0.0, minus_delivered=0.01 if index % 2 == 0 else 0.0, plus_delivered=-0.01 if index % 2 == 0 else 0.0,
    )
    b.probes[point.control_ns] = SimpleNamespace(
      rows=(row,), control_ns=point.control_ns, apply_ns=point.apply_ns, profile_sha256=b.pending[0][4], model_revision=b.certificate.model_revision
    )
  b._finish()
  (episode,) = b.drain()
  (sensitivity,) = episode.sensitivity
  assert sensitivity.name == name
  assert math.isclose(sensitivity.error_feature_sum, -1.6)
  assert sensitivity.feature_energy == 16.0 and sensitivity.native_separated == 2
  assert sensitivity.native_endpoints == 4 and sensitivity.max_step_physical_rms == 0.01


def test_same_stamp_different_control_is_not_accepted_as_fresh_input():
  b = owner()
  assert tick(b, 0, 0.1)
  assert not tick(b, 0, 0.1)  # exact duplicate doesn't refresh
  assert len(b.ticks) == 1
  assert not tick(b, 0, 0.2)  # same stamp mutation invalidates lineage
  assert not b.ticks


def numeric_pose(builder, control, *, yaw=0.01):
  from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose

  stamp = control.control_ns + builder.certificate.lag_ns
  return OriginalSourcePose(
    stamp,
    stamp,
    0,
    0,
    0,
    physical_calibration_generation(builder.binding),
    '0' * 64,
    ('synthetic-numeric-join-only',),
    yaw,
    0.0,
    '',
    True,
    True,
    builder.binding.source_epoch,
    builder.binding.identity,
    builder.binding.baseline_revision,
    builder.binding.controller_revision,
    control.control_ns,
    control.control_ns,
    observed_ns=stamp,
    completed_ns=stamp,
    native_required=False,
    source_lower_ns=stamp,
    source_upper_ns=stamp,
    source_timestamp_ns=stamp,
    speed_mps=20.0,
  )


def numerical_join_validation_only(monkeypatch, builder):
  # These two seams isolate adapter stamp/context checks. They do NOT claim
  # native/source-clock admission; root actual source joins remain required.
  monkeypatch.setattr('openpilot.starpilot.flm.live_tracking_episodes.validate_pose', lambda pose, now: None)
  monkeypatch.setattr('openpilot.starpilot.flm.live_tracking_episodes.validate_response_witness', lambda witness, binding, now: witness)


def test_pose_uses_same_frame_force_target_and_independent_real_stamp_without_upsampling(monkeypatch):
  b = owner()
  numerical_join_validation_only(monkeypatch, b)
  for index in range(30):
    assert tick(b, index, 0.1)
  for index in (0, 5, 10, 15):
    c, o, _ = pair(b, index, 0.1)
    pose = numeric_pose(b, c)
    assert b.observe_pose(
      pose,
      c,
      o,
      witness=SimpleNamespace(certificate=b.certificate, last_control_ns=c.control_ns, last_apply_ns=o.apply_ns, last_pose_ns=pose.mono_ns),
      now_ns=pose.mono_ns,
      current_native=True,
    )
    assert not b.observe_pose(
      pose,
      c,
      o,
      witness=SimpleNamespace(certificate=b.certificate, last_control_ns=c.control_ns, last_apply_ns=o.apply_ns, last_pose_ns=pose.mono_ns),
      now_ns=pose.mono_ns,
      current_native=True,
    )
  assert len(b.endpoints) == 4  # 30 control ticks are not30 physical samples.
  assert all(math.isclose(point.error, -0.1) for point in b.endpoints)
  assert b.endpoints[0].pose_ns == BASE + b.certificate.lag_ns


@pytest.mark.parametrize('change', ['lag', 'calibration', 'controller', 'unseen_control', 'stale', 'native'])
def test_invalid_pose_association_resets_partial_window(monkeypatch, change):
  b = owner()
  numerical_join_validation_only(monkeypatch, b)
  for index in range(20):
    assert tick(b, index, 0.5)
  c, o, _ = pair(b, 5, 0.5)
  pose = numeric_pose(b, c)
  now, current = pose.mono_ns, True
  if change == 'lag':
    pose = replace(pose, source_lower_ns=pose.source_lower_ns - 1)
  elif change == 'calibration':
    pose = replace(pose, calibration='9' * 64)
  elif change == 'controller':
    pose = replace(pose, controller_revision='other')
  elif change == 'unseen_control':
    c, o, _ = pair(b, 100, 0.5)
    pose = numeric_pose(b, c)
    now = pose.mono_ns
  elif change == 'stale':
    now += 100_000_001
  elif change == 'native':
    current = False
  assert not b.observe_pose(
    pose,
    c,
    o,
    witness=SimpleNamespace(certificate=b.certificate, last_control_ns=c.control_ns, last_apply_ns=o.apply_ns, last_pose_ns=pose.mono_ns),
    now_ns=now,
    current_native=current,
  )
  assert not b.ticks and not b.pending and not b.endpoints


def test_plot_contains_only_last_completed_actual_endpoint_stamps_and_values():
  b = completed_builder(count=4, error=0.2)
  b._finish()
  (episode,) = b.drain()
  plot = b.tracking_plot()
  assert plot['episodeId'] == episode.episode_id
  assert len(plot['points']) == 4 and plot['lagNs'] == b.certificate.lag_ns
  for row, original in zip(plot['points'], b.endpoints, strict=True):
    assert (row['controlMonoTime'], row['applyMonoTime'], row['poseMonoTime']) == (original.control_ns, original.apply_ns, original.pose_ns)
    assert (row['desiredForceAccel'], row['measuredForceAccel'], row['errorForceAccel']) == (0.5, 0.3, 0.2)
  plot['points'].clear()
  assert len(b.tracking_plot()['points']) == 4
  b.interrupt('driver')
  assert b.tracking_plot() is None


def test_missing_owner_frame_is_not_a_continuous_whole_episode():
  b = owner()
  assert tick(b, 0, 0.5)
  c, o, profile = pair(b, 1, 0.5)
  values = tuple((key, 3.0 if key == 'control_frame' else value) for key, value in c.replay_inputs)
  c = replace(c, replay_inputs=values)
  assert b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  assert b.start == c.control_ns and not b.pending
  assert len(b.ticks) == 1


def test_goal_shape_uses_original_fullrate_force_in_eight_real_time_bins():
  b = completed_builder(count=4, error=0.2)
  b._finish()
  (episode,) = b.drain()
  ticks = [tick for stamp, tick in b.ticks.items() if episode.start_ns <= stamp <= episode.end_ns]
  bins = [[] for _ in range(8)]
  for tick in ticks:
    index = min(7, (tick.control.control_ns - episode.start_ns) * 8 // (episode.end_ns - episode.start_ns + 1))
    bins[index].append(tick.goal)
  assert len(episode.goal_shape) == 8
  assert episode.goal_shape == tuple(sum(values) / len(values) / episode.goal_rms for values in bins)
  assert episode.goal_shape[0] == 1.0 and episode.goal_shape[-1] == 0.0


def test_missing_real_goal_bin_never_interpolates_a_shape():
  b = completed_builder(count=4)
  start, end, *_ = b.pending[0]
  # Numerical finalization-only corrupted-inventory negative; no source proof.
  for stamp in tuple(b.ticks):
    if start + 100_000_000 <= stamp < start + 200_000_000:
      del b.ticks[stamp]
  b._finish()
  assert not b.drain()


def test_actual_neutral_legacy_two_space_is_not_mistaken_for_missing_full30():
  b = owner()
  c, o, profile = pair(b, 0)
  assert c.applied_coordinates == (0.0, 0.0) and c.live_applied_coordinates == ()
  assert b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)


def test_explicit_actual_live_map_not_pending_offer_supplies_nonzero_full30():
  b = owner()
  values = (0.001,) + (0.0,) * 29
  c, o, profile = pair(b, 0, coordinates=values)
  assert b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  b = owner()
  c, o, _ = pair(b, 0)
  assert not b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)


def test_old_nonneutral_selected_residual_is_not_unseen_full30():
  b = owner()
  c, o, profile = pair(b, 0)
  c = replace(c, applied_coordinates=(0.001, 0.0))
  assert not b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)


def test_existing_full_effect_trace_uses_actual_map_and_original_binding():
  from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope
  from openpilot.starpilot.flm.effect_trace import FullEffectTrace, VERSION as TRACE_VERSION

  b = owner()
  c, o, profile = pair(b, 0)
  env = FullSurfaceEnvelope.initial(
    cp_sha256=b.binding.cp_sha256,
    controller_revision=b.binding.controller_revision,
    source_epoch=b.binding.source_epoch,
    reset_generation=b.binding.reset_generation,
    selected_baseline_revision=b.binding.baseline_revision,
    persistent_source_epoch=b.binding.persistent_source_epoch,
    trial_names=(NAMES[0],),
  )
  trace = FullEffectTrace(TRACE_VERSION, env, c.control_ns, 0, True, env.coordinates, env.coordinates, env.coordinates, profile)
  c = replace(
    c,
    effect_trace=trace,
    effect_prepare_ns=c.control_ns,
    applied_profile_sequence=1,
    applied_profile_observed_ns=c.control_ns,
    applied_profile_expires_ns=c.control_ns + 500_000_000,
  )
  assert b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  b = owner()
  assert not b.observe_control(replace(c, live_applied_coordinates=(0.0,) * 30), o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)


@pytest.mark.parametrize('emit_every', [1, 2])
def test_fullrate_geometry_is_independent_of_native_emission_cadence(emit_every):
  b = owner()
  for index in range(130):
    geometry = 0.1 if index < 25 or 75 <= index < 100 else -0.1
    c, o, profile = pair(b, index, geometry)
    assert b.observe_source_control(c, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
    if index % emit_every == 0:
      assert b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  assert len(b.ticks) == 130 and len(b.pending) == 1
  assert sum(tick.output is not None for tick in b.ticks.values()) == (130 + emit_every - 1) // emit_every
  assert b.pending[0][:4] == (BASE + 25 * DT, BASE + 100 * DT - 1, 'subtle', -1)
  assert not b.endpoints  # Native outputs are not physical samples.


def test_non_emitted_source_tick_can_never_be_a_physical_endpoint(monkeypatch):
  b = owner()
  numerical_join_validation_only(monkeypatch, b)
  c, o, profile = pair(b, 0, 0.5)
  assert b.observe_source_control(c, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  assert b.ticks[c.control_ns].output is None
  pose = numeric_pose(b, c)
  witness = SimpleNamespace(certificate=b.certificate, last_control_ns=c.control_ns, last_apply_ns=o.apply_ns, last_pose_ns=pose.mono_ns)
  assert not b.observe_pose(pose, c, o, witness=witness, now_ns=pose.mono_ns, current_native=True)
  assert not b.endpoints


def test_delayed_real_native_join_does_not_discard_newer_source_geometry():
  b = owner()
  for index in range(4):
    c, o, profile = pair(b, index, 0.5)
    assert b.observe_source_control(c, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  c, o, profile = pair(b, 1, 0.5)
  assert b.observe_control(c, o, profile_sha256=profile, now_ns=BASE + 3 * DT, current_native=True)
  assert len(b.ticks) == 4 and b.start == BASE
  assert b.ticks[c.control_ns].output == o


def test_actual_output_clipping_and_controller_saturation_interrupt_all_cadences():
  b = owner()
  for index in range(4):
    c, o, profile = pair(b, index, 0.5)
    assert b.observe_source_control(c, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  c, o, profile = pair(b, 2, 0.5)
  assert not b.observe_control(c, replace(o, limited=True), profile_sha256=profile, now_ns=BASE + 3 * DT, current_native=True)
  assert not b.ticks
  c, o, profile = pair(b, 4, 0.5)
  assert not b.observe_source_control(replace(c, saturated=True), profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  assert not b.ticks


def test_configured_turn_assist_is_source_context_not_a_false_clipping_flag():
  b = owner()
  assert tick(b, 0, 0.1, turn_assist=True)
  c, o, profile = pair(b, 1, 0.1)
  c = replace(c, turn_assist=True)
  assert not b.observe_control(c, replace(o, limited=True), profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  assert b.last_input_status == 'invalid' and not b.ticks


def test_pre_boundary_lagged_pose_does_not_deadlock_new_profile_history(monkeypatch):
  b = owner()
  numerical_join_validation_only(monkeypatch, b)
  old_c, old_o, _ = pair(b, 0, 0.1)
  assert tick(b, 0, 0.1)
  c, o, profile = pair(b, 1, 0.1, coordinates=(0.001,) + (0.0,) * 29)
  assert b.observe_control(c, o, profile_sha256=profile, now_ns=c.control_ns, current_native=True)
  before = tuple(b.ticks.items())
  pose = numeric_pose(b, old_c)
  witness = SimpleNamespace(certificate=b.certificate, last_control_ns=old_c.control_ns, last_apply_ns=old_o.apply_ns, last_pose_ns=pose.mono_ns)
  assert not b.observe_pose(pose, old_c, old_o, witness=witness, now_ns=pose.mono_ns, current_native=True)
  assert b.last_input_status == 'ignored' and tuple(b.ticks.items()) == before
  assert not b.endpoints and b.watermark == 0 and b.last_pose_ns == 0
  assert not b.observe_control(old_c, old_o, profile_sha256=profile, now_ns=old_o.apply_ns, current_native=True)
  assert b.last_input_status == 'ignored' and tuple(b.ticks.items()) == before


def requested_completed_builder(monkeypatch):
  # Pure numerical support-seal seam; no child/native ownership is claimed.
  monkeypatch.setattr('openpilot.starpilot.flm.live_tracking_probe.validate_probe_result', lambda value, binding: value)
  from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS

  b = completed_builder(count=4, error=0.2)
  name = NAMES[0]
  group = next(group for group, names in GROUPS.items() if name in names)
  queued = 1_000_000_000_000
  for point in b.endpoints:
    assert b.request_probe(point.control_ns, group=group, names=(name,), queued_ns=queued)
  b._finish()
  assert not b.drain(now_ns=queued) and len(b.completed) == 1
  return b, group, name, queued


def numerical_probe(b, point, group, name):
  return SimpleNamespace(
    binding=b.binding,
    control_ns=point.control_ns,
    apply_ns=point.apply_ns,
    profile_sha256=next(iter(b.completed.values()))['episode'].profile_sha256,
    model_revision=b.certificate.model_revision,
    group=group,
    rows=(SimpleNamespace(
      name=name, physical_derivative=-2.0, native_separated=True, max_step_physical_delta=0.01, status='separated',
      baseline_coordinate=0.0, minus_coordinate=-0.05, plus_coordinate=0.05,
      baseline_delivered=0.0, minus_delivered=0.01, plus_delivered=-0.01,
    ),),
  )


def test_late_four_endpoint_support_keeps_exact_episode_once_without_pose_renewal(monkeypatch):
  b, group, name, queued = requested_completed_builder(monkeypatch)
  saved = next(iter(b.completed.values()))
  episode_id = saved['episode'].episode_id
  points = saved['points']
  clock = (b.last_pose_ns, b.watermark)
  b.ticks.clear()  # Original control history may age out; exact completed endpoints remain.
  for index, point in enumerate(points):
    assert b.observe_probe(numerical_probe(b, point, group, name), now_ns=queued + index + 1)
    if index < 3:
      assert not b.drain(now_ns=queued + index + 1)
  (episode,) = b.drain(now_ns=queued + 5)
  assert episode.episode_id == episode_id and episode.sensitivity[0].native_endpoints == 4
  assert (b.last_pose_ns, b.watermark) == clock
  assert not b.drain(now_ns=queued + 30_000_000_001)
  assert not b.observe_probe(numerical_probe_from_saved(b, saved, points[0], group, name), now_ns=queued + 6)


def numerical_probe_from_saved(b, saved, point, group, name):
  return SimpleNamespace(
    binding=b.binding,
    control_ns=point.control_ns,
    apply_ns=point.apply_ns,
    profile_sha256=saved['episode'].profile_sha256,
    model_revision=b.certificate.model_revision,
    group=group,
    rows=(SimpleNamespace(
      name=name, physical_derivative=-2.0, native_separated=True, max_step_physical_delta=0.01, status='separated',
      baseline_coordinate=0.0, minus_coordinate=-0.05, plus_coordinate=0.05,
      baseline_delivered=0.0, minus_delivered=0.01, plus_delivered=-0.01,
    ),),
  )


def test_probe_deadline_is_first_request_fixed_and_partial_support_remains_partial(monkeypatch):
  b, group, name, queued = requested_completed_builder(monkeypatch)
  saved = next(iter(b.completed.values()))
  point = saved['points'][0]
  assert b.request_probe(point.control_ns, group=group, names=(name,), queued_ns=queued + 29_000_000_000)
  assert saved['deadline'] == queued + 30_000_000_000
  assert b.observe_probe(numerical_probe(b, point, group, name), now_ns=queued + 1)
  assert not b.drain(now_ns=queued + 29_999_999_999)
  (episode,) = b.drain(now_ns=queued + 30_000_000_000)
  assert episode.sensitivity[0].native_endpoints == 1  # Never pretend four.
  assert b.reason == 'whole_episode_probe_deadline_partial_support'
  assert not b.request_probe(point.control_ns, group=group, names=(name,), queued_ns=queued + 30_000_000_001)
  assert not b.drain(now_ns=queued + 60_000_000_000)


@pytest.mark.parametrize('change', ['apply', 'profile', 'model', 'group', 'names', 'late'])
def test_late_probe_cannot_change_geometry_or_cross_numeric_context(monkeypatch, change):
  b, group, name, queued = requested_completed_builder(monkeypatch)
  saved = next(iter(b.completed.values()))
  result = numerical_probe(b, saved['points'][0], group, name)
  now = queued + 1
  if change == 'apply':
    result.apply_ns += 1
  elif change == 'profile':
    result.profile_sha256 = '9' * 64
  elif change == 'model':
    result.model_revision = '9' * 64
  elif change == 'group':
    result.group = 'not-the-requested-group'
  elif change == 'names':
    result.rows = ()
  else:
    now = queued + 30_000_000_001
  assert not b.observe_probe(result, now_ns=now)
  assert not saved['probes'] and len(b.completed) == 1


def test_interruption_discards_retained_geometry_and_cannot_reemit_old_episode(monkeypatch):
  b, group, name, queued = requested_completed_builder(monkeypatch)
  saved = next(iter(b.completed.values()))
  result = numerical_probe(b, saved['points'][0], group, name)
  b.interrupt('actual_source_changed')
  assert not b.completed and not b.probe_requests
  assert not b.observe_probe(result, now_ns=queued + 1)
  assert not b.drain(now_ns=queued + 30_000_000_000)


def test_unimplemented_steady_kind_and_constructor_policy_are_explicitly_rejected():
  b = completed_builder(count=4, error=0.0)
  b._finish()
  (episode,) = b.drain()
  with pytest.raises(ValueError, match='complete real tracking episode required'):
    replace(episode, kind='steady', direction=0).validate()
  cert = certificate()
  with pytest.raises(TypeError):
    cast(Callable[..., TrackingEpisodeBuilder], TrackingEpisodeBuilder)(cert.binding, cert, enable_steady=True)


def test_supported_cyclic_geometry_keeps_perfect_future_tracking_without_error_floor():
  b = owner()
  for index in range(150):
    geometry = 0.1 if index < 25 or 75 <= index < 100 else -0.1
    assert tick(b, index, geometry)
  # Numerical physical endpoints only; actual GM/Toyota/HKG native cadence
  # remains exercised separately without changing its assertions or verdicts.
  for index in range(25, 100, 5):
    goal = 0.1 if index >= 75 else -0.1
    stamp = BASE + index * DT
    b.endpoints.append(_Endpoint(stamp, stamp, stamp + b.certificate.lag_ns, 20.0, goal, 0.0, goal))
  b.watermark = BASE + 149 * DT
  b._finish()
  (episode,) = b.drain()
  assert episode.kind == 'subtle' and episode.direction == -1
  assert episode.start_ns == BASE + 25 * DT and episode.end_ns == BASE + 100 * DT - 1
  assert episode.error_rms == 0.0 and episode.reversal_hz == 0.0 and episode.endpoints == 15
  assert episode.sensitivity == () and episode.control_max_gap_ns == DT
  assert not b.drain()


@pytest.mark.parametrize('bad_reference', (None, 'missing', 'stale', 'future_only', 'profile', 'changed_goal', 'epoch', 'invalid'))
def test_tracking_reference_matches_physical_response_time_without_retiming_command(monkeypatch, bad_reference):
  b = owner()
  numerical_join_validation_only(monkeypatch, b)
  for index in range(41):
    assert tick(b, index, 0.1 if index < 17 else 0.3)
  c, o, _ = pair(b, 0, 0.1)
  pose = numeric_pose(b, c, yaw=0.01)
  reference_stamp = BASE + 17 * DT
  if bad_reference == 'missing':
    for stamp in list(b.ticks):
      if stamp > BASE:
        del b.ticks[stamp]
  elif bad_reference == 'stale':
    for stamp in list(b.ticks):
      if stamp > BASE + 13 * DT:
        del b.ticks[stamp]
  elif bad_reference == 'future_only':
    for stamp in list(b.ticks):
      if BASE < stamp <= pose.mono_ns:
        del b.ticks[stamp]
  elif bad_reference == 'profile':
    b.ticks[reference_stamp] = replace(b.ticks[reference_stamp], profile='9' * 64)
  elif bad_reference == 'changed_goal':
    b.ticks[reference_stamp] = replace(b.ticks[reference_stamp], goal=0.9)
  elif bad_reference == 'epoch':
    b.ticks[reference_stamp] = replace(b.ticks[reference_stamp], control=replace(b.ticks[reference_stamp].control, source_epoch='9' * 64))
  elif bad_reference == 'invalid':
    b.ticks[reference_stamp] = replace(b.ticks[reference_stamp], control=replace(b.ticks[reference_stamp].control, valid=False))
  result = b.observe_pose(
    pose,
    c,
    o,
    witness=SimpleNamespace(certificate=b.certificate, last_control_ns=c.control_ns, last_apply_ns=o.apply_ns, last_pose_ns=pose.mono_ns),
    now_ns=pose.mono_ns,
    current_native=True,
  )
  if bad_reference is not None:
    assert result is False and not b.endpoints
  else:
    assert result is True and len(b.endpoints) == 1
    point = b.endpoints[0]
    assert point.control_ns == c.control_ns and point.apply_ns == o.apply_ns
    assert point.pose_ns == pose.mono_ns and point.reference_ns == reference_stamp
    assert math.isclose(point.goal, 0.3) and math.isclose(point.measured, 0.2) and math.isclose(point.error, 0.1)
    assert not math.isclose(point.error, 0.1 - point.measured)


def test_comparison_releases_real_measurements_before_optional_probe_deadline_without_claiming_gradients(monkeypatch):
  b, group, name, queued = requested_completed_builder(monkeypatch)
  row = next(iter(b.completed.values()))
  original = row['episode']
  deadline = row['deadline']
  assert queued + 1 < deadline and not b.drain(now_ns=queued + 1)
  assert b.release_measured_only()
  measured = b.drain(now_ns=queued + 1)
  assert len(measured) == 1
  current = measured[0]
  for field in (
    'binding',
    'profile_sha256',
    'episode_id',
    'start_ns',
    'end_ns',
    'kind',
    'direction',
    'goal_rms',
    'error_rms',
    'endpoints',
    'control_max_gap_ns',
    'goal_shape',
  ):
    assert getattr(current, field) == getattr(original, field)
  assert current.sensitivity == () and current.native_gradient_complete is False
  assert row['deadline'] == deadline
  assert not b.completed and not b.probe_requests
  assert b.closed_through_ns == original.end_ns
  assert b.release_measured_only() and not b.drain(now_ns=queued + 2)
  assert not b.request_probe(row['points'][0].control_ns, group=group, names=(name,), queued_ns=queued + 2)


def test_measured_only_release_rejects_overflow_instead_of_dropping_or_fabricating_episodes(monkeypatch):
  b, _, _, queued = requested_completed_builder(monkeypatch)
  row = next(iter(b.completed.values()))
  # Explicit malformed-backlog injection; never source/native evidence.
  b.ready.extend([row['episode']] * b.ready.maxlen)
  assert not b.release_measured_only()
  assert b.reason == 'measured_episode_backlog'
  assert not b.ready and not b.completed and not b.probe_requests


def test_tracking_goal_uses_original_actual_controller_target_not_future_feedforward_request():
  from openpilot.starpilot.flm.live_tracking_episodes import tracking_reference_force

  b = owner()
  c, _, _ = pair(b, 0, geometry=-0.1)
  # Actual delayed setpoint is negative while future model demand has already
  # become positive. All ORIGINAL times remain unchanged.
  c = replace(c, replay_inputs=tuple((key, 0.3 / 400.0 if key == 'desired_curvature' else 0.02 if key == 'roll' else value) for key, value in c.replay_inputs))
  expected = -0.1 - 9.81 * math.sin(0.02)
  assert tracking_reference_force(c) == pytest.approx(expected)
  assert tracking_reference_force(c) != pytest.approx(0.3 - 9.81 * math.sin(0.02))
  assert c.control_ns == BASE and dict(c.stage_values)['setpoint'] == -0.1


def test_actual_standard_torque_controller_reports_a_different_target_from_its_future_request():
  from opendbc.car.toyota.values import CAR
  from opendbc.car.car_helpers import interfaces
  from opendbc.car import structs
  from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
  from openpilot.starpilot.flm.live_tracking_episodes import tracking_reference_force

  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  controller, vm = SelectedControllerFactory(cp, 'standard').create()
  cs = structs.CarState.new_message(vEgo=20.0, steeringAngleDeg=0.0, steeringPressed=False, steeringRateDeg=0.0, standstill=False)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  for index in range(40):
    curvature = (index + 1) * 0.00001
    _, _, log = controller.update(True, cs, vm, params, False, curvature, False, 0.2)
  assert abs(log.desiredLateralAccel - curvature * cs.vEgo**2) > 0.03
  c, _, _ = pair(owner(), 0, geometry=float(log.desiredLateralAccel))
  c = replace(c, replay_inputs=tuple((key, curvature if key == 'desired_curvature' else value) for key, value in c.replay_inputs))
  assert tracking_reference_force(c) == pytest.approx(float(log.desiredLateralAccel))


@pytest.mark.parametrize(
  'errors,expected',
  [
    ((-0.03, -0.02, 0.003, -0.002, 0.004, -0.003, 0.02, 0.03), 0.5),
    ((0.004, -0.004, 0.003, -0.003, 0.002, -0.002), 0.0),
    ((0.03, -0.03, 0.03, -0.03, 0.03), 2.0),
  ],
)
def test_weave_counter_uses_actionable_excursions_not_reference_zero_jitter(errors, expected):
  from openpilot.starpilot.flm.live_tracking_episodes import tracking_reversal_rate

  assert tracking_reversal_rate(errors, 2.0, 0.0) == expected


def test_weave_counter_retains_raw_error_and_respects_identified_measurement_floor():
  from openpilot.starpilot.flm.live_tracking_episodes import tracking_reversal_rate

  errors = (0.04, -0.04, 0.04)
  original = tuple(errors)
  assert tracking_reversal_rate(errors, 2.0, 0.02) == 0.0
  assert tracking_reversal_rate((0.08, -0.08, 0.08), 2.0, 0.02) == 1.0
  assert errors == original
