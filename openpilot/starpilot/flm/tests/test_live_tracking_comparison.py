"""Uncensored score-baseline and physical-drift seams, synthetic data only."""
from dataclasses import replace
from types import SimpleNamespace

from openpilot.starpilot.flm.tests.test_live_tracking_rollout import _optimizer, _refresh, _episode, NS
from openpilot.starpilot.flm.live_tracking_response import ResponseMonitor
from openpilot.starpilot.flm.tests.test_live_tracking_requalification import certificate, synthetic_pair


def test_error_selected_discovery_is_not_reused_as_comparison_baseline():
  owner = _optimizer()
  index = 0
  while owner.discovery is None and index < 24:
    index += 1
    now = index*8*NS
    _refresh(owner, now)
    owner.observe_episode(_episode(owner, index, now, owner.accepted, .6, .35, 22.), now_ns=now)
  assert owner.discovery is not None
  assert owner.offer_state is None
  discovery = dict(owner.discovery)
  comparison = []
  for _ in range(discovery['quota']):
    index += 1
    now = index*8*NS
    _refresh(owner, now)
    # Every newly requested comparison episode is retained, even perfect0.
    row = _episode(owner, index, now, owner.accepted, .6, 0., 22.)
    comparison.append(row)
    owner.observe_episode(row, now_ns=now)
  if owner.offer_state is not None:
    assert owner.before == tuple(comparison)
    assert all(row.error_rms == 0. for row in owner.before)
  else:
    assert owner.reason == 'independent_baseline_trial_power_insufficient'
  assert owner.discovery is None


def test_overlapping_comparison_does_not_count_twice():
  owner = _optimizer()
  index = 0
  while owner.discovery is None and index < 24:
    index += 1
    now = index*8*NS
    _refresh(owner, now)
    owner.observe_episode(_episode(owner, index, now, owner.accepted, .6, .35, 22.), now_ns=now)
  assert owner.discovery is not None
  index += 1
  now = index*8*NS
  _refresh(owner, now)
  first = _episode(owner, index, now, owner.accepted, .6, .35, 22.)
  owner.observe_episode(first, now_ns=now)
  overlap = replace(first, episode_id='1'*64, start_ns=first.start_ns+1, end_ns=first.end_ns+1)
  assert not owner.observe_episode(overlap, now_ns=now+1)
  assert owner.comparison == [first]


def test_constant_command_point12_physical_shift_cannot_refresh_response_witness():
  cert = certificate()
  source = SimpleNamespace(_binding=cert.binding, original=cert)
  monitor = ResponseMonitor(cert)
  for index in range(8):
    control, output, pose = synthetic_pair(source, index, command=.1)
    assert monitor.observe(control, output, pose, now_ns=pose.mono_ns, binding=cert.binding, current_native=True)
  assert monitor.witness(pose.mono_ns) is not None
  for index in range(8, 16):
    control, output, pose = synthetic_pair(source, index, command=.1)
    shifted_yaw = pose.yaw_rate+.12/pose.speed_mps
    pose = replace(pose, yaw_rate=shifted_yaw, angular_velocity_device=(0., 0., shifted_yaw))
    monitor.observe(control, output, pose, now_ns=pose.mono_ns, binding=cert.binding, current_native=True)
    assert monitor.witness(pose.mono_ns) is None


def test_changed_actual_commands_explain_point12_shift_without_proving_trial_benefit():
  cert = certificate()
  source = SimpleNamespace(_binding=cert.binding, original=cert)
  monitor = ResponseMonitor(cert)
  for index in range(16):
    control, output, pose = synthetic_pair(source, index, command=.1 if index < 8 else .14)
    assert monitor.observe(control, output, pose, now_ns=pose.mono_ns, binding=cert.binding, current_native=True)
  assert monitor.witness(pose.mono_ns) is not None
  # A response witness licenses the numerical model, never causal attribution
  # of a tracking improvement to the offered knob rather than another input.
