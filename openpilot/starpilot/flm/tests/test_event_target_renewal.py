"""Collector lifecycle regression; synthetic adapter grants are NOT vehicle proof."""
from dataclasses import replace

import pytest

from openpilot.starpilot.flm.adaptive_profile import CurrentAdmission, ReplayValidation, Result
from openpilot.starpilot.flm.canonical_surface_adapter import full_profile_sha256
from openpilot.starpilot.flm.effect_trace import FullEffectTrace, ordered, step_full
from openpilot.starpilot.flm.event_collector import EventFullrateCollector
from openpilot.starpilot.flm.tests.test_full_surface_profile import base, wire_profile
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import BASE, SyntheticAdapter, snapshots


class BoundaryAdapter(SyntheticAdapter):
  target_changed = False
  current_physical_valid = False
  last_physical_ns = 0

  def __init__(self, profile):
    super().__init__()
    self.fit_profile = profile

  def parameters(self):
    return self.fit_profile.parameters

  def cohort(self, control, physical):
    self.current_physical_valid = True
    self.last_physical_ns = physical['response_ns']
    return super().cohort(control, physical)

  def current_admission(self, profile, control, output, context):
    healthy = self.current_physical_valid and self.last_physical_ns > control.control_ns
    return CurrentAdmission(control.identity, 'a' * 64, 'b' * 64, min(control.control_ns, output.apply_ns),
      healthy, control.enabled, control.manual, healthy)


def fixture():
  full = base().with_proposal({'ff_gain_left': .005})
  profile, _ = wire_profile(full)
  profile = replace(profile, observed_ns=BASE, expires_ns=BASE + 500_000_000)
  validation = ReplayValidation(profile, 'a' * 64, 'b' * 64, True, True, .1)
  collector = EventFullrateCollector(BoundaryAdapter(profile))
  old, output, pose = snapshots(BASE)
  old = replace(old, identity=profile.identity, baseline_revision='a' * 64, source_epoch='e' * 64,
    controller_revision=full.controller_revision, native_identity='9' * 64)
  assert collector.control(old, BASE + 1_000_000)
  assert collector.output(output, BASE + 1_000_000)
  collector.active_since = BASE - 3_000_000_000
  collector.physical_context = ('cal', 'vm', 'lag', 0)
  assert collector.install_fit(Result('qualified', 'synthetic', profile), validation)
  collector.blocks.append('old-training')
  stamp = BASE + 10_000_000
  applied = ordered(step_full(dict(full.prior_coordinates), dict(full.coordinates),
    now_ns=stamp, previous_ns=0, qualified=True))
  trace = FullEffectTrace(1, full, stamp, 0, True, full.prior_coordinates, full.coordinates, applied,
    full_profile_sha256(full.selected_baseline_revision, applied)).validate()
  current, output, pose = snapshots(stamp)
  current = replace(current, identity=old.identity, baseline_revision=old.baseline_revision,
    source_epoch=old.source_epoch, controller_revision=old.controller_revision,
    native_identity=old.native_identity, effect_trace=trace, effect_prepare_ns=stamp,
    applied_profile_sequence=1, applied_profile_observed_ns=BASE, applied_profile_expires_ns=BASE + 500_000_000)
  collector.adapter.target_changed = True
  return collector, current, output, pose


def test_exact_accepted_target_keeps_fit_but_requires_new_physical_admission():
  collector, control, output, pose = fixture()
  old_fit, active = collector.fit_result, collector.active_since
  assert collector.control(control, output.apply_ns)
  assert collector.fit_result is old_fit and collector.active_since == active
  assert not collector.blocks and not collector.raw_points
  assert collector.last_qualified_response == 0
  assert collector.adapter.current_physical_valid is False
  assert collector.revalidate_fit(output.apply_ns) is None
  assert collector.output(output, output.apply_ns)
  assert collector.revalidate_fit(output.apply_ns) is None
  assert collector.physical(pose, pose.received_ns)
  assert collector.revalidate_fit(pose.received_ns) is not None
  assert collector.revalidate_fit(pose.received_ns + 101_000_000) is None


@pytest.mark.parametrize('change', ['fault', 'stale', 'source', 'native', 'other_target'])
def test_replaced_or_unhealthy_target_never_retains_fit(change):
  collector, control, output, _ = fixture()
  now = output.apply_ns
  if change == 'fault':
    control = replace(control, fault=True)
  elif change == 'stale':
    now += 101_000_000
  elif change == 'source':
    control = replace(control, source_epoch='0' * 64)
  elif change == 'native':
    control = replace(control, native_identity='0' * 64)
  else:
    collector.fit_result = None
  collector.control(control, now)
  assert collector.fit_result is None
  assert collector.revalidate_fit(now) is None
