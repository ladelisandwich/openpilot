"""Numerical intent only; actual command/native leases are never restored."""
import pytest
from openpilot.starpilot.flm.tests.test_live_tracking import optimizer, ready, refresh, acknowledge, episode


def active(owner=None, error=.1):
  owner = optimizer() if owner is None else owner
  state, now = ready(owner)
  assert state is not None
  pending = owner.offer(now)
  assert pending is not None and acknowledge(owner, pending)
  row = episode(owner, 20, 'ff_gain_left', profile=owner._hash(state['target']), error=error, start=now+2_000_000_000)
  refresh(owner, row.end_ns)
  owner.observe_episode(row, now_ns=row.end_ns)
  assert len(owner.future) == 1
  return owner, row.end_ns, state.copy(), owner.before, tuple(owner.future)


def rollback_ack(owner, now):
  refresh(owner, now)
  offer = owner.offer(now)
  assert offer is not None and offer.phase == 'rollback'
  assert acknowledge(owner, offer, stamp=now+10_000_000)


@pytest.mark.parametrize('error', (.1, .201))
def test_evidence_gap_retains_study_but_requires_fresh_baseline_ack_and_new_wire_trial(error):
  owner, now, old, before, future = active(error=error)
  owner.interrupt('acknowledged_trial_profile_withdrawn')
  assert owner.offer_state['phase'] == 'rollback' and owner.settled_ns is None
  assert owner.before == () and owner.future == []
  rollback_ack(owner, now+100_000_000)
  resumed = owner.offer_state
  assert resumed['phase'] == 'trial' and resumed['trial_id'] != old['trial_id']
  assert resumed['target_sequence'] > old['target_sequence'] and resumed['published'] is False
  assert owner.before == before and tuple(owner.future) == future and owner.settled_ns is None
  for field in ('target', 'baseline', 'started_ns', 'deadline_ns', 'alpha', 'required_episodes', 'look_quotas'):
    assert resumed[field] == old[field]
  assert owner.paused_intent is None


@pytest.mark.parametrize('reason', ('manual_profile_override', 'episode actual native join unqualified', 'persistent_physical_response_drift',
  'invalid_control', 'ineligible_applied_output', 'current_source_or_native_permission_unavailable'))
def test_driver_fault_or_model_exclusions_cancel_inert_intent(reason):
  owner, now, _old, _before, _future = active()
  owner.interrupt('acknowledged_trial_profile_withdrawn')
  assert owner.paused_intent is not None
  owner.interrupt(reason)
  assert owner.paused_intent is None
  rollback_ack(owner, now+100_000_000)
  assert owner.offer_state['phase'] == 'rollback' and owner.offer_state['accepted']


def test_identity_change_and_original_deadline_never_resume_intent():
  owner, now, _old, _before, _future = active()
  owner.interrupt('acknowledged_trial_profile_withdrawn')
  owner.interrupt('source_epoch_changed', context_changed=True)
  assert owner.paused_intent is None
  owner, now, old, _before, _future = active()
  owner.interrupt('acknowledged_trial_profile_withdrawn')
  rollback_ack(owner, old['deadline_ns']+100_000_000)
  assert owner.offer_state['phase'] == 'rollback' and owner.offer_state['accepted']
  assert owner.paused_intent is None


def test_stale_baseline_ack_does_not_resume_inert_intent():
  owner, now, _old, _before, _future = active()
  owner.interrupt('acknowledged_trial_profile_withdrawn')
  pending = owner.offer(now)
  assert pending is not None
  assert not owner.acknowledge_application(trial_id=pending.trial_id, target_sequence=pending.target_sequence,
    phase='rollback', target_sha256=pending.target_sha256, applied_full30=pending.target,
    control_ns=now+200_000_000, binding=pending.binding, now_ns=now+200_000_000, status='applied')
  assert owner.offer_state['phase'] == 'rollback' and owner.paused_intent is not None


"""Input rejection and inert study recovery; no mocked native authority."""
from dataclasses import replace
import pytest
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import snapshots, SyntheticAdapter, BASE
from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
from openpilot.starpilot.flm.live_tracking_response import response_witness_value


@pytest.mark.parametrize('error', (.1, .201))
def test_real_original_witness_expiry_recovers_complete_scores_only_after_new_baseline_ack(error):
  owner, _now, old, before, future = active(error=error)
  witness = response_witness_value(owner.response_witness)
  now = witness.valid_until_ns + 1
  assert owner._witness_expired(now)
  assert owner.offer(now) is None
  assert owner.paused_intent is not None and owner.offer_state['phase'] == 'rollback'
  assert owner.before == () and owner.future == [] and owner.settled_ns is None
  for reason in ('learning_evidence_delayed', 'stale_control', 'stale_applied_output', 'current_evidence_delayed',
                 'fresh_physical_residual_window_required'):
    owner.interrupt(reason)
    assert owner.paused_intent is not None
  rollback_ack(owner, now+100_000_000)
  resumed = owner.offer_state
  assert resumed['phase'] == 'trial' and resumed['trial_id'] != old['trial_id']
  assert resumed['target_sequence'] > old['target_sequence'] and not resumed['published']
  assert owner.before == before and tuple(owner.future) == future and owner.settled_ns is None
  for name in ('target', 'baseline', 'started_ns', 'deadline_ns', 'alpha', 'required_episodes', 'look_quotas'):
    assert resumed[name] == old[name]


@pytest.mark.parametrize('field,value', [('independent_poses', 0), ('binding', None), ('last_pose_ns', -1)])
def test_malformed_original_witness_never_classified_as_expiry(field, value):
  owner, _now, _old, _before, _future = active()
  original = response_witness_value(owner.response_witness)
  owner.response_witness = replace(original, **{field: value})
  assert not owner._witness_expired(original.valid_until_ns+1)
  assert owner.offer(original.valid_until_ns+1) is None
  assert owner.paused_intent is None


class Interruptions:
  def __init__(self):
    self.rows = []

  def interrupt(self, reason, *, context_changed=False):
    self.rows.append((reason, context_changed))


def collector():
  result = FullrateCollector(SyntheticAdapter())
  events = Interruptions()
  result.live_tracking = events
  return result, events


@pytest.mark.parametrize('field,value,reason', [
  ('fault', False, 'stale_control'), ('fault', True, 'ineligible_control'),
  ('manual', True, 'ineligible_control'), ('override', True, 'ineligible_control'),
  ('valid', False, 'ineligible_control'), ('identity', 'bad', 'invalid_control'),
  ('state_ns', BASE+1, 'invalid_control'), ('requested', float('nan'), 'invalid_control'),
])
def test_old_control_fault_and_structural_exclusions_take_precedence(field, value, reason):
  c, events = collector()
  control, _output, _pose = snapshots(BASE)
  assert not c.control(replace(control, **{field: value}), BASE+150_000_000)
  assert c.reason == reason and events.rows[-1][0] == reason
  assert not c.controls and not c.ready


@pytest.mark.parametrize('field,value,reason', [
  ('fault', False, 'stale_applied_output'), ('fault', True, 'ineligible_applied_output'),
  ('override', True, 'ineligible_applied_output'), ('active', False, 'ineligible_applied_output'),
  ('applied', float('nan'), 'invalid_applied_output'), ('domain', 'wrong', 'consumed_source_mismatch'),
])
def test_old_output_fault_and_structural_exclusions_take_precedence(field, value, reason):
  c, events = collector()
  _control, output, _pose = snapshots(BASE)
  assert not c.output(replace(output, **{field: value}), BASE+150_000_000)
  assert c.reason == reason and events.rows[-1][0] == reason
  assert not c.applied and not c.ready


@pytest.mark.parametrize('field,value', [('independent_poses', 0), ('binding', None), ('last_pose_ns', -1)])
def test_malformed_witness_after_a_pause_cancels_the_inert_study(field, value):
  owner, _now, _old, _before, _future = active()
  original = response_witness_value(owner.response_witness)
  owner.offer(original.valid_until_ns+1)
  assert owner.paused_intent is not None
  owner.response_witness = replace(original, **{field: value})
  assert owner.offer(original.valid_until_ns+2) is None
  assert owner.paused_intent is None


def test_explicit_native_refusal_is_not_classified_as_input_delay():
  c, events = collector()
  control, _output, _pose = snapshots(BASE)
  control = replace(control, native_required=True, native_allowed=False, native_identity='4'*64,
    native_observed_ns=BASE-50_000_000, native_expires_ns=BASE+150_000_000)
  assert not c.control(control, BASE+150_000_000)
  assert c.reason == 'original_native_admission_unavailable'
  assert events.rows[-1][0] == c.reason and not c.ready


"""Real current-authority parser, declared original CC and output delivery seam."""
from types import SimpleNamespace

import pytest
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.tests.test_native_health import toyota_cp, typed_state
from openpilot.starpilot.flm.runtime_adapter_registry import current_authority


# Original timestamps retained from the full-cycle native-join refusal. The
# current wire/CP are declared unit inputs, not reconstruction of a lost wire.
OBSERVED, CONTROL, APPLY, EXPIRES, RECEIVED = 8549444345139, 8549483378931, 8549484378931, 8549644345139, 8549490482223


def original_control(identity):
  return SimpleNamespace(native_required=True, native_allowed=True, native_identity=identity,
    native_observed_ns=OBSERVED, control_ns=CONTROL, native_expires_ns=EXPIRES)


class Delivery:
  def __init__(self, control, storage):
    self.controls = {} if storage == 'recent' else {control.control_ns: control}
    self.recent_controls = {control.control_ns: control} if storage == 'recent' else {}
    self._learning_current_native_allowed = False  # Real reset/empty-ready condition.
    self.calls = []

  def output(self, value, now_ns):
    self.calls.append((value, now_ns, self._learning_current_native_allowed))
    return self._learning_current_native_allowed


@pytest.mark.parametrize('storage', ('pending', 'recent'))
@pytest.mark.parametrize('fault', ('none', 'current_denied', 'current_stale', 'record_denied', 'record_stale', 'identity'))
def test_each_output_rechecks_its_actual_consumed_control_against_real_current_native(storage, fault):
  cp = toyota_cp()
  healthy = typed_state(cp, RECEIVED-1_000_000)
  allowed, _native, identity = current_authority(healthy, cp, now_ns=RECEIVED, required=True)
  assert allowed
  control = original_control(identity)
  if fault == 'record_denied':
    control.native_allowed = False
  elif fault == 'record_stale':
    control.native_expires_ns = CONTROL-1
  elif fault == 'identity':
    control.native_identity = '0'*64
  sm = typed_state(cp, RECEIVED-1_000_000, denied=True) if fault == 'current_denied' else healthy
  collector = Delivery(control, storage)
  output = SimpleNamespace(control_ns=control.control_ns, apply_ns=APPLY)
  now = RECEIVED+300_000_000 if fault == 'current_stale' else RECEIVED
  # Old direct delivery inherits the false verdict even with a healthy actual CC.
  assert collector.output(output, now) is False
  result = steerlearnerd._collect_applied_output(collector, output, sm, cp, now)
  assert result is (fault == 'none')
  assert collector.calls[-1] == (output, now, fault == 'none')
  assert control.control_ns == output.control_ns  # No timestamp is rewritten.


def test_missing_consumed_control_cannot_inherit_another_frames_healthy_native_verdict():
  cp = toyota_cp()
  sm = typed_state(cp, RECEIVED-1_000_000)
  allowed, _native, identity = current_authority(sm, cp, now_ns=RECEIVED, required=True)
  assert allowed
  control = original_control(identity)
  collector = Delivery(control, 'recent')
  collector._learning_current_native_allowed = True
  output = SimpleNamespace(control_ns=control.control_ns+1)
  assert steerlearnerd._collect_applied_output(collector, output, sm, cp, RECEIVED) is False
  assert collector.calls[-1] == (output, RECEIVED, False)
