"""Real encoder and child-worker tests; no claim of route or physical fit."""
from dataclasses import replace

import pytest
from opendbc.car.toyota.values import CAR
from opendbc.car.steering_projection import inspect_projection
from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence
from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory, EncoderBranch
from openpilot.starpilot.flm.torque_eligibility import bounded_context_output, exact_unclipped_pair
from openpilot.starpilot.flm.tests.test_torque_emission_join import BASE, actual_public_health, cc, interface, settle, source
from openpilot.starpilot.flm.tests import test_actual_raw_worker as raw_worker


def clipped_context(tmp_path):
  ci = interface(CAR.TOYOTA_COROLLA_TSS2)
  settle(ci, cc(0.0))
  limits = ci.CC.params
  command = cc((limits.STEER_DELTA_UP + 1) / limits.STEER_MAX)
  for _ in range(5):
    stamp = BASE + ci.CC.frame * 10_000_000
    output, packets = ci.apply(command, stamp)
    witness = ci.CC.learning_projection
    if witness is not None and witness.unlimited_raw != witness.final_raw:
      break
  else:
    raise AssertionError('actual registered encoder did not clip')
  assert (witness.can_address, witness.can_data, witness.can_bus) in packets
  assert inspect_projection(ci, command, output, witness) is False
  control = source(ci, tmp_path, float(command.actuators.torque), stamp)
  health = actual_public_health(ci.CP, stamp)
  evidence = AppliedEvidence(stamp, stamp - 1, stamp, 'normalized_torque', float(output.torque),
                             True, True, False, False, True, health, witness, ci.CC.frame, stamp)
  return ci, control, evidence


def test_actual_clipped_emission_is_context_not_estimation(tmp_path):
  ci, control, output = clipped_context(tmp_path)
  assert bounded_context_output(output)
  assert not exact_unclipped_pair(control, output)
  history = EmissionHistory(ci.CP, identity=control.identity, baseline_revision=control.baseline_revision)
  assert history.record_carrier(output) and history.record(control, output)
  event = history.event(control.control_ns)
  history.verify(control, output)
  actual = EncoderBranch()
  actual.step(event, control.requested, actual=True)
  assert actual.previous == output.projection.final_raw
  assert event.limited and output.limited  # original observations stay immutable


def test_candidate_cannot_increase_clipping_in_context(tmp_path):
  ci, control, output = clipped_context(tmp_path)
  history = EmissionHistory(ci.CP, identity=control.identity, baseline_revision=control.baseline_revision)
  assert history.record_carrier(output) and history.record(control, output)
  event = history.event(control.control_ns)
  branch = EncoderBranch()
  with pytest.raises(ValueError, match='clipping regression'):
    branch.step(event, control.requested + 5 / output.projection.effective_max)


@pytest.mark.parametrize('field', ('fault', 'override'))
def test_clipped_context_never_relaxes_fault_or_override(tmp_path, field):
  ci, control, output = clipped_context(tmp_path)
  changed = replace(output, **{field: True})
  assert not bounded_context_output(changed)
  history = EmissionHistory(ci.CP)
  assert history.record_carrier(changed)
  assert not history.record(control, changed)


def test_actual_raw_child_accepts_exact_false_hint_without_rewriting_inputs(monkeypatch):
  original = raw_worker.ControlEvidence
  observed = []

  def asynchronous_hint(**kwargs):
    evidence = original(**kwargs)
    assert not dict(evidence.replay_inputs)['safety_limited']
    changed = replace(evidence, limited=True)
    assert changed.replay_inputs == evidence.replay_inputs
    observed.append(changed)
    return changed

  monkeypatch.setattr(raw_worker, 'ControlEvidence', asynchronous_hint)
  raw_worker.test_actual_raw_registered_worker_report_reaches_controls()
  assert len(observed) == 3200 and all(row.limited for row in observed)


def test_exact_unclipped_hint_keeps_original_source_native_deadlines(tmp_path):
  from types import SimpleNamespace
  from openpilot.starpilot.flm.tests.test_torque_emission_join import emitted

  ci = interface(CAR.TOYOTA_COROLLA_TSS2)
  settle(ci, cc(0.01))
  control, output = emitted(ci, cc(0.01), tmp_path)
  control = SimpleNamespace(**vars(control), limited=True, saturated=False, domain='normalized_torque',
                            stage_schema_revision='torque-selected-stage-snapshot-v1',
                            replay_inputs=(('curvature_limited', 0.0),), state_ns=output.state_ns,
                            source_receipt_ns=control.control_ns - 1)
  assert exact_unclipped_pair(control, output)
  assert control.limited and not output.limited
  old_source = SimpleNamespace(**{**vars(control), 'source_receipt_ns': control.control_ns - 500_000_001})
  assert not exact_unclipped_pair(old_source, output)
  assert not exact_unclipped_pair(control, replace(output, state_ns=control.state_ns + 30_000_001,
                                                  apply_ns=control.control_ns + 30_000_002))
  native = SimpleNamespace(**{**vars(control), 'native_required': True, 'native_allowed': True,
                              'native_identity': 'a' * 64, 'native_observed_ns': control.control_ns - 1,
                              'native_expires_ns': output.apply_ns})
  assert exact_unclipped_pair(native, output)
  assert not exact_unclipped_pair(native, replace(output, apply_ns=output.apply_ns + 1))
  assert not exact_unclipped_pair(SimpleNamespace(**{**vars(control), 'replay_inputs': (('curvature_limited', 1.0),)}), output)


def test_candidate_cannot_reverse_context_clipping_sign(tmp_path):
  ci, control, output = clipped_context(tmp_path)
  history = EmissionHistory(ci.CP)
  assert history.record_carrier(output) and history.record(control, output)
  with pytest.raises(ValueError, match='clipping regression'):
    EncoderBranch().step(history.event(control.control_ns), -0.1)


def test_candidate_uses_own_previous_state_for_context_demand(tmp_path):
  ci, control, output = clipped_context(tmp_path)
  history = EmissionHistory(ci.CP)
  assert history.record_carrier(output) and history.record(control, output)
  branch = EncoderBranch()
  branch.previous = output.projection.previous_torque - 20
  with pytest.raises(ValueError, match='clipping regression'):
    branch.step(history.event(control.control_ns), control.requested)


def test_clipping_guard_runs_in_unscored_warmup_context(tmp_path):
  ci, control, output = clipped_context(tmp_path)
  history = EmissionHistory(ci.CP)
  assert history.record_carrier(output) and history.record(control, output)
  history.verify(control, output)
  # The only emission precedes the scored window; it still advances encoder
  # state and must enforce candidate demand rather than disappearing as warmup.
  window = ((control.control_ns + 1, control.control_ns + 10_000_000),)
  with pytest.raises(ValueError, match='clipping regression'):
    history.rollout({control.control_ns: control.requested + 5 / output.projection.effective_max},
                    control.control_ns, control.control_ns, window)
