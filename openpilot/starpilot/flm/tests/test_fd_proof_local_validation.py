"""Actual controller/native numerical checks; original seals remain unit-mocked.

Root's authentic owner/component run is required for source admission and gain.
"""
from types import MethodType
from dataclasses import replace
import pytest

from openpilot.starpilot.flm.tests.test_closed_fd_reference_reuse import closed_engine, certificate


def actual_key_engine(monkeypatch, domain='torque'):
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  engine, evidence = closed_engine(monkeypatch, domain)
  # Use actual full-body causal keys, not the imported fixture's constant key.
  monkeypatch.setattr(engine, '_causal_input_key', MethodType(EventControllerReplay._causal_input_key, engine))
  from openpilot.starpilot.flm import feedback_replay
  monkeypatch.setattr(feedback_replay, 'require_feedback_engine', lambda owner: owner.__dict__['verified_feedback_tape'])
  return engine, evidence


@pytest.mark.parametrize('domain', ['torque', 'starpilot', 'pid'])
def test_actual_strict_reference_retains_two_full_checks_and_removes_only_middle_require(monkeypatch, domain):
  from openpilot.starpilot.flm import feedback_replay, candidate_feedback
  engine, evidence = actual_key_engine(monkeypatch, domain)
  cert, result = certificate(engine)
  calls = []
  monkeypatch.setattr(feedback_replay, 'require_feedback_engine', lambda owner: (calls.append(owner), evidence)[1])
  key = engine._causal_input_key
  keys = []
  monkeypatch.setattr(engine, '_causal_input_key', lambda epsilon: (keys.append(epsilon), key(epsilon))[1])
  actual = engine._strict_fd_reference(engine.reference, cert)
  assert actual == result[1] and actual is not result[1]
  assert keys == [.01, .01]
  assert calls == [engine, engine]
  proof, expected = cert[1][2]
  calls.clear()
  assert candidate_feedback.strict_reference_proof_key(proof, engine, engine.reference, len(result[0])) == expected
  assert calls == [engine]  # Public API still owns its fresh complete require.


@pytest.mark.parametrize('change', ['factory_cp', 'native_cp', 'records', 'evidence', 'policy', 'native_events'])
def test_full_post_key_still_declines_mutation_during_proof_projection(monkeypatch, change):
  from openpilot.starpilot.flm import candidate_feedback
  engine, evidence = actual_key_engine(monkeypatch)
  cert, _ = certificate(engine)
  original = candidate_feedback._strict_reference_fields

  def mutated(*args):
    value = original(*args)
    if change in ('factory_cp', 'native_cp'):
      owner = engine.factory if change == 'factory_cp' else engine.emissions
      builder = owner.cp.as_builder()
      builder.steerRatio += .125
      owner.cp = builder.as_reader()
    elif change == 'records':
      engine.records[0] = replace(engine.records[0], desired_physical_accel=1.25)
    elif change == 'evidence':
      engine.verified_feedback_tape = replace(evidence, binding='f' * 64)
    elif change == 'policy':
      object.__setattr__(evidence, 'candidate_policy', candidate_feedback.LEGACY_POLICY)
    else:
      first = next(iter(engine.emissions.events))
      engine.emissions.events[first] = replace(engine.emissions.events[first], state_ns=1)
    return value

  monkeypatch.setattr(candidate_feedback, '_strict_reference_fields', mutated)
  assert engine._strict_fd_reference(engine.reference, cert) is None


def test_private_projection_is_not_a_wire_or_unsealed_proof_minter(monkeypatch):
  from openpilot.starpilot.flm import candidate_feedback
  from openpilot.starpilot.flm.feedback_replay import FeedbackReplayEvidence
  engine, evidence = actual_key_engine(monkeypatch)
  cert, result = certificate(engine)
  proof, expected = cert[1][2]
  fn = candidate_feedback._strict_reference_fields
  assert fn(proof, evidence, engine.reference, len(result[0])) == expected
  assert fn(replace(proof, _seal=object()), evidence, engine.reference, len(result[0])) is None
  assert fn(proof, replace(evidence, _seal=object()), engine.reference, len(result[0])) is None
  wire = FeedbackReplayEvidence(evidence.binding, evidence.original, evidence.windows,
    evidence.engine_input_digest, object(), evidence.candidate_policy)
  assert fn(proof, wire, engine.reference, len(result[0])) is None
