from dataclasses import replace
from unittest import mock
import pytest
from openpilot.starpilot.flm import effect_trace
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture, NOW
from openpilot.starpilot.flm.torque_adapter import Admission


def trace():
  owner, p, v, context = fixture()
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  return owner.trace


def test_same_sender_trace_skips_only_duplicate_immutable_validation():
  value = trace()
  with mock.patch('openpilot.starpilot.flm.effect_admission_cache.validated_envelope', side_effect=AssertionError('duplicate immutable validation')):
    assert value.validate() is value
  # A new decoded/receiver object is NOT an identity-cache hit, even if equal.
  decoded = replace(value)
  with mock.patch(
    'openpilot.starpilot.flm.effect_admission_cache.validated_envelope',
    wraps=__import__('openpilot.starpilot.flm.effect_admission_cache', fromlist=['validated_envelope']).validated_envelope,
  ) as verify:
    decoded.validate()
    assert verify.call_count == 1


@pytest.mark.parametrize('kind', ['signed_zero', 'mutable_rows', 'hash', 'envelope'])
def test_changed_trace_or_envelope_never_reuses_sender_proof(kind):
  value = trace()
  if kind == 'signed_zero':
    rows = list(value.applied)
    index = next(i for i, (name, _) in enumerate(rows) if name == 'ff_gain_right')
    rows[index] = (rows[index][0], -0.0)
    object.__setattr__(value, 'applied', tuple(rows))
  elif kind == 'mutable_rows':
    object.__setattr__(value, 'before', tuple(list(row) for row in value.before))
  elif kind == 'hash':
    object.__setattr__(value, 'applied_full_profile_sha256', '0' * 64)
  else:
    # Keep exact envelope object identity but replace a nested immutable field.
    object.__setattr__(value.envelope, 'full_profile_sha256', '0' * 64)
  with pytest.raises(ValueError):
    value.validate()


def test_sender_cache_has_fixed_capacity_and_cannot_retain_receiver_history():
  for _ in range(12):
    trace()
  assert len(effect_trace._SENDER_PROOFS) <= 4
  original = trace()
  for _ in range(5):
    trace()
  assert id(original) not in effect_trace._SENDER_PROOFS
  with mock.patch(
    'openpilot.starpilot.flm.effect_admission_cache.validated_envelope',
    wraps=__import__('openpilot.starpilot.flm.effect_admission_cache', fromlist=['validated_envelope']).validated_envelope,
  ) as verify:
    original.validate()
    assert verify.call_count == 1


def test_changed_legal_first_frame_clock_revalidates_instead_of_using_sender_proof():
  value = trace()
  assert value.previous_ns == 0
  # First-frame transition deliberately uses its fixed 10 ms step. Moving its
  # timestamp remains structurally valid, but must never reuse the old proof.
  object.__setattr__(value, 'control_ns', value.control_ns + 10_000_000)
  with mock.patch(
    'openpilot.starpilot.flm.effect_admission_cache.validated_envelope',
    wraps=__import__('openpilot.starpilot.flm.effect_admission_cache', fromlist=['validated_envelope']).validated_envelope,
  ) as verify:
    assert value.validate() is value
    assert verify.call_count == 1


def test_changed_clock_past_existing_transition_window_rejects_old_applied_map():
  owner, p, v, context = fixture()
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  owner.prepare(Admission(p, NOW + 10_000_000, True, False, True, v), active=True, current_context=context)
  value = owner.trace
  assert value.previous_ns == NOW
  assert any(coordinate != 0 for _, coordinate in value.applied)
  # The existing step_full contract withdraws all coordinates after 50 ms;
  # retaining the old nonzero map must fail exact transition validation.
  object.__setattr__(value, 'control_ns', value.previous_ns + 50_000_001)
  with pytest.raises(ValueError, match='effect transition or applied hash changed'):
    value.validate()
