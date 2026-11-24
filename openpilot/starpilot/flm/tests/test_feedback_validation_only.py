from collections.abc import Callable
from typing import cast, NoReturn
from dataclasses import fields
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm import feedback_observation as f
from openpilot.starpilot.flm import feedback_replay as replay


def record(kind, **overrides):
  defaults: dict[object, object] = {int: 1, float: 0.25, bool: True, str: 'a' * 64}
  values = {field.name: defaults[field.type] for field in fields(kind)}
  values.update(overrides)
  return kind(**values)


def tape(*, session='a' * 64):
  rows = tuple(f.FeedbackJoin(record(f.CoConsumption, control_ns=stamp), record(f.CoPublication, session=session))
               for stamp in (100, 200))
  binding = 'a' * 64
  parent = 'b' * 64
  digest = f._tape_digest(rows, (), binding, parent)
  return f.VerifiedFeedbackTape(f.PROTOCOL, rows, binding, digest, (), parent, f._VERIFIED)


def test_closed_validation_uses_one_fresh_key_and_no_descendant(monkeypatch):
  value = tape()
  calls = []
  original = f._tape_digest_key
  monkeypatch.setattr(f, '_tape_digest_key', lambda *args: (calls.append(args), original(*args))[1])
  monkeypatch.setattr(f, '_mint_subset', lambda *args: cast(Callable[[str], NoReturn], pytest.fail)('discarded descendant mint'))
  assert f.validate_subset_verified(value, (100, 200), binding_digest=value.binding_digest) is None
  assert len(calls) == 1
  assert f.validate_subset_verified(value, (100, 200), binding_digest=value.binding_digest) is None
  assert len(calls) == 2
  object.__setattr__(value.rows[0].consumption, 'requested_torque', 0.75)
  with pytest.raises(ValueError, match='owned verified feedback tape/exact window required'):
    f.validate_subset_verified(value, (100, 200), binding_digest=value.binding_digest)
  assert len(calls) == 3


def test_public_mint_preserves_parent_rows_digest():
  value = tape()
  subset = f.subset_verified(value, (100,), binding_digest=value.binding_digest)
  assert subset.rows == value.rows[:1]
  assert subset.parent_digest == value.digest
  assert subset.digest == f._tape_digest(value.rows[:1], (), value.binding_digest, value.digest)
  assert subset._seal is f._VERIFIED


@pytest.mark.parametrize('controls', [(), [], (999,), (100, 999), (200, 100)])
def test_validation_preserves_public_guard_errors_and_causes(controls):
  outcomes = []
  for fn in (f.subset_verified, f.validate_subset_verified):
    value = tape()
    try:
      fn(value, controls, binding_digest=value.binding_digest)
    except Exception as error:
      outcomes.append((type(error), str(error), type(error.__cause__), str(error.__cause__)))
  assert len(outcomes) == 2
  assert outcomes[0] == outcomes[1]


@pytest.mark.parametrize('session', ['a' * 129, '\ud800'])
def test_unknown_key_domain_retains_mint(monkeypatch, session):
  value = tape(session=session)
  if f._tape_digest_key(value.rows, (), value.binding_digest, value.parent_digest) is not None:
    # A supported surrogate remains a safe exact immutable scalar.
    assert session == '\ud800'
    return
  calls = []
  original = f._mint_subset
  monkeypatch.setattr(f, '_mint_subset', lambda *args: (calls.append(args), original(*args))[1])
  f.validate_subset_verified(value, (100, 200), binding_digest=value.binding_digest)
  assert len(calls) == 1


def test_custom_control_equality_keeps_original_mint_after_callback(monkeypatch):
  value = tape()
  class Stamp:
    def __eq__(self, other):
      object.__setattr__(value.rows[0].consumption, 'requested_torque', 0.5)
      return other == 100
  calls = []
  original = f._mint_subset
  monkeypatch.setattr(f, '_mint_subset', lambda *args: (calls.append(args), original(*args))[1])
  f.validate_subset_verified(value, (Stamp(), 200), binding_digest=value.binding_digest)
  assert len(calls) == 1
  assert value.rows[0].consumption.requested_torque == 0.5


def test_actual_require_caller_keeps_evidence_and_activation_checks(monkeypatch):
  value = tape()
  evidence = replay.FeedbackReplayEvidence(value.binding_digest, value, (((100, 200), value, 150),), 'owned', replay._SEAL)
  engine = SimpleNamespace(verified_feedback_tape=evidence, records=(SimpleNamespace(control_ns=100), SimpleNamespace(control_ns=200)),
                           replay_windows=((100, 200),), activation_plan=SimpleNamespace(activation_for=lambda *args: (150,)))
  def inputs(owner, *, candidate_policy):
    assert owner is engine
    assert candidate_policy == replay.LEGACY_POLICY
    return 'owned'
  monkeypatch.setattr(replay, '_inputs', inputs)
  monkeypatch.setattr(f, '_mint_subset', lambda *args: cast(Callable[[str], NoReturn], pytest.fail)('require discarded descendant mint'))
  assert replay.require_feedback_engine(engine) is evidence
  assert replay.owned_feedback_key(engine)[-1] == (((100, 200), value.digest, 150),)
  engine.activation_plan.activation_for = lambda *args: (151,)
  with pytest.raises(ValueError, match='feedback activation changed'):
    replay.require_feedback_engine(engine)


def old_hex(value, size=64):
  return type(value) is str and len(value) == size and all(c in '0123456789abcdef' for c in value)


@pytest.mark.parametrize('size', [-1, 0, 1, 64, True, None, 1.0, '1'])
def test_hex_exact_equivalence_full_byte_alphabet_and_sizes(size):
  for value in ('', '0' * 64, 'f' * 64, 'a' * 63 + 'F', *[chr(index) for index in range(256)],
                '\u0100', '\uff10', '\U0001f642', '\ud800', 'a\n', 'a\x00'):
    assert f._hex(value, size) == old_hex(value, size)


def test_hex_str_subclass_and_nonstring_short_circuit():
  class Text(str):
    def __len__(self):
      raise AssertionError('subclass must short circuit')
  for value in (Text('a'), None, b'a', 1, True, [], {}):
    assert f._hex(value, 1) is False
    assert old_hex(value, 1) is False


def test_hex_custom_size_comparison_order_and_errors():
  class Size:
    def __init__(self, result=False, raises=False):
      self.result, self.raises, self.calls = result, raises, []
    def __eq__(self, value):
      self.calls.append(value)
      if self.raises:
        raise ValueError('size callback')
      return self.result
  for text in ('a', 'z', ''):
    for result in (True, False):
      left, right = Size(result), Size(result)
      assert f._hex(text, left) == old_hex(text, right)
      assert left.calls == right.calls == [len(text)]
  for operation in (f._hex, old_hex):
    size = Size(raises=True)
    with pytest.raises(ValueError, match='size callback'):
      operation('a', size)
    assert size.calls == [1]
