from collections.abc import Callable
from typing import cast, NoReturn
"""Exact formatting vectors only; no vehicle/native/profile acceptance."""
from dataclasses import asdict, fields

import pytest

from openpilot.starpilot.flm import feedback_observation as feedback
from openpilot.starpilot.flm import physical_input_observation as physical


def test_hash_ascii_fullmatch_matches_original_all_byte_positions_and_unicode():
  def original(value):
    return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)
  for position in range(64):
    for char in (*map(chr, range(256)), '\u0100', '\uff10', '\U0001f642', '\ud800'):
      value = 'a' * position + char + 'f' * (63 - position)
      assert physical._hash(value) is original(value)
  for value in ('', 'a' * 63, 'a' * 65, None, b'a' * 64, 1, True, [], {}):
    assert physical._hash(value) is original(value)


def test_hash_exact_str_guard_rejects_subclass_without_callbacks():
  class Text(str):
    def __len__(self):
      raise AssertionError('unsupported subclass callback')
  assert physical._hash(Text('a' * 64)) is False


def feedback_record(kind, **changes):
  defaults: dict[object, object] = {str: 'a' * 64, int: 1, float: -0., bool: True}
  values = {field.name: defaults[field.type] for field in fields(kind)}
  values.update(changes)
  return kind(**values)


@pytest.mark.parametrize('value', [None, True, 1, 2**120, 1., -0., float('inf'), float('nan'),
                                  'plain', '\ud800', b'\x00\xff'])
def test_feedback_exact_record_all_literal_types_and_hashes_match_asdict(value):
  consumption = feedback_record(feedback.CoConsumption, reported_torque=value)
  publication = feedback_record(feedback.CoPublication, reported_torque=value)
  for record in (consumption, publication, feedback.FeedbackJoin(consumption, publication)):
    expected = asdict(record)
    actual = feedback._feedback_asdict(record)
    # Whole typed digest compares NaN/error and signed-zero semantics honestly.
    outcomes = []
    for body in (expected, actual):
      try:
        outcomes.append(('result', physical.body_digest(body)))
      except Exception as error:
        outcomes.append(('error', type(error), str(error)))
    assert outcomes[0] == outcomes[1]


def test_feedback_unknown_leaf_whole_fallback_keeps_deepcopy_mutation_order():
  outcomes = []
  for converter in (asdict, feedback._feedback_asdict):
    consumption = feedback_record(feedback.CoConsumption)
    publication = feedback_record(feedback.CoPublication)
    log = []
    class Callback:
      def __deepcopy__(self, memo, log=log, publication=publication):
        log.append('consumption callback')
        object.__setattr__(publication, 'sequence', 99)
        return 'copied'
    object.__setattr__(consumption, 'requested_torque', Callback())
    result = converter(feedback.FeedbackJoin(consumption, publication))
    outcomes.append((result, tuple(log)))
  assert outcomes[0] == outcomes[1]
  assert outcomes[0][0]['publication']['sequence'] == 99


def test_feedback_subclass_falls_back_to_original_asdict(monkeypatch):
  class Publication(feedback.CoPublication):
    pass
  value = Publication(**asdict(feedback_record(feedback.CoPublication)))
  observed = []
  original = feedback.asdict
  monkeypatch.setattr(feedback, 'asdict', lambda item: (observed.append(item), original(item))[1])
  assert feedback._feedback_asdict(value) == original(value)
  assert observed == [value]


def test_feedback_exact_record_avoids_deepcopy_and_still_reads_every_field(monkeypatch):
  record = feedback_record(feedback.CoConsumption)
  expected = asdict(record)
  monkeypatch.setattr(feedback, 'asdict', lambda *args: cast(Callable[[str], NoReturn], pytest.fail)('closed immutable primitive deepcopy'))
  assert feedback._feedback_asdict(record) == expected
  object.__setattr__(record, 'requested_torque', .5)
  assert feedback._feedback_asdict(record)['requested_torque'] == .5


def test_feedback_early_unknown_deepcopy_restores_missing_later_field():
  outcomes = []
  for converter in (asdict, feedback._feedback_asdict):
    record = feedback_record(feedback.CoConsumption)
    log = []
    class Restore:
      def __deepcopy__(self, memo, log=log, record=record):
        log.append('early deepcopy')
        object.__setattr__(record, 'cp_sha256', 'b' * 64)
        return 'copied'
    object.__setattr__(record, 'session' if hasattr(record, 'session') else 'version', Restore())
    object.__delattr__(record, 'cp_sha256')
    outcomes.append((converter(record), tuple(log)))
  assert outcomes[0] == outcomes[1]
  assert outcomes[0][0]['cp_sha256'] == 'b' * 64


def test_feedback_unknown_consumption_can_restore_publication_before_original_read():
  outcomes = []
  for converter in (asdict, feedback._feedback_asdict):
    consumption = feedback_record(feedback.CoConsumption)
    publication = feedback_record(feedback.CoPublication)
    class Restore:
      def __deepcopy__(self, memo, publication=publication):
        object.__setattr__(publication, 'cp_sha256', 'b' * 64)
        return 'copied'
    object.__setattr__(consumption, 'version', Restore())
    object.__delattr__(publication, 'cp_sha256')
    outcomes.append(converter(feedback.FeedbackJoin(consumption, publication)))
  assert outcomes[0] == outcomes[1]


def test_feedback_descriptor_unknown_leaf_runs_once_in_original_order(monkeypatch):
  outcomes = []
  for converter in (asdict, feedback._feedback_asdict):
    record = feedback_record(feedback.CoConsumption)
    log = []
    class Copy:
      def __deepcopy__(self, memo, log=log):
        log.append('deepcopy')
        return 'copied'
    def getter(self, log=log, Copy=Copy):
      log.append('getter')
      return Copy()
    with monkeypatch.context() as scope:
      scope.setattr(feedback.CoConsumption, 'version', property(getter), raising=False)
      outcomes.append((converter(record), tuple(log)))
  assert outcomes[0] == outcomes[1]
  assert outcomes[0][1] == ('getter', 'deepcopy')


def test_feedback_colliding_extra_key_falls_back_before_any_key_callback():
  outcomes = []
  for converter in (asdict, feedback._feedback_asdict):
    record = feedback_record(feedback.CoConsumption)
    owned = object.__getattribute__(record, '__dict__')
    log = []
    class Collision:
      def __hash__(self):
        return hash('cp_sha256')
      def __eq__(self, other, log=log):
        log.append('unexpected key comparison')
        return False
    key = Collision()
    class Restore:
      def __deepcopy__(self, memo, log=log, owned=owned, key=key, record=record):
        log.append('early deepcopy')
        owned.pop(key)
        object.__setattr__(record, 'cp_sha256', 'b' * 64)
        return 'copied'
    object.__setattr__(record, 'version', Restore())
    object.__delattr__(record, 'cp_sha256')
    owned[key] = 'extra ignored by original asdict'
    log.clear()
    outcomes.append((converter(record), tuple(log)))
  assert outcomes[0] == outcomes[1]
  assert outcomes[0][1] == ('early deepcopy',)


def original_large_sequence(value, depth=0):
  if type(value) in (tuple, list):
    return len(value) > 256
  if type(value) is dict and depth < 3:
    return any(original_large_sequence(child, depth + 1) for child in value.values())
  return False


@pytest.mark.parametrize('length', (0, 1, 255, 256, 257, 258))
@pytest.mark.parametrize('depth', range(6))
def test_digest_size_selection_keeps_original_boundary_and_depth(length, depth):
  for container in (list, tuple):
    value = container(range(length))
    for i in range(depth):
      value = {str(i): value}
    for initial_depth in range(5):
      assert physical._large_sequence(value, initial_depth) is original_large_sequence(value, initial_depth)
    assert physical._body_digest_python(value) == physical._body_digest_original(value)


def test_digest_selection_preserves_custom_type_callbacks_and_iteration_failures():
  calls = []
  class Meta(type):
    def __eq__(cls, other):
      calls.append(other.__name__)
      return False
  class Custom(metaclass=Meta):
    pass
  for value in ({'first': Custom(), 'nested': {'second': Custom()}},
                {'first': list(range(257)), 'unused': Custom()}):
    expected = original_large_sequence(value)
    original_calls = calls[:]
    calls.clear()
    assert physical._large_sequence(value) is expected
    assert calls == original_calls
    calls.clear()
  def mutating_parent():
    parent: dict[str, object] = {}
    class MutatingMeta(type):
      def __eq__(cls, other):
        parent['new'] = 1
        return False
    class Mutating(metaclass=MutatingMeta):
      pass
    parent['first'] = Mutating()
    return parent
  for select in (original_large_sequence, physical._large_sequence):
    parent = mutating_parent()
    with pytest.raises(RuntimeError, match='dictionary changed size during iteration'):
      select(parent)


def test_digest_ordinary_nested_bodies_keep_original_bits_types_and_refusals():
  import random
  randomizer = random.Random(42)
  def body(depth=0):
    if depth == 5 or randomizer.random() < .5:
      return randomizer.choice((None, True, False, randomizer.randrange(-10**15, 10**15),
                                randomizer.uniform(-1, 1), -0., 'x\\ü🙂', '', b'\x00\xff'))
    if randomizer.randrange(3) == 0:
      return {str(k): body(depth + 1) for k in range(randomizer.randrange(6))}
    values = [body(depth + 1) for _ in range(randomizer.randrange(8))]
    return values if randomizer.randrange(2) else tuple(values)
  for _ in range(4000):
    value = body()
    assert physical._large_sequence(value) is original_large_sequence(value)
    assert physical._body_digest_python(value) == physical._body_digest_original(value)
  for value in (float('inf'), float('nan'), {1: 0}, {'x': object()}, {'x': bytearray(b'x')}):
    with pytest.raises(ValueError) as original:
      physical._body_digest_original(value)
    with pytest.raises(type(original.value), match=str(original.value)):
      physical._body_digest_python(value)
