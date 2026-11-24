"""Real Cereal trace retention shares immutable content, never per-frame clocks."""
from dataclasses import replace
import struct

import pytest

from openpilot.cereal import custom
from openpilot.starpilot.flm import learning_wire as wire
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256
from openpilot.starpilot.flm.effect_trace import FullEffectTrace
from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope


def trace(stamp=1_000_000_000, epoch='e' * 64):
  envelope = FullSurfaceEnvelope.initial(cp_sha256='c' * 64,
    controller_revision='selected-controller-v2:' + 'd' * 64, source_epoch=epoch,
    reset_generation='0', selected_baseline_revision='a' * 64,
    trial_names=('ff_gain_left',), persistent_source_epoch='9' * 64)
  rows = envelope.coordinates
  return FullEffectTrace(1, envelope, stamp, stamp - 10_000_000, True,
                         rows, rows, rows, envelope.full_profile_sha256).validate()


def decode(value):
  payload = custom.LearningControlEvidence.new_message(effectTrace=wire._write_effect_trace(value)).to_bytes()
  with custom.LearningControlEvidence.from_bytes(payload) as message:
    return wire._read_effect_trace(message.effectTrace)


def test_actual_cereal_retains_shared_content_and_distinct_original_clocks():
  retained = [decode(trace(1_000_000_000 + i * 10_000_000)) for i in range(100)]
  first, last = retained[0], retained[-1]
  assert first is not last
  assert first.envelope is last.envelope
  for field in ('before', 'target', 'applied'):
    assert getattr(first, field) is getattr(last, field)
    assert tuple(name for name, _ in getattr(first, field)) == NAMES
  assert [row.control_ns for row in retained] == [1_000_000_000 + i * 10_000_000 for i in range(100)]
  assert all(row.previous_ns == row.control_ns - 10_000_000 for row in retained)
  assert all(row.validate() == trace(row.control_ns) for row in retained)


def test_actual_cereal_source_identity_never_aliases_a_different_envelope():
  first, other = decode(trace()), decode(trace(epoch='f' * 64))
  assert first.envelope is not other.envelope
  assert first.envelope.source_epoch == 'e' * 64
  assert other.envelope.source_epoch == 'f' * 64
  assert first.applied is other.applied


def test_actual_cereal_changed_coordinate_is_preserved():
  base = trace()
  rows = tuple((name, .001 if name == 'ff_gain_left' else value) for name, value in base.before)
  # Withdrawing a nonzero prior effect must still produce the exact zero command.
  changed = replace(base, before=rows, qualified=False).validate()
  decoded = decode(changed)
  assert decoded == changed
  assert decoded.before is not decode(base).before
  assert dict(decoded.before)['ff_gain_left'] == .001


@pytest.mark.parametrize('change', ['version', 'names', 'bounds', 'nan', 'hash', 'context', 'manifest'])
def test_actual_cereal_invalid_trace_is_still_rejected_after_cache_warmup(change):
  base = trace()
  decode(base)
  data = wire._write_effect_trace(base)
  if change == 'version':
    data['version'] = 2
  elif change == 'names':
    data['before'][0]['name'] = 'foreign'
  elif change == 'bounds':
    data['before'].pop()
  elif change == 'nan':
    data['before'][0]['value'] = float('nan')
  elif change == 'hash':
    data['appliedFullProfileSha256'] = '0' * 64
  elif change == 'context':
    data['envelope']['contextKey'] = 'f' * 64
  else:
    data['envelope']['manifestSha256'] = '0' * 64
  payload = custom.LearningControlEvidence.new_message(effectTrace=data).to_bytes()
  with custom.LearningControlEvidence.from_bytes(payload) as message, pytest.raises(ValueError):
    wire._read_effect_trace(message.effectTrace)


def test_coordinate_cache_preserves_signed_zero_and_has_fixed_bound():
  wire._effect_coordinate_rows.cache_clear()
  positive = tuple((name, 0.) for name in NAMES)
  negative = tuple((name, -0.) for name in NAMES)
  first = wire._shared_effect_rows(positive)
  second = wire._shared_effect_rows(negative)
  assert first is not second
  assert struct.pack('!d', second[0][1]) == struct.pack('!d', -0.)
  for i in range(160):
    rows = tuple((name, float(i)) for name in NAMES)
    assert wire._shared_effect_rows(rows) == rows
  assert wire._effect_coordinate_rows.cache_info().currsize == 128
  assert wire._shared_effect_rows(positive) == positive
  assert wire._shared_effect_rows(positive) is not first


def test_actual_cereal_signed_zero_hash_is_not_normalized():
  base = trace()
  negative = tuple((name, -0.) for name in NAMES)
  # Even numerically equal zero maps have a different canonical hash.
  assert full_profile_sha256('a' * 64, negative) != base.applied_full_profile_sha256
  changed = replace(base, applied=negative, applied_full_profile_sha256=full_profile_sha256('a' * 64, negative)).validate()
  decoded = decode(changed)
  assert struct.pack('!d', decoded.applied[0][1]) == struct.pack('!d', -0.)
  assert decoded.applied_full_profile_sha256 == changed.applied_full_profile_sha256
