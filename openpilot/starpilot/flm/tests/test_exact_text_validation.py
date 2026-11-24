"""Equivalent bounded text predicates, including malformed wire text."""
from dataclasses import replace
from types import SimpleNamespace

from openpilot.starpilot.flm.live_trial_contract import _digest, _text
from openpilot.starpilot.flm.runtime_adapter_registry import SourceState
from openpilot.starpilot.flm.steerlearnerd import _source_config_known


class TextSubclass(str):
  pass


def old_text(value, maximum):
  return type(value) is str and 0 < len(value) <= maximum and value.isascii() and all(32 <= ord(c) < 127 for c in value)


def old_digest(value):
  return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def test_text_and_digest_match_exact_prior_predicates():
  values = [None, False, 0, b'a', [], {}, SimpleNamespace(), TextSubclass('a'), '', 'a' * 193]
  alphabet = [chr(i) for i in range(256)] + ['\u200b', '\u2028', '\ud800', '\uffff', '\U0001f642']
  for char in alphabet:
    values.extend((char, char * 64, 'a' * 31 + char + '0' * 32, 'a' * 63 + char, char + 'a' * 63))
  for value in values:
    assert _digest(value) is old_digest(value)
    for bound in (0, 1, 32, 64, 96, 192):
      assert _text(value, bound) is old_text(value, bound)


def test_source_binding_hex_remains_strict():
  source = SourceState(True, False, '0', 'a' * 64, 100, 500_000_100, 'b' * 64,
    calibration_generation='c' * 32, persistent_source_epoch='d' * 64)
  assert _source_config_known(source, 200)
  for field in ('cp_sha256', 'reset', 'calibration_generation', 'persistent_source_epoch'):
    original = getattr(source, field)
    for bad in ('g', 'A', ' ', '\n', '\x00', '\u00e9'):
      assert not _source_config_known(replace(source, **{field: original[:-1] + bad}), 200)
  assert not _source_config_known(source, 500_000_101)
