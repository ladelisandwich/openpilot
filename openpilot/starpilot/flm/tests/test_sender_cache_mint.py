from dataclasses import fields
import pytest
from openpilot.starpilot.flm import effect_trace
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture, NOW
from openpilot.starpilot.flm.torque_adapter import Admission


def original():
  owner, p, v, context = fixture()
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  return owner.trace


def test_virtual_validation_subclass_cannot_mint_sender_cache_proof():
  class Forged(effect_trace.FullEffectTrace):
    def validate(self):
      return self

  real = original()
  forged = Forged(**{field.name: getattr(real, field.name) for field in fields(real)})
  with pytest.raises(ValueError):
    effect_trace.retain_sender_verification(forged)
  assert id(forged) not in effect_trace._SENDER_PROOFS


@pytest.mark.parametrize('kind', ['subclass', 'short', 'nonhex'])
def test_noncanonical_digest_cannot_mint_sender_cache_proof(kind):
  class MutableString(str):
    pass

  value = original()
  digest = MutableString(value.applied_full_profile_sha256) if kind == 'subclass' else 'a' * 63 if kind == 'short' else 'z' * 64
  object.__setattr__(value, 'applied_full_profile_sha256', digest)
  with pytest.raises(ValueError):
    effect_trace.retain_sender_verification(value)
