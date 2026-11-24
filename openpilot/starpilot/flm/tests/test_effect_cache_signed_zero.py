from copy import deepcopy
import pytest
from openpilot.starpilot.flm.effect_admission_cache import validated_envelope
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture


def test_changed_signed_zero_cannot_hit_prior_verified_content_hash():
  _, p, _, _ = fixture()
  validated_envelope(p.effect_envelope, p)
  changed = deepcopy(p.effect_envelope)
  prior = list(changed.prior_coordinates)
  prior[0] = (prior[0][0], -0.0)
  object.__setattr__(changed, 'prior_coordinates', tuple(prior))
  # Keep old +0.0hash deliberately: strict original hash must fail.
  with pytest.raises(ValueError):
    validated_envelope(changed, p)
