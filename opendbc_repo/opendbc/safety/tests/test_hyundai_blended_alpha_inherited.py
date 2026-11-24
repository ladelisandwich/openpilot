"""An ordinary alpha fixture selects its profile after explicit experience reset."""
import unittest

from opendbc.safety.tests import test_hyundai_blended_alpha_closure as closure


class TestOrdinaryAlphaInheritedExperience(unittest.TestCase):
  def test_prior_aol_policy_cannot_change_ordinary_primary_and_relay_contract(self):
    for experience in (32, 0, 1):
      with self.subTest(inherited_experience=experience):
        case = closure.TestHyundaiBlendedAlphaClosure(methodName='test_mirror_relay_and_profile_reinit_fail_closed')
        case.setUp()
        try:
          case.safety.set_alternative_experience(experience)
          case.test_mirror_relay_and_profile_reinit_fail_closed()
        finally:
          case.doCleanups()
