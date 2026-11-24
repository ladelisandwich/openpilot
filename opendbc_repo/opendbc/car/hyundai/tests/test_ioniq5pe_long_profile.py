import unittest

from opendbc.car.hyundai.ioniq5pe_long_profile import candidate, qualified, required
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.ev9_longitudinal import copy_cp


class TestPELongProfile(unittest.TestCase):
  def test_explicit_developer_choice_serializes_separate_cp_without_mutating_stock(self):
    stock = params()
    before = stock.to_dict()
    cp = candidate(stock, enabled=True, is_release=False)
    self.assertIsNotNone(cp)
    self.assertTrue(qualified(cp))
    self.assertTrue(required(cp))
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 0x5495)
    self.assertAlmostEqual(cp.deprecated.startAccel, 1.)
    self.assertAlmostEqual(cp.deprecated.vEgoStarting, .4)
    self.assertAlmostEqual(cp.longitudinalActuatorDelay, .35)
    self.assertAlmostEqual(cp.deprecated.stoppingDecelRate, .4)
    self.assertEqual(stock.to_dict(), before)
    self.assertTrue(qualified(copy_cp(cp)))

  def test_default_off_release_and_sibling_do_not_prepare_cp(self):
    for enabled, release in ((False, False), (False, True), (True, True)):
      stock = params()
      before = stock.to_dict()
      self.assertIsNone(candidate(stock, enabled=enabled, is_release=release))
      self.assertEqual(stock.to_dict(), before)
    self.assertIsNone(candidate(params(candidate=CAR.KIA_EV9), enabled=True, is_release=False))

  def test_exact_alternative_experience_survives_serialization(self):
    for experience in (0, 1, 32, 33, 64):
      with self.subTest(experience=experience):
        cp = candidate(params(), enabled=True, is_release=False)
        cp.alternativeExperience = experience
        self.assertTrue(required(cp))
        self.assertEqual(qualified(cp), experience in (0, 32))
        self.assertEqual(qualified(copy_cp(cp)), experience in (0, 32))

  def test_malformed_active_pe_cannot_bypass_required_owner(self):
    for experience in (0, 32):
      for field, value in (('passive', True), ('dashcamOnly', True), ('pcmCruise', True),
                           ('startAccel', .2), ('vEgoStarting', .5)):
        with self.subTest(experience=experience, field=field):
          cp = candidate(params(), enabled=True, is_release=False)
          cp.alternativeExperience = experience
          self.assertTrue(qualified(cp))
          setattr(cp.deprecated if field in ('startAccel', 'vEgoStarting', 'vEgoStopping', 'stoppingDecelRate') else cp, field, value)
          self.assertTrue(required(cp))
          self.assertFalse(qualified(cp), field)
      with self.subTest(experience=experience, field='safetyParam'):
        cp = candidate(params(), enabled=True, is_release=False)
        cp.alternativeExperience = experience
        self.assertTrue(qualified(cp))
        cp.safetyConfigs[0].safetyParam = 0x5C95
        self.assertFalse(qualified(cp))

  def test_unobserved_topologies_and_missing_physical_identity_sources_stay_stock(self):
    for topology in ('lka', 'lfa'):
      self.assertIsNone(candidate(params(topology=topology), enabled=True, is_release=False))
    for missing in (0x35, 0x110, 0x362, 0x1CF, 0x1A0):
      self.assertIsNone(candidate(params(missing=missing), enabled=True, is_release=False))

  def test_nonfinite_calibration_cannot_enter_the_sealed_long_owner(self):
    for experience in (0, 32):
      for field in ('startAccel', 'vEgoStarting', 'vEgoStopping', 'stoppingDecelRate', 'longitudinalActuatorDelay'):
        for value in (float('nan'), float('inf'), float('-inf')):
          with self.subTest(experience=experience, field=field, value=value):
            cp = candidate(params(), enabled=True, is_release=False)
            cp.alternativeExperience = experience
            self.assertTrue(qualified(cp))
            setattr(cp.deprecated if field in ('startAccel', 'vEgoStarting', 'vEgoStopping', 'stoppingDecelRate') else cp, field, value)
            self.assertTrue(required(cp))
            self.assertFalse(qualified(cp))

  def test_recorded_2027_inherits_pe_capability_and_release_stays_stock(self):
    from opendbc.car.hyundai.tests.test_ioniq5pe_2027 import params as recorded_params
    for release in (False, True):
      stock = recorded_params(alpha=True, release=release)
      self.assertEqual(stock.alphaLongitudinalAvailable, not release)
      self.assertFalse(stock.openpilotLongitudinalControl)
      cp = candidate(stock, enabled=True, is_release=release)
      self.assertEqual(cp is not None, not release)
      if cp is not None:
        self.assertTrue(qualified(cp))
      if release:
        self.assertIsNone(candidate(stock, enabled=True, is_release=False))
