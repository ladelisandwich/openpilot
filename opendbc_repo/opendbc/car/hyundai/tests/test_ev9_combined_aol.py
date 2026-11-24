import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.hyundai.ev9_longitudinal import candidate, qualified as long_qualified, lateral_request_allowed
from opendbc.car.hyundai.ev9_long_aol import qualified, temporary_restriction
from opendbc.car.hyundai.canfd_angle_aol import qualified as permission_owner
from opendbc.car.hyundai.ev9_pending_startup import EV9PendingStartup
from opendbc.car.hyundai.tests.test_ev9_pending_startup import EV9StartupFixture
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.radar_interface import RadarInterface
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.intent import AOL_TOGGLE
from openpilot.starpilot.car.hyundai.aol import policy_for, native_accepts_cp


class TestEV9CombinedProfile(unittest.TestCase):
  def test_exact_profile_and_marker_preserve_stock_settings_and_long_calibration(self):
    stock = params(candidate=CAR.KIA_EV9)
    before = stock.to_dict()
    cp = candidate(stock, enabled=True, is_release=False)
    ordinary = cp.to_dict()
    self.assertTrue(qualified(cp))
    self.assertFalse(qualified(cp, marked_only=True))
    self.assertFalse(permission_owner(cp, marked_only=True))
    self.assertTrue(policy_for(cp).settings_supported)
    cp.alternativeExperience = 32
    self.assertTrue(qualified(cp, marked_only=True) and long_qualified(cp))
    self.assertTrue(permission_owner(cp, marked_only=True))
    self.assertTrue(native_accepts_cp(cp, int(cp.safetyConfigs[0].safetyModel.raw), 0x5c95))
    expected = dict(ordinary, alternativeExperience=32)
    self.assertEqual(cp.to_dict(), expected)
    self.assertEqual(stock.to_dict(), before)
    for field, value in (('carFingerprint', CAR.HYUNDAI_IONIQ_5_PE), ('alternativeExperience', 1),
                         ('pcmCruise', True), ('passive', True),
                         ('steerControlType', structs.CarParams.SteerControlType.torque)):
      bad = cp.as_reader().as_builder()
      setattr(bad, field, value)
      self.assertFalse(qualified(bad), field)
    for word in (0x5c91, 0x5495, 0x5c97, 0x5cb5):
      bad = cp.as_reader().as_builder()
      bad.safetyConfigs[0].safetyParam = word
      self.assertFalse(qualified(bad))

  def test_pedals_temporary_faults_and_nondrive_withhold_output_without_forgetting_intent(self):
    cp = candidate(params(candidate=CAR.KIA_EV9), enabled=True, is_release=False)
    cp.alternativeExperience = 32
    cs = structs.CarState(canValid=True, gearShifter='drive', vEgo=10., vEgoRaw=10.)
    cc = structs.CarControl(latActive=True)
    self.assertFalse(lateral_request_allowed(cc, cs, False))
    self.assertTrue(lateral_request_allowed(cc, cs, False, independent=qualified(cp, marked_only=True)))
    ordinary = structs.CarControl(enabled=True, latActive=True)
    for field in ('gasPressed', 'brakePressed'):
      setattr(cs, field, True)
      self.assertTrue(temporary_restriction(cp, cs), field)  # Intent retention is not output authority.
      self.assertTrue(lateral_request_allowed(cc, cs, False, independent=qualified(cp, marked_only=True)), field)
      self.assertFalse(lateral_request_allowed(ordinary, cs, False), field)
      self.assertFalse(lateral_request_allowed(structs.CarControl(), cs, False, independent=True), field)
      self.assertFalse(lateral_request_allowed(structs.CarControl(enabled=True), cs, False, independent=True), field)
      for refusal, value in (('canValid', False), ('canTimeout', True), ('steerFaultTemporary', True),
                             ('steerFaultPermanent', True), ('standstill', True), ('gearShifter', 'reverse')):
        before = getattr(cs, refusal)
        setattr(cs, refusal, value)
        self.assertFalse(lateral_request_allowed(cc, cs, False, independent=True), (field, refusal))
        setattr(cs, refusal, before)
      self.assertFalse(lateral_request_allowed(cc, cs, True, independent=True), field)
      setattr(cs, field, False)
    for field in ('steerFaultTemporary', 'standstill'):
      setattr(cs, field, True)
      self.assertTrue(temporary_restriction(cp, cs), field)
      self.assertFalse(lateral_request_allowed(cc, cs, False, independent=True), field)
      setattr(cs, field, False)
    cs.steerFaultPermanent = True
    self.assertFalse(temporary_restriction(cp, cs))
    self.assertFalse(lateral_request_allowed(cc, cs, False, independent=True))


class TestEV9CombinedPending(EV9StartupFixture):
  def test_actual_card_initialization_negotiates_only_marker_and_seals_saved_setting(self):
    for aol in (False, True):
      with self.subTest(aol=aol), OpenpilotPrefix(), patch.dict('os.environ', {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
        saved = Params()
        saved.put_bool('OpenpilotEnabledToggle', True, block=True)
        saved.put_bool('AlphaLongitudinalEnabled', True, block=True)
        saved.put_bool('AlwaysOnLateral', aol, block=True)
        saved.put_bool('IsReleaseBranch', False, block=True)
        saved.put('LKASButtonControl', AOL_TOGGLE, block=True)
        saved.put('MainCruiseButtonControl', 0, block=True)
        keys = ('AlwaysOnLateral', 'AlphaLongitudinalEnabled', 'LKASButtonControl', 'MainCruiseButtonControl')
        before = {key: saved.get(key) for key in keys}
        owner, ci, _ = self.instance(seal=False)
        cp_before = ci.CP.to_dict()
        card = Car(CI=ci, RI=RadarInterface(ci.CP), startup_owner=owner)
        self.assertIsInstance(card.vehicle_startup.owner, EV9PendingStartup)
        self.assertTrue(owner.prepared_for(card.CP) and owner.owner.published)
        self.assertEqual(card.CP.to_dict(), dict(cp_before, alternativeExperience=32 if aol else 0))
        self.assertEqual({key: saved.get(key) for key in keys}, before)
        self.assertEqual(self.queries, [])
        self.assertEqual(self.sent, [])
        with structs.CarParams.from_bytes(saved.get('CarParams')) as published:
          self.assertEqual(published.to_dict(), card.CP.to_dict())
        self.assertEqual(card.aol_permission_owners()[0], aol)
        owner.close()

  def test_marker_finalization_cannot_relabel_other_cp_fields_or_follow_publication(self):
    owner, ci, _ = self.instance(seal=False)
    ci.CP.alternativeExperience = 32
    ci.CP.steerRatio += 1
    with self.assertRaises(RuntimeError):
      owner.finalize_aol_configuration(ci)
    owner.close()
    owner, ci, _ = self.instance()
    ci.CP.alternativeExperience = 32
    with self.assertRaises(RuntimeError):
      owner.finalize_aol_configuration(ci)
    owner.close()
