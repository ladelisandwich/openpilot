import unittest

from opendbc.car import structs
from opendbc.car.hyundai.ioniq5pe_long_profile import candidate
from opendbc.car.hyundai.ioniq5pe_stopping import PEStoppingPolicy
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.starpilot.longitudinal.extension import LongitudinalContext


class TestPEStopping(unittest.TestCase):
  def loop(self):
    loop = LongControl(candidate(params(), enabled=True, is_release=False))
    self.assertIsInstance(loop.extension.stopping_policy, PEStoppingPolicy)
    return loop

  @staticmethod
  def state(speed=0., standstill=False):
    cs = structs.CarState(vEgo=speed, canValid=True)
    cs.cruiseState.standstill = standstill
    return cs

  @staticmethod
  def context(**changes):
    fields = dict(has_lead=False, traffic_mode=False, custom_acceleration=False, profile_max_accel=0.)
    fields.update(changes)
    return LongitudinalContext(**fields)

  def test_pe_start_is_original_one_and_point_four_not_ev9_point_two_and_point_five(self):
    loop = self.loop()
    cs = self.state()
    output = loop.update(True, cs, .1, False, (-3.5, 2.), context=self.context())
    self.assertEqual(loop.long_control_state, LongCtrlState.starting)
    self.assertAlmostEqual(output, 1.)
    cs.vEgo = float(loop.CP.deprecated.vEgoStarting)
    loop.update(True, cs, .1, False, (-3.5, 2.), context=self.context())
    self.assertEqual(loop.long_control_state, LongCtrlState.starting)
    cs.vEgo += .0001
    loop.update(True, cs, .1, False, (-3.5, 2.), context=self.context())
    self.assertEqual(loop.long_control_state, LongCtrlState.pid)

  def test_traffic_custom_lead_and_selected_profile_limit_are_preserved(self):
    for changes, expected in (({'traffic_mode': True}, .1), ({'custom_acceleration': True}, .1),
                              ({'has_lead': True}, .1), ({'profile_max_accel': .3}, .3)):
      loop = self.loop()
      loop.long_control_state = LongCtrlState.starting
      output = loop.update(True, self.state(), .1, False, (-3.5, 2.), context=self.context(**changes))
      self.assertAlmostEqual(output, expected)
    loop = self.loop()
    loop.long_control_state = LongCtrlState.starting
    self.assertAlmostEqual(loop.update(True, self.state(), .1, False, (-3.5, 2.), context=LongitudinalContext()), .1)

  def test_original_release_dwell_and_stopping_ramp_use_existing_bounds(self):
    loop = self.loop()
    loop.long_control_state = LongCtrlState.stopping
    cs = self.state(standstill=True)
    for _ in range(34):
      loop.update(True, cs, .2, False, (-3.5, 2.), context=self.context())
      self.assertEqual(loop.long_control_state, LongCtrlState.stopping)
    loop.update(True, cs, .2, False, (-3.5, 2.), context=self.context())
    self.assertEqual(loop.long_control_state, LongCtrlState.starting)
    loop = self.loop()
    loop.long_control_state = LongCtrlState.stopping
    self.assertAlmostEqual(loop.update(True, self.state(), -1., True, (-3.5, 2.), context=self.context()), -.004)

  def test_current_output_cap_and_disengagement_remain_final(self):
    loop = self.loop()
    loop.long_control_state = LongCtrlState.starting
    self.assertLessEqual(loop.update(True, self.state(), 100., False, (-3.5, .25), context=self.context()), .25)
    self.assertEqual(loop.update(False, self.state(), 100., False, (-3.5, 2.), context=self.context()), 0.)
    self.assertEqual(loop.long_control_state, LongCtrlState.off)

  def test_original_zero_default_pid_gains_and_unit_feedforward_are_preserved(self):
    loop = self.loop()
    for speed in (0., .4, 5., 20., 35.):
      loop.long_control_state = LongCtrlState.pid
      for target in (-1., 0., .5):
        output = loop.update(True, self.state(speed), target, False, (-3.5, 2.), context=self.context())
        self.assertEqual(loop.pid.k_p, 0.)
        self.assertEqual(loop.pid.k_i, 0.)
        self.assertAlmostEqual(output, target)
