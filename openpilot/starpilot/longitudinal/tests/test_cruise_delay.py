"""Exercise the live planner caller and native MPC for the #39127 regression."""
import unittest
from unittest.mock import patch
from typing import Any

import numpy as np

from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.selfdrive.controls.lib import longitudinal_planner as planner_module
from openpilot.starpilot.longitudinal.experimental_release import SOURCES
from openpilot.starpilot.longitudinal.profile_runtime import ProfileTuning
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages, message_bytes
from openpilot.starpilot.longitudinal.tests.test_throttle_gate import Frame

NOW = 10_000_000_000
DRIVE = NOW - 1_000_000_000
KEY = ('cruise-delay-test', 1, DRIVE, 'conditional_experimental')


class NativeCruiseDelayTests(unittest.TestCase):
  def test_lower_set_speed_uses_actual_previous_output_with_native_mpc(self):
    # The old lower jerk bound takes many frames to unwind positive cruise state
    # even when lead/model arbitration already commanded deceleration.
    for case in ('ordinary', 'profile', 'cem', 'experimental', 'stock'):
      with self.subTest(case=case):
        cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
        cp.openpilotLongitudinalControl = case != 'stock'
        cp_before = cp.to_bytes()
        cp.clear_write_flag()
        current = planner_module.LongitudinalPlanner(cp, init_v=20.)
        legacy = planner_module.LongitudinalPlanner(cp, init_v=20.)
        for planner in (current, legacy):
          planner.a_cruise = 1.
          planner.output_a_target = -.2
        sm, envelopes = messages(e2e=case == 'experimental')
        sm = Frame(sm)
        sm.logMonoTime = dict.fromkeys(SOURCES, NOW - 5_000_000)
        sm.valid = dict.fromkeys(SOURCES, True)
        sm.alive = dict.fromkeys(SOURCES, True)
        sm['carControl'].longActive = case != 'stock'
        sm['carState'].canValid = True
        sm['carState'].aEgo = -.35
        sm['modelV2'].action.shouldStop = False
        # Lower the prior fixture set speed from 100 to 36 km/h at 20 m/s.
        self.assertEqual(sm['carState'].vCruise, 100.)
        sm['carState'].vCruise = 36.
        kwargs: dict[str, Any] = {'now_ns': NOW, 'drive_id': DRIVE}
        if case == 'profile':
          kwargs['profile_tuning'] = ProfileTuning('standard', 1.8, 1., 1., 1., 1., 1.,
                                                   acceleration_max=.7, cruise_brake_magnitude=.6)
        if case in ('cem', 'experimental'):
          kwargs['conditional_handoff'] = KEY
        # SubMaster supplies immutable payload readers; reading an absent struct
        # on a fixture builder otherwise allocates wire storage.
        for service, payload in sm.items():
          sm[service] = payload.as_reader()
        before = message_bytes(envelopes)
        real_helper = planner_module.get_cruise_accel
        forwarded = []

        def observe(*args, forwarded=forwarded, real_helper=real_helper, **options):
          forwarded.append(options['a_target_prev'])
          return real_helper(*args, **options)

        def old_lower_bound(*args, real_helper=real_helper, **options):
          # Counterfactual for the exact pre-#39127 lower bound; native MPC,
          # production caller, profiles and final candidate selection stay real.
          options['a_target_prev'] = None
          return real_helper(*args, **options)

        for planner, helper in ((current, observe), (legacy, old_lower_bound)):
          for field in ('params', 'x_sol', 'u_sol', 'x0'):
            self.assertTrue(np.isfinite(getattr(planner.mpc, field)).all())
          # run() clears the failing status during reset(), so status alone
          # cannot prove a successful native solve. Observe every post-init reset.
          with patch.object(planner.mpc, 'reset', wraps=planner.mpc.reset) as reset, \
               patch.object(planner_module, 'get_cruise_accel', side_effect=helper):
            planner.update(sm, **kwargs)
          reset.assert_not_called()
          for field in ('params', 'x_sol', 'u_sol', 'x0'):
            self.assertTrue(np.isfinite(getattr(planner.mpc, field)).all())
          self.assertTrue(np.isfinite([planner.a_cruise, planner.output_a_target]).all())
        self.assertEqual(forwarded, [-.2])
        self.assertEqual(message_bytes(envelopes), before)
        self.assertEqual(cp.to_bytes(), cp_before)
        cp.clear_write_flag()
        self.assertEqual(current.mpc.solution_status, 0)
        self.assertEqual(legacy.mpc.solution_status, 0)
        self.assertGreater(legacy.a_cruise, .9)
        self.assertLess(current.a_cruise, -.2)
        self.assertLessEqual(current.output_a_target, current.a_cruise)
        self.assertGreaterEqual(current.output_a_target, planner_module.ACCEL_MIN)
        self.assertLessEqual(current.output_a_target, planner_module.ACCEL_MAX)
        self.assertEqual(current.last_profile, legacy.last_profile)
        self.assertEqual(current.allow_throttle, legacy.allow_throttle)
        # Cruise candidates are outside the MPC: same real solver trajectory,
        # constraints and weights must survive the response change.
        self.assertEqual(current.mpc._cached_cost_weights, legacy.mpc._cached_cost_weights)
        for field in ('params', 'x_sol', 'u_sol', 'x0'):
          np.testing.assert_array_equal(getattr(current.mpc, field), getattr(legacy.mpc, field))
        expected_state = (20., -.35) if case == 'stock' else (20., -.2)
        np.testing.assert_allclose(current.mpc.x0[1:], expected_state)
        if case == 'profile':
          self.assertIsNotNone(current.last_profile)
          # A valid ordinary profile fades from native 1.45 by dt=.05.
          self.assertAlmostEqual(current.last_profile.follow_seconds, 1.5)
          self.assertAlmostEqual(float(current.mpc.params[0, 4]), current.last_profile.follow_seconds)
        if case in ('cem', 'experimental'):
          # This single-cycle fixture proves handoff admission, not a transition hold.
          self.assertEqual(current.experimental_release.key, KEY)
        if case == 'experimental':
          self.assertLessEqual(current.output_a_target, -2.)


if __name__ == '__main__':
  unittest.main()
