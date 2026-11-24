"""Root-only actual canonical helper/call-contract tests, private candidate."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch
from openpilot.starpilot.flm.canonical_surface_adapter import CanonicalSurfaceEffects, NAMES, GROUPS, coordinate_bounds, original_anchor
from openpilot.starpilot.flm import torque_surface as source


class Frame(SimpleNamespace):
  def valid(self):
    return True


class CanonicalSurfaceTest(unittest.TestCase):
  def frame(self, **overrides):
    values = {
      'speed': 12.0,
      'setpoint': 0.2,
      'jerk': 0.4,
      'baseline_ff': 0.3,
      'baseline_threshold': 0.3,
      'active': True,
      'driver_override': False,
      'limited': False,
    }
    values.update(overrides)
    return Frame(**values)

  def make(self, coordinates, accumulated=None):
    return CanonicalSurfaceEffects(coordinates, 'a' * 64, accumulated=accumulated)

  def test_exact_canonical30_catalog_and_identifiable_groups(self):
    self.assertEqual(set(NAMES), set(source._GM_NEUTRAL))
    self.assertEqual(len(NAMES), 30)
    self.assertEqual({n for names in GROUPS.values() for n in names}, set(NAMES))
    self.assertTrue(all(1 <= len(names) <= 4 for names in GROUPS.values()))
    self.assertNotIn('STEER_MAX', NAMES)
    self.assertNotIn('selected_friction_amplitude', NAMES)

  def test_neutral_selected_tune_is_exact_across_arbitrary_vehicle_stages(self):
    for ff in (-0.8, -0.3, 0.0, 0.3, 0.8):
      for threshold in (0.05, 0.3, 0.8):
        with self.subTest(ff=ff, threshold=threshold):
          effect = self.make({'ff_gain_left': 0.0})
          frame = self.frame(baseline_ff=ff, baseline_threshold=threshold)
          self.assertEqual(effect.selected_stage(frame, original_anchor=effect.original_effect_anchor), (ff, threshold))
          self.assertEqual(effect.selected_friction(-0.07, original_anchor=effect.original_effect_anchor), -0.07)
          self.assertEqual(effect.selected_deadzone(12.0, 0.0, 0.02, 0.6, 0.2, original_anchor=effect.original_effect_anchor), 0.0)
          self.assertEqual(effect.selected_output(2.0, 0.2, 50.0, 10.0, 0.3, 0.1, original_anchor=effect.original_effect_anchor), 0.3)

  def test_stage_calls_existing_dom_helpers_in_canonical_order(self):
    effect = self.make({'ff_gain_left': 0.05})
    frame = self.frame()
    with (
      patch.object(source, '_gm_get_flm_full_surface_center_taper_scale', wraps=source._gm_get_flm_full_surface_center_taper_scale) as center,
      patch.object(source, '_gm_get_flm_full_surface_ff_scale', wraps=source._gm_get_flm_full_surface_ff_scale) as ff,
      patch.object(source, '_gm_get_flm_full_surface_friction_threshold', wraps=source._gm_get_flm_full_surface_friction_threshold) as threshold,
    ):
      value = effect.selected_stage(frame, original_anchor=effect.original_effect_anchor)
    center.assert_called_once_with(effect.surface, 'torque_universal', 0.2, 12.0, True)
    ff.assert_called_once_with(effect.surface, 'torque_universal', 0.2, 0.4, 12.0, True)
    threshold.assert_called_once_with(effect.surface, 'torque_universal', 0.3, 12.0, 0.2, 0.4, True)
    expected_ff = (
      0.3
      * source._gm_get_flm_full_surface_ff_scale(effect.surface, 'torque_universal', 0.2, 0.4, 12.0, True)
      * source._gm_get_flm_full_surface_center_taper_scale(effect.surface, 'torque_universal', 0.2, 12.0, True)
    )
    self.assertEqual(value[0], expected_ff)

  def test_actual_deadband_interpolation_and_own_vehicle_vm_scale(self):
    effect = self.make({'center_deadband_low_deg': 0.01})
    # Own frictionaccel .1 / VM .02=5deg; coefficient .01 => .05deg.
    actual = effect.selected_deadzone(5.0, 0.0, 0.02, 0.1, 0.2, original_anchor=effect.original_effect_anchor)
    raw = dict(effect.surface.knobs)
    raw['center_deadband_low_deg'] = 0.05
    s = source.GmSurface('torque_universal', raw)
    expected = source._gm_get_flm_full_surface_center_deadband_deg(s, s.profile, 5.0) * 0.02
    self.assertAlmostEqual(actual, expected)
    self.assertGreater(actual, 0.0)

  def test_actual_angle_helper_uses_dom_internal_policy_sign_directly(self):
    effect = self.make({'low_speed_angle_assist_max_torque': 0.1})
    raw = dict(effect.surface.knobs)
    raw['low_speed_angle_assist_max_torque'] = 0.1 * 0.3
    s = source.GmSurface('torque_universal', raw)
    expected = source._gm_get_flm_full_surface_low_speed_angle_assist_torque(s, s.profile, 50.0, 10.0, 0.3, 2.0)
    actual = effect.selected_output(2.0, 0.2, 50.0, 10.0, 0.3, 0.1, original_anchor=effect.original_effect_anchor)
    self.assertEqual(actual, expected)

  def test_standstill_retains_output_and_original_native_limits_are_not_inputs(self):
    effect = self.make({'low_speed_angle_assist_max_torque': 0.1})
    self.assertEqual(effect.selected_output(0.0, 0.2, 50.0, 10.0, 0.3, 0.1, original_anchor=effect.original_effect_anchor), 0.3)

  def test_windows_intersect_exact_existing_source_hard_bounds(self):
    low, high = coordinate_bounds('curvy_speed_max')
    self.assertLess(high, 0.25)
    original = source._GM_NEUTRAL['curvy_speed_max']
    self.assertAlmostEqual(original * (1 + high), source._GM_BOUNDS['curvy_speed_max'][1])
    with self.assertRaises(ValueError):
      self.make({'curvy_speed_max': high + 1e-6})

  def test_per_frame_use_and_original_anchor_are_not_renewable(self):
    effect = self.make({'ff_gain_left': 0.01})
    frame = self.frame()
    effect.begin_frame(1)
    effect.selected_stage(frame, original_anchor=effect.original_effect_anchor)
    with self.assertRaises(ValueError):
      effect.selected_stage(frame, original_anchor=effect.original_effect_anchor)
    with self.assertRaises(ValueError):
      effect.begin_frame(1)
    effect.begin_frame(2)
    effect.selected_stage(frame, original_anchor=effect.original_effect_anchor)
    with self.assertRaises(ValueError):
      effect.selected_friction(0.1, original_anchor='b' * 64)
    self.assertEqual(effect.original_effect_anchor, original_anchor('a' * 64, ('ff_gain_left',)))

  def test_accumulated_profile_binds_values_without_recentering_original(self):
    a = self.make({'ff_gain_left': 0.01})
    b = self.make({'ff_gain_left': 0.01}, accumulated={'center_taper_max': 0.01})
    self.assertEqual(a.original_effect_anchor, b.original_effect_anchor)
    self.assertNotEqual(a.full_profile_sha256, b.full_profile_sha256)
    with self.assertRaises(ValueError):
      self.make({'ff_gain_left': 0.251})


if __name__ == '__main__':
  unittest.main()
