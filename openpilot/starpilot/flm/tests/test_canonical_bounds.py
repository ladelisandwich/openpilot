"""Actual source hard-domain agreement for every canonical normalized knob."""

import pytest
from openpilot.starpilot.flm import canonical_surface_adapter as adapter, torque_surface as source


def test_raw_domains_bind_exact_existing_universal_defaults_and_limits():
  limits = dict(source._GM_BOUNDS)
  limits.update(ff_gain_left=(-0.4, 0.6), ff_gain_right=(-0.4, 0.6))
  assert len(adapter.RAW_DOMAINS) == 30
  for name, original, hardbounds in adapter.RAW_DOMAINS:
    assert original == source._GM_NEUTRAL[name]
    assert hardbounds == limits[name]


@pytest.mark.parametrize('name', adapter.NAMES)
def test_all30_normalization_endpoints_preserve_actual_constructor_domains(name):
  low, high = adapter.coordinate_bounds(name)
  for coordinate in (low, 0.0, high):
    raw = dict(source._GM_NEUTRAL)
    if name in adapter.WINDOWS:
      raw[name] *= 1 + coordinate
    elif name in adapter.DEADBANDS or name == 'low_speed_angle_assist_max_torque':
      # Actual0normalizer mustfreeze output; nonzero normalizers intersect unitraw
      # domains at application rather than confusing coefficient withdegrees/torque.
      for normalizer in (0.0, 0.01, 0.5, 1.0, 10.0, 100.0):
        dl, dh = adapter.dynamic_coordinate_bounds(name, normalizer)
        assert 0 <= dl <= dh <= 0.25
        selected = min(max(coordinate, dl), dh)
        dynamic = dict(raw)
        dynamic[name] = selected * normalizer
        source.GmSurface('torque_universal', dynamic)
      continue
    else:
      raw[name] = coordinate
    source.GmSurface('torque_universal', raw)


@pytest.mark.parametrize(
  'name,expected', [('center_taper_max', 0.18), ('highway_center_taper_max', 0.18), ('curvy_turn_in_trim_left', 0.20), ('curvy_turn_in_trim_right', 0.20)]
)
def test_stored_profile_cannot_admit_raw_amplitudes_above_existing_domain(name, expected):
  assert adapter.coordinate_bounds(name)[1] == expected
  with pytest.raises(ValueError):
    adapter.validate_trial({name: expected + 1e-10})
  values = dict.fromkeys(adapter.NAMES, 0.0)
  values[name] = expected + 1e-10
  with pytest.raises(ValueError):
    adapter.full_profile_sha256('a' * 64, values)


def test_window_joint_extremes_keep_order_and_rawbounds():
  for a in adapter.coordinate_bounds('curvy_speed_min'):
    for b in adapter.coordinate_bounds('curvy_speed_max'):
      for c in adapter.coordinate_bounds('curvy_turn_in_trim_speed_min'):
        for d in adapter.coordinate_bounds('curvy_turn_in_trim_speed_max'):
          effect = adapter.CanonicalSurfaceEffects(dict(zip(adapter.WINDOWS, (a, b, c, d), strict=True)), 'a' * 64)
          assert effect.surface.knobs['curvy_speed_min'] < effect.surface.knobs['curvy_speed_max']
          assert effect.surface.knobs['curvy_turn_in_trim_speed_min'] < effect.surface.knobs['curvy_turn_in_trim_speed_max']


def test_true_invalid_dynamic_coordinates_fail_at_actual_hook_without_cap_relaxation():
  effects = adapter.CanonicalSurfaceEffects({'center_deadband_highway_deg': 0.25}, 'a' * 64)
  with pytest.raises(ValueError):
    effects.selected_deadzone(30.0, 0.0, 0.01, 0.1, 0.0, original_anchor=effects.original_effect_anchor)
  assert source._GM_BOUNDS['center_deadband_highway_deg'] == (0.0, 0.08)
