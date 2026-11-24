"""Rich camera color: faint when disengaged, muted for AOL, vivid with clear tones engaged."""

from openpilot.starpilot.ui.runtime_snapshot import display_message

ROAD_COLOR_SHADER = """
highp vec3 road_color(highp vec3 rgb, highp float strength) {
  const highp vec3 weights = vec3(0.299, 0.587, 0.114);
  // YUV conversion can produce channels outside the display range.
  rgb = clamp(rgb, 0.0, 1.0);
  // Preserve the muted disengaged look; active states keep the video's original midtones.
  if (strength <= 0.2) {
    rgb = pow(rgb, vec3(1.0 / 1.28));
    highp float gray = dot(rgb, weights);
    rgb = clamp((vec3(gray) + 1.45 * (rgb - vec3(gray)) - 0.5) * 1.25 + 0.5, 0.0, 1.0);
    return mix(vec3(dot(rgb, weights)), rgb, 0.2);
  }
  highp float high = max(rgb.r, max(rgb.g, rgb.b));
  highp float low = min(rgb.r, min(rgb.g, rgb.b));
  highp float saturation = (high - low) / max(high, 0.000001);
  // Taper the boost near neutral, black, and already saturated colors without singling out a hue.
  highp float boost = 0.95 * smoothstep(0.04, 0.20, saturation) * smoothstep(0.03, 0.12, high);
  rgb = vec3(high) - (vec3(high) - rgb) * (1.0 + boost * (1.0 - saturation));
  rgb *= 1.0 + 0.12 * (high * (3.0 - 2.0 * high) - 1.0);
  return mix(vec3(dot(rgb, weights)), rgb, strength);
}
"""


def road_color_strength(ui) -> float:
  if not ui.started:
    return 0.2
  now = ui.display_time_ns()
  control = display_message(ui.sm, 'carControl', now, after_frame=ui.started_frame)
  if control is None:
    return 0.2
  selfdrive = display_message(ui.sm, 'selfdriveState', now, after_frame=ui.started_frame)
  # Enabled includes stock ACC and temporary pedal/steering overrides.
  if control.longActive or (selfdrive is not None and selfdrive.enabled):
    return 1.0
  return 0.45 if control.latActive else 0.2
