"""Ascent2023 angle target shaping retained from pinned Dom."""

_ASCENT_ANGLE_TRACKING_GAIN = 0.25
_ASCENT_ANGLE_TRACKING_MAX_CORRECTION = 8.0
_ASCENT_ANGLE_TRACKING_MIN_SPEED = 9.0
_ASCENT_ANGLE_TRACKING_FULL_SPEED = 15.0
_ASCENT_ANGLE_TRACKING_TURN_START = 15.0
_ASCENT_ANGLE_TRACKING_TURN_FULL = 35.0
_ASCENT_LOW_SPEED_FILTER_MAX_SPEED = 10.0
_ASCENT_LOW_SPEED_FILTER_CENTER_ANGLE = 35.0
_ASCENT_LOW_SPEED_FILTER_TIME_CONSTANT = 0.18


def _clipped_weight(value: float, start: float, end: float) -> float:
  return max(0.0, min(1.0, (value - start) / (end - start)))


def _ascent_angle_tracking_target(target_angle: float, steering_angle: float,
                                  v_ego: float, steering_pressed: bool) -> float:
  if steering_pressed:
    return target_angle

  speed_weight = _clipped_weight(v_ego, _ASCENT_ANGLE_TRACKING_MIN_SPEED, _ASCENT_ANGLE_TRACKING_FULL_SPEED)
  turn_weight = _clipped_weight(abs(target_angle), _ASCENT_ANGLE_TRACKING_TURN_START, _ASCENT_ANGLE_TRACKING_TURN_FULL)
  correction = (target_angle - steering_angle) * _ASCENT_ANGLE_TRACKING_GAIN * max(speed_weight, turn_weight)
  correction = max(-_ASCENT_ANGLE_TRACKING_MAX_CORRECTION,
                   min(_ASCENT_ANGLE_TRACKING_MAX_CORRECTION, correction))
  return target_angle + correction


def _ascent_low_speed_angle_target(target_angle: float, previous_target: float,
                                   v_ego: float, steering_pressed: bool, dt: float) -> float:
  if steering_pressed:
    return target_angle

  speed_weight = 1.0 - _clipped_weight(v_ego, 0.0, _ASCENT_LOW_SPEED_FILTER_MAX_SPEED)
  center_weight = 1.0 - _clipped_weight(abs(target_angle), 0.0, _ASCENT_LOW_SPEED_FILTER_CENTER_ANGLE)
  time_constant = _ASCENT_LOW_SPEED_FILTER_TIME_CONSTANT * speed_weight * center_weight
  if time_constant <= 0.0:
    return target_angle

  alpha = dt / (time_constant + dt)
  return previous_target + alpha * (target_angle - previous_target)
