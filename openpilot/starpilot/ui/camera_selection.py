"""One saved camera choice and temporary reverse policy for both UI profiles."""
from openpilot.starpilot.ui.appearance_preferences import CameraViewChoice

WIDE_CAM_MAX_SPEED = 5.0
ROAD_CAM_MIN_SPEED = 10.0


def selected_camera_stream(choice, *, reverse_driver, current, available, experimental, speed, narrow, wide, driver):
  if choice == CameraViewChoice.NONE:
    return None
  if reverse_driver or choice == CameraViewChoice.DRIVER:
    return driver
  if choice == CameraViewChoice.STANDARD:
    return narrow
  if choice == CameraViewChoice.WIDE:
    return wide if wide in available else narrow
  if experimental and wide in available:
    if speed < WIDE_CAM_MAX_SPEED:
      return wide
    if speed > ROAD_CAM_MIN_SPEED:
      return narrow
    return current if current in (narrow, wide) else narrow
  return narrow
