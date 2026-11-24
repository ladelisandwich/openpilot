"""Find gamepads even when udev does not create /dev/input/by-id links."""

from pathlib import Path
import re

import inputs


def refresh_gamepads() -> None:
  inputs.devices = inputs.DeviceManager()
  known = {pad.get_char_device_path() for pad in inputs.devices.gamepads}
  try:
    devices = Path('/proc/bus/input/devices').read_text().split('\n\n')
  except OSError:
    return

  for device in devices:
    handlers = re.search(r'^H: Handlers=(.*)$', device, re.MULTILINE)
    if handlers is None or not any(re.fullmatch(r'js\d+', h) for h in handlers[1].split()):
      continue
    event = next((h for h in handlers[1].split() if re.fullmatch(r'event\d+', h)), None)
    if event is None:
      continue
    path = Path('/dev/input') / event
    if str(path) not in known and path.exists():
      # inputs only discovers devices through udev links; comma may have none for Bluetooth HID.
      try:
        pad = inputs.GamePad(inputs.devices, '/dev/input/by-id/usb-fallback-event-joystick', str(path))
      except OSError:
        continue
      inputs.devices.gamepads.append(pad)
      known.add(str(path))
