"""Dom's two Resume/Set copies, bound to one current neutral physical source."""

import time
from opendbc.car import structs
from opendbc.car.subaru.software_cruise import selected


def send(controller, cc, cs, now_ns):
  if not selected(controller.CP):
    return []
  source = cs.software_button_source_ns
  now_boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  command = controller.software_cruise_command
  state = cs.out
  buttons = cs.software_buttons_msg
  if (
    not selected(controller.CP)
    or command not in (1, 2)
    or not cc.enabled
    or cc.cruiseControl.cancel
    or cc.cruiseControl.override
    or cc.cruiseControl.resume
    or not state.cruiseState.enabled
    or not state.cruiseState.available
    or not state.canValid
    or state.canTimeout
    or state.brakePressed
    or state.gasPressed
    or state.accFaulted
    or state.steerFaultPermanent
    or state.standstill
    or state.gearShifter != structs.CarState.GearShifter.drive
    or not 0 < controller.software_cruise_command_ns <= now_ns
    or now_ns - controller.software_cruise_command_ns > 30_000_000
    or not 0 < source <= now_boot
    or now_boot - source > 100_000_000
    or source <= controller.software_cruise_sent_source
    or controller.frame - controller.software_cruise_last_frame < 10
    or now_ns - controller.software_cruise_last_send_ns < 100_000_000
    or not buttons
    or any(buttons[name] for name in ('Main', 'Set', 'Resume'))
  ):
    return []
  controller.software_cruise_sent_source = source
  controller.software_cruise_last_frame = controller.frame
  controller.software_cruise_last_send_ns = now_ns
  messages = []
  for index in (1, 2):
    values = {name: buttons[name] for name in ('Signal1', 'Signal2')}
    values.update(COUNTER=(int(buttons['COUNTER']) + index) % 16, Main=0, Set=int(command == 2), Resume=int(command == 1))
    messages.append(controller.packer.make_can_msg('Cruise_Buttons', 0, values))
  return messages
