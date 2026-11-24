"""Preserve complete physical payloads, including bits absent from the current DBC."""

import time
from opendbc.car.subaru.sng import selected


def replacement(address, data, *, throttle=False, speed=False):
  if len(data) != 8 or address not in (0x40, 0x139):
    raise ValueError('Unexpected Subaru standstill source')
  result = bytearray(data)
  result[1] = (result[1] & 0xF0) | ((result[1] + 1) & 15)
  if address == 0x40 and throttle:
    result[4] = max(result[4], 5)
  if address == 0x139 and speed:
    # Dom's 3 kph hold pulse, quantized by the current 0.05625 kph DBC.
    result[2] = 53
    result[3] &= 0xF0
  result[0] = ((address & 255) + (address >> 8) + sum(result[1:])) & 255
  return address, bytes(result), 2


def send(controller, cc, cs, now_ns):
  if not selected(controller.CP):
    return []
  fresh = 0 < controller.sng_command_ns <= now_ns and now_ns - controller.sng_command_ns <= 30_000_000
  now_boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  physical_ready = True
  for address, age in ((0x40, 20_000_000), (0x139, 40_000_000)):
    sample = cs.sng_sources.get(address)
    physical_ready = physical_ready and sample is not None and cs.sng_source_valid.get(address, False)
    if sample is not None:
      source_ns, raw = sample
      physical_ready = physical_ready and 0 < source_ns <= now_boot and now_boot - source_ns <= age
      physical_ready = physical_ready and (raw[4] == 0 if address == 0x40 else (raw[4] >> 4 | raw[5] << 4) == 0)
  cs.sng_lead_ready = bool(physical_ready and cs.sng_lead_present and 0 < cs.sng_lead_source_ns <= now_boot and now_boot - cs.sng_lead_source_ns <= 100_000_000)
  if fresh:
    throttle, speed = controller.sng_owner.update(cc, cs, controller.frame)
  else:
    controller.sng_owner.reset(controller.frame)
    throttle, speed = False, False
  if not cs.out.standstill:
    controller.sng_stationary_since_ns = 0
  elif not controller.sng_stationary_since_ns:
    controller.sng_stationary_since_ns = now_ns
  stationary_age = now_ns - controller.sng_stationary_since_ns
  messages = []
  for address, (source_ns, data) in cs.sng_sources.items():
    age = 20_000_000 if address == 0x40 else 40_000_000
    if (
      not cs.sng_source_valid.get(address, False)
      or not cs.out.canValid
      or cs.out.canTimeout
      or not 0 < source_ns <= now_boot
      or now_boot - source_ns > age
      or source_ns <= controller.sng_sent_sources.get(address, 0)
    ):
      continue
    if address == 0x40 and (cs.out.gasPressed or data[4] != 0) or address == 0x139 and (cs.out.brakePressed or (data[4] >> 4 | data[5] << 4) != 0):
      continue
    resume = throttle and address == 0x40 and data[4] == 0
    elapsed = now_ns - controller.sng_pulse_ns
    pulse = (
      speed
      and stationary_age >= 500_000_000
      and address == 0x139
      and ((data[3] & 15) << 8 | data[2]) <= 53
      and (not controller.sng_pulse_ns or elapsed <= 50_000_000 or elapsed >= 550_000_000)
    )
    if pulse and (not controller.sng_pulse_ns or elapsed >= 550_000_000):
      controller.sng_pulse_ns = now_ns
    messages.append(replacement(address, data, throttle=resume, speed=pulse))
    controller.sng_sent_sources[address] = source_ns
  return messages
