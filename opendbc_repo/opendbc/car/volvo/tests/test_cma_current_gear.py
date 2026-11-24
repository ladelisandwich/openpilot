"""Actual received gear controls host/native command eligibility, never MAIN."""
import pytest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.volvo.tests.test_cma import CMA, interface, sources, packet
from opendbc.safety.tests.libsafety import libsafety_py


def received(identity, gear):
  packer = CANPacker('volvo_mid_1')
  return [packer.make_can_msg('GEAR_POSITION', 0, {'GEAR_POSITION': gear}) if msg[0] == 0x80 else msg
          for msg in sources(identity)]


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('gear', [0, 1, 2, 5, 6, 7])
def test_actual_host_gear_never_creates_modified_steering_in_non_drive(identity, gear):
  _, ci = interface(identity)
  data = received(identity, gear)
  ci.update([(1_000_000_000, data)])
  cc = structs.CarControl.new_message()
  cc.latActive = True  # stale/incorrect upstream active input must not change the physical gear.
  _, output = ci.apply(cc.as_reader(), 1_000_000_000)
  source = {address: payload for address, payload, _ in data}
  for address, payload, _ in output:
    assert payload == source[address]


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('gear', [3, 4])
def test_actual_drive_and_b_both_preserve_selected_angle_control(identity, gear):
  _, ci = interface(identity)
  ci.update([(1_000_000_000, received(identity, gear))])
  cc = structs.CarControl.new_message()
  cc.latActive = True
  _, output = ci.apply(cc.as_reader(), 1_000_000_000)
  steering = next(payload for address, payload, _ in output if address == 0x58)
  # The selected Dom envelope begins rebuilding; its positive authority is nonzero.
  assert (((steering[0] & 7) << 8) | steering[1]) > 0


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('gear', [0, 1, 2, 5, 6, 7])
def test_real_native_gear_vetoes_valid_crc_source_bound_host_forgery(identity, gear):
  cp, drive_ci = interface(identity)
  lib = libsafety_py.libsafety
  assert lib.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  lib.init_tests()
  lib.set_timer(10000)
  # Native consumes the actual non-drive gear, while a distinct host state is
  # deliberately supplied Drive. No helper sets native controlsAllowed.
  for msg in received(identity, gear):
    assert lib.safety_rx_hook(packet(msg))
  lib.safety_tick()
  assert lib.get_controls_allowed()  # real stock cruise alone is not a Drive assertion.
  drive_ci.update([(1_000_000_000, received(identity, 3))])
  cc = structs.CarControl.new_message()
  cc.latActive = True
  _, output = drive_ci.apply(cc.as_reader(), 1_000_000_000)
  lca = next(msg for msg in output if msg[0] == 0x58)
  assert not lib.safety_tx_hook(packet(lca))
  # Current exact stock packets can still cross the harness without invented authority.
  stock = next(msg for msg in received(identity, gear) if msg[0] == 0x58)
  assert lib.safety_tx_hook(packet((stock[0], stock[1], 2)))


@pytest.mark.parametrize('identity', CMA)
def test_fresh_other_receivers_and_controls_cannot_renew_missing_gear(identity):
  cp, ci = interface(identity)
  lib = libsafety_py.libsafety
  assert lib.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  lib.init_tests()
  initial = received(identity, 3)
  lib.set_timer(10000)
  for msg in initial:
    assert lib.safety_rx_hook(packet(msg))
  ci.update([(1_000_000_000, initial)])
  fresh = [msg for msg in received(identity, 3) if msg[0] != 0x80]
  lib.set_timer(110001)
  for msg in fresh:
    assert lib.safety_rx_hook(packet(msg))
  assert lib.get_controls_allowed()
  ci.update([(1_100_001_000, fresh)])
  cc = structs.CarControl.new_message()
  cc.latActive = True
  _, output = ci.apply(cc.as_reader(), 1_100_001_000)
  lca = next(msg for msg in output if msg[0] == 0x58)
  assert not lib.safety_tx_hook(packet(lca))
  gear = next(msg for msg in initial if msg[0] == 0x80)
  for bus in (0, 1, 2):
    assert not lib.safety_tx_hook(packet((gear[0], gear[1], bus)))
