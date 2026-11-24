"""Every exact composite retains its parent's RX contracts and physical gates."""
import pytest
from opendbc.car import structs
from opendbc.car.toyota.pedal_composite import PARENT_WORDS
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers

source_matched_native = helpers.source_matched_native
libsafety_py.ffi.cdef('extern safety_config current_safety_config;')


def rx_catalog(safety):
  cfg = safety.current_safety_config
  fields = ('addr', 'bus', 'len', 'frequency', 'ignore_checksum', 'ignore_counter', 'max_counter', 'ignore_quality_flag')
  return [tuple(int(getattr(cfg.rx_checks[i].msg[0], key)) for key in fields) for i in range(cfg.rx_checks_len)]


@pytest.mark.parametrize('word,parent', sorted(PARENT_WORDS.items()))
def test_parent_catalog_and_warm_switch_are_exact(word, parent):
  safety = libsafety_py.libsafety
  safety.init_tests()
  safety.set_alternative_experience(32)
  try:
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, parent) == 0
    original = rx_catalog(safety)
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, word) == 0
    composite = rx_catalog(safety)
    assert 5 <= len(composite) <= 8
    assert composite[:len(original)] == original
    extra_eps = not any(row[0] == 0x262 and row[1] == 0 for row in original)
    extra_main = not any(row[0] in (0x1D3, 0x365) and row[1] == 0 for row in original)
    assert len(composite) == len(original) + 1 + int(extra_eps) + int(extra_main)
    assert sum(row[0] == 0x2FF for row in composite) == sum(row[0] == 0x2FF for row in original)
    assert sum(row[:3] == (0x201, 0, 6) for row in composite) == 1
    assert sum(row[0] == 0x262 and row[1] == 0 for row in composite) == 1
    assert sum(row[0] in (0x1D3, 0x365) and row[1] == 0 for row in composite) == 1
    pedal = next(row for row in composite if row[0] == 0x201)
    assert pedal[3:] == (50, 0, 0, 15, 0)
    assert not safety.safety_config_valid() and safety.aol_get_permission_mask() == 0
    # A warm switch must not retain the pedal's extra catalog or authority.
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, parent) == 0
    assert rx_catalog(safety) == original
    assert not safety.safety_config_valid() and safety.aol_get_permission_mask() == 0
  finally:
    safety.set_alternative_experience(0)
    safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)


@pytest.mark.parametrize('word', (0xF149, 0xF349, 0xF04D, 0xF24D))
@pytest.mark.parametrize('fault', ('main_off', 'main_stale'))
def test_composite_alternate_main_keeps_its_independent_freshness(word, fault):
  from opendbc.can import CANPacker
  from opendbc.car import Bus
  from opendbc.car.toyota import pedal
  from opendbc.car.toyota.values import CAR, DBC

  safety = libsafety_py.libsafety
  safety.init_tests()
  safety.set_alternative_experience(32)
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, word) == 0
  safety.set_aol_test_heartbeat(True)
  identity = CAR.LEXUS_GS_F if word in (0xF149, 0xF349) else CAR.LEXUS_IS
  packer = CANPacker(DBC[identity][Bus.pt])
  now = 1_000_000
  counter = 0

  def tick(*, main=True, omit_main=False):
    nonlocal now, counter
    now += 10_000
    safety.set_timer(now)
    frames = helpers.physical(packer, cruise=True)
    frames.append(packer.make_can_msg('PCS_HUD', 0, {}))
    frames.append(packer.make_can_msg('PCM_CRUISE_ALT', 0, {'UI_SET_SPEED': 90}))
    if not omit_main:
      frames.append(packer.make_can_msg('DSU_CRUISE', 0, {'MAIN_ON': main, 'SET_SPEED': 90}))
    address, data, bus = packer.make_can_msg('GAS_SENSOR', 0, {
      'INTERCEPTOR_GAS': 0, 'INTERCEPTOR_GAS2': 0, 'COUNTER_PEDAL': counter, 'STATE': 0})
    counter = (counter + 1) & 15
    data = bytearray(data)
    data[-1] = pedal.checksum(data)
    frames.append((address, bytes(data), bus))
    for address, data, bus in frames:
      safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
    safety.safety_tick()
    safety.aol_set_host_request(3)

  try:
    assert safety.current_safety_config.rx_checks_len == 8
    for _ in range(30):
      tick()
    expected = 1 if word in (0xF349, 0xF24D) else 3
    assert safety.safety_config_valid()
    assert safety.aol_get_permission_mask() == expected
    steering = packer.make_can_msg('STEERING_LKA', 0, {'STEER_REQUEST': 1, 'STEER_TORQUE_CMD': 1})
    def transmit_steering():
      address, data, bus = steering
      return bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)))
    assert transmit_steering()
    for _ in range(40):
      tick(main=False, omit_main=fault == 'main_stale')
    assert safety.aol_get_permission_mask() == 0
    assert not transmit_steering()
    for _ in range(30):
      tick()
    assert safety.aol_get_permission_mask() == expected
    assert transmit_steering()
  finally:
    safety.set_alternative_experience(0)
    safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
