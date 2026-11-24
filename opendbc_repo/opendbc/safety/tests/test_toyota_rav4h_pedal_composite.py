"""Physical native parent/pedal leases, independent gas vetoes and recovery."""

import pytest
import json
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.toyota import pedal
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers

source_matched_native = helpers.source_matched_native
libsafety_py.ffi.cdef("extern safety_config current_safety_config;")


@pytest.mark.parametrize('word', (0xB049, 0xB249))
@pytest.mark.parametrize('experience', (0, 32))
@pytest.mark.parametrize(
  'fault', ('pedal_crc', 'pedal_counter', 'pedal_state', 'pedal_drop', 'parent_drop', 'parent_aeb', 'pcm_gas', 'pedal_gas', 'main', 'eps')
)
def test_native_independent_parent_pedal_and_physical_gas(word, experience, fault):
  safety = libsafety_py.libsafety
  safety.init_tests()
  safety.set_alternative_experience(experience)
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, word) == 0
  safety.set_aol_test_heartbeat(True)
  packer = CANPacker('toyota_tnga_k_pt_generated')
  now = 1_000_000
  counter = 0

  def tick(bad=None, cruise=True):
    nonlocal now, counter
    now += 10_000
    safety.set_timer(now)
    frames = helpers.physical(
      packer,
      cruise=cruise,
      gas=bad == 'pcm_gas',
      main=bad != 'main',
      eps=9 if bad == 'eps' else 1,
      sdsu=None if bad == 'parent_drop' else 7 if bad == 'parent_aeb' else 0,
    )
    if bad != 'pedal_drop':
      a, d, b = packer.make_can_msg(
        'GAS_SENSOR',
        0,
        {
          'INTERCEPTOR_GAS': 806 if bad == 'pedal_gas' else 0,
          'INTERCEPTOR_GAS2': 806 if bad == 'pedal_gas' else 0,
          'COUNTER_PEDAL': 0 if bad == 'pedal_counter' else counter,
          'STATE': 1 if bad == 'pedal_state' else 0,
        },
      )
      counter = (counter + 1) & 15
      d = bytearray(d)
      d[-1] = pedal.checksum(d) ^ int(bad == 'pedal_crc')
      frames.append((a, bytes(d), b))
    for a, d, b in frames:
      safety.safety_rx_hook(libsafety_py.make_CANPacket(a, b, d))
    safety.safety_tick()
    safety.aol_set_host_request(3)

  def tx(value):
    a, d, b = pedal.command(packer, value, 0)
    return bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d)))

  def actual_rx_details():
    ffi = libsafety_py.ffi
    def flatten(value):
      if isinstance(value, (int, float, bool)):
        return value
      kind = ffi.typeof(value).kind
      if kind == 'struct':
        return {name: flatten(getattr(value, name)) for name, _ in ffi.typeof(value).fields}
      if kind == 'array':
        return [flatten(value[i]) for i in range(len(value))]
      return int(value)
    cfg = safety.current_safety_config
    return json.dumps([flatten(cfg.rx_checks[i]) for i in range(cfg.rx_checks_len)])
  try:
    assert not tx(0.1)
    for _ in range(25):
      tick()
    assert safety.safety_config_valid(), actual_rx_details()
    assert tx(0.1) == (word == 0xB049) and tx(0), actual_rx_details()
    assert safety.aol_get_permission_mask() == ((3 if word == 0xB049 else 1) if experience else 0)
    for _ in range(40):
      tick(fault)
    assert not tx(0.1) and tx(0)
    if fault in ('pcm_gas', 'pedal_gas'):
      assert safety.get_gas_pressed_prev()
    if fault in ('parent_aeb', 'pcm_gas', 'pedal_gas'):
      assert safety.aol_get_permission_mask() == (1 if experience else 0)
    else:
      assert safety.aol_get_permission_mask() == 0
    tick(cruise=False)
    for _ in range(30):
      tick()
    assert tx(0.1) == (word == 0xB049) and tx(0), actual_rx_details()
  finally:
    safety.set_alternative_experience(0)
    safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
