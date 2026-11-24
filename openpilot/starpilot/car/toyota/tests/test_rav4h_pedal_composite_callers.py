"""Real caller/native composition; only producers, acquired FW and clocks are synthetic."""

import pytest
from opendbc.car.toyota import pedal
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('kind', ('nominal', 'physical', 'parent_drop', 'pedal_drop'))
def test_actual_independent_parent_and_pedal_callers(stock, kind, host_heartbeat_bridge):
  tick = 0
  pedal_sent = False

  def physical(packer, **fields):
    nonlocal tick
    tick += 1
    parent_drop = kind == 'parent_drop' and 260 <= tick < 311
    pedal_drop = kind == 'pedal_drop' and 260 <= tick < 311
    if parent_drop:
      fields['sdsu'] = None
    frames = helpers.physical(packer, **fields)
    # Exercise the pedal below its existing transition speed; parent ACC alone
    # owns the original 72 km/h fixture. This changes physical CAN input only.
    frames = [frame for frame in frames if frame[0] != 0xAA]
    frames.append(packer.make_can_msg('WHEEL_SPEEDS', 0, {
      'WHEEL_SPEED_' + wheel: 0.0 if fields.get('stopped') else 18.0 for wheel in ('FL', 'FR', 'RL', 'RR')}))
    if not pedal_drop:
      a, d, b = packer.make_can_msg(
        'GAS_SENSOR',
        0,
        {'INTERCEPTOR_GAS': 806 if fields.get('gas') else 0, 'INTERCEPTOR_GAS2': 806 if fields.get('gas') else 0, 'COUNTER_PEDAL': tick & 15, 'STATE': 0},
      )
      d = bytearray(d)
      d[-1] = pedal.checksum(d)
      frames.append((a, bytes(d), b))
    return frames

  def observe(tick, host, cs, cc, safety, native):
    if kind.endswith('_drop') and 301 <= tick < 308:
      assert not cc.latActive and not cc.longActive and not cs.canValid
      assert safety.aol_get_permission_mask() == 0

  def tx_check(tick, frame, cp, safety):
    nonlocal pedal_sent
    if frame.address == 0x200:
      data = bytes(frame.dat)
      assert len(data) == 6 and data[-1] == pedal.checksum(data)
      if data[4] & 128:
        assert not stock and safety.aol_get_permission_mask() & 2 and not safety.get_gas_pressed_prev() and not safety.get_brake_pressed_prev()
        pedal_sent = True
      else:
        assert data[:4] == bytes(4)
    return True

  helpers.run_case(
    stock,
    'main_lease' if kind.endswith('_drop') else kind,
    bridge=host_heartbeat_bridge,
    input_clock=True,
    sdsu_words=(0xB049, 0xB249),
    fingerprint_frames=[(0x2FF, bytes(8), 0), (0x201, bytes(6), 0)],
    physical_frames=physical,
    on_sample=observe,
    brake_stop_check_start=355,
    native_tx_expected=tx_check,
  )
  assert pedal_sent == (not stock)
