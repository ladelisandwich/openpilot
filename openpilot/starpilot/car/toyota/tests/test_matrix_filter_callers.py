"""Manual Matrix filter through real factory, daemons, C++ wire and native TX.

Manual selection and CAN/perception/USB/input-clock producers are fixtures.
No firmware match, health, permission or physical-road acceptance is claimed.
"""
import struct

import pytest
from opendbc.car.toyota import matrix_filter
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


@pytest.mark.parametrize('length', (4, 8))
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('scenario', ('nominal', 'physical'))
def test_manual_matrix_filter_actual_callers(length, stock, scenario, host_heartbeat_bridge):
  def physical(packer, **fields):
    if length == 4:
      fields['sdsu'] = None
    frames = helpers.physical(packer, **fields)
    assert len(next(data for address, data, bus in frames if address == 0x262 and bus == 0)) == 5
    assert len(next(data for address, data, bus in frames if address == 0x1D3 and bus == 0)) == 8
    if length == 4:
      # The high nibble of byte2 is not an SDSU state for this protocol.
      frames.append((0x2FF, bytes.fromhex('8000f4ff'), 0))
    return frames

  def state(tick, host, cs):
    cp = host.CP
    assert cp.carFingerprint == CAR.TOYOTA_MATRIX_RETROFIT
    assert cp.safetyConfigs[0].safetyParam == matrix_filter.PROTOCOLS[length][stock]
    assert cp.flags == int(ToyotaFlags.RADAR_CAN_FILTER if length == 4 else ToyotaFlags.LONG_FILTER)
    assert cp.lateralTuning.torque.latAccelFactor == struct.unpack('<f', struct.pack('<f', 4.05))[0]
    assert cp.lateralTuning.torque.friction == struct.unpack('<f', struct.pack('<f', .1))[0]
    assert not cp.autoResumeSng and not cp.alphaLongitudinalAvailable

  def observe(tick, host, cs, cc, safety, native):
    if scenario == 'physical' and tick == 355:
      assert cs.brakePressed and cs.standstill and cc.latActive
      assert not cc.longActive and safety.aol_get_permission_mask() == 1
    if stock:
      assert not cc.longActive and not safety.aol_get_permission_mask() & 2
    if scenario == 'nominal' and tick >= 220:
      assert cc.latActive and cc.longActive == (not stock)
      assert safety.aol_get_permission_mask() == (1 if stock else 3)
    if scenario == 'physical' and tick == 425:
      assert str(cs.gearShifter) == 'reverse'
      assert not cc.latActive and not cc.longActive and safety.aol_get_permission_mask() == 0

  helpers.run_case(stock, scenario, candidate=CAR.TOYOTA_MATRIX_RETROFIT,
                   bridge=host_heartbeat_bridge, input_clock=True, brake_stop_check_start=360,
                   sdsu_words=matrix_filter.PROTOCOLS[length], firmware_override=[],
                   fingerprint_frames=[(0x2FF, bytes(length), 0)], manual_selection=True, manual_filter_length=length,
                   filter_advert=length == 4, physical_frames=physical, on_state=state, on_sample=observe)
