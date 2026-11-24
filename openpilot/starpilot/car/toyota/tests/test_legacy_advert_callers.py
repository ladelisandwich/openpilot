"""Four-byte EPS5 ownership through actual factory/Card/SD/Controls/C++/native."""
import pytest
from unittest.mock import patch
from dataclasses import replace
from openpilot.starpilot.aol.wire import decode_safety, encode_safety
from opendbc.car import Bus
from opendbc.car.toyota import legacy_advert_filter
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope
REPRESENTATIVES = (CAR.TOYOTA_RAV4H, CAR.TOYOTA_COROLLA, CAR.LEXUS_GS_F, CAR.LEXUS_IS)


def run(candidate, stock, bridge, scenario='nominal', *, aol=True, fault=None):
  tick = 0
  alternate = candidate in legacy_advert_filter.ALT_MAIN_CARS

  def physical(packer, **fields):
    nonlocal tick
    tick += 1
    if scenario == 'physical':
      if 410 <= tick < 415:
        fields['gear'] = 32  # Park, before the existing Neutral and Reverse samples.
      if 475 <= tick < 480:
        fields.update(main=False, cruise=False)  # Deliberate fresh physical recovery.
    if fault == 'configured-brake' and 260 <= tick < 311:
      fields.update(brake=True, cruise=False, stopped=False)
    if fault in ('eps-temporary', 'eps-permanent') and 260 <= tick < 311:
      fields['eps'] = 9 if fault == 'eps-temporary' else 3
    frames = helpers.physical(packer, **fields)
    if scenario == 'physical' and 470 <= tick < 475:
      frames = [f for f in frames if f[0] != 0x620]
      frames.append(packer.make_can_msg('BODY_CONTROL_STATE', 0, {'DOOR_OPEN_FL': 1, 'SEATBELT_DRIVER_UNLATCHED': 0}))
    frames = [f for f in frames if f[0] != 0x2FF]
    frames = [f for f in frames if f[0] != 0x412]
    frames.append(packer.make_can_msg('LKAS_HUD', 2, {'LDA_ON_MESSAGE': 1 if 220 <= tick < 225 else 0}))
    frames += [(0x2FF, bytes.fromhex('80000409' if 220 <= tick < 240 else '00000409'), 0),
               packer.make_can_msg('PCS_HUD', 0, {})]
    if alternate:
      frames += [packer.make_can_msg('DSU_CRUISE', 0, {'MAIN_ON': fields.get('main', True), 'SET_SPEED': 90}),
                 packer.make_can_msg('PCM_CRUISE_ALT', 0, {'UI_SET_SPEED': 90})]
    if fault is not None and 260 <= tick < 311:
      if fault == 'aeb':
        frames = [f for f in frames if f[0] != 0x283]
        aeb = packer.make_can_msg('PRE_COLLISION', 0, {'PRECOLLISION_ACTIVE': 1, 'FORCE': -20})
        assert int.from_bytes(aeb[1][2:4], 'big', signed=True) < 0 and aeb[1][5] & 2
        frames.append(aeb)
      elif fault in ('configured-brake', 'eps-temporary', 'eps-permanent') or fault.startswith('native-'):
        pass  # Wire corruption below leaves all physical CAN sources unchanged.
      else:
        source, kind = fault.split('-')
        address = (0x365 if alternate else 0x1D3) if source == 'main' else 0x262
        kept = []
        for a, data, bus in frames:
          if a == address and bus == 0:
            if kind == 'drop':
              continue
            if kind == 'length':
              data = data[:-1]
            elif kind == 'checksum':
              assert address != 0x365, 'DSU MAIN has no checksum'
              data = data[:-1] + bytes([data[-1] ^ 1])
            else:
              assert kind == 'bus'
              bus = 2
          kept.append((a, data, bus))
        frames = kept
    return frames

  def parsed_state(tick, host, cs):
    if fault == 'aeb' and 260 <= tick < 311:
      assert cs.stockAeb, 'the actual parser must see representable physical AEB before daemon consumers run'
      assert host.CI.can_parsers[Bus.pt].vl['PRE_COLLISION']['FORCE'] < 0

  def observe(tick, host, cs, cc, safety, native):
    del native
    parser = host.CI.can_parsers[Bus.pt]
    assert isinstance(parser, legacy_advert_filter.LegacyAdvertCANParser)
    assert 0x2FF not in parser.message_states
    assert host.CP.safetyConfigs[0].safetyParam == legacy_advert_filter.PROFILES[candidate][stock]
    if scenario == 'physical' and (410 <= tick < 430 or 470 <= tick < 475):
      assert host.CI.CC.last_torque == 0, 'fresh physical source veto must beat a cached axis acknowledgement'
      if not stock:
        assert host.CI.CC.standstill_req, 'inactive physical owner must withdraw standstill release'
    if tick == 220:
      from opendbc.car import structs
      assert any(e.type == structs.CarState.ButtonEvent.Type.lkas and e.pressed for e in cs.buttonEvents)
      assert any(e.type == structs.CarState.ButtonEvent.Type.lkas and not e.pressed for e in cs.buttonEvents)
    if 225 <= tick < 235:
      assert host.CI.CS.distance_button == 1
    if fault is not None and 305 <= tick < 309:
      assert not cc.latActive and not cc.longActive
      if fault == 'aeb':
        assert cs.stockAeb and not cs.accFaulted
      else:
        assert safety.aol_get_permission_mask() == 0
    assert safety.safety_fwd_hook(2, 0x343) == 0

  def native_rx_expected(address, data, bus, default, actual_cp):
    if fault == 'eps-checksum' and 260 <= tick < 311 and address == 0x262 and bus == 0:
      from opendbc.car.toyota.toyotacan import toyota_checksum
      assert legacy_advert_filter.configured(actual_cp) and actual_cp.carFingerprint == candidate
      assert len(data) == 5 and data[-1] != toyota_checksum(address, None, bytearray(data))
      return False  # A required source with a bad checksum must be rejected.
    return default

  original_native = helpers.native_events

  def native_events(*args, **kwargs):
    messages = original_native(*args, **kwargs)
    if fault in ('native-identity', 'native-session') and 260 <= tick < 311:
      original_wire = next(e for e in messages if e.which() == 'aolSafetyWire')
      reader = original_wire.as_reader() if hasattr(original_wire, 'as_reader') else original_wire
      wire = reader.as_builder()
      state = decode_safety(bytes(wire.aolSafetyWire))
      assert state is not None
      # USB producer boundary: alter identity/session only; retain every real
      # native permission/request/health field and all physical CAN producers.
      wrong = replace(state, safetyParam=state.safetyParam ^ 1) if fault == 'native-identity' else replace(state, axisSessionId='f' * 32)
      wire.aolSafetyWire = encode_safety(wrong)
      messages = [wire.as_reader() if e.which() == 'aolSafetyWire' else e for e in messages]
    return messages

  with patch.object(helpers, 'native_events', native_events):
    helpers.run_case(stock, scenario, candidate=candidate, bridge=bridge, input_clock=True,
      aol=aol, sdsu_words=legacy_advert_filter.PROFILES[candidate], alpha_long=False,
      physical_frames=physical, filter_advert=True, on_sample=observe, brake_stop_check_start=355,
      brake_pause_mps=25.0 if fault == 'configured-brake' else None, native_rx_expected=native_rx_expected, on_state=parsed_state,
      firmware_override=[] if candidate == CAR.TOYOTA_MATRIX_RETROFIT else None,
      manual_selection=candidate == CAR.TOYOTA_MATRIX_RETROFIT,
      manual_filter_length=4 if candidate == CAR.TOYOTA_MATRIX_RETROFIT else None,
      fingerprint_frames=[(0x2FF, bytes(4), 0)] if candidate == CAR.TOYOTA_MATRIX_RETROFIT else None)



@pytest.mark.parametrize('candidate', sorted(legacy_advert_filter.PROFILES))
@pytest.mark.parametrize('stock', (False, True))
def test_each_legacy_four_byte_factory_caller(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('scenario', ('physical', 'off', 'cold_low'))
def test_legacy_physical_saved_off_cold(candidate, stock, scenario, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, scenario)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize('fault', ('main-drop', 'main-length', 'main-bus', 'eps-drop', 'eps-length', 'eps-checksum', 'eps-bus', 'aeb'))
def test_required_can_sources_remain_independent(candidate, fault, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'main_lease', fault=fault)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
def test_ordinary_cruise_without_aol(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, aol=False)


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_RAV4H, CAR.LEXUS_IS))
@pytest.mark.parametrize('fault', ('native-identity', 'native-session'))
def test_native_wire_identity_and_session_refusal(candidate, fault, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'main_lease', fault=fault)


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_RAV4H, CAR.LEXUS_IS))
@pytest.mark.parametrize('fault', ('eps-temporary', 'eps-permanent'))
def test_eps_faults_use_real_status_frames(candidate, fault, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'main_lease', fault=fault)


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_RAV4H, CAR.TOYOTA_COROLLA))
def test_configured_moving_brake_pause_is_not_bypassed(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'main_lease', fault='configured-brake')
