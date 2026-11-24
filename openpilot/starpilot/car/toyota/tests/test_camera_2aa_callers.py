"""Known camera2AA configurations through the actual serial caller/native chain."""

from contextlib import nullcontext
from dataclasses import replace
from unittest.mock import patch
import pytest
from openpilot.starpilot.aol.wire import decode_safety, encode_safety
from opendbc.car import Bus, structs
from opendbc.car.toyota import camera_2aa_filter
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


def run(candidate, stock, bridge, scenario='nominal', *, aol=True, fault=None, mirror=False):
  tick = 0

  def physical(packer, **fields):
    nonlocal tick
    tick += 1
    if scenario == 'physical':
      if 410 <= tick < 415:
        fields['gear'] = 32
      if 475 <= tick < 480:
        fields.update(main=False, cruise=False)
    if scenario == 'identity_refusal':
      fields.update(cruise=tick >= 150 and not 330 <= tick < 360, main=not 340 <= tick < 350)
    if fault in ('eps-temporary', 'eps-permanent') and 260 <= tick < 311:
      fields['eps'] = 9 if fault == 'eps-temporary' else 3
    if fault == 'configured-brake' and 260 <= tick < 311:
      fields.update(brake=True, cruise=False, stopped=False)
    frames = [f for f in helpers.physical(packer, **fields) if f[0] not in (0x260, 0x262, 0x2FF, 0x283, 0x412)]
    frames += [
      packer.make_can_msg('STEER_TORQUE_SENSOR', 0, {'STEER_ANGLE': 0.0573}),
      packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': fields.get('eps', 1), 'LTA_STATE': fields.get('eps', 1)}),
      packer.make_can_msg('LKAS_HUD', 2, {'LDA_ON_MESSAGE': 1 if 220 <= tick < 225 else 0}),
      packer.make_can_msg('PCS_HUD', 2, {}),
      packer.make_can_msg('PRE_COLLISION', 2, {}),
      (0x2AA, bytes.fromhex('8000700000000000' if 220 <= tick < 240 else '0000000000000000'), 0),
    ]
    if mirror:
      frames.append((0x2AA, bytes(8), 2))
    if scenario == 'physical' and 470 <= tick < 475:
      frames = [f for f in frames if f[0] != 0x620]
      frames.append(packer.make_can_msg('BODY_CONTROL_STATE', 0, {'DOOR_OPEN_FL': 1}))
    if fault == 'aeb' and 260 <= tick < 311:
      frames = [frame for frame in frames if frame[0] != 0x283]
      aeb = packer.make_can_msg('PRE_COLLISION', 2, {'PRECOLLISION_ACTIVE': 1, 'FORCE': -20})
      assert int.from_bytes(aeb[1][2:4], 'big', signed=True) < 0 and aeb[1][5] & 2
      frames.append(aeb)
    if fault is not None and fault.startswith('identity-') and tick == 260:
      markers = {
        'identity-status': (0x2FF, bytes.fromhex('0000700000000000'), 0),
        'identity-advert': (0x2FF, bytes.fromhex('80000409'), 0),
        'identity-pedal': (0x201, bytes(6), 0),
        'identity-dlc': (0x2AA, bytes(7), 0),
        'identity-bus': (0x2AA, bytes(8), 1),
      }
      frames.append(markers[fault])
    if fault is not None and fault.startswith('identity-') and tick == 275:
      # A genuine invalid-source reset must not erase contradictory identity.
      frames = [frame for frame in frames if frame[0] != 0x262]
      a, data, bus = packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': 1, 'LTA_STATE': 1})
      frames.append((a, data[:-1] + bytes([data[-1] ^ 1]), bus))
    if fault is not None and fault.split('-')[0] in ('main', 'eps') and fault not in ('eps-temporary', 'eps-permanent') and 260 <= tick < 311:
      source, kind = fault.split('-')
      address = 0x1D3 if source == 'main' else 0x262
      kept = []
      for a, data, bus in frames:
        if a == address and bus == 0:
          if kind == 'drop':
            continue
          if kind == 'length':
            data = data[:-1]
          elif kind == 'checksum':
            data = data[:-1] + bytes([data[-1] ^ 1])
          else:
            assert kind == 'bus'
            bus = 2
        kept.append((a, data, bus))
      frames = kept
    return frames

  def parsed_state(tick, host, cs):
    if fault == 'aeb' and 260 <= tick < 311:
      assert cs.stockAeb
      assert host.CI.can_parsers[Bus.cam].vl['PRE_COLLISION']['FORCE'] < 0

  def observe(tick, host, cs, cc, safety, native):
    del native
    parser = host.CI.can_parsers[Bus.pt]
    assert isinstance(parser, camera_2aa_filter.CameraTwoAACANParser)
    assert host.CP.safetyConfigs[0].safetyParam == (camera_2aa_filter.STOCK_WORD if stock else camera_2aa_filter.LONG_WORD)
    assert 0x2FF not in parser.message_states and 0x2AA not in parser.message_states
    assert parser.advert_distance_button == 0 and host.CI.CS.distance_button == 0
    assert not any(e.type == structs.CarState.ButtonEvent.Type.gapAdjustCruise for e in cs.buttonEvents)
    assert safety.safety_fwd_hook(2, 0x343) == 0 and safety.safety_fwd_hook(2, 0x4CB) == 0
    if tick == 220:
      assert any(e.type == structs.CarState.ButtonEvent.Type.lkas and e.pressed for e in cs.buttonEvents)
    if scenario == 'physical' and (410 <= tick < 430 or 470 <= tick < 475):
      assert host.CI.CC.last_torque == 0
      if not stock:
        assert host.CI.CC.standstill_req
    if fault is not None and 301 <= tick < 308:
      assert not cc.latActive and not cc.longActive and safety.aol_get_permission_mask() == 0
      if fault == 'aeb':
        assert cs.stockAeb and not cs.accFaulted
    if scenario == 'identity_refusal' and tick >= 270:
      assert parser.hardware_ambiguous and not parser.main_valid and not parser.eps_valid
      assert not cc.latActive and not cc.longActive and safety.aol_get_permission_mask() == 0
      if tick >= 390:
        assert safety.safety_config_valid(), 'all real required sources recovered, but identity must stay refused'

  def rx_expected(address, data, bus, default, cp):
    if fault is not None and fault.startswith('identity-') and tick == 275 and address == 0x262 and bus == 0:
      from opendbc.car.toyota.toyotacan import toyota_checksum

      assert camera_2aa_filter.configured(cp) and len(data) == 8 and data[-1] != toyota_checksum(address, None, bytearray(data))
      return False
    if fault == 'eps-checksum' and address == 0x262 and bus == 0 and 260 <= tick < 311:
      assert camera_2aa_filter.configured(cp) and len(data) == 8
      return False
    if fault == 'main-checksum' and address == 0x1D3 and bus == 0 and 260 <= tick < 311:
      assert camera_2aa_filter.configured(cp) and len(data) == 8
      return False
    return default

  original_native = helpers.native_events

  def native_events(*args, **kwargs):
    messages = original_native(*args, **kwargs)
    if fault in ('native-identity', 'native-session') and 260 <= tick < 311:
      original = next(event for event in messages if event.which() == 'aolSafetyWire')
      reader = original.as_reader() if hasattr(original, 'as_reader') else original
      wire = reader.as_builder()
      state = decode_safety(bytes(wire.aolSafetyWire))
      assert state is not None
      # USB producer boundary: change identity/session only, preserving all actual native bits.
      wrong = replace(state, safetyParam=state.safetyParam ^ 1) if fault == 'native-identity' else replace(state, axisSessionId='f' * 32)
      wire.aolSafetyWire = encode_safety(wrong)
      messages = [wire.as_reader() if event.which() == 'aolSafetyWire' else event for event in messages]
    return messages

  advertisements = [(0x2AA, bytes(8), 0)] + ([(0x2AA, bytes(8), 2)] if mirror else [])
  context = patch.object(helpers, 'native_events', native_events) if fault in ('native-identity', 'native-session') else nullcontext()
  with context:
    helpers.run_case(
      stock,
      scenario,
      candidate=candidate,
      bridge=bridge,
      input_clock=True,
      aol=aol,
      sdsu_words=(camera_2aa_filter.LONG_WORD, camera_2aa_filter.STOCK_WORD),
      physical_frames=physical,
      fingerprint_frames=advertisements,
      on_sample=observe,
      native_rx_expected=rx_expected,
      brake_stop_check_start=355,
      brake_pause_mps=25.0 if fault == 'configured-brake' else None,
      on_state=parsed_state,
    )


@pytest.mark.parametrize('candidate', sorted(camera_2aa_filter.CARS))
@pytest.mark.parametrize('stock', (False, True))
def test_each_camera_twoaa_factory_caller(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_COROLLA_TSS2, CAR.LEXUS_LC_TSS2))
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('scenario', ('physical', 'off', 'cold_low'))
def test_camera_twoaa_physical_off_cold(candidate, stock, scenario, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, scenario)


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_COROLLA_TSS2, CAR.LEXUS_LC_TSS2))
@pytest.mark.parametrize('fault', ('main-drop', 'main-length', 'main-checksum', 'main-bus', 'eps-drop', 'eps-length', 'eps-checksum', 'eps-bus'))
def test_camera_twoaa_source_refusals(candidate, fault, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'main_lease', fault=fault)


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_COROLLA_TSS2, CAR.LEXUS_LC_TSS2))
def test_camera_twoaa_ordinary_and_forwarded(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, aol=False, mirror=True)


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_COROLLA_TSS2, CAR.LEXUS_LC_TSS2))
@pytest.mark.parametrize('stock', (False, True))
def test_camera_twoaa_actual_camera_aeb(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, 'main_lease', fault='aeb')


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_COROLLA_TSS2, CAR.LEXUS_LC_TSS2))
@pytest.mark.parametrize('fault', ('eps-temporary', 'eps-permanent', 'native-identity', 'native-session', 'configured-brake'))
def test_camera_twoaa_native_session_eps_and_brake(candidate, fault, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'main_lease', fault=fault)


@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('fault', ('identity-status', 'identity-advert', 'identity-pedal', 'identity-dlc', 'identity-bus'))
def test_camera_twoaa_identity_survives_source_reset_and_physical_cycle(stock, fault, host_heartbeat_bridge):
  run(CAR.TOYOTA_COROLLA_TSS2, stock, host_heartbeat_bridge, 'identity_refusal', fault=fault)
