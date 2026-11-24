"""Draft actual full caller catalog for original Prius four-byte advertisements."""

from dataclasses import replace
import pytest
from opendbc.car import Bus
from opendbc.car.toyota import prius_advert_filter as owner
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.aol.wire import decode_safety, encode_safety
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_prius_sdsu_filter import original_firmware, ORIGINAL
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


def run(version, stock, bridge, scenario='nominal', *, aol=True, fault=None, mirror=False):
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
    frames = helpers.physical(packer, **fields)
    if scenario == 'physical' and 470 <= tick < 475:
      frames = [f for f in frames if f[0] != 0x620]
      frames.append(packer.make_can_msg('BODY_CONTROL_STATE', 0, {'DOOR_OPEN_FL': 1}))
    frames = [f for f in frames if f[0] != 0x2FF]
    frames += [(0x2FF, bytes.fromhex('80007409' if 220 <= tick < 240 else '00007409'), 0)]
    if mirror:
      frames.append((0x2FF, bytes.fromhex('80007409'), 2))
    if fault == 'aeb' and 260 <= tick < 311:
      frames = [f for f in frames if f[0] != 0x283]
      frames.append(packer.make_can_msg('PRE_COLLISION', 0, {'PRECOLLISION_ACTIVE': 1, 'FORCE': -20}))
    if fault is not None and fault.startswith('identity-') and tick == 260:
      markers = {
        'identity-advert': (0x2AA, bytes(8), 0),
        'identity-pedal': (0x201, bytes(6), 0),
        'identity-bus': (0x2FF, bytes(8), 1),
        'identity-status-advert': (0x2FF, bytes(8), 0),
        'identity-mirror-advert': (0x2FF, bytes(8), 2),
        'identity-length': (0x2FF, bytes(7), 0),
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

  def observe(tick, host, cs, cc, safety, native):
    del native
    parser = host.CI.can_parsers[Bus.pt]
    assert isinstance(parser, owner.PriusAdvertCANParser)
    assert 0x2FF not in parser.message_states and 'SDSU' not in parser.vl
    if tick == 220:
      assert host.CI.CS.distance_button == parser.advert_distance_button == 1
    if tick == 240:
      assert host.CI.CS.distance_button == parser.advert_distance_button == 0, 'Mirror button cannot replace primary release'
    if 200 <= tick < 250:
      assert not cs.stockAeb, 'Advertisement nibble7 is not SDSU STATE/AEB'
    assert host.CI.CS.eps_torque_scale == 0.66
    assert abs(host.CP.longitudinalActuatorDelay - 0.05) < 1e-6
    assert host.CP.safetyConfigs[0].safetyParam == (owner.STOCK_WORD if stock else owner.LONG_WORD)
    assert safety.safety_fwd_hook(2, 0x343) == 0 and safety.safety_fwd_hook(2, 0x4CB) == 0
    if scenario == 'physical' and (410 <= tick < 430 or 470 <= tick < 475):
      assert host.CI.CC.last_torque == 0
      if not stock:
        assert host.CI.CC.standstill_req
    if fault == 'aeb' and 270 <= tick < 300:
      assert cs.stockAeb and not cc.latActive and not cc.longActive
    if fault is not None and 301 <= tick < 308:
      assert not cc.latActive and not cc.longActive and safety.aol_get_permission_mask() == 0
    if scenario == 'identity_refusal' and tick >= 270:
      assert parser.hardware_ambiguous
      assert not cc.latActive and not cc.longActive and safety.aol_get_permission_mask() == 0
      if tick >= 390:
        assert safety.safety_config_valid(), 'all real required sources recovered, but identity must stay refused'

  def rx_expected(address, data, bus, default, cp):
    if fault is not None and fault.startswith('identity-') and tick == 275 and address == 0x262 and bus == 0:
      from opendbc.car.toyota.toyotacan import toyota_checksum

      assert owner.configured(cp) and len(data) == 8 and data[-1] != toyota_checksum(address, None, bytearray(data))
      return False
    if fault == 'eps-checksum' and address == 0x262 and bus == 0 and 260 <= tick < 311:
      assert owner.configured(cp) and len(data) == 8
      return False
    if fault == 'main-checksum' and address == 0x1D3 and bus == 0 and 260 <= tick < 311:
      assert owner.configured(cp) and len(data) == 8
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

  with pytest.MonkeyPatch.context() as monkeypatch:
    if fault in ('native-identity', 'native-session'):
      monkeypatch.setattr(helpers, 'native_events', native_events)
    helpers.run_case(
      stock,
      scenario,
      candidate=CAR.TOYOTA_PRIUS,
      firmware_override=original_firmware(version),
      bridge=bridge,
      input_clock=True,
      aol=aol,
      sdsu_words=(owner.LONG_WORD, owner.STOCK_WORD),
      physical_frames=physical,
      fingerprint_frames=[(0x2FF, bytes(4), 0)] + ([(0x2FF, bytes(4), 2)] if mirror else []),
      on_sample=observe,
      native_rx_expected=rx_expected,
      brake_stop_check_start=355,
      brake_pause_mps=25.0 if fault == 'configured-brake' else None,
    )


@pytest.mark.parametrize('version', ORIGINAL)
@pytest.mark.parametrize('stock', (False, True))
def test_original_prius_advert_all_known_firmware(version, stock, host_heartbeat_bridge):
  run(version, stock, host_heartbeat_bridge, mirror=True)


@pytest.mark.parametrize('version', (ORIGINAL[0], next(v for v in ORIGINAL if v.startswith(b'8965B47060'))))
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('scenario', ('physical', 'off', 'cold_low', 'ordinary_aol_off'))
def test_original_prius_advert_physical_and_preferences(version, stock, scenario, host_heartbeat_bridge):
  run(version, stock, host_heartbeat_bridge, 'nominal' if scenario == 'ordinary_aol_off' else scenario, aol=scenario != 'ordinary_aol_off', mirror=True)


@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize(
  'fault',
  (
    'main-drop',
    'main-length',
    'main-checksum',
    'main-bus',
    'eps-drop',
    'eps-length',
    'eps-checksum',
    'eps-bus',
    'eps-temporary',
    'eps-permanent',
    'native-identity',
    'native-session',
    'identity-advert',
    'identity-pedal',
    'identity-bus',
    'identity-status-advert',
    'identity-mirror-advert',
    'identity-length',
    'configured-brake',
    'aeb',
  ),
)
def test_original_prius_advert_actual_source_refusals(stock, fault, host_heartbeat_bridge):
  run(ORIGINAL[0], stock, host_heartbeat_bridge, 'identity_refusal' if fault.startswith('identity-') else 'main_lease', fault=fault, mirror=True)
