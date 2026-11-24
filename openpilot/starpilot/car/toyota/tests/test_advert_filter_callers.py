import pytest
from opendbc.car import Bus, structs
from opendbc.car.toyota import advert_filter
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope
REPRESENTATIVES = (CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_CAMRY_TSS2, CAR.TOYOTA_AVALON_2019, CAR.LEXUS_LS)


def run(candidate, stock, bridge, scenario='nominal', aol=True, alpha=False, hybrid=False, fault=None):
  tick = 0
  camera = bool(candidate.config.flags & ToyotaFlags.TSS2)

  def physical(packer, **fields):
    nonlocal tick
    tick += 1
    if fault in ('temporary', 'permanent') and 260 <= tick < 311:
      fields['eps'] = 9 if fault == 'temporary' else 3
    frames = [f for f in helpers.physical(packer, **fields) if f[0] not in (0x260, 0x262, 0x2FF)]
    frames += [
      packer.make_can_msg('STEER_TORQUE_SENSOR', 0, {'STEER_ANGLE': 0.0573}),
      packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': fields.get('eps', 1), 'LTA_STATE': fields.get('eps', 1)}),
      packer.make_can_msg('PCS_HUD', 2 if camera else 0, {}),
      (0x2FF, bytes.fromhex('80000409' if 220 <= tick < 240 else '00000409'), 0),
    ]
    if camera:
      frames.append(packer.make_can_msg('PRE_COLLISION', 2, {}))
    # Intercepted stock ACC_CONTROL is deliberately absent. The four-byte
    # advertisement's FD_BUTTON is informational; MAIN/EPS remain independent.
    if fault is not None and '-' in fault and 260 <= tick < 311:
      source, kind = fault.split('-')
      target = 0x1D3 if source == 'main' else 0x262
      kept = []
      for address, data, bus in frames:
        if address == target and bus == 0:
          if kind == 'drop':
            continue
          if kind == 'length':
            data = data[:7]
          elif kind == 'checksum':
            data = data[:-1] + bytes([data[-1] ^ 1])
          else:
            assert kind == 'bus'
            bus = 2
        kept.append((address, data, bus))
      frames = kept
    return frames

  def observe(tick, host, cs, cc, safety, native):
    parser = host.CI.can_parsers[Bus.pt]
    assert isinstance(parser, advert_filter.AdvertFilterCANParser)
    assert 0x2FF not in parser.message_states
    assert not host.CP.flags & ToyotaFlags.LONG_FILTER
    if 225 <= tick < 235:
      assert host.CI.CS.distance_button == 1
    if fault is not None and 301 <= tick < 308:
      assert not cc.latActive and not cc.longActive
      assert safety.aol_get_permission_mask() == 0
    assert safety.safety_fwd_hook(2, 0x343) == 0

  fw = helpers.firmware(candidate)
  if hybrid:
    fw.append(
      structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.hybrid, address=0x7E2, fwVersion=b'synthetic hybrid ECU response', brand='toyota', bus=0, logging=True)
    )
  helpers.run_case(
    stock,
    scenario,
    candidate=candidate,
    bridge=bridge,
    input_clock=True,
    aol=aol,
    alpha_long=alpha,
    sdsu_words=(advert_filter.LONG_WORD, advert_filter.STOCK_WORD),
    physical_frames=physical,
    firmware_override=fw,
    assert_hybrid_release=hybrid,
    filter_advert=True,
    brake_stop_check_start=355,
    on_sample=observe,
  )


@pytest.mark.parametrize('candidate', sorted(advert_filter.CARS))
@pytest.mark.parametrize('stock', [False, True])
def test_each_advertised_filter_actual_callers(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize('scenario', ['physical', 'off', 'cold_low'])
def test_physical_saved_off_and_cold_lateral(candidate, stock, scenario, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, scenario)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize('stock', [False, True])
def test_ordinary_cruise_without_aol(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, aol=False)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize('stock', [False, True])
def test_alpha_does_not_remove_default_long(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, alpha=True)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
def test_active_hybrid_resume(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'hybrid_stop', hybrid=True)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_CAMRY_TSS2, CAR.LEXUS_LS])
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize(
  'fault', ['main-drop', 'main-length', 'main-checksum', 'main-bus', 'eps-drop', 'eps-length', 'eps-checksum', 'eps-bus', 'temporary', 'permanent']
)
def test_required_sources_withdraw_and_recover_through_actual_can(candidate, stock, fault, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, 'main_lease', fault=fault)
