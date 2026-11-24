import pytest
from opendbc.car import structs
from openpilot.starpilot.aol.wire import decode_safety
from opendbc.car.toyota import legacy_sdsu
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


def run(candidate, stock, bridge, scenario='nominal', aol=True, alpha=False, hybrid=False, main_fault=None, camera_messages=False):
  tick = 0

  def physical(packer, **fields):
    nonlocal tick
    tick += 1
    frames = helpers.physical(packer, **fields)
    frames.append(packer.make_can_msg('PCS_HUD', 0, {}))
    if candidate in legacy_sdsu.ALT_MAIN_CARS:
      main = packer.make_can_msg('DSU_CRUISE', 0, {'MAIN_ON': fields.get('main', True), 'SET_SPEED': 90})
      if main_fault is not None and 260 <= tick < 311:
        if main_fault == 'length':
          frames.append((main[0], main[1][:6], main[2]))
        elif main_fault == 'bus':
          frames.append((main[0], main[1], 2))
        else:
          assert main_fault == 'drop'
      else:
        frames.append(main)
      frames.append(packer.make_can_msg('PCM_CRUISE_ALT', 0, {'UI_SET_SPEED': 90}))
    return frames

  def observe(tick, host, cs, cc, safety, native_messages):
    if scenario == 'physical' and 340 <= tick < 355:
      feedback = decode_safety(bytes(native_messages[0].aolSafetyWire))
      assert feedback is not None
      if not feedback.requestedLateral or not feedback.lateralAllowed:
        assert not cc.latActive, 'LAT must wait for its real native acknowledgement'
    if main_fault is not None and 301 <= tick < 308:
      assert not cc.latActive and not cc.longActive
      assert not cs.cruiseState.available
      assert safety.aol_get_permission_mask() == 0

  fw = helpers.firmware(candidate)
  if hybrid and candidate == CAR.TOYOTA_MATRIX_RETROFIT:
    from opendbc.car.toyota.interface import CarInterface
    cp = CarInterface.get_params(candidate, {0: {0x2FF: 8}, 1: {}, 2: {}},
      [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.hybrid, address=0x7E2,
        fwVersion=b"synthetic hybrid ECU response", brand="toyota", bus=0, logging=True)], False, False, False)
    assert not legacy_sdsu.configured(cp)
    return
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
    sdsu_words=legacy_sdsu.PROFILES[candidate],
    alpha_long=alpha,
    physical_frames=physical,
    brake_stop_check_start=355,
    firmware_override=fw,
    assert_hybrid_release=hybrid,
    on_sample=observe,
    camera_messages=camera_messages,
    manual_selection=candidate == CAR.TOYOTA_MATRIX_RETROFIT,
    manual_filter_length=8 if candidate == CAR.TOYOTA_MATRIX_RETROFIT else None,
    fingerprint_frames=[(0x2FF, bytes(8), 0)] if candidate == CAR.TOYOTA_MATRIX_RETROFIT else None,
  )


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.PROFILES))
@pytest.mark.parametrize('stock', [False, True])
def test_each_legacy_sdsu_actual_callers(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_COROLLA, CAR.TOYOTA_RAV4, CAR.LEXUS_CTH, CAR.LEXUS_GS_F, CAR.LEXUS_IS])
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize('scenario', ['physical', 'faults', 'bad_dlc', 'off'])
def test_legacy_sdsu_faults_physical_and_saved_off(candidate, stock, scenario, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, scenario)


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.PROFILES))
@pytest.mark.parametrize('stock', [False, True])
def test_legacy_ordinary_cruise_without_aol(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, aol=False)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_COROLLA, CAR.LEXUS_CTH, CAR.LEXUS_GS_F])
@pytest.mark.parametrize('stock', [False, True])
def test_legacy_alpha_does_not_replace_nongated_owner(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, alpha=True)


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.PROFILES))
def test_legacy_hybrid_active_resume(candidate, host_heartbeat_bridge):
  # CT supports active stop/start. Other models retain their original minimum
  # engagement speed; test their hybrid command behavior above that speed.
  scenario = "hybrid_stop" if candidate == CAR.LEXUS_CTH else "nominal"
  run(candidate, False, host_heartbeat_bridge, scenario, hybrid=True)


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.ALT_MAIN_CARS))
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize('bad', ['drop', 'length', 'bus'])
def test_dsu_main_and_status_are_independent_sources(candidate, stock, bad, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, 'main_lease', main_fault=bad)


@pytest.mark.parametrize('candidate', sorted(legacy_sdsu.PROFILES))
@pytest.mark.parametrize('stock', [False, True])
def test_legacy_lateral_below_stock_engagement_speed(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, 'cold_low')
