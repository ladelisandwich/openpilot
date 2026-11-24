import pytest
from opendbc.car.toyota import eps8_sdsu
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


def run(candidate, stock, bridge, scenario='nominal', aol=True, alpha=True, disable_setting=None, firmware=None,
        hybrid_release=False, sdsu_mirror=False, camera_messages=False):
  angle = candidate == CAR.TOYOTA_RAV4_TSS2_2023
  camera = bool(candidate.config.flags & ToyotaFlags.TSS2 and not candidate.config.flags & ToyotaFlags.RADAR_ACC)

  def physical(packer, **fields):
    frames = helpers.physical(packer, **fields)
    frames = [f for f in frames if f[0] not in (0x260, 0x262)]
    frames += [
      packer.make_can_msg('STEER_TORQUE_SENSOR', 0, {'STEER_ANGLE': 0.0573}),
      packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': fields.get('eps', 1), 'LTA_STATE': fields.get('eps', 1)}),
      packer.make_can_msg('PCS_HUD', 2 if camera else 0, {}),
    ]
    if camera:
      frames.append(packer.make_can_msg('PRE_COLLISION', 2, {}))
    return frames

  helpers.run_case(
    stock,
    scenario,
    candidate=candidate,
    bridge=bridge,
    input_clock=True,
    aol=aol,
    sdsu_words=eps8_sdsu.words(candidate),
    alpha_long=alpha,
    disable_setting=disable_setting,
    physical_frames=physical,
    angle=angle,
    brake_stop_check_start=355,
    firmware_override=firmware,
    assert_hybrid_release=hybrid_release,
    sdsu_mirror=sdsu_mirror,
    camera_messages=camera_messages,
  )


@pytest.mark.parametrize('candidate', sorted(eps8_sdsu.CARS))
@pytest.mark.parametrize('stock', [False, True])
def test_each_eps8_full_caller(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_CAMRY, CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_RAV4_TSS2_2023])
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize('scenario', ['physical', 'faults', 'bad_dlc', 'off'])
def test_eps8_fault_source_and_preference_callers(candidate, stock, scenario, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, scenario)


@pytest.mark.parametrize('candidate', [CAR.TOYOTA_CAMRY, CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_RAV4_TSS2_2023])
@pytest.mark.parametrize('stock', [False, True])
def test_eps8_ordinary_cruise_callers(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge, aol=False)


@pytest.mark.parametrize(
  'candidate', [CAR.TOYOTA_CAMRY, CAR.TOYOTA_CHR, CAR.TOYOTA_CHR_TSS2, CAR.TOYOTA_RAV4_TSS2_2022, CAR.TOYOTA_RAV4_TSS2_2023, CAR.TOYOTA_COROLLA_TSS2]
)
def test_eps8_alpha_off_actual_callers(candidate, host_heartbeat_bridge):
  stock = bool(candidate.config.flags & ToyotaFlags.RADAR_ACC or not candidate.config.flags & ToyotaFlags.TSS2)
  run(candidate, stock, host_heartbeat_bridge, alpha=False, disable_setting=False)


@pytest.mark.parametrize("candidate", [CAR.TOYOTA_CAMRY, CAR.TOYOTA_CAMRY_TSS2, CAR.TOYOTA_RAV4_TSS2_2023])
def test_detected_hybrid_preserves_active_resume(candidate, host_heartbeat_bridge):
  from opendbc.car import structs
  from opendbc.car.toyota.values import FW_QUERY_CONFIG

  # Firmware acquisition boundary: model a response from Toyota's real logging
  # hybrid query. Keep the catalog EPS/camera/radar identity used for enrollment.
  assert (structs.CarParams.Ecu.hybrid, 0x7E2, None) in FW_QUERY_CONFIG.extra_ecus
  fw = helpers.firmware(candidate)
  fw.append(
    structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.hybrid, address=0x7E2, fwVersion=b"synthetic hybrid ECU response", brand="toyota", bus=0, logging=True)
  )
  run(candidate, False, host_heartbeat_bridge, scenario="hybrid_stop", firmware=fw, hybrid_release=True)


@pytest.mark.parametrize("candidate", [CAR.TOYOTA_AVALON_2019, CAR.LEXUS_LS])
def test_nongated_legacy_sdsu_alpha_off(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, alpha=False, disable_setting=False)
