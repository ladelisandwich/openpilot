"""Actual legacy stop-latch callers; only CAN/FW/perception/USB producers are synthetic."""
from contextlib import ExitStack
from unittest.mock import patch

import pytest
from opendbc.car import structs
from opendbc.car.toyota import advert_filter, eps8_sdsu, legacy_sdsu
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope
CARS = (CAR.TOYOTA_CAMRY, CAR.TOYOTA_CHR, CAR.TOYOTA_AVALON_2019, CAR.LEXUS_LS, CAR.LEXUS_NX, CAR.LEXUS_RX, CAR.LEXUS_CTH)
ADVERT_CARS = (CAR.TOYOTA_CAMRY, CAR.TOYOTA_CHR, CAR.TOYOTA_AVALON_2019, CAR.LEXUS_LS)


def run(candidate, stock, bridge, *, advert=False, hybrid=False, scenario='hybrid_stop', aol=True):
  tick = 0
  samples = []
  latched = []
  decoded_acc = []

  def physical(packer, **fields):
    nonlocal tick
    tick += 1
    frames = helpers.physical(packer, **fields)
    if scenario == 'hybrid_stop' and 200 <= tick < 205:
      frames = [f for f in frames if f[0] != 0x1D2]
      frames.append(packer.make_can_msg('PCM_CRUISE', 0, {'CRUISE_ACTIVE': True, 'GAS_RELEASED': True, 'CRUISE_STATE': 8}))
    if candidate in eps8_sdsu.CARS:
      frames = [f for f in frames if f[0] not in (0x260, 0x262)]
      frames += [packer.make_can_msg('STEER_TORQUE_SENSOR', 0, {'STEER_ANGLE': .0573}),
                 packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': fields.get('eps', 1), 'LTA_STATE': fields.get('eps', 1)})]
    frames.append(packer.make_can_msg('PCS_HUD', 0, {}))
    if advert:
      frames = [f for f in frames if f[0] != 0x2FF]
      frames.append((0x2FF, bytes.fromhex('00000409'), 0))
    return frames

  def plan_inputs(original):
    def produce(*args, **kwargs):
      messages = original(*args, **kwargs)
      result = []
      for message in messages:
        if message.which() == 'longitudinalPlan' and 195 <= tick < 230:
          message = message.as_builder() if hasattr(message, 'as_builder') else message
          message.longitudinalPlan.aTarget = -.5
          message.longitudinalPlan.shouldStop = True
        result.append(message)
      return result
    return produce

  original_decode = helpers.decode

  def observe_decode(cp, frame, name):
    value = original_decode(cp, frame, name)
    if name == 'ACC_CONTROL':
      decoded_acc[:] = [(tick, value)]
    return value

  def observe(tick, host, cs, cc, safety, native):
    del native
    if scenario == 'hybrid_stop' and 200 <= tick < 205 and cs.standstill and cc.longActive:
      assert host.CI.CS.pcm_acc_status == 8 and host.CI.CC.standstill_req, 'Dom PCM stop latch must pulse'
      latched.append(tick)
    if scenario == 'hybrid_stop' and 215 <= tick < 228:
      assert cs.standstill and cs.cruiseState.standstill and host.CI.CS.pcm_acc_status == 7
      assert cc.longActive == (not stock)
      if not stock:
        assert cc.actuators.accel <= 0, 'perception must exercise nonpositive stop demand'
        assert not host.CI.CC.standstill_req, ('Dom releases after PCM latches stop', candidate, tick)
        assert decoded_acc and decoded_acc[0][1]['RELEASE_STANDSTILL'] == 1
        assert safety.aol_get_permission_mask() & 2
        samples.append(tick)

  fw = helpers.firmware(candidate)
  if hybrid:
    fw.append(structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.hybrid, address=0x7E2,
      fwVersion=b'synthetic hybrid ECU response', brand='toyota', bus=0, logging=True))
  words = ((advert_filter.LONG_WORD, advert_filter.STOCK_WORD) if advert else
           legacy_sdsu.PROFILES[candidate] if candidate in legacy_sdsu.PROFILES else
           eps8_sdsu.words(candidate) if candidate in eps8_sdsu.CARS else (0x9049, 0x9249))
  with ExitStack() as stack:
    stack.enter_context(patch.object(helpers, 'selfdrive_inputs', plan_inputs(helpers.selfdrive_inputs)))
    stack.enter_context(patch.object(helpers, 'control_inputs', plan_inputs(helpers.control_inputs)))
    stack.enter_context(patch.object(helpers, 'decode', observe_decode))
    helpers.run_case(stock, scenario, candidate=candidate, bridge=bridge, input_clock=True,
      aol=aol, sdsu_words=words, alpha_long=True, firmware_override=fw,
      physical_frames=physical, filter_advert=advert, on_sample=observe, brake_stop_check_start=355)
  if scenario == 'hybrid_stop' and not stock:
    assert latched and len(samples) == 13


@pytest.mark.parametrize('candidate', CARS)
def test_legacy_pcm_stop_latch(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', ADVERT_CARS)
def test_legacy_four_byte_pcm_stop_latch(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, advert=True)


@pytest.mark.parametrize('candidate', (CAR.LEXUS_CTH, CAR.TOYOTA_CAMRY))
def test_hybrid_pcm_latch(candidate, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, hybrid=True)


@pytest.mark.parametrize('candidate', (CAR.LEXUS_NX, CAR.TOYOTA_CAMRY))
def test_stock_pcm_stop_latch(candidate, host_heartbeat_bridge):
  run(candidate, True, host_heartbeat_bridge)


@pytest.mark.parametrize('scenario', ('cold_low', 'off', 'physical', 'faults'))
def test_legacy_inactive_source_and_physical_guards(scenario, host_heartbeat_bridge):
  run(CAR.LEXUS_NX, False, host_heartbeat_bridge, scenario=scenario)
