"""Factory, Card, SelfdriveD, Controls, CI, C++ heartbeat and native bypass."""

import pytest
from opendbc.car.toyota import dsu_bypass
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_dsu_bypass import acquired, fingerprint
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope
REPRESENTATIVES = (CAR.TOYOTA_RAV4H, CAR.LEXUS_ES, CAR.TOYOTA_CAMRY, CAR.TOYOTA_PRIUS, CAR.TOYOTA_COROLLA, CAR.TOYOTA_PRIUS_V, CAR.LEXUS_IS, CAR.LEXUS_GS_F)


def run(candidate, stock, bridge, scenario='nominal', aol=True):
  def physical(packer, **fields):
    fields['sdsu'] = None
    frames = helpers.physical(packer, **fields)
    if candidate in dsu_bypass.ALT_MAIN_CARS:
      frames += [
        packer.make_can_msg('DSU_CRUISE', 0, {'MAIN_ON': fields.get('main', True), 'SET_SPEED': 90}),
        packer.make_can_msg('PCM_CRUISE_ALT', 0, {'UI_SET_SPEED': 90}),
      ]
    frames += [packer.make_can_msg('ACC_CONTROL', 2, {'ACC_TYPE': 1}), packer.make_can_msg('PRE_COLLISION', 2, {}), packer.make_can_msg('PCS_HUD', 2, {})]
    return frames

  def observe(tick, host, cs, cc, safety, native):
    del tick, cs, cc, native
    assert dsu_bypass.configured(host.CP)
    assert safety.safety_fwd_hook(2, 0x343) == (0 if stock else -1)
    assert safety.safety_fwd_hook(2, 0x283) == 0

  fp = [(a, bytes(length), bus) for bus, messages in fingerprint(candidate).items() for a, length in messages.items()]
  manual = candidate in (CAR.TOYOTA_MATRIX_RETROFIT, CAR.TOYOTA_PRIUS_RETROFIT)
  helpers.run_case(
    stock,
    scenario,
    candidate=candidate,
    bridge=bridge,
    input_clock=True,
    sdsu_words=(dsu_bypass.PROFILES[candidate], dsu_bypass.PROFILES[candidate] | 0x200),
    physical_frames=physical,
    on_sample=observe,
    firmware_override=acquired(candidate),
    fingerprint_frames=fp,
    manual_selection=manual,
    manual_bypass=manual,
    bypass_owner=True,
    aol=aol,
    brake_stop_check_start=355,
  )


@pytest.mark.parametrize('candidate', sorted(dsu_bypass.PROFILES))
@pytest.mark.parametrize('stock', (False, True))
def test_each_bypass_identity_full_caller(candidate, stock, host_heartbeat_bridge):
  run(candidate, stock, host_heartbeat_bridge)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize('scenario', ('physical', 'off', 'ordinary'))
def test_profile_physical_saved_off_and_ordinary_paths(candidate, scenario, host_heartbeat_bridge):
  run(candidate, False, host_heartbeat_bridge, 'nominal' if scenario == 'ordinary' else scenario, aol=scenario != 'ordinary')
