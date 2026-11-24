"""ALT_RADAR stock AOL keeps actual factory ownership and exact MDX TJA semantics."""
import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, HondaSafetyFlags

IDENTITIES = (CAR.ACURA_RDX_3G_MMR, CAR.HONDA_ODYSSEY_5G_MMR, CAR.ACURA_TLX_2G, CAR.ACURA_MDX_4G)


def factory(identity, *, alt=False, release=False, alpha=False):
  fp = gen_empty_fingerprint()
  fp[1][0x1A3] = 8
  if alt:
    fp[1][0x1BE] = 3
  return CarInterface.get_params(identity, fp, [], alpha, release, False)


@pytest.mark.parametrize('identity', IDENTITIES)
@pytest.mark.parametrize('alt', (False, True))
@pytest.mark.parametrize('release', (False, True))
def test_alt_radar_stock_factory_and_native_marking(identity, alt, release):
  from opendbc.car.honda.stock_aol import qualified
  from openpilot.starpilot.car.honda.aol import native_accepts_cp, policy_for
  cp = factory(identity, alt=alt, release=release)
  if identity == CAR.ACURA_RDX_3G_MMR and release:
    assert cp.dashcamOnly and not qualified(cp)
    return
  assert qualified(cp)
  assert cp.pcmCruise and not cp.openpilotLongitudinalControl
  policy = policy_for(cp)
  assert policy.explicit_latch and policy.normal_runtime_supported
  expected = int(HondaSafetyFlags.STOCK_TJA) if identity == CAR.ACURA_MDX_4G else 0
  assert policy.safety_param_addition == expected
  base = cp.safetyConfigs[0].safetyParam
  cp.safetyConfigs[0].safetyParam |= expected
  cp.alternativeExperience = 32
  assert qualified(cp, marked_only=True)
  assert cp.safetyConfigs[0].safetyParam == base | expected
  assert native_accepts_cp(cp, 20, base | expected)
  bad = cp.as_reader().as_builder()
  bad.safetyConfigs[0].safetyParam ^= 512
  assert not qualified(bad)
  assert qualified(factory(identity, alt=alt, alpha=True))


@pytest.mark.parametrize('identity', IDENTITIES)
@pytest.mark.parametrize('alt', (False, True))
def test_actual_card_controls_and_native_sources_use_final_stock_cp(identity, alt):
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import honda_family_case
  from opendbc.safety.tests.libsafety import libsafety_py
  from opendbc.safety.tests.test_honda_stock_aol import TestHondaStockAol
  cp = factory(identity, alt=alt)
  with OpenpilotPrefix():
    case = honda_family_case('honda', False)
    case.family_cp = cp.as_reader()
    native = TestHondaStockAol(methodName='runTest')
    try:
      sd, _, drive, card, receive = case.family_callers(False)
      assert cp.pcmCruise and not card.CP.openpilotLongitudinalControl
      word = card.CP.safetyConfigs[0].safetyParam
      native.configure(('alt-radar', 20, word, 1, card.CI.CC.packer.dbc.name, False, 'GEARBOX_AUTO', 4))
      native.stream()
      native.normal_engage()
      native.press_brake()
      assert not native.safety.get_controls_allowed()
      permission = native.safety.aol_get_permission_mask()
      assert permission == 1
      axis, _, command = drive(2, allowed=bool(permission & 1), native_request_long=False,
                               panda_controls_allowed=bool(native.safety.get_controls_allowed()))
      assert axis.lateralActive and command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert receive(10_020_000_000, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      native.stream(eps=1)
      permission = native.safety.aol_get_permission_mask()
      assert permission == int(identity == CAR.ACURA_MDX_4G)
      if identity == CAR.ACURA_MDX_4G:
        # The configured current parser explicitly regards this status as healthy.
        from opendbc.can import CANPacker
        packer = CANPacker(card.CI.CC.packer.dbc.name)
        msg = packer.make_can_msg('STEER_STATUS', 1, {'STEER_STATUS': 1})
        card.CI.update([])
        parsed = card.CI.update([(10_030_000_000, [msg])])
        assert not parsed.steerFaultPermanent and not parsed.steerFaultTemporary
      axis, _, command = drive(3, allowed=bool(permission & 1), native_request_long=False,
                               panda_controls_allowed=bool(native.safety.get_controls_allowed()))
      assert bool(axis.lateralActive) == bool(permission & 1)
      assert not command.longActive
      native.stream(eps=7)
      native.stream(eps=0)
      assert native.safety.aol_get_permission_mask() == 0
    finally:
      libsafety_py.libsafety.set_alternative_experience(0)
      case.doCleanups()


@pytest.mark.parametrize('word', (512, 513))
def test_tja_marker_cannot_be_reused_by_other_host_identity(word):
  from opendbc.car.honda.stock_aol import qualified
  for identity in (CAR.ACURA_RDX_3G_MMR, CAR.HONDA_ODYSSEY_5G_MMR, CAR.ACURA_TLX_2G):
    cp = factory(identity)
    cp.alternativeExperience = 32
    cp.safetyConfigs[0].safetyParam = word
    assert not qualified(cp)
  cp = factory(CAR.ACURA_MDX_4G)
  cp.alternativeExperience = 32
  cp.safetyConfigs = [structs.CarParams.SafetyConfig(safetyModel='noOutput'), cp.safetyConfigs[0]]
  assert not qualified(cp)
