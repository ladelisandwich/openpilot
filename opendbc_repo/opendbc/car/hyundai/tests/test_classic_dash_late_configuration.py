"""Exercise Card's interface-before-AOL-configuration construction order."""
import pytest

from opendbc.car.hyundai.classic_scc_aol import aol_word
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.tests.test_classic_aol_dash import controller, step


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_SONATA_HYBRID, CAR.HYUNDAI_SONATA,
                                     CAR.HYUNDAI_ELANTRA_2021, CAR.HYUNDAI_KONA_EV])
def test_late_aol_configuration_updates_actual_encoded_dash(identity):
  ci, parser = controller(identity, marked=False)
  assert ci.CC.classic_dash_icons is None
  step(ci, parser)
  assert parser.vl['LFAHDA_MFC']['LFA_Icon_State'] == 0
  ci.CP.safetyConfigs[0].safetyParam = aol_word(ci.CP)
  ci.CP.alternativeExperience = 32
  original = ci.CP.to_dict()
  # This is the same shared CP object finalized by Card after CI construction.
  assert ci.CC.CP.safetyConfigs[0].safetyParam == ci.CP.safetyConfigs[0].safetyParam
  ci.CC.frame = 5
  actuators, packets = step(ci, parser, lateral=True)
  assert parser.vl['LKAS11']['CF_Lkas_FcwOpt_USM'] == 2
  assert parser.vl['LFAHDA_MFC']['LFA_Icon_State'] == 2
  assert actuators.torque == 0 and actuators.accel == 0
  assert not any(address in (0x420, 0x421) for address, _, _ in packets)
  for frame in range(6, 107):
    step(ci, parser)
    expected = 3 if frame < 105 else 0
    if frame % 5 == 0:
      assert parser.vl['LFAHDA_MFC']['LFA_Icon_State'] == expected
    assert parser.vl['LKAS11']['CF_Lkas_FcwOpt_USM'] == (expected or 1)
  assert ci.CP.to_dict() == original
