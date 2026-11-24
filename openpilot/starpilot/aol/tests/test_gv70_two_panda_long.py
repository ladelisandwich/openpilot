"""Factory owner selection stays exact when first-generation GV70 LONG uses slot1."""
import pytest
from opendbc.car import structs
from opendbc.car.hyundai.canfd_owner import config_index
from opendbc.car.hyundai.canfd_stock_aol import qualified_long
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.starpilot.car.hyundai.aol import native_accepts_cp, policy_for
from openpilot.starpilot.tests.test_captured_adrv_diagnostic_startup import factory


@pytest.mark.parametrize('pandas', [1, 2])
def test_selected_gv70_long_owner_preserves_factory_configuration(pandas):
  cp, _, _ = factory(CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, offset=4 * (pandas - 1))
  original = cp.to_dict()
  assert qualified_long(cp) and config_index(cp) == pandas - 1
  policy = policy_for(cp)
  assert policy.explicit_latch and policy.safety_param_addition == 0x800
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  assert qualified_long(cp, marked_only=True) and config_index(cp) == pandas - 1
  assert native_accepts_cp(cp, int(structs.CarParams.SafetyModel.hyundaiCanfd), 0x2815)
  assert not native_accepts_cp(cp, int(structs.CarParams.SafetyModel.noOutput), 0)
  assert not native_accepts_cp(cp, int(structs.CarParams.SafetyModel.hyundaiCanfd), 0x895)
  marked = cp.to_dict()
  assert {k:v for k,v in marked.items() if k not in ('safetyConfigs', 'alternativeExperience')} == {
    k:v for k,v in original.items() if k not in ('safetyConfigs', 'alternativeExperience')}
  assert marked['safetyConfigs'][:-1] == original['safetyConfigs'][:-1]
  assert marked['alternativeExperience'] == 32


@pytest.mark.parametrize('fault', ['missing_damping', 'aux_word', 'aux_model', 'order', 'extra_slot', 'owner_model', 'owner_word',
                                  'experience', 'foreign_identity', 'angle', 'alt_lka', 'alt_buttons', 'camera',
                                  'ccnc', 'hybrid', 'passive', 'dashcam', 'not_car', 'pcm', 'missing_long', 'steer_type'])
def test_two_panda_gv70_long_refuses_unqualified_configuration(fault):
  cp, _, _ = factory(CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN)
  cp.safetyConfigs[-1].safetyParam = 0x2815
  cp.alternativeExperience = 32
  assert qualified_long(cp, marked_only=True)
  assert native_accepts_cp(cp, int(structs.CarParams.SafetyModel.hyundaiCanfd), 0x2815)
  if fault == 'missing_damping':
    cp.safetyConfigs[-1].safetyParam = 0x815
  elif fault == 'aux_word':
    cp.safetyConfigs[0].safetyParam = 1
  elif fault == 'aux_model':
    cp.safetyConfigs[0].safetyModel = 'silent'
  elif fault == 'order':
    cp.safetyConfigs = list(reversed(cp.safetyConfigs))
  elif fault == 'extra_slot':
    cp.safetyConfigs = [*cp.safetyConfigs, structs.CarParams.SafetyConfig(safetyModel='noOutput')]
  elif fault == 'owner_model':
    cp.safetyConfigs[-1].safetyModel = 'hyundai'
  elif fault == 'owner_word':
    cp.safetyConfigs[-1].safetyParam = 0x2895
  elif fault == 'experience':
    cp.alternativeExperience = 0
  elif fault == 'foreign_identity':
    cp.carFingerprint = CAR.HYUNDAI_IONIQ_5
  elif fault in ('angle', 'alt_lka', 'alt_buttons', 'camera', 'ccnc', 'hybrid'):
    cp.flags = int(cp.flags) | int({'angle': HyundaiFlags.CANFD_ANGLE_STEERING, 'alt_lka': HyundaiFlags.CANFD_LKA_STEER_MSG_ALT,
                 'alt_buttons': HyundaiFlags.CANFD_ALT_BUTTONS, 'camera': HyundaiFlags.CANFD_CAMERA_SCC,
                 'ccnc': HyundaiFlags.CCNC, 'hybrid': HyundaiFlags.HYBRID}[fault])
  elif fault == 'passive':
    cp.passive = True
  elif fault == 'dashcam':
    cp.dashcamOnly = True
  elif fault == 'not_car':
    cp.notCar = True
  elif fault == 'pcm':
    cp.pcmCruise = True
  elif fault == 'missing_long':
    cp.openpilotLongitudinalControl = False
  elif fault == 'steer_type':
    cp.steerControlType = 'angle'
  assert not qualified_long(cp, marked_only=True)
  assert not native_accepts_cp(cp, int(structs.CarParams.SafetyModel.hyundaiCanfd), 0x2815)
