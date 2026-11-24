from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.cereal import log
from openpilot.starpilot.aol.panda_faults import blocking_fault_list, blocking_fault_mask


def recorded():
  fp = gen_empty_fingerprint()
  fp[0].update({0x391: 8, 0x38d: 8})
  fp[2][0x485] = 4
  fp[1][0x500] = 8
  cp = CarInterface.get_params(CAR.HYUNDAI_SONATA_HYBRID, fp, [], False, False, False)
  cp.safetyConfigs[0].safetyParam |= 0xC00
  cp.alternativeExperience = 32
  assert cp.flags == 0x1011580 and cp.safetyConfigs[0].safetyParam == 0xC02
  return cp


@pytest.mark.parametrize('mask', [1, 2, 4, 16, 32, 64, 256, 1 << 18, 9, 10, 12, 24, 0xffffffff, -1, 1 << 32, None, 'invalid'])
def test_other_mixed_or_malformed_faults_remain_blocking(mask):
  assert blocking_fault_mask(mask, recorded())


@pytest.mark.parametrize('field,value', [('brand', 'ford'), ('carFingerprint', CAR.HYUNDAI_SONATA),
  ('flags', 0x1011580 ^ 0x80), ('alternativeExperience', 0), ('alternativeExperience', 1),
  ('alternativeExperience', 33), ('passive', True), ('dashcamOnly', True), ('notCar', True),
  ('openpilotLongitudinalControl', True), ('pcmCruise', False)])
def test_unrecorded_configuration_does_not_inherit_diagnostic_classification(field, value):
  cp = recorded()
  setattr(cp, field, value)
  assert blocking_fault_mask(8, cp)


@pytest.mark.parametrize('word', [2, 0x402, 0xC00, 0xC01, 0xC04, 0xC0A, 0x840A])
def test_other_native_profiles_remain_blocking(word):
  cp = recorded()
  cp.safetyConfigs[0].safetyParam = word
  assert blocking_fault_mask(8, cp)


def test_raw_fault_list_and_carparams_are_retained():
  cp = recorded()
  before = cp.to_dict()
  faults = [log.PandaState.FaultType.interruptRateCan2]
  assert not blocking_fault_mask(0, cp)
  assert not blocking_fault_mask(8, cp)
  assert not blocking_fault_list(faults, cp)
  assert faults == [log.PandaState.FaultType.interruptRateCan2]
  for fault in (log.PandaState.FaultType.relayMalfunction, log.PandaState.FaultType.interruptRateCan1,
                log.PandaState.FaultType.registerDivergent):
    assert blocking_fault_list([*faults, fault], cp)
  assert cp.to_dict() == before


def test_extra_owner_and_legacy_model_remain_blocking():
  cp = recorded()
  cp.safetyConfigs[0].safetyModel = 'hyundaiLegacy'
  assert blocking_fault_mask(8, cp)
  cp = recorded()
  cp.safetyConfigs = [cp.safetyConfigs[0].to_dict(), cp.safetyConfigs[0].to_dict()]
  assert blocking_fault_mask(8, cp)


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_IONIQ_6, CAR.HYUNDAI_SONATA])
def test_unrelated_hyundai_skips_flag_construction_and_qualification(identity):
  cp = recorded()
  cp.carFingerprint = identity
  with patch.object(HyundaiFlags, '__or__', side_effect=AssertionError('unrelated flag work')), \
       patch('opendbc.car.hyundai.classic_scc_aol.qualified', side_effect=AssertionError('unrelated qualification')):
    assert blocking_fault_mask(8, cp)
