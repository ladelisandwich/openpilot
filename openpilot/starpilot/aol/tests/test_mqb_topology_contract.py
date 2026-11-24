"""Family CP and ordered native inventory contracts for the new topology contexts."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.stock_aol import MQB_CARS, safety_config
from openpilot.starpilot.aol.runtime import native_inventory_matches_cp
from openpilot.starpilot.aol.vehicle import policy_for, native_matches_cp
from openpilot.starpilot.aol.wire import PandaSlot
from openpilot.starpilot.car.volkswagen.aol import qualified


def params(identity, manual, alpha, offset, fixed):
  fp = gen_empty_fingerprint()
  if not manual:
    fp[offset][0xAD] = 8
  fp[offset + 1][0xFD] = 8
  cp = CarInterface.get_params(identity, fp, [], alpha, False, False)
  cp.fingerprintSource = structs.CarParams.FingerprintSource.fixed if fixed else structs.CarParams.FingerprintSource.fw
  return cp


@pytest.mark.parametrize('identity', sorted(MQB_CARS))
@pytest.mark.parametrize('manual', (False, True))
@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('offset,fixed', ((0, True), (4, False), (4, True)))
def test_exact_family_identity_topology_and_owner(identity, manual, alpha, offset, fixed):
  cp = params(identity, manual, alpha, offset, fixed)
  policy = policy_for(cp)
  assert policy.intent_supported
  owner = safety_config(cp)
  assert owner is not None and owner.safetyModel == structs.CarParams.SafetyModel.volkswagen
  owner.safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  word = (24 if manual else 8) | int(alpha)
  assert qualified(cp, marked_only=True) and native_matches_cp(cp, 15, word)
  assert cp.pcmCruise != alpha and cp.openpilotLongitudinalControl == alpha
  if offset:
    assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput
    assert cp.safetyConfigs[0].safetyParam == 0
  assert not native_matches_cp(cp, 15, word ^ 16)
  assert not native_matches_cp(cp, 34, word)


def test_exact_ordered_inventory_denies_incoherent_or_active_prefix():
  cp = params(sorted(MQB_CARS)[0], False, False, 4, True)
  policy = policy_for(cp)
  safety_config(cp).safetyParam |= policy.safety_param_addition
  cp.alternativeExperience = 32
  quiet = PandaSlot(0, 'quiet', int(structs.CarParams.SafetyModel.noOutput), 0, 32, False, False, False, 0)
  active = PandaSlot(1, 'owner', 15, 8, 32, True, False, False, 0)
  def receipt(slots, serial='owner', stamp=123):
    return SimpleNamespace(pandaInventory=slots, pandaSerial=serial, sourcePandaStatesMonoTime=stamp,
                           safetyModel=15, safetyParam=8)
  assert native_inventory_matches_cp(receipt((quiet, active)), cp)
  for slots in ((active, quiet), (quiet,), (quiet, active, active),
                (replace(quiet, controlsAllowed=True), active),
                (replace(quiet, safetyParam=8), active),
                (replace(quiet, hardwareSerial='owner'), active),
                (quiet, replace(active, heartbeatLost=True)),
                (quiet, replace(active, safetyRxChecksInvalid=True))):
    assert not native_inventory_matches_cp(receipt(slots), cp)
  assert not native_inventory_matches_cp(receipt((quiet, active), serial='quiet'), cp)
  assert not native_inventory_matches_cp(receipt((quiet, active), stamp=0), cp)
  assert not native_inventory_matches_cp(receipt((), stamp=0), cp)
