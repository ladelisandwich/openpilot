import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.rivian.stock_aol import qualified
from opendbc.car.rivian.values import CAR
from openpilot.starpilot.aol.vehicle import policy_for


@pytest.mark.parametrize('alpha', [False, True])
def test_actual_registered_gen1_owner(alpha):
  fp = gen_empty_fingerprint()
  fp[0][0x321] = 7
  cp = interfaces[CAR.RIVIAN_R1].get_params(CAR.RIVIAN_R1, fp, [], alpha, False, False)
  assert qualified(cp)
  assert policy_for(cp).intent_supported
  assert cp.openpilotLongitudinalControl == alpha
  assert cp.safetyConfigs[0].safetyParam == int(alpha)


@pytest.mark.parametrize('mutation', ['gen2', 'fixed', 'passive', 'not_car', 'dashcam', 'multi_panda', 'unknown_param', 'unknown_flag'])
def test_family_owner_rejects_unsupported_configuration(mutation):
  fp = gen_empty_fingerprint()
  if mutation != 'gen2':
    fp[0][0x321] = 7
  cp = interfaces[CAR.RIVIAN_R1].get_params(CAR.RIVIAN_R1, fp, [], False, False, False)
  if mutation == 'fixed':
    cp.fingerprintSource = structs.CarParams.FingerprintSource.fixed
  elif mutation == 'passive':
    cp.passive = True
  elif mutation == 'not_car':
    cp.notCar = True
  elif mutation == 'dashcam':
    cp.dashcamOnly = True
  elif mutation == 'multi_panda':
    cp.safetyConfigs = [cp.safetyConfigs[0], cp.safetyConfigs[0]]
  elif mutation == 'unknown_param':
    cp.safetyConfigs[0].safetyParam = 4
  elif mutation == 'unknown_flag':
    cp.flags = 2
  assert not qualified(cp)
  assert not policy_for(cp).intent_supported


@pytest.mark.parametrize('mutation', [
  'none', 'gen2', 'permanent', 'brake', 'regen', 'gas', 'door', 'belt', 'acc', 'sensors',
  'steering', 'button', 'enabled', 'unavailable', 'reverse', 'invalid',
])
def test_temporary_eps_registration_preserves_physical_disarms(mutation):
  from opendbc.car.rivian.stock_aol import temporary_eps_restriction

  fp = gen_empty_fingerprint()
  fp[0][0x321] = 7
  cp = interfaces[CAR.RIVIAN_R1].get_params(CAR.RIVIAN_R1, fp, [], False, False, False)
  cp.alternativeExperience = 32
  cs = structs.CarState(canValid=True, steerFaultTemporary=True, gearShifter=structs.CarState.GearShifter.drive,
                        cruiseState=structs.CarState.CruiseState(available=True))
  names = {'permanent': 'steerFaultPermanent', 'brake': 'brakePressed', 'regen': 'regenBraking',
           'gas': 'gasPressed', 'door': 'doorOpen', 'belt': 'seatbeltUnlatched', 'acc': 'accFaulted',
           'sensors': 'vehicleSensorsInvalid', 'steering': 'steeringDisengage'}
  if mutation in names:
    setattr(cs, names[mutation], True)
  elif mutation == 'gen2':
    cp.flags = 1
  elif mutation == 'button':
    cs.buttonEvents = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.cancel, pressed=True)]
  elif mutation == 'enabled':
    cs.cruiseState.enabled = True
  elif mutation == 'unavailable':
    cs.cruiseState.available = False
  elif mutation == 'reverse':
    cs.gearShifter = structs.CarState.GearShifter.reverse
  elif mutation == 'invalid':
    cs.canValid = False
  assert temporary_eps_restriction(cp, cs) == (mutation == 'none')
