from dataclasses import replace

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.runtime import AxisDecision
from openpilot.starpilot.aol.wire import IntentState, PandaSlot, SafetyState
from openpilot.starpilot.car.mazda.aol import physical_cancel_lateral_allowed


def proof(identity):
  cp = CarInterface.get_params(identity, gen_empty_fingerprint(), [], False, False, False)
  cp.alternativeExperience = 32
  cs = structs.CarState(canValid=True, vEgo=20., gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = True
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.cancel, pressed=True)]
  settings = AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0))
  intent = IntentState('card-session', 1, 1_000_000_000, 1_000_000_000, 1_200_000_000,
                       True, False, False, True, True)
  native = SafetyState(1, True, 1_000_000_000, 1_200_000_000, 13, 0, True, False, True, False,
                       'mazda-panda', 'axis-session', 1_000_000_000,
                       (PandaSlot(0, 'mazda-panda', 13, 0, 32),))
  context = {'settings': settings, 'previous': AxisDecision(True, False, True, False, True),
             'source_ns': 1_000_000_000, 'now_ns': 1_010_000_000, 'axis_session_id': 'axis-session'}
  return cp, cs, intent, native, context


@pytest.mark.parametrize('identity', (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021))
def test_cancel_keeps_existing_native_lateral_and_handles_release(identity):
  cp, cs, intent, native, context = proof(identity)
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context)
  cs.buttonEvents[0].pressed = False
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context)


@pytest.mark.parametrize('failure', (
  'disabled', 'unarmed', 'paused',
  'intent_stale', 'source_mismatch', 'main_off', 'invalid_can', 'timeout', 'temporary_eps', 'permanent_eps',
  'acc_fault', 'no_cancel', 'native_missing', 'native_denied', 'native_unrequested', 'native_stale',
  'native_session', 'native_model', 'native_param', 'native_rx_fault', 'native_fault', 'no_inventory',
  'no_previous', 'previous_inactive', 'profile_unmarked', 'dashcam',
))
def test_cancel_exception_never_invents_intent_or_permission(failure):
  cp, cs, intent, native, context = proof(CAR.MAZDA_CX5_2022)
  if failure == 'disabled':
    context['settings'] = replace(context['settings'], enabled=False)
  elif failure == 'unarmed':
    intent = replace(intent, allowedLatch=False, lateralArmed=False)
  elif failure == 'paused':
    intent = replace(intent, pauseLateral=True)
  elif failure == 'intent_stale':
    intent = replace(intent, observedMonoTime=700_000_000, carStateLogMonoTime=700_000_000)
  elif failure == 'source_mismatch':
    context['source_ns'] -= 1
  elif failure == 'main_off':
    cs.cruiseState.available = False
  elif failure == 'invalid_can':
    cs.canValid = False
  elif failure == 'timeout':
    cs.canTimeout = True
  elif failure == 'temporary_eps':
    cs.steerFaultTemporary = True
  elif failure == 'permanent_eps':
    cs.steerFaultPermanent = True
  elif failure == 'acc_fault':
    cs.accFaulted = True
  elif failure == 'no_cancel':
    cs.buttonEvents = []
  elif failure == 'native_missing':
    native = None
  elif failure == 'native_denied':
    native = replace(native, lateralAllowed=False)
  elif failure == 'native_unrequested':
    native = replace(native, requestedLateral=False)
  elif failure == 'native_stale':
    native = replace(native, observedMonoTime=700_000_000, validUntilMonoTime=900_000_000)
  elif failure == 'native_session':
    native = replace(native, axisSessionId='other-axis')
  elif failure == 'native_model':
    native = replace(native, safetyModel=0)
  elif failure == 'native_param':
    native = replace(native, safetyParam=2)
  elif failure == 'native_rx_fault':
    native = replace(native, pandaInventory=(replace(native.pandaInventory[0], safetyRxChecksInvalid=True),))
  elif failure == 'native_fault':
    native = replace(native, pandaInventory=(replace(native.pandaInventory[0], faults=1),))
  elif failure == 'no_inventory':
    native = replace(native, pandaInventory=())
  elif failure == 'no_previous':
    context['previous'] = None
  elif failure == 'previous_inactive':
    context['previous'] = AxisDecision()
  elif failure == 'profile_unmarked':
    cp.alternativeExperience = 0
  elif failure == 'dashcam':
    cp.dashcamOnly = True
  assert not physical_cancel_lateral_allowed(cp, cs, intent, native, **context)


@pytest.mark.parametrize('action', (3, 4, 9))
@pytest.mark.parametrize('slot', range(3))
def test_mapped_cancel_uses_current_owner_intent(action, slot):
  cp, cs, intent, native, context = proof(CAR.MAZDA_CX5_2022)
  actions = [0, 0, 0]
  actions[slot] = action
  context['settings'] = replace(context['settings'], cancel_actions=tuple(actions))
  assert physical_cancel_lateral_allowed(cp, cs, intent, native, **context)
  assert not physical_cancel_lateral_allowed(cp, cs, replace(intent, pauseLateral=True), native, **context)
  assert not physical_cancel_lateral_allowed(cp, cs, replace(intent, allowedLatch=False, lateralArmed=False), native, **context)
