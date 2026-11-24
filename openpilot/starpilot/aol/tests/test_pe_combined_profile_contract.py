"""PE Alpha LONG must receive its own paired AOL contract, never EV9 admission."""
import pytest
from opendbc.car import structs

from opendbc.car.hyundai.ioniq5pe_long_profile import candidate, qualified
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from openpilot.starpilot.car.hyundai.aol import policy_for, native_accepts_cp


def test_pe_alpha_long_retains_requested_lateral_capability():
  cp = candidate(params(), enabled=True, is_release=False)
  assert cp is not None and qualified(cp)
  assert cp.safetyConfigs[0].safetyParam == 0x5495 and cp.alternativeExperience == 0
  policy = policy_for(cp)
  assert policy.settings_supported and policy.intent_supported and policy.runtime_supported


def test_pe_long_controller_profile_survives_its_own_aol_marker():
  cp = candidate(params(), enabled=True, is_release=False)
  assert cp is not None and qualified(cp)
  cp.alternativeExperience = 32
  assert qualified(cp)


def test_pe_must_never_borrow_ev9_native_word():
  cp = candidate(params(), enabled=True, is_release=False)
  assert cp is not None and qualified(cp)
  cp.alternativeExperience = 32
  assert not native_accepts_cp(cp, 28, 0x5c95)


@pytest.mark.parametrize('case', ['gas', 'brake', 'temporary_eps', 'permanent_eps', 'can_invalid', 'timeout',
                                  'unknown_gear', 'nonfinite', 'ev9_word', 'unmarked'])
def test_pe_temporary_restrictions_remain_exact_and_healthy(case):
  from opendbc.car.hyundai.pe_long_aol import temporary_restriction
  cp = candidate(params(), enabled=True, is_release=False)
  assert cp is not None and qualified(cp)
  cp.alternativeExperience = 32
  state = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive, vEgoRaw=20., gasPressed=True)
  if case == 'brake':
    state.gasPressed, state.brakePressed = False, True
  elif case == 'temporary_eps':
    state.gasPressed, state.steerFaultTemporary = False, True
  elif case == 'permanent_eps':
    state.steerFaultPermanent = True
  elif case == 'can_invalid':
    state.canValid = False
  elif case == 'timeout':
    state.canTimeout = True
  elif case == 'unknown_gear':
    state.gearShifter = structs.CarState.GearShifter.unknown
  elif case == 'nonfinite':
    state.vEgoRaw = float('nan')
  elif case == 'ev9_word':
    cp.safetyConfigs[0].safetyParam = 0x5c95
  elif case == 'unmarked':
    cp.alternativeExperience = 0
  assert temporary_restriction(cp, state) == (case in ('gas', 'brake', 'temporary_eps'))


@pytest.mark.parametrize('case', ['recovery', 'physical_main_off', 'unobserved', 'future_receipt', 'expired', 'newer_event',
                                  'soft_fault', 'immediate_fault', 'other_user_event', 'can_invalid', 'permanent_eps',
                                  'wrong_session', 'wrong_producer', 'long_granted', 'no_long_request', 'no_native',
                                  'wrong_inventory', 'unmarked'])
def test_pe_main_recovery_requires_original_healthy_receipt(monkeypatch, case):
  from types import SimpleNamespace as NS
  from dataclasses import replace
  from openpilot.cereal import log
  from openpilot.starpilot.aol import transport_pause as module
  from openpilot.starpilot.aol.wire import SafetyState, PandaSlot
  cp = candidate(params(), enabled=True, is_release=False)
  assert cp is not None
  cp.alternativeExperience = 32
  if case == 'unmarked':
    cp.alternativeExperience = 0
  now = 1_000_000_000
  state = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive)
  state.cruiseState.available = case != 'physical_main_off'
  event = NS(name=log.OnroadEvent.EventName.wrongCarMode, immediateDisable=False, softDisable=False, userDisable=True)
  events = [event]
  class SM(dict):
    seen = valid = alive = {'onroadEvents': True}
    logMonoTime = {'onroadEvents': now - 10_000_000}
  sm = SM(onroadEvents=events, pandaStates=[NS(controlsAllowed=False)])
  feedback = module.TransportPauseFeedback(cp)
  feedback.session, feedback.availability_producer, feedback.availability_epoch = 'session', 'producer', 4
  feedback.ccnc_main_off_source_ns = now - 10_000_000
  feedback.lateral_baseline_ns = feedback.lateral_native_ns = feedback.lateral_source_ns = now - 1_000_000
  feedback.lateral_inventory = ('panda', ('panda',))
  axis = NS(sessionId='session', intentProducerSessionId='producer', intentIntegrityEpoch=4,
    faultReason='none', mode='lateralOnly', nativeAcknowledged=True, desiredLateral=True,
    lateralActive=True, longitudinalActive=False, sourceCarStateMonoTime=now - 1_000_000, observedMonoTime=now)
  slot = PandaSlot(0, 'panda', 28, 0x5495, 32, False, False)
  native = SafetyState(1, True, now, now + 30_000_000, 28, 0x5495, True, False, True, True,
    'panda', 'session', now, (slot,))
  if case == 'unobserved':
    feedback.ccnc_main_off_source_ns = 0
  elif case == 'future_receipt':
    feedback.ccnc_main_off_source_ns = now + 1
  elif case == 'expired':
    feedback.ccnc_main_off_source_ns = now - 30_000_001
    sm.logMonoTime = {'onroadEvents': now - 30_000_001}
  elif case == 'newer_event':
    sm.logMonoTime = {'onroadEvents': now - 9_000_000}
  elif case == 'soft_fault':
    event.softDisable = True
  elif case == 'immediate_fault':
    event.immediateDisable = True
  elif case == 'other_user_event':
    event.name = log.OnroadEvent.EventName.buttonCancel
  elif case == 'can_invalid':
    state.canValid = False
  elif case == 'permanent_eps':
    state.steerFaultPermanent = True
  elif case == 'wrong_session':
    axis.sessionId = 'other'
  elif case == 'wrong_producer':
    axis.intentProducerSessionId = 'other'
  elif case == 'long_granted':
    native = replace(native, longitudinalAllowed=True)
  elif case == 'no_long_request':
    native = replace(native, requestedLongitudinal=False)
  elif case == 'no_native':
    native = None
  elif case == 'wrong_inventory':
    feedback.lateral_inventory = ('other', ('other',))
  monkeypatch.setattr(module, 'independent_lateral_health', lambda *a: native)
  before = feedback.ccnc_main_off_source_ns
  result = feedback._native_long_withdrawal_lateral_current(sm, cp, state, axis, now, latched=True, producer='producer')
  assert (result is not None) == (case in ('recovery', 'physical_main_off'))
  assert feedback.ccnc_main_off_source_ns == before
