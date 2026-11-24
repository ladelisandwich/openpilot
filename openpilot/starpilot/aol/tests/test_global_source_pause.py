"""Shared source-pause intent contract for every registered make and Volt owner.

Native peer wires are controlled fixtures; no physical USB qualification is claimed.
"""
from dataclasses import replace
from functools import lru_cache

import pytest

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol import vehicle
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.runtime import decide_axes
from openpilot.starpilot.aol.tests.test_crossmake_native_admission import BRANDS, profile
from openpilot.starpilot.aol.tests.test_gm_held_native import Peers
from openpilot.starpilot.aol.tests.test_panda_inventory import bound_state
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.wire import encode_safety

NOW = 10_000_000_000
PRODUCER, SESSION = 'global-source-card', 'global-source-axis'
VOLTS = ('volt_cc', 'volt_gateway', 'volt_ascm_stock', 'volt_ascm_alpha', 'volt_sdgm_stock', 'volt_sdgm_alpha',
         'volt_camera_stock', 'volt_camera_alpha')
PROFILES = (*BRANDS, *VOLTS)


@pytest.fixture(autouse=True)
def namespace():
  with OpenpilotPrefix():
    yield


@lru_cache(maxsize=len(PROFILES))
def configured_bytes(identity):
  if identity in BRANDS:
    cp = profile(identity)
  else:
    from opendbc.car.gm.values import CAR
    if identity == 'volt_cc':
      from opendbc.car.gm.tests.test_volt_cc_f1 import params
      cp = params()
    elif identity in ('volt_gateway', 'volt_ascm_stock', 'volt_ascm_alpha'):
      from opendbc.car.gm.tests.test_bolt_volt_configurations import ordinary_params
      cp = ordinary_params(CAR.CHEVROLET_VOLT if identity == 'volt_gateway' else CAR.CHEVROLET_VOLT_ASCM,
                           alpha=identity.endswith('alpha'), sascm=identity != 'volt_gateway', radar=True)
    elif identity.startswith('volt_sdgm'):
      from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
      cp = sdgm_params(alpha=identity.endswith('alpha'), radar=True)
    else:
      from opendbc.car.gm.tests.test_volt_camera_control import camera_params
      cp = camera_params(alpha=identity.endswith('alpha'), radar=True)
    policy = vehicle.policy_for(cp)
    assert policy.intent_supported and not cp.dashcamOnly
    cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
    cp.alternativeExperience |= policy.alternative_experience_addition
  assert vehicle.native_matches_cp(cp, cp.safetyConfigs[-1].safetyModel.raw, cp.safetyConfigs[-1].safetyParam)
  return cp.as_reader().as_builder().to_bytes()


def parameters(identity):
  with structs.CarParams.from_bytes(configured_bytes(identity)) as cp:
    return cp.as_builder()


def state():
  cs = structs.CarState(canValid=True, gearShifter='drive', vEgo=20., vEgoRaw=20.)
  cs.cruiseState.available = True
  return cs


def sample(cp, now, sequence, *, phase='active', withdrawn=False, order='matched', warning='commIssue'):
  longitudinal = bool(cp.openpilotLongitudinalControl)
  native = bound_state(cp, now=now, session=SESSION)
  native = replace(native, requestedLongitudinal=longitudinal, longitudinalAllowed=longitudinal)
  if withdrawn:
    native = replace(native, requestedLateral=False, lateralAllowed=False, requestedLongitudinal=False, longitudinalAllowed=False)
  wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
  wire.aolSafetyWire = encode_safety(native)
  pandas = messaging.new_message('pandaStates', len(cp.safetyConfigs), valid=True, logMonoTime=native.sourcePandaStatesMonoTime)
  for panda, slot in zip(pandas.pandaStates, native.pandaInventory, strict=True):
    panda.safetyModel, panda.safetyParam = slot.safetyModel, slot.safetyParam
    panda.alternativeExperience, panda.controlsAllowed = slot.alternativeExperience, slot.controlsAllowed
  proof = messaging.new_message('aolAxisState', valid=True, logMonoTime=now)
  axis = proof.aolAxisState
  axis.sessionId = axis.faultSessionId = SESSION
  axis.intentProducerSessionId, axis.intentIntegrityEpoch = PRODUCER, 0
  axis.sequence, axis.qualified = sequence, True
  axis.observedMonoTime, axis.validUntilMonoTime = now, now + 30_000_000
  axis.sourceCarStateMonoTime = 0 if phase == 'pause' else now
  axis.desiredLateral = phase != 'pause'
  axis.desiredLongitudinal = longitudinal and phase != 'pause'
  axis.lateralActive = phase == 'active'
  axis.longitudinalActive = longitudinal and phase == 'active'
  axis.nativeAcknowledged = phase != 'pause'
  axis.mode = ('combined' if longitudinal else 'lateralOnly') if phase == 'active' else 'off'
  axis.faultReason = 'availabilityPause' if phase == 'pause' else 'none'
  axis.faultEventMonoTime = now if phase == 'pause' else NOW
  matched = phase == 'pause' and order == 'matched'
  event_stamp = now if matched else now + 1_000_000 if order == 'recovery_first' else NOW
  events = messaging.new_message('onroadEvents', int(matched), valid=True, logMonoTime=event_stamp)
  if matched:
    events.onroadEvents[0].name, events.onroadEvents[0].softDisable = warning, True
  return Peers([wire.as_reader(), pandas.as_reader(), proof.as_reader(), events.as_reader()]), native


def baseline(identity):
  cp, cs = parameters(identity), state()
  feedback = TransportPauseFeedback(cp)
  for tick, phase in ((0, 'active'), (10_000_000, 'inactive')):
    sm, _ = sample(cp, NOW + tick, tick + 1, phase=phase)
    assert not feedback.observe(sm, cp, cs, NOW + tick, latched=True, intent_producer_session=PRODUCER)
  assert feedback.ever_lateral_active and not feedback.lateral_only_baseline
  return cp, cs, feedback


def change(sm, service, update):
  event = sm.messages[service].as_builder()
  update(getattr(event, service))
  sm.messages[service] = event.as_reader()


@pytest.mark.parametrize('identity', PROFILES)
@pytest.mark.parametrize('withdrawn', [False, True])
@pytest.mark.parametrize('order', ['matched', 'axis_first', 'recovery_first'])
@pytest.mark.parametrize('warning', ['commIssue', 'commIssueAvgFreq', 'selfdrivedLagging'])
def test_every_make_retains_source_pause_intent_and_requires_current_output_grant(identity, withdrawn, order, warning):
  cp, cs, feedback = baseline(identity)
  now = NOW + 42_000_000
  sm, _ = sample(cp, now, 20_000_001, phase='pause', withdrawn=withdrawn, order=order, warning=warning)
  before = dict(vars(feedback))
  assert not feedback.observe(sm, cp, cs, now + 2_000_000, latched=True, intent_producer_session=PRODUCER)
  assert not feedback.failed
  for key in ('signature', 'sequence', 'healthy_feedback_ns', 'healthy_native_ns', 'lateral_source_ns'):
    assert vars(feedback)[key] == before[key]
  recovered, native = sample(cp, now + 10_000_000, 20_000_002, phase='inactive', withdrawn=True)
  assert not feedback.observe(recovered, cp, cs, now + 10_000_000, latched=True, intent_producer_session=PRODUCER)
  intent = type('Intent', (), {'allowedLatch': True, 'pauseLateral': False, 'pauseLongitudinal': False})()
  def decision(native):
    return decide_axes(standard_lateral=False, standard_longitudinal=False, intent=intent, native=native, car_state=cs,
                       initialized=True, model_ready=True, no_entry=False, immediate_disable=False,
                       dm_lockout=False, pause_brake_mps=0.)
  axes = decision(native)
  assert not axes.lateral_active and not axes.longitudinal_active
  granted, native = sample(cp, now + 20_000_000, 20_000_003, phase='active')
  assert not feedback.observe(granted, cp, cs, now + 20_000_000, latched=True, intent_producer_session=PRODUCER)
  assert decision(native).lateral_active


@pytest.mark.parametrize('identity', PROFILES)
@pytest.mark.parametrize('fault', ['source_expired', 'packet_expired', 'native_expired', 'native_denial', 'native_fault',
                                  'can', 'timeout', 'permanent_eps', 'button', 'main_off', 'producer', 'epoch',
                                  'session', 'cp', 'event_fault', 'event_cancel', 'combined_cancel', 'unarmed'])
def test_every_make_keeps_existing_fault_cancel_identity_and_stale_vetoes(identity, fault):
  cp, cs, feedback = baseline(identity)
  now = NOW + (210_000_001 if fault == 'source_expired' else 42_000_000)
  sm, native = sample(cp, now, 20_000_001, phase='pause')
  producer, latched = PRODUCER, fault != 'unarmed'
  if fault == 'packet_expired':
    now += 30_000_001
  elif fault in ('can', 'timeout', 'permanent_eps'):
    setattr(cs, {'can': 'canValid', 'timeout': 'canTimeout', 'permanent_eps': 'steerFaultPermanent'}[fault], fault != 'can')
  elif fault == 'main_off':
    cs.cruiseState.available = False
  elif fault == 'button':
    cs.buttonEvents = [{'type': 'cancel', 'pressed': True}]
  elif fault == 'producer':
    producer = 'different'
  elif fault in ('epoch', 'session'):
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch' if fault == 'epoch' else 'sessionId', 1 if fault == 'epoch' else 'different'))
  elif fault == 'cp':
    cp.flags ^= 1
  elif fault == 'combined_cancel':
    event = messaging.new_message('onroadEvents', 2, valid=True, logMonoTime=now)
    event.onroadEvents[0] = sm['onroadEvents'][0]
    event.onroadEvents[1].name, event.onroadEvents[1].userDisable = 'buttonCancel', True
    sm.messages['onroadEvents'] = event.as_reader()
  elif fault in ('event_fault', 'event_cancel'):
    def alter(events):
      events[0].name = 'controlsMismatch' if fault == 'event_fault' else 'buttonCancel'
      events[0].immediateDisable, events[0].userDisable = fault == 'event_fault', fault == 'event_cancel'
    change(sm, 'onroadEvents', alter)
  elif fault in ('native_expired', 'native_denial', 'native_fault'):
    updates = {'native_expired': {'validUntilMonoTime': now - 1}, 'native_denial': {'lateralAllowed': False},
               'native_fault': {'pandaInventory': (*native.pandaInventory[:-1], replace(native.pandaInventory[-1], heartbeatLost=True))}}[fault]
    event = sm.messages['aolSafetyWire'].as_builder()
    event.aolSafetyWire = encode_safety(replace(native, **updates))
    sm.messages['aolSafetyWire'] = event.as_reader()
  assert not feedback._inactive_source_pause_current(sm, cp, cs, sm['aolAxisState'], sm.logMonoTime['aolAxisState'], now,
                                                    latched=latched, producer=producer)


@pytest.mark.parametrize('identity', ('gm', *VOLTS))
@pytest.mark.parametrize('withdrawn', [False, True])
@pytest.mark.parametrize('warning', ['commIssue', 'commIssueAvgFreq', 'selfdrivedLagging'])
def test_gm_and_volt_actual_intent_owner_keeps_main_arm_during_temporary_pause(identity, withdrawn, warning):
  cp, cs, feedback = baseline(identity)
  settings = AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0))
  owner = vehicle.create_intent(cp, settings, vehicle.policy_for(cp))
  owner.update(cs, fault_active=False, now_ns=NOW)
  assert owner.allowed_latch and not owner._main_cycle_required
  now = NOW + 42_000_000
  sm, _ = sample(cp, now, 20_000_001, phase='pause', withdrawn=withdrawn, warning=warning)
  reset = feedback.observe(sm, cp, cs, now, latched=owner.allowed_latch, intent_producer_session=PRODUCER)
  owner.update(cs, fault_active=reset, now_ns=now)
  assert owner.allowed_latch and not owner._main_cycle_required
  cs.cruiseState.available = False
  owner.update(cs, fault_active=False, now_ns=now + 10_000_000)
  assert not owner.allowed_latch


def test_registered_owner_matrix_is_exhaustive():
  assert set(BRANDS) == set(vehicle._PORTS)
