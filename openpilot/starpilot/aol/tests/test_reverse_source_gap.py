"""A source-zero OFF publication cannot erase fresh, already inactive reverse intent."""
from contextlib import ExitStack
from dataclasses import replace

import pytest

from opendbc.car import structs
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.runtime import decide_axes
from openpilot.starpilot.aol.tests import test_ioniq_availability as callers
from openpilot.starpilot.aol.tests.test_gm_held_native import Peers
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety
from openpilot.starpilot.controllers.tests.test_wheel_actions import candidate

NOW = 10_000_000_000
PRODUCER, SESSION = 'reverse-card', 'reverse-axis'


def sample(cp, now=NOW, sequence=1, *, active=False, gap=False):
  cfg = cp.safetyConfigs[0]
  slot = PandaSlot(0, 'reverse-panda', cfg.safetyModel.raw, cfg.safetyParam, cp.alternativeExperience, False)
  native = SafetyState(1, True, now, now + 200_000_000, cfg.safetyModel.raw, cfg.safetyParam,
                       active, False, active, False, slot.hardwareSerial, SESSION,
                       sourcePandaStatesMonoTime=now, pandaInventory=(slot,))
  wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
  wire.aolSafetyWire = encode_safety(native)
  axis = messaging.new_message('aolAxisState', valid=not gap, logMonoTime=now)
  a = axis.aolAxisState
  a.sessionId = a.faultSessionId = SESSION
  a.sequence, a.qualified, a.faultReason = sequence, not gap, 'none'
  a.observedMonoTime, a.sourceCarStateMonoTime, a.validUntilMonoTime = now, 0 if gap else now, now + 30_000_000
  a.intentProducerSessionId, a.intentIntegrityEpoch = PRODUCER, 0
  a.faultEventMonoTime = NOW
  a.desiredLateral = a.lateralActive = active
  a.nativeAcknowledged = not gap
  a.mode = 'lateralOnly' if active else 'off'
  pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
  p = pandas.pandaStates[0]
  p.safetyModel, p.safetyParam, p.alternativeExperience = cfg.safetyModel, cfg.safetyParam, cp.alternativeExperience
  events = messaging.new_message('onroadEvents', 2, valid=True, logMonoTime=NOW)
  events.onroadEvents[0].name, events.onroadEvents[0].softDisable = 'wrongGear', True
  events.onroadEvents[1].name, events.onroadEvents[1].userDisable = 'reverseGear', True
  sm = Peers([wire.as_reader(), axis.as_reader(), pandas.as_reader(), events.as_reader()])
  sm.valid['aolAxisState'] = not gap
  return sm, native


def baseline():
  _, cp = candidate(True)
  cp.safetyConfigs[0].safetyParam |= 0x800
  cs = structs.CarState(canValid=True, gearShifter='drive')
  cs.cruiseState.available = True
  feedback = TransportPauseFeedback(cp)
  active, _ = sample(cp, NOW - 10_000_000, active=True)
  assert not feedback.observe(active, cp, cs, NOW - 10_000_000, latched=True, intent_producer_session=PRODUCER)
  cs.gearShifter = 'reverse'
  off, _ = sample(cp, NOW, 2)
  assert not feedback.observe(off, cp, cs, NOW, latched=True, intent_producer_session=PRODUCER)
  assert feedback.ever_lateral_active and not feedback.lateral_only_baseline
  return cp, cs, feedback


def change(sm, service, update):
  event = sm.messages[service].as_builder()
  update(getattr(event, service))
  sm.messages[service] = event.as_reader()


@pytest.mark.parametrize('gear', ['neutral', 'reverse'])
def test_reverse_off_source_gap_preserves_only_existing_intent_and_original_deadlines(gear):
  cp, cs, feedback = baseline()
  cs.gearShifter = gear
  cs.gasPressed = True  # The recorded failure occurred while backing up on gas.
  sm, native = sample(cp, NOW + 32_000_000, 3, gap=True)
  before = dict(vars(feedback))
  assert not feedback.observe(sm, cp, cs, NOW + 32_000_000, latched=True, intent_producer_session=PRODUCER)
  assert vars(feedback) == before
  # Repeated invalid samples cannot renew the original 200ms intent bound.
  for elapsed in (100_000_000, 200_000_000):
    sm, _ = sample(cp, NOW + elapsed, 4, gap=True)
    assert not feedback.observe(sm, cp, cs, NOW + elapsed, latched=True, intent_producer_session=PRODUCER)
    assert vars(feedback) == before
  sm, _ = sample(cp, NOW + 200_000_001, 5, gap=True)
  assert feedback.observe(sm, cp, cs, NOW + 200_000_001, latched=True, intent_producer_session=PRODUCER)
  intent = type('Intent', (), {'allowedLatch': True, 'pauseLateral': False, 'pauseLongitudinal': False})()
  decision = decide_axes(standard_lateral=False, standard_longitudinal=False, intent=intent, native=native,
                        car_state=cs, initialized=True, model_ready=True, no_entry=False, immediate_disable=False,
                        dm_lockout=False, pause_brake_mps=0.)
  assert not any((decision.desired_lateral, decision.desired_longitudinal, decision.lateral_active, decision.longitudinal_active))


@pytest.mark.parametrize('fault', ['drive', 'park', 'unknown', 'can', 'timeout', 'eps', 'sensors', 'acc', 'door', 'belt',
                                  'steering_disengage', 'main_off', 'button', 'cp', 'producer', 'epoch', 'session',
                                  'fault_session', 'critical', 'axis_active', 'axis_request', 'source', 'sequence',
                                  'axis_expired', 'events_invalid', 'event_fault', 'event_cancel', 'native_request',
                                  'native_allow', 'native_long', 'native_fault', 'native_expired', 'inventory', 'unlatched'])
def test_reverse_gap_does_not_bypass_fault_identity_configuration_or_native_veto(fault):
  cp, cs, feedback = baseline()
  now = NOW + 32_000_000
  sm, native = sample(cp, now, 3, gap=True)
  producer, latched = PRODUCER, True
  if fault in ('drive', 'park', 'unknown'):
    cs.gearShifter = fault
  elif fault in ('can', 'timeout', 'eps', 'sensors', 'acc', 'door', 'belt', 'steering_disengage', 'main_off'):
    if fault == 'main_off':
      cs.cruiseState.available = False
    else:
      setattr(cs, {'can': 'canValid', 'timeout': 'canTimeout', 'eps': 'steerFaultTemporary', 'sensors': 'vehicleSensorsInvalid',
                     'acc': 'accFaulted', 'door': 'doorOpen', 'belt': 'seatbeltUnlatched', 'steering_disengage': 'steeringDisengage'}[fault], fault != 'can')
  elif fault == 'button':
    cs.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
  elif fault == 'cp':
    cp.flags ^= 1
  elif fault == 'producer':
    producer = 'different'
  elif fault == 'unlatched':
    latched = False
  elif fault in ('epoch', 'session', 'fault_session', 'critical', 'axis_active', 'axis_request', 'source', 'sequence', 'axis_expired'):
    field, value = {'epoch': ('intentIntegrityEpoch', 1), 'session': ('sessionId', 'different'),
                    'fault_session': ('faultSessionId', 'different'), 'critical': ('faultReason', 'critical'),
                    'axis_active': ('lateralActive', True), 'axis_request': ('desiredLateral', True),
                    'source': ('sourceCarStateMonoTime', NOW), 'sequence': ('sequence', 2),
                    'axis_expired': ('validUntilMonoTime', now - 1)}[fault]
    change(sm, 'aolAxisState', lambda a: setattr(a, field, value))
  elif fault == 'events_invalid':
    sm.valid['onroadEvents'] = False
  elif fault in ('event_fault', 'event_cancel'):
    def update(events):
      events[0].name = 'controlsMismatch' if fault == 'event_fault' else 'buttonCancel'
      events[0].immediateDisable = fault == 'event_fault'
      events[0].userDisable = fault == 'event_cancel'
    change(sm, 'onroadEvents', update)
  elif fault.startswith('native_') or fault == 'inventory':
    updates = {'native_request': {'requestedLateral': True}, 'native_allow': {'lateralAllowed': True},
               'native_long': {'requestedLongitudinal': True}, 'native_expired': {'validUntilMonoTime': now - 1},
               'native_fault': {'pandaInventory': (replace(native.pandaInventory[0], heartbeatLost=True),)},
               'inventory': {'pandaSerial': 'changed', 'pandaInventory': (replace(native.pandaInventory[0], hardwareSerial='changed'),)}}[fault]
    altered = replace(native, **updates)
    event = sm.messages['aolSafetyWire'].as_builder()
    event.aolSafetyWire = encode_safety(altered)
    sm.messages['aolSafetyWire'] = event.as_reader()
  assert not feedback._inactive_gear_source_gap(sm, cp, cs, sm['aolAxisState'], now, now, latched=latched, producer=producer)


def test_actual_card_reverse_source_zero_keeps_latch_and_drive_requires_current_ack():
  case = callers.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      sd, _, step, card, receive = case._ioniq_availability_callers(full_op=False)
      reverse = callers.car_state(brake=True)
      reverse.gearShifter = 'reverse'
      for tick in (2, 3):
        step(tick, observed_state=reverse, native_request_lat=False, allowed=False, native_request_long=False)
        assert receive(NOW + tick * 10_000_000, state=reverse, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      broken = sd.pm.messages['aolAxisState'].as_builder()
      broken.valid = False
      broken.logMonoTime = NOW + 55_000_000
      axis = broken.aolAxisState
      axis.sequence += 1
      axis.observedMonoTime, axis.validUntilMonoTime = broken.logMonoTime, broken.logMonoTime + 30_000_000
      axis.sourceCarStateMonoTime, axis.qualified, axis.nativeAcknowledged = 0, False, False
      before = dict(vars(card.aol_transport_feedback))
      assert receive(NOW + 62_000_000, state=reverse, axis=broken.as_reader())
      assert not card.aol_transport_feedback.failed
      assert card.aol_transport_feedback.healthy_feedback_ns == before['healthy_feedback_ns']
      for tick in (7, 8):
        step(tick, observed_state=reverse, native_request_lat=False, allowed=False, native_request_long=False)
        assert receive(NOW + tick * 10_000_000, state=reverse, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      for tick, allowed in ((9, False), (10, True)):
        axis, _, command = step(tick, observed_state=callers.car_state(), native_request_lat=allowed, allowed=allowed, native_request_long=False)
        assert bool(axis.lateralActive and command.latActive) == allowed
        assert not axis.longitudinalActive and not command.longActive
