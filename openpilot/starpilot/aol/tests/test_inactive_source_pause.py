"""Source-zero availability withdraws output, not accepted LAT/FULL driver intent."""
from contextlib import ExitStack
from dataclasses import replace

import pytest

from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as callers
from openpilot.starpilot.aol.tests.test_reverse_source_gap import baseline as reverse_baseline, sample, change, NOW, PRODUCER
from openpilot.starpilot.aol.wire import decode_safety, encode_safety


def full_native(sm, native, longitudinal):
  native = replace(native, requestedLateral=True, lateralAllowed=True,
                   requestedLongitudinal=longitudinal, longitudinalAllowed=longitudinal,
                   pandaInventory=(replace(native.pandaInventory[0], controlsAllowed=longitudinal),))
  event = sm.messages['aolSafetyWire'].as_builder()
  event.aolSafetyWire = encode_safety(native)
  sm.messages['aolSafetyWire'] = event.as_reader()
  change(sm, 'pandaStates', lambda pandas: setattr(pandas[0], 'controlsAllowed', longitudinal))
  return native


def healthy_events(sm, stamp):
  event = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=stamp)
  event.onroadEvents[0].name = 'cruiseMismatch'
  sm.messages['onroadEvents'] = event.as_reader()
  sm.logMonoTime['onroadEvents'] = stamp


def baseline(longitudinal):
  cp, cs, feedback = reverse_baseline()
  cs.gearShifter = 'drive'
  sm, native = sample(cp, NOW + 10_000_000, 3, active=True)
  full_native(sm, native, longitudinal)
  def inactive(axis):
    axis.mode = 'off'
    axis.lateralActive = axis.longitudinalActive = False
    axis.desiredLongitudinal = longitudinal
  change(sm, 'aolAxisState', inactive)
  healthy_events(sm, NOW)
  assert not feedback.observe(sm, cp, cs, NOW + 10_000_000, latched=True, intent_producer_session=PRODUCER)
  assert not feedback.lateral_only_baseline and feedback.ever_lateral_active
  return cp, cs, feedback


def paused(cp, now, longitudinal, event_order='matched'):
  sm, native = sample(cp, now, 4, active=False)
  native = full_native(sm, native, longitudinal)
  def pause(axis):
    axis.sourceCarStateMonoTime = 0
    axis.nativeAcknowledged = False
    axis.faultReason = 'availabilityPause'
    axis.faultEventMonoTime = now
  change(sm, 'aolAxisState', pause)
  event = messaging.new_message('onroadEvents', 2, valid=True, logMonoTime=now)
  event.onroadEvents[0].name, event.onroadEvents[0].softDisable = 'commIssue', True
  event.onroadEvents[1].name = 'cruiseMismatch'
  sm.messages['onroadEvents'] = event.as_reader()
  sm.logMonoTime['onroadEvents'] = now
  if event_order != 'matched':
    healthy_events(sm, now + 1_000_000 if event_order == 'recovery_first' else NOW)
  return sm, native


@pytest.mark.parametrize('longitudinal', [False, True])
@pytest.mark.parametrize('order', ['matched', 'axis_first', 'recovery_first'])
@pytest.mark.parametrize('withdrawn', [False, True])
def test_source_zero_pause_retains_original_intent_without_renewing_authority(longitudinal, order, withdrawn):
  cp, cs, feedback = baseline(longitudinal)
  now = NOW + 42_000_000
  sm, native = paused(cp, now, longitudinal, order)
  if withdrawn:
    event = sm.messages['aolSafetyWire'].as_builder()
    event.aolSafetyWire = encode_safety(replace(native, requestedLateral=False, lateralAllowed=False,
                                               requestedLongitudinal=False, longitudinalAllowed=False))
    sm.messages['aolSafetyWire'] = event.as_reader()
  before = dict(vars(feedback))
  assert not feedback.observe(sm, cp, cs, now + 2_000_000, latched=True, intent_producer_session=PRODUCER)
  assert not feedback.failed
  for field in ('healthy_feedback_ns', 'healthy_native_ns', 'signature', 'sequence', 'lateral_source_ns', 'lateral_baseline_ns'):
    assert vars(feedback)[field] == before[field]
  assert feedback.retention_qualified == (order == 'matched')
  # The recovered host proof can still be inactive while its native grants are
  # healthy; retaining intent must not require control output to turn on first.
  recovered, native = sample(cp, now + 10_000_000, 5, active=True)
  full_native(recovered, native, longitudinal)
  def inactive(axis):
    axis.mode = 'off'
    axis.lateralActive = axis.longitudinalActive = False
    axis.desiredLongitudinal = longitudinal
  change(recovered, 'aolAxisState', inactive)
  healthy_events(recovered, now + 10_000_000)
  assert not feedback.observe(recovered, cp, cs, now + 10_000_000, latched=True, intent_producer_session=PRODUCER)
  assert not feedback.failed


@pytest.mark.parametrize('fault', ['source_bound', 'packet_bound', 'can', 'eps', 'door', 'belt', 'button', 'cp', 'producer',
                                  'session', 'epoch', 'source', 'active', 'critical', 'event_fault', 'event_cancel',
                                  'native_denial', 'native_long_denial', 'inventory', 'native_expired'])
def test_inactive_source_pause_preserves_fault_identity_and_original_deadlines(fault):
  cp, cs, feedback = baseline(True)
  now = NOW + 42_000_000
  sm, native = paused(cp, now, True)
  producer = PRODUCER
  if fault == 'source_bound':
    now = NOW + 210_000_001
    sm, native = paused(cp, now, True)
  elif fault == 'packet_bound':
    now += 30_000_001
  elif fault in ('can', 'eps', 'door', 'belt'):
    setattr(cs, {'can': 'canValid', 'eps': 'steerFaultTemporary', 'door': 'doorOpen', 'belt': 'seatbeltUnlatched'}[fault], fault != 'can')
  elif fault == 'button':
    cs.buttonEvents = [{'type': 'cancel', 'pressed': True}]
  elif fault == 'cp':
    cp.flags ^= 1
  elif fault == 'producer':
    producer = 'changed'
  elif fault in ('session', 'epoch', 'source', 'active', 'critical'):
    key, value = {'session': ('sessionId', 'changed'), 'epoch': ('intentIntegrityEpoch', 1),
                  'source': ('sourceCarStateMonoTime', NOW), 'active': ('lateralActive', True),
                  'critical': ('faultReason', 'critical')}[fault]
    change(sm, 'aolAxisState', lambda axis: setattr(axis, key, value))
  elif fault in ('event_fault', 'event_cancel'):
    def altered(events):
      events[0].name = 'controlsMismatch' if fault == 'event_fault' else 'buttonCancel'
      events[0].immediateDisable = fault == 'event_fault'
      events[0].userDisable = fault == 'event_cancel'
    change(sm, 'onroadEvents', altered)
  elif fault in ('native_denial', 'native_long_denial', 'inventory', 'native_expired'):
    updates = {'native_denial': {'lateralAllowed': False}, 'native_long_denial': {'longitudinalAllowed': False},
               'inventory': {'pandaSerial': 'changed', 'pandaInventory': (replace(native.pandaInventory[0], hardwareSerial='changed'),)},
               'native_expired': {'validUntilMonoTime': now - 1}}[fault]
    event = sm.messages['aolSafetyWire'].as_builder()
    event.aolSafetyWire = encode_safety(replace(native, **updates))
    sm.messages['aolSafetyWire'] = event.as_reader()
  assert not feedback._inactive_source_pause_current(sm, cp, cs, sm['aolAxisState'], sm.logMonoTime['aolAxisState'], now,
                                                    latched=True, producer=producer)


@pytest.mark.parametrize('full_op,stock_angle', [(True, False), (False, False), (False, True)])
@pytest.mark.parametrize('native_withdrawn', [False, True])
def test_actual_card_keeps_arm_through_fully_withdrawn_source_pause(full_op, stock_angle, native_withdrawn):
  case = callers.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    with OpenpilotPrefix():
      sd, _, step, card, receive = case._ioniq_availability_callers(full_op=full_op, stock_angle=stock_angle)
      step(3, native_request_long=full_op)
      proof = sd.pm.messages['aolAxisState'].as_builder()
      proof.aolAxisState.mode = 'off'
      proof.aolAxisState.lateralActive = proof.aolAxisState.longitudinalActive = False
      assert receive(NOW + 30_000_000, axis=proof.as_reader(), event=sd.pm.messages['onroadEvents'])
      if native_withdrawn:
        native = decode_safety(sd.sm['aolSafetyWire'])
        assert native is not None
        wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=NOW + 50_000_000)
        wire.aolSafetyWire = encode_safety(replace(native, observedMonoTime=wire.logMonoTime,
          validUntilMonoTime=wire.logMonoTime + 200_000_000, requestedLateral=False, lateralAllowed=False,
          requestedLongitudinal=False, longitudinalAllowed=False))
        sd.sm.update_msgs((NOW + 50_000_000) / 1e9, [wire.as_reader()])
      pause = proof.as_reader().as_builder()
      pause.logMonoTime = NOW + 55_000_000
      axis = pause.aolAxisState
      axis.sequence += 1
      axis.observedMonoTime = axis.faultEventMonoTime = pause.logMonoTime
      axis.validUntilMonoTime, axis.sourceCarStateMonoTime = pause.logMonoTime + 30_000_000, 0
      axis.desiredLateral = axis.desiredLongitudinal = axis.nativeAcknowledged = False
      axis.faultReason = 'availabilityPause'
      events = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=pause.logMonoTime)
      events.onroadEvents[0].name, events.onroadEvents[0].softDisable = 'commIssue', True
      assert receive(NOW + 62_000_000, axis=pause.as_reader(), event=events.as_reader())
      assert not card.aol_transport_feedback.failed
      assert card.aol_card_intent.allowed_latch
