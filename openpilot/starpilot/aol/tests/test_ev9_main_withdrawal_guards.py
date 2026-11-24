"""Exact EV9 MAIN withdrawal retains intent only from native LAT permission."""
from dataclasses import replace

import pytest
from opendbc.car import structs
from opendbc.car.hyundai.ev9_longitudinal import candidate
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR
from openpilot.cereal import log
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample


@pytest.mark.parametrize('fault', ['none', 'unlatched', 'producer', 'epoch', 'session', 'failed', 'paused',
  'pending', 'no_baseline', 'no_inventory', 'source_future', 'source_zero', 'expired_source', 'can', 'timeout',
  'temporary_eps', 'permanent_eps', 'acc', 'steering', 'brake', 'gas', 'door', 'belt', 'gear', 'cancel',
  'native_lat', 'native_long', 'native_no_long_request', 'native_fault', 'native_identity', 'panda_fault',
  'event_soft', 'event_immediate', 'event_user', 'main_still_available', 'cruise_still_enabled', 'other_cp'])
def test_ev9_main_off_keeps_only_current_independent_lateral_proof(fault):
  cp = candidate(params(candidate=CAR.KIA_EV9), enabled=True, is_release=False)
  cp.alternativeExperience = 32
  cs = structs.CarState(canValid=True, gearShifter='drive', vEgo=15.)
  cs.cruiseState.available = True
  feedback = TransportPauseFeedback(cp)
  sm, _ = sample(cp, NOW, NOW, 1)
  event = sm.messages['onroadEvents'].as_builder()
  event.onroadEvents = []
  sm.messages['onroadEvents'] = event.as_reader()
  assert not feedback.observe(sm, cp, cs, NOW, latched=True, intent_producer_session=PRODUCER)
  assert feedback.lateral_only_baseline
  now = NOW + 10_000_000
  sm, native = sample(cp, now, now, 2, event_stamp=now)
  native = native_change(sm, native, requestedLongitudinal=True)
  change(sm, 'aolAxisState', lambda axis: setattr(axis, 'mode', 'combined'))
  change(sm, 'aolAxisState', lambda axis: setattr(axis, 'desiredLongitudinal', True))
  change(sm, 'aolAxisState', lambda axis: setattr(axis, 'longitudinalActive', True))
  change(sm, 'onroadEvents', lambda events: setattr(events[0], 'name', log.OnroadEvent.EventName.wrongCarMode))
  cs.cruiseState.available = False
  latched, producer = fault != 'unlatched', 'foreign' if fault == 'producer' else PRODUCER
  changes = {'can': ('canValid', False), 'timeout': ('canTimeout', True), 'temporary_eps': ('steerFaultTemporary', True),
    'permanent_eps': ('steerFaultPermanent', True), 'acc': ('accFaulted', True), 'steering': ('steeringDisengage', True),
    'brake': ('brakePressed', True), 'gas': ('gasPressed', True), 'door': ('doorOpen', True),
    'belt': ('seatbeltUnlatched', True), 'gear': ('gearShifter', structs.CarState.GearShifter.park)}
  if fault in changes:
    setattr(cs, *changes[fault])
  attrs = {'failed': ('failed', True), 'paused': ('paused', True), 'pending': ('pending_event', 1),
    'no_baseline': ('lateral_baseline_ns', 0), 'no_inventory': ('lateral_inventory', None)}
  if fault in attrs:
    setattr(feedback, *attrs[fault])
  axes = {'epoch': ('intentIntegrityEpoch', 2), 'session': ('sessionId', 'foreign'),
    'source_future': ('sourceCarStateMonoTime', now + 1), 'source_zero': ('sourceCarStateMonoTime', 0),
    'expired_source': ('sourceCarStateMonoTime', now - 30_000_001)}
  if fault in axes:
    change(sm, 'aolAxisState', lambda axis: setattr(axis, *axes[fault]))
  if fault == 'cancel':
    cs.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
  native_fields = {'native_lat': {'lateralAllowed': False}, 'native_long': {'longitudinalAllowed': True},
    'native_no_long_request': {'requestedLongitudinal': False}}
  if fault in native_fields:
    native = native_change(sm, native, **native_fields[fault])
  if fault in ('native_fault', 'native_identity'):
    slot = replace(native.pandaInventory[0], **({'faults': 1} if fault == 'native_fault' else {'safetyParam': 0}))
    native = native_change(sm, native, pandaInventory=(slot,))
  if fault == 'panda_fault':
    change(sm, 'pandaStates', lambda pandas: setattr(pandas[0], 'faults', ['relayMalfunction']))
  if fault in ('event_soft', 'event_immediate'):
    change(sm, 'onroadEvents', lambda events: setattr(events[0], 'softDisable' if fault == 'event_soft' else 'immediateDisable', True))
  if fault == 'event_user':
    change(sm, 'onroadEvents', lambda events: setattr(events[0], 'name', log.OnroadEvent.EventName.buttonCancel))
  if fault == 'main_still_available':
    cs.cruiseState.available = True
  if fault == 'cruise_still_enabled':
    cs.cruiseState.enabled = True
  if fault == 'other_cp':
    cp.carFingerprint = CAR.HYUNDAI_IONIQ_5_PE
  original = dict(vars(feedback))
  result = feedback._native_long_withdrawal_lateral_current(sm, cp, cs, sm['aolAxisState'], now,
    latched=latched, producer=producer)
  assert (result is not None) == (fault in ('none', 'cruise_still_enabled'))
  assert vars(feedback) == original
  if result is not None:
    assert result.lateralAllowed and not result.longitudinalAllowed
    assert result.observedMonoTime == now and result.validUntilMonoTime == now + 200_000_000
