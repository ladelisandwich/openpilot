"""Fresh OFF receipts retain intent without acquiring active-axis authority."""
from dataclasses import replace

import pytest

from openpilot.cereal import log
from openpilot.starpilot.aol.tests.test_stock_location_recovery import established_stock as established_stock
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample


def inactive(cp, now, sequence):
  sm, native = sample(cp, now, now, sequence, active=False, requested=False, allowed=False, event_stamp=now)
  change(sm, 'aolAxisState', lambda axis: setattr(axis, 'nativeAcknowledged', False))
  return sm, native


@pytest.mark.parametrize('unbelted', (False, True))
def test_fresh_off_retains_intent_without_minting_active_baseline(established_stock, unbelted):
  cp, cs, feedback = established_stock
  cs.seatbeltUnlatched = unbelted
  for tick in range(2, 65):
    now = NOW + tick * 10_000_000
    sm, native = inactive(cp, now, tick)
    if unbelted:
      event = sm.messages['onroadEvents'].as_builder()
      event.onroadEvents = [{'name': 'pcmDisable', 'userDisable': True},
                           {'name': 'seatbeltNotLatched', 'softDisable': True, 'noEntry': True}]
      sm.messages['onroadEvents'] = event.as_reader()
    assert not feedback.observe(sm, cp, cs, now, latched=True, intent_producer_session=PRODUCER, intent_paused=True)
    assert not feedback.failed and not feedback.lateral_only_baseline
    assert not feedback.lateral_baseline_ns and not feedback.lateral_source_ns
    assert not native.requestedLateral and not native.lateralAllowed
    assert not native.requestedLongitudinal and not native.longitudinalAllowed
    assert sm['aolAxisState'].validUntilMonoTime == now + 30_000_000


@pytest.mark.parametrize('bad', ('invalid_can', 'main_off', 'permanent_eps', 'temporary_eps', 'door', 'seatbelt_event_without_source',
  'sensors', 'acc_fault', 'steering_disengage', 'wrong_epoch', 'wrong_producer', 'wrong_session', 'source_future',
  'expired', 'native_invalid', 'native_request', 'inventory', 'immediate', 'soft', 'other_user'))
def test_inactive_receipt_cannot_bypass_existing_veto(established_stock, bad):
  cp, cs, feedback = established_stock
  now = NOW + 10_000_000
  sm, native = inactive(cp, now, 2)
  producer = PRODUCER
  fields = {'invalid_can': ('canValid', False), 'permanent_eps': ('steerFaultPermanent', True),
            'temporary_eps': ('steerFaultTemporary', True), 'door': ('doorOpen', True),
            'sensors': ('vehicleSensorsInvalid', True),
            'acc_fault': ('accFaulted', True), 'steering_disengage': ('steeringDisengage', True)}
  if bad in fields:
    name, value = fields[bad]
    setattr(cs, name, value)
  elif bad == 'main_off':
    cs.cruiseState.available = False
  elif bad == 'wrong_epoch':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch', 9))
  elif bad == 'wrong_producer':
    producer = 'other'
  elif bad == 'wrong_session':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'sessionId', 'other'))
  elif bad == 'source_future':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'sourceCarStateMonoTime', now + 1))
  elif bad == 'expired':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'validUntilMonoTime', now - 1))
  elif bad == 'native_invalid':
    sm.valid['aolSafetyWire'] = False
  elif bad == 'native_request':
    native_change(sm, native, requestedLateral=True)
  elif bad == 'inventory':
    native_change(sm, native, pandaSerial='different',
                  pandaInventory=(replace(native.pandaInventory[0], hardwareSerial='different'),))
  elif bad in ('immediate', 'soft'):
    change(sm, 'onroadEvents', lambda events: setattr(events[0],
           'immediateDisable' if bad == 'immediate' else 'softDisable', True))
  elif bad == 'seatbelt_event_without_source':
    event = sm.messages['onroadEvents'].as_builder()
    event.onroadEvents = [{'name': 'seatbeltNotLatched', 'softDisable': True, 'noEntry': True}]
    sm.messages['onroadEvents'] = event.as_reader()
  elif bad == 'other_user':
    change(sm, 'onroadEvents', lambda events: setattr(events[0], 'name', log.OnroadEvent.EventName.pedalPressed))
  assert feedback.observe(sm, cp, cs, now, latched=True, intent_producer_session=producer, intent_paused=bad != 'temporary_eps')
