"""Expired command proofs may retain an existing intent, never native authority."""
from dataclasses import replace
from types import SimpleNamespace

import pytest
from opendbc.car import structs
from opendbc.car.chrysler.values import CAR as Chrysler
from opendbc.car.ford.values import CAR as Ford
from opendbc.car.hyundai.values import CAR as Hyundai
from opendbc.safety.tests.test_ioniq6_native_pedal_order import profile
from openpilot.starpilot.aol.tests import test_stock_location_recovery as stock
from openpilot.starpilot.aol.runtime import decide_axes
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample


def exact_sample(cp, source, observed, sequence, **kwargs):
  sm, native = sample(cp, source, observed, sequence, **kwargs)
  if cp.alternativeExperience != 32:
    native = native_change(sm, native, pandaInventory=(replace(native.pandaInventory[0], alternativeExperience=cp.alternativeExperience),))
    change(sm, 'pandaStates', lambda pandas: setattr(pandas[0], 'alternativeExperience', cp.alternativeExperience))
  return sm, native


@pytest.fixture(params=['ioniq6_std', 'ioniq6_alt', Chrysler.CHRYSLER_PACIFICA_2018, Chrysler.RAM_1500_5TH_GEN,
                        Ford.FORD_MUSTANG_MACH_E_MK1, Ford.FORD_ESCAPE_MK4, Hyundai.HYUNDAI_SONATA_HYBRID])
def established(request):
  if request.param not in ('ioniq6_std', 'ioniq6_alt'):
    return stock.established_stock.__wrapped__(request)
  cp = profile(request.param == 'ioniq6_alt')
  state = structs.CarState(canValid=True, gearShifter='drive', vEgo=15.)
  state.cruiseState.available = True
  feedback = TransportPauseFeedback(cp)
  sm, _ = exact_sample(cp, NOW, NOW, 1)
  assert not feedback.observe(sm, cp, state, NOW, latched=True, intent_producer_session=PRODUCER)
  assert feedback.lateral_only_baseline
  return cp, state, feedback


def clocks(feedback):
  return (feedback.healthy_feedback_ns, feedback.healthy_native_ns, feedback.lateral_baseline_ns,
          feedback.lateral_source_ns, feedback.lateral_native_ns)


def test_zero_request_retains_only_prior_intent_until_original_source_bound(established):
  cp, state, feedback = established
  original = clocks(feedback)
  for age in (40_000_000, 190_000_000):
    now = NOW + age
    sm, native = exact_sample(cp, NOW, now, 1, requested=False, allowed=False)
    assert not feedback.observe(sm, cp, state, now, latched=True, intent_producer_session=PRODUCER)
    assert clocks(feedback) == original
    assert feedback.source_retention_started_ns in (0, NOW + 40_000_000)
    assert sm['aolAxisState'].validUntilMonoTime == NOW + 30_000_000 < now
    axes = decide_axes(standard_lateral=False, standard_longitudinal=False,
      intent=SimpleNamespace(allowedLatch=True, pauseLateral=False, pauseLongitudinal=False),
      native=native, car_state=state, initialized=True, model_ready=True, no_entry=False,
      immediate_disable=False, dm_lockout=False, pause_brake_mps=0.)
    assert not axes.lateral_active and not axes.longitudinal_active
  now = NOW + 200_000_001
  sm, _ = exact_sample(cp, NOW, now, 1, requested=False, allowed=False)
  assert feedback.observe(sm, cp, state, now, latched=True, intent_producer_session=PRODUCER)
  assert clocks(feedback) == original


@pytest.mark.parametrize('fault', ['invalid_can', 'timeout', 'permanent_eps', 'temporary_eps', 'requested_denied',
  'unexpected_long', 'native_grant', 'native_invalid', 'native_expired', 'native_future', 'native_old',
  'native_serial', 'native_fault', 'native_rx', 'native_heartbeat', 'wrong_session', 'wrong_epoch',
  'wrong_producer', 'cp_changed', 'no_baseline', 'immediate', 'other_soft'])
def test_zero_request_keeps_all_existing_source_and_native_vetoes(established, fault):
  cp, state, feedback = established
  now = NOW + 40_000_000
  sm, native = exact_sample(cp, NOW, now, 1, requested=False, allowed=False)
  producer = PRODUCER
  if fault in ('invalid_can', 'timeout', 'permanent_eps', 'temporary_eps'):
    field = {'invalid_can': 'canValid', 'timeout': 'canTimeout', 'permanent_eps': 'steerFaultPermanent', 'temporary_eps': 'steerFaultTemporary'}[fault]
    setattr(state, field, fault != 'invalid_can')
  elif fault == 'requested_denied':
    native_change(sm, native, requestedLateral=True)
  elif fault == 'unexpected_long':
    native_change(sm, native, requestedLongitudinal=True)
  elif fault == 'native_grant':
    native_change(sm, native, lateralAllowed=True)
  elif fault == 'native_invalid':
    sm.valid['aolSafetyWire'] = False
  elif fault == 'native_expired':
    native_change(sm, native, validUntilMonoTime=now - 1)
  elif fault == 'native_future':
    native_change(sm, native, observedMonoTime=now + 1)
  elif fault == 'native_old':
    native_change(sm, native, observedMonoTime=NOW - 1)
  elif fault == 'native_serial':
    native_change(sm, native, pandaSerial='other', pandaInventory=(replace(native.pandaInventory[0], hardwareSerial='other'),))
  elif fault in ('native_fault', 'native_rx', 'native_heartbeat'):
    field = {'native_fault': 'faults', 'native_rx': 'safetyRxChecksInvalid', 'native_heartbeat': 'heartbeatLost'}[fault]
    native_change(sm, native, pandaInventory=(replace(native.pandaInventory[0], **{field: 1 if field == 'faults' else True}),))
  elif fault == 'wrong_session':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'sessionId', 'other'))
  elif fault == 'wrong_epoch':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch', 2))
  elif fault == 'wrong_producer':
    producer = 'other'
  elif fault == 'cp_changed':
    cp.flags ^= 1
  elif fault == 'no_baseline':
    feedback.lateral_baseline_ns = 0
  elif fault in ('immediate', 'other_soft'):
    event = sm.messages['onroadEvents'].as_builder()
    event.onroadEvents = ([{'name': 'relayMalfunction', 'immediateDisable': True}] if fault == 'immediate' else
                          [{'name': 'lowMemory', 'softDisable': True}])
    sm.messages['onroadEvents'] = event.as_reader()
  assert feedback.observe(sm, cp, state, now, latched=True, intent_producer_session=producer)


def test_an_explicit_off_intent_does_not_enter_retention(established):
  cp, state, feedback = established
  now = NOW + 40_000_000
  sm, native = exact_sample(cp, NOW, now, 1, requested=False, allowed=False)
  assert not feedback.observe(sm, cp, state, now, latched=False, intent_producer_session=PRODUCER)
  assert not feedback.source_retention_started_ns
  axes = decide_axes(standard_lateral=False, standard_longitudinal=False,
    intent=SimpleNamespace(allowedLatch=False, pauseLateral=False, pauseLongitudinal=False),
    native=native, car_state=state, initialized=True, model_ready=True, no_entry=False,
    immediate_disable=False, dm_lockout=False, pause_brake_mps=0.)
  assert not axes.desired_lateral and not axes.lateral_active and not axes.longitudinal_active
