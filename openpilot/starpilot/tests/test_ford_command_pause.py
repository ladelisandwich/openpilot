"""A 30ms output lease expiry must not erase a still-fresh stock-Ford arm."""

from dataclasses import replace
import pytest

from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, baseline, sample, change, native_change
from openpilot.starpilot.car.ford.command_pause import retain_command_pause_intent


def pause():
  cp, cs, feedback = baseline()
  sm, native = sample(cp, NOW, NOW + 40_000_000, 1, requested=False, allowed=False)
  return cp, cs, feedback, sm, native


def test_output_expiry_retains_only_original_intent_then_recovers_with_real_native_grant():
  cp, cs, feedback, sm, native = pause()
  before = dict(vars(feedback))
  for delta in (40_000_000, 60_000_000, 150_000_000):
    assert retain_command_pause_intent(feedback, sm, cp, cs, sm['aolAxisState'], NOW + delta, latched=True, producer=PRODUCER)
    assert not feedback.observe(sm, cp, cs, NOW + delta, latched=True, intent_producer_session=PRODUCER)
    assert vars(feedback) == before
  assert not retain_command_pause_intent(feedback, sm, cp, cs, sm['aolAxisState'], NOW + 200_000_001, latched=True, producer=PRODUCER)
  fresh, _ = sample(cp, NOW + 160_000_000, NOW + 160_000_000, 2)
  assert not feedback.observe(fresh, cp, cs, NOW + 160_000_000, latched=True, intent_producer_session=PRODUCER)
  assert feedback.lateral_source_ns == NOW + 160_000_000 and not feedback.failed


@pytest.mark.parametrize(
  'fault',
  [
    'unlatched',
    'can',
    'main',
    'brake',
    'gas',
    'eps',
    'long',
    'producer',
    'epoch',
    'session',
    'event',
    'native-rx',
    'native-heartbeat',
    'interrupt8',
    'inventory',
    'requested',
  ],
)
def test_command_pause_cannot_hide_health_identity_longitudinal_or_driver_veto(fault):
  cp, cs, feedback, sm, native = pause()
  latched, producer = True, PRODUCER
  if fault == 'unlatched':
    latched = False
  elif fault == 'can':
    cs.canValid = False
  elif fault == 'main':
    cs.cruiseState.available = False
  elif fault == 'brake':
    cs.brakePressed = True
  elif fault == 'gas':
    cs.gasPressed = True
  elif fault == 'eps':
    cs.steerFaultPermanent = True
  elif fault == 'long':
    cp.openpilotLongitudinalControl = True
  elif fault == 'producer':
    producer = 'other-card'
  elif fault == 'epoch':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'intentIntegrityEpoch', 2))
  elif fault == 'session':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'sessionId', 'other-axis'))
  elif fault == 'event':
    change(sm, 'onroadEvents', lambda e: setattr(e[0], 'immediateDisable', True))
  elif fault == 'requested':
    native_change(sm, native, requestedLateral=True)
  elif fault == 'inventory':
    native_change(sm, native, pandaSerial='other-panda')
  else:
    slot = replace(
      native.pandaInventory[0],
      **{'native-rx': {'safetyRxChecksInvalid': True}, 'native-heartbeat': {'heartbeatLost': True}, 'interrupt8': {'faults': 8}}[fault],
    )
    native_change(sm, native, pandaInventory=(slot,))
  before = dict(vars(feedback))
  assert not retain_command_pause_intent(feedback, sm, cp, cs, sm['aolAxisState'], NOW + 40_000_000, latched=latched, producer=producer)
  assert vars(feedback) == before
