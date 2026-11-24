"""Independent typed gas LAT proof; no combined-health or LONG permission renewal."""

from dataclasses import replace

import pytest
from openpilot.starpilot.tests.ford_alpha_test_support import NOW, PRODUCER, baseline, change, native_change, receipt
from openpilot.starpilot.car.ford.gas_override import retained_gas_lateral_native

NEGATIVES = (
  'released',
  'brake',
  'eps',
  'can',
  'main',
  'unlatched',
  'failed',
  'producer',
  'epoch',
  'source',
  'prior-expired',
  'long-active',
  'no-long-desire',
  'event',
  'event-user',
  'native-long-grant',
  'native-no-lat',
  'native-controls',
  'panda-controls',
  'native-rx',
  'native-heartbeat',
  'serial',
  'legacy',
)


def setup():
  cp, cs, f = baseline(combined=True)
  cs.gasPressed = True
  sm, native = receipt(cp, NOW + 10_000_000, NOW + 20_000_000, 2, gas=True)
  return cp, cs, f, sm, native


def test_typed_gas_proof_records_only_current_independent_lateral_and_recovers_normally():
  cp, cs, f, sm, native = setup()
  before = dict(vars(f))
  assert retained_gas_lateral_native(f, sm, cp, cs, sm['aolAxisState'], NOW + 20_000_000, latched=True, producer=PRODUCER) == native
  assert vars(f) == before
  for index in range(1, 5):
    sm, _ = receipt(cp, NOW + index * 20_000_000 - 10_000_000, NOW + index * 20_000_000, index + 1, gas=True)
    lease = sm['aolAxisState'].validUntilMonoTime
    assert not f.observe(sm, cp, cs, NOW + index * 20_000_000, latched=True, intent_producer_session=PRODUCER)
    assert f.lateral_source_ns == sm['aolAxisState'].sourceCarStateMonoTime
    assert f.healthy_feedback_ns == before['healthy_feedback_ns'] and f.healthy_native_ns == before['healthy_native_ns']
    assert sm['aolAxisState'].validUntilMonoTime == lease
    assert not f.lateral_only_baseline and not f.failed
  cs.gasPressed = False
  sm, _ = receipt(cp, NOW + 90_000_000, NOW + 90_000_000, 6, combined=True)
  assert retained_gas_lateral_native(f, sm, cp, cs, sm['aolAxisState'], NOW + 90_000_000, latched=True, producer=PRODUCER) is None
  assert not f.observe(sm, cp, cs, NOW + 90_000_000, latched=True, intent_producer_session=PRODUCER)


@pytest.mark.parametrize('fault', NEGATIVES)
def test_typed_gas_never_masks_fault_or_extends_an_expired_prior_proof(fault):
  cp, cs, f, sm, native = setup()
  now, latched, producer = NOW + 20_000_000, True, PRODUCER
  if fault in ('released', 'brake', 'eps', 'can'):
    field, value = {'released': ('gasPressed', False), 'brake': ('brakePressed', True), 'eps': ('steerFaultTemporary', True), 'can': ('canValid', False)}[fault]
    setattr(cs, field, value)
  elif fault == 'main':
    cs.cruiseState.enabled = False
  elif fault == 'unlatched':
    latched = False
  elif fault == 'failed':
    f.failed = True
  elif fault == 'producer':
    producer = 'other'
  elif fault == 'epoch':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'intentIntegrityEpoch', 2))
  elif fault == 'source':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'sourceCarStateMonoTime', now + 1))
  elif fault == 'prior-expired':
    now = NOW + 40_000_000
    sm, native = receipt(cp, NOW + 30_000_000, now, 3, gas=True)
  elif fault in ('long-active', 'no-long-desire'):
    field, value = ('longitudinalActive', True) if fault == 'long-active' else ('desiredLongitudinal', False)
    change(sm, 'aolAxisState', lambda a: setattr(a, field, value))
  elif fault in ('event', 'event-user'):
    if fault == 'event':
      change(sm, 'onroadEvents', lambda es: setattr(es[0], 'name', 'controlsMismatch'))
    else:
      change(sm, 'onroadEvents', lambda es: setattr(es[0], 'userDisable', True))
  elif fault == 'native-long-grant':
    native_change(sm, native, longitudinalAllowed=True)
  elif fault == 'native-no-lat':
    native_change(sm, native, lateralAllowed=False)
  elif fault in ('native-controls', 'native-rx', 'native-heartbeat'):
    field, value = {'native-controls': ('controlsAllowed', False), 'native-rx': ('safetyRxChecksInvalid', True), 'native-heartbeat': ('heartbeatLost', True)}[
      fault
    ]
    native_change(sm, native, pandaInventory=(replace(native.pandaInventory[0], **{field: value}),))
  elif fault == 'panda-controls':
    change(sm, 'pandaStates', lambda ps: setattr(ps[0], 'controlsAllowed', False))
  elif fault == 'serial':
    native_change(sm, native, pandaSerial='other', pandaInventory=(replace(native.pandaInventory[0], hardwareSerial='other'),))
  elif fault == 'legacy':
    native_change(sm, native, pandaInventory=(), sourcePandaStatesMonoTime=0)
  else:
    raise AssertionError(fault)
  assert retained_gas_lateral_native(f, sm, cp, cs, sm['aolAxisState'], now, latched=latched, producer=producer) is None
