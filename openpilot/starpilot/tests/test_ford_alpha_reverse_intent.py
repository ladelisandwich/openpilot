"""Alpha CP may retain only a proven LAT-only Reverse intent, never LONG."""

from dataclasses import replace

import pytest
from openpilot.starpilot.tests.ford_alpha_test_support import NOW, PRODUCER, baseline, change, native_change, receipt
from opendbc.car import structs
from openpilot.starpilot.car.ford.reverse_intent import retain_reverse_intent

NEGATIVES = (
  'lat-only-origin',
  'axis-long-desire',
  'axis-long-active',
  'native-long-request',
  'native-long-grant',
  'native-controls',
  'raw-controls',
  'native-rx',
  'native-heartbeat',
  'expired',
  'event',
  'epoch',
)


def setup():
  cp, cs, f = baseline(combined=False)
  cs.gearShifter = structs.CarState.GearShifter.reverse
  sm, native = receipt(cp, NOW + 10_000_000, NOW + 20_000_000, 2, reverse=True)
  return cp, cs, f, sm, native


def test_alpha_reverse_uses_original_lat_only_proof_with_zero_long_and_no_renewal():
  cp, cs, f, sm, _ = setup()
  before = dict(vars(f))
  assert retain_reverse_intent(f, sm, cp, cs, sm['aolAxisState'], NOW + 20_000_000, latched=True, producer=PRODUCER)
  assert vars(f) == before
  assert not f.observe(sm, cp, cs, NOW + 20_000_000, latched=True, intent_producer_session=PRODUCER)
  assert f.sequence == before['sequence'] and f.signature == before['signature']
  assert f.healthy_native_ns == before['healthy_native_ns'] and f.lateral_source_ns == before['lateral_source_ns']


@pytest.mark.parametrize('fault', NEGATIVES)
def test_alpha_reverse_cannot_borrow_combined_permission_or_mask_native_fault(fault):
  cp, cs, f, sm, native = setup()
  now = NOW + 20_000_000
  if fault == 'lat-only-origin':
    f.lateral_only_baseline = False
  elif fault in ('axis-long-desire', 'axis-long-active'):
    change(sm, 'aolAxisState', lambda a: setattr(a, 'desiredLongitudinal' if fault == 'axis-long-desire' else 'longitudinalActive', True))
  elif fault in ('native-long-request', 'native-long-grant'):
    native_change(sm, native, **{'requestedLongitudinal' if fault == 'native-long-request' else 'longitudinalAllowed': True})
  elif fault in ('native-controls', 'native-rx', 'native-heartbeat'):
    field = {'native-controls': 'controlsAllowed', 'native-rx': 'safetyRxChecksInvalid', 'native-heartbeat': 'heartbeatLost'}[fault]
    native_change(sm, native, pandaInventory=(replace(native.pandaInventory[0], **{field: True}),))
  elif fault == 'raw-controls':
    change(sm, 'pandaStates', lambda ps: setattr(ps[0], 'controlsAllowed', True))
  elif fault == 'expired':
    now = NOW + 40_000_000
    sm, _ = receipt(cp, NOW + 30_000_000, now, 3, reverse=True)
  elif fault == 'event':
    change(sm, 'onroadEvents', lambda es: setattr(es[0], 'immediateDisable', True))
  elif fault == 'epoch':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'intentIntegrityEpoch', 2))
  else:
    raise AssertionError(fault)
  assert not retain_reverse_intent(f, sm, cp, cs, sm['aolAxisState'], now, latched=True, producer=PRODUCER)
