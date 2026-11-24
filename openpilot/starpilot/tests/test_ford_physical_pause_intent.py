"""Controlled first-join guards; real producer/native acceptance is a separate retry."""

from dataclasses import replace

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.car.ford.physical_pause import retain_physical_pause_intent
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample

REASONS = ('alpha-gas', 'alpha-brake', 'stock-eps', 'alpha-eps')
NEGATIVES = (
  'unlatched',
  'failed',
  'pending',
  'unqualified',
  'main-off',
  'wrong-cruise-state',
  'reverse',
  'brake',
  'mixed-pedal-eps',
  'can-invalid',
  'timeout',
  'permanent-eps',
  'sensors',
  'acc-fault',
  'door',
  'seatbelt',
  'steering',
  'button',
  'wrong-producer',
  'wrong-epoch',
  'old-proof',
  'renewed-proof',
  'future-source',
  'wrong-session',
  'critical-axis',
  'event-fault',
  'event-unseen',
  'native-rx',
  'native-heartbeat',
  'native-controls',
  'panda-controls',
  'panda-rx',
  'wrong-native-denial',
  'changed-inventory',
  'stale-native',
  'legacy-native',
)


def setup(reason):
  alpha = reason != 'stock-eps'
  fp = gen_empty_fingerprint()
  fp[0][0x5A] = 8
  fp[2].update({0x3D6: 8, 0x186: 8})
  with OpenpilotPrefix():
    cp = CarInterface.get_params(CAR.FORD_MONDEO_MK5, fp, [], alpha, False, False)
  cp.alternativeExperience = 32
  assert cp.safetyConfigs[0].safetyParam == 10 | int(alpha)
  assert cp.openpilotLongitudinalControl == alpha
  cs = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = cs.cruiseState.enabled = True
  feedback = TransportPauseFeedback(cp)

  def receipt(source, observed, sequence, denied):
    sm, native = sample(cp, source, observed, sequence)
    ordinary = not (denied and reason == 'alpha-brake')
    slot = replace(native.pandaInventory[0], controlsAllowed=ordinary)
    native = native_change(
      sm,
      native,
      requestedLongitudinal=alpha,
      longitudinalAllowed=alpha and not (denied and reason in ('alpha-gas', 'alpha-brake')),
      lateralAllowed=not (denied and reason in ('stock-eps', 'alpha-eps')),
      pandaInventory=(slot,),
    )
    change(sm, 'pandaStates', lambda ps: setattr(ps[0], 'controlsAllowed', ordinary))
    change(sm, 'aolAxisState', lambda a: setattr(a, 'mode', 'combined' if alpha else 'lateralOnly'))
    change(sm, 'aolAxisState', lambda a: setattr(a, 'desiredLongitudinal', alpha))
    change(sm, 'aolAxisState', lambda a: setattr(a, 'longitudinalActive', alpha))
    sm.messages['onroadEvents'] = messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=NOW).as_reader()
    return sm, native

  healthy, _ = receipt(NOW, NOW, 1, False)
  assert not feedback.observe(healthy, cp, cs, NOW, latched=True, intent_producer_session=PRODUCER)
  assert feedback.signature and feedback.healthy_inventory
  cs.gasPressed = reason == 'alpha-gas'
  cs.steerFaultTemporary = reason in ('stock-eps', 'alpha-eps')
  cs.brakePressed = reason == 'alpha-brake'
  cs.cruiseState.enabled = reason != 'alpha-brake'
  sm, native = receipt(NOW + 10_000_000, NOW + 20_000_000, 2, True)
  return cp, cs, feedback, sm, native


@pytest.mark.parametrize('reason', REASONS)
def test_first_physical_join_keeps_only_intent_with_no_state_or_lease_renewal(reason):
  cp, cs, feedback, sm, _ = setup(reason)
  before = dict(vars(feedback))
  assert retain_physical_pause_intent(feedback, sm, cp, cs, sm['aolAxisState'], NOW + 20_000_000, latched=True, producer=PRODUCER)
  assert vars(feedback) == before
  assert not feedback.observe(sm, cp, cs, NOW + 20_000_000, latched=True, intent_producer_session=PRODUCER)
  for field in (
    'sequence',
    'signature',
    'healthy_feedback_ns',
    'healthy_native_ns',
    'healthy_inventory',
    'lateral_baseline_ns',
    'lateral_source_ns',
    'lateral_native_ns',
  ):
    assert getattr(feedback, field) == before[field]
  assert not feedback.failed


@pytest.mark.parametrize('reason', REASONS)
@pytest.mark.parametrize('fault', NEGATIVES)
def test_first_join_cannot_hide_authority_health_identity_or_original_deadline_loss(reason, fault):
  cp, cs, feedback, sm, native = setup(reason)
  now, latched, producer = NOW + 20_000_000, True, PRODUCER
  physical = {'main-off': ('available', False), 'wrong-cruise-state': ('enabled', not cs.cruiseState.enabled)}
  fields = {
    'can-invalid': ('canValid', False),
    'timeout': ('canTimeout', True),
    'permanent-eps': ('steerFaultPermanent', True),
    'sensors': ('vehicleSensorsInvalid', True),
    'acc-fault': ('accFaulted', True),
    'door': ('doorOpen', True),
    'seatbelt': ('seatbeltUnlatched', True),
    'steering': ('steeringDisengage', True),
    'brake': ('brakePressed', not cs.brakePressed),
  }
  if fault == 'unlatched':
    latched = False
  elif fault == 'failed':
    feedback.failed = True
  elif fault == 'pending':
    feedback.pending_event = NOW
  elif fault == 'unqualified':
    cp.alternativeExperience = 0
  elif fault in physical:
    setattr(cs.cruiseState, *physical[fault])
  elif fault in fields:
    setattr(cs, *fields[fault])
  elif fault == 'reverse':
    cs.gearShifter = structs.CarState.GearShifter.reverse
  elif fault == 'mixed-pedal-eps':
    cs.gasPressed = cs.steerFaultTemporary = True
  elif fault == 'button':
    cs.buttonEvents = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.cancel, pressed=True)]
  elif fault == 'wrong-producer':
    producer = 'foreign'
  elif fault == 'wrong-epoch':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'intentIntegrityEpoch', 2))
  elif fault in ('old-proof', 'renewed-proof'):
    now = NOW + 40_000_000
    if fault == 'old-proof':
      now = int(sm['aolAxisState'].validUntilMonoTime) + 1
      assert now > sm['aolAxisState'].validUntilMonoTime
    if fault == 'renewed-proof':
      event = sm.messages['aolAxisState'].as_builder()
      event.logMonoTime = event.aolAxisState.observedMonoTime = event.aolAxisState.sourceCarStateMonoTime = NOW + 30_000_000
      event.aolAxisState.validUntilMonoTime = NOW + 60_000_000
      event.aolAxisState.sequence = 3
      sm.messages['aolAxisState'] = event.as_reader()
      sm.logMonoTime['aolAxisState'] = NOW + 30_000_000
  elif fault == 'future-source':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'sourceCarStateMonoTime', now + 1))
  elif fault == 'wrong-session':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'sessionId', 'foreign'))
  elif fault == 'critical-axis':
    change(sm, 'aolAxisState', lambda a: setattr(a, 'faultReason', 'critical'))
  elif fault == 'event-fault':
    event = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=NOW)
    event.onroadEvents[0].name, event.onroadEvents[0].immediateDisable = 'controlsMismatch', True
    sm.messages['onroadEvents'] = event.as_reader()
  elif fault == 'event-unseen':
    sm.seen['onroadEvents'] = False
  elif fault in ('native-rx', 'native-heartbeat', 'native-controls'):
    field = {'native-rx': 'safetyRxChecksInvalid', 'native-heartbeat': 'heartbeatLost', 'native-controls': 'controlsAllowed'}[fault]
    native_change(
      sm,
      native,
      pandaInventory=(replace(native.pandaInventory[0], **{field: not native.pandaInventory[0].controlsAllowed if fault == 'native-controls' else True}),),
    )
  elif fault in ('panda-controls', 'panda-rx'):
    field = 'controlsAllowed' if fault == 'panda-controls' else 'safetyRxChecksInvalid'
    change(sm, 'pandaStates', lambda ps: setattr(ps[0], field, not ps[0].controlsAllowed if fault == 'panda-controls' else True))
  elif fault == 'wrong-native-denial':
    native_change(sm, native, lateralAllowed=not native.lateralAllowed)
  elif fault == 'changed-inventory':
    native_change(sm, native, pandaSerial='foreign', pandaInventory=(replace(native.pandaInventory[0], hardwareSerial='foreign'),))
  elif fault == 'stale-native':
    native_change(sm, native, observedMonoTime=NOW - 300_000_000, validUntilMonoTime=NOW - 100_000_000)
  elif fault == 'legacy-native':
    native_change(sm, native, pandaInventory=(), sourcePandaStatesMonoTime=0)
  else:
    raise AssertionError(fault)
  assert not retain_physical_pause_intent(feedback, sm, cp, cs, sm['aolAxisState'], now, latched=latched, producer=producer)
  if fault in ('native-rx', 'native-heartbeat', 'panda-rx', 'wrong-producer', 'critical-axis', 'old-proof'):
    assert feedback.observe(sm, cp, cs, now, latched=latched, intent_producer_session=producer)
    assert feedback.failed


@pytest.mark.parametrize('offset', [-1, 0, 1])
def test_native_first_brake_retains_intent_only_through_original_source_deadline(offset):
  cp, cs, feedback, sm, _ = setup('alpha-brake')
  axis_before = sm['aolAxisState'].to_dict()
  native_before = bytes(sm['aolSafetyWire'])
  deadline = int(sm['aolAxisState'].validUntilMonoTime)
  now = deadline + offset
  assert feedback.observe(sm, cp, cs, now, latched=True, intent_producer_session=PRODUCER) == (offset > 0)
  assert sm['aolAxisState'].to_dict() == axis_before
  assert sm['aolAxisState'].validUntilMonoTime == deadline
  assert bytes(sm['aolSafetyWire']) == native_before
