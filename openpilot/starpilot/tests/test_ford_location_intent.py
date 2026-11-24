"""Controlled wire boundaries; actual Ford/native caller coverage runs separately."""
from dataclasses import replace

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample


IDENTITIES = (
  CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_EDGE_MK2, CAR.FORD_ESCAPE_MK4, CAR.FORD_ESCAPE_MK4_5,
  CAR.FORD_EXPLORER_MK6, CAR.FORD_EXPEDITION_MK4, CAR.FORD_F_150_MK14, CAR.FORD_F_150_LIGHTNING_MK1,
  CAR.FORD_FOCUS_MK4, CAR.FORD_MONDEO_MK5, CAR.FORD_MAVERICK_MK1, CAR.FORD_MUSTANG_MACH_E_MK1,
  CAR.FORD_RANGER_MK2, CAR.FORD_TRANSIT_MK5,
)


def location_case(identity=CAR.FORD_ESCAPE_MK4, alpha=False):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x5A] = 8
  fingerprint[2].update({0x3D6: 8, 0x186: 8})
  with OpenpilotPrefix():
    cp = CarInterface.get_params(identity, fingerprint, [], alpha, False, False)
  cp.alternativeExperience = 32
  cs = structs.CarState(canValid=True, canTimeout=False, vEgo=15., gearShifter='drive')
  cs.cruiseState.available = True
  feedback = TransportPauseFeedback(cp)
  healthy, _ = sample(cp, NOW, NOW, 1)
  assert not feedback.observe(healthy, cp, cs, NOW, latched=True, intent_producer_session=PRODUCER)
  assert feedback.lateral_only_baseline
  stamp = NOW + 10_000_000
  peers, native = sample(cp, stamp, stamp, 2, active=False, requested=False, allowed=False, event_stamp=stamp)
  def withdrawn(axis):
    axis.faultReason = 'critical'
    axis.nativeAcknowledged = False
  change(peers, 'aolAxisState', withdrawn)
  events = messaging.new_message('onroadEvents', 2, valid=True, logMonoTime=stamp)
  events.onroadEvents = [
    {'name': 'pcmDisable', 'userDisable': True},
    {'name': 'locationdTemporaryError', 'softDisable': True, 'noEntry': True},
  ]
  peers.messages['onroadEvents'] = events.as_reader()
  peers.logMonoTime['onroadEvents'] = stamp
  return cp, cs, feedback, peers, native, stamp


@pytest.mark.parametrize('identity', IDENTITIES)
@pytest.mark.parametrize('alpha', [False, True], ids=['stock', 'alpha-after-brake'])
@pytest.mark.parametrize('event_first', [False, True])
def test_location_loss_retains_only_existing_ford_lateral_intent(identity, alpha, event_first):
  cp, cs, feedback, peers, _, stamp = location_case(identity, alpha)
  baseline = (feedback.lateral_baseline_ns, feedback.lateral_source_ns, feedback.lateral_native_ns,
              feedback.healthy_feedback_ns, feedback.healthy_native_ns)
  sources = dict(peers.logMonoTime)
  if event_first:
    early, _ = sample(cp, NOW, stamp, 1)
    early.messages['onroadEvents'] = peers.messages['onroadEvents']
    early.logMonoTime['onroadEvents'] = stamp
    assert not feedback.observe(early, cp, cs, stamp, latched=True, intent_producer_session=PRODUCER)
    # The earlier SOFT event is owned by the shared ordinary-warning join.
    # The inactive, matched location proof below still uses the host pause owner.
    assert feedback.ordinary_warning_qualified and feedback.ordinary_warning_retained
    assert not feedback.temporary_host_qualified and not feedback.failed
    assert feedback.lateral_only_baseline and feedback.pending_event == 0
    assert baseline == (feedback.lateral_baseline_ns, feedback.lateral_source_ns, feedback.lateral_native_ns,
                        feedback.healthy_feedback_ns, feedback.healthy_native_ns)
  assert not feedback.observe(peers, cp, cs, stamp, latched=True, intent_producer_session=PRODUCER)
  assert feedback.temporary_host_qualified and not feedback.failed
  assert baseline == (feedback.lateral_baseline_ns, feedback.lateral_source_ns, feedback.lateral_native_ns,
                      feedback.healthy_feedback_ns, feedback.healthy_native_ns)
  assert peers.logMonoTime == sources
  assert not any((peers['aolAxisState'].desiredLateral, peers['aolAxisState'].lateralActive,
                  peers['aolAxisState'].desiredLongitudinal, peers['aolAxisState'].longitudinalActive))
  recovered, _ = sample(cp, stamp + 10_000_000, stamp + 10_000_000, 3, event_stamp=stamp + 10_000_000)
  assert not feedback.observe(recovered, cp, cs, stamp + 10_000_000, latched=True, intent_producer_session=PRODUCER)
  assert not feedback.failed and not feedback.temporary_host_qualified
  assert feedback.lateral_source_ns == stamp + 10_000_000


@pytest.mark.parametrize('fault', [
  'unlatched', 'invalid-can', 'can-timeout', 'main-off', 'brake', 'regen', 'gas', 'door', 'seatbelt',
  'permanent-eps', 'temporary-eps', 'reverse', 'unknown-gear', 'explicit-cancel', 'hard-fault',
  'producer', 'epoch', 'session', 'future-source', 'stale-source', 'native-rx', 'native-heartbeat',
  'interrupt-can2', 'native-identity', 'native-denial', 'long-active', 'aol-off',
])
def test_location_exception_cannot_hide_independent_fault_or_driver_veto(fault):
  cp, cs, feedback, peers, native, stamp = location_case()
  latched, producer = True, PRODUCER
  if fault == 'unlatched':
    latched = False
  elif fault in ('invalid-can', 'can-timeout', 'main-off', 'brake', 'regen', 'gas', 'door', 'seatbelt',
                 'permanent-eps', 'temporary-eps', 'reverse', 'unknown-gear'):
    if fault == 'main-off':
      cs.cruiseState.available = False
    elif fault in ('reverse', 'unknown-gear'):
      cs.gearShifter = 'reverse' if fault == 'reverse' else 'unknown'
    else:
      field = {'invalid-can': 'canValid', 'can-timeout': 'canTimeout', 'brake': 'brakePressed',
               'regen': 'regenBraking', 'gas': 'gasPressed', 'door': 'doorOpen', 'seatbelt': 'seatbeltUnlatched',
               'permanent-eps': 'steerFaultPermanent', 'temporary-eps': 'steerFaultTemporary'}[fault]
      setattr(cs, field, fault != 'invalid-can')
  elif fault in ('explicit-cancel', 'hard-fault'):
    if fault == 'explicit-cancel':
      change(peers, 'onroadEvents', lambda events: setattr(events[0], 'name', 'buttonCancel'))
    else:
      change(peers, 'onroadEvents', lambda events: setattr(events[0], 'immediateDisable', True))
  elif fault == 'producer':
    producer = 'other-card'
  elif fault in ('epoch', 'session', 'future-source', 'stale-source', 'long-active'):
    field, value = {'epoch': ('intentIntegrityEpoch', 2), 'session': ('sessionId', 'other-axis'),
                    'future-source': ('sourceCarStateMonoTime', stamp + 1),
                    'stale-source': ('sourceCarStateMonoTime', stamp - 30_000_001),
                    'long-active': ('longitudinalActive', True)}[fault]
    change(peers, 'aolAxisState', lambda axis: setattr(axis, field, value))
  elif fault == 'native-identity':
    native_change(peers, native, pandaSerial='other-panda')
  elif fault == 'native-denial':
    native_change(peers, native, requestedLateral=True, lateralAllowed=False)
  elif fault == 'aol-off':
    cp.alternativeExperience = 0
  else:
    updates = {'native-rx': {'safetyRxChecksInvalid': True}, 'native-heartbeat': {'heartbeatLost': True},
               'interrupt-can2': {'faults': 8}}[fault]
    native_change(peers, native, pandaInventory=(replace(native.pandaInventory[0], **updates),))
  before = dict(vars(feedback))
  assert not feedback._temporary_host_intent_current(peers, cp, cs, peers['aolAxisState'], stamp, stamp,
                                                    latched=latched, producer=producer)
  assert vars(feedback) == before


@pytest.mark.parametrize('fault', ['unlatched', 'invalid-can', 'can-timeout', 'main-off', 'door', 'permanent-eps',
                                 'native-denial', 'native-rx', 'native-heartbeat', 'interrupt-can2',
                                 'producer', 'epoch', 'session', 'expired-native', 'future-event',
                                 'explicit-cancel', 'hard-fault'])
def test_event_first_ordinary_warning_rejects_independent_faults_without_mutation(fault):
  cp, cs, feedback, withdrawn, _, stamp = location_case()
  early, native = sample(cp, NOW, stamp, 1)
  early.messages['onroadEvents'] = withdrawn.messages['onroadEvents']
  early.logMonoTime['onroadEvents'] = stamp
  latched, producer = fault != 'unlatched', PRODUCER
  if fault in ('invalid-can', 'can-timeout', 'main-off', 'door', 'permanent-eps'):
    if fault == 'main-off':
      cs.cruiseState.available = False
    else:
      field = {'invalid-can': 'canValid', 'can-timeout': 'canTimeout',
               'door': 'doorOpen', 'permanent-eps': 'steerFaultPermanent'}[fault]
      setattr(cs, field, fault != 'invalid-can')
  elif fault == 'native-denial':
    native_change(early, native, requestedLateral=True, lateralAllowed=False)
  elif fault in ('native-rx', 'native-heartbeat', 'interrupt-can2'):
    updates = {'native-rx': {'safetyRxChecksInvalid': True}, 'native-heartbeat': {'heartbeatLost': True},
               'interrupt-can2': {'faults': 8}}[fault]
    native_change(early, native, pandaInventory=(replace(native.pandaInventory[0], **updates),))
  elif fault == 'producer':
    producer = 'foreign-card'
  elif fault in ('epoch', 'session'):
    field, value = ('intentIntegrityEpoch', 2) if fault == 'epoch' else ('sessionId', 'foreign-axis')
    change(early, 'aolAxisState', lambda axis: setattr(axis, field, value))
  elif fault == 'expired-native':
    native_change(early, native, validUntilMonoTime=stamp - 1)
  elif fault == 'future-event':
    early.logMonoTime['onroadEvents'] = stamp + 1
  elif fault == 'explicit-cancel':
    change(early, 'onroadEvents', lambda events: setattr(events[0], 'name', 'buttonCancel'))
  elif fault == 'hard-fault':
    change(early, 'onroadEvents', lambda events: setattr(events[0], 'immediateDisable', True))
  before = dict(vars(feedback))
  assert not feedback._ordinary_warning_intent_current(early, cp, cs, early['aolAxisState'], NOW, stamp,
                                                      latched=latched, producer=producer)
  assert vars(feedback) == before
