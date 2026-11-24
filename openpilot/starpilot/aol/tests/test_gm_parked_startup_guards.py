"""Parked intent exemptions require fresh source, idle native axes and known events."""

from dataclasses import replace

import pytest
from opendbc.car.structs import car
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from openpilot.cereal import messaging
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.vehicle import policy_for
from openpilot.starpilot.aol.wire import IntentState, PandaSlot, SafetyState, encode_safety

NOW = 10_000_000_000


def setup():
  cp = sdgm_params(alpha=False)
  policy = policy_for(cp)
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  cs = car.CarState(canValid=True, gearShifter='park', standstill=True, vEgo=0.0, seatbeltUnlatched=True)
  cs.cruiseState.available = True
  intent = IntentState('producer', 1, NOW, NOW, NOW + 200_000_000, False, False, False, True, True)
  sm = messaging.SubMaster(['aolSafetyWire', 'pandaStates', 'aolAxisState', 'onroadEvents'])
  events = Events()
  events.add(EventName.wrongGear)
  events.add(EventName.seatbeltNotLatched)
  slot = PandaSlot(0, 'panda', 4, cp.safetyConfigs[0].safetyParam, cp.alternativeExperience)
  native = SafetyState(1, True, NOW, NOW + 200_000_000, 4, cp.safetyConfigs[0].safetyParam, False, False, False, False, 'panda', 'session', NOW, (slot,))

  def publish(native=native):
    wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=NOW)
    wire.aolSafetyWire = encode_safety(native)
    panda = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=NOW)
    p = panda.pandaStates[0]
    p.safetyModel, p.safetyParam, p.alternativeExperience = 4, cp.safetyConfigs[0].safetyParam, cp.alternativeExperience
    event = messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=NOW)
    event.onroadEvents = events.to_msg()
    sm.update_msgs(NOW / 1e9 + (sm.frame + 1) * 0.000001, [wire.as_reader(), panda.as_reader(), event.as_reader()])

  publish()
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP, sd.sm, sd.events = cp, sm, events
  sd.enabled = sd.active = sd.aol_ever_active = sd.aol_authority_lost = False
  sd.initialized = sd.conditional_car_state_valid = True
  sd.aol_car_state_log_ns, sd.aol_session_id = NOW, 'session'
  return sd, cs, intent, native, publish


@pytest.mark.parametrize(
  'field,value',
  [
    ('settingsQualified', False),
    ('lateralArmed', False),
    ('allowedLatch', True),
    ('pauseLateral', True),
    ('pauseLongitudinal', True),
    ('carStateLogMonoTime', NOW - 1),
    ('observedMonoTime', NOW + 1),
    ('validUntilMonoTime', NOW - 1),
  ],
)
def test_source_intent_must_match_current_park(field, value):
  sd, cs, intent, _, _ = setup()
  assert sd.cold_gm_park_readiness(cs, intent, NOW)
  assert not sd.cold_gm_park_readiness(cs, replace(intent, **{field: value}), NOW)


@pytest.mark.parametrize(
  'field,value',
  [
    ('initialized', False),
    ('enabled', True),
    ('active', True),
    ('aol_ever_active', True),
    ('aol_authority_lost', True),
    ('conditional_car_state_valid', False),
  ],
)
def test_only_cold_inactive_source_can_wait(field, value):
  sd, cs, intent, _, _ = setup()
  assert sd.cold_gm_park_readiness(cs, intent, NOW)
  setattr(sd, field, value)
  assert not sd.cold_gm_park_readiness(cs, intent, NOW)


@pytest.mark.parametrize(
  'field,value',
  [
    ('gearShifter', 'unknown'),
    ('gearShifter', 'drive'),
    ('gearShifter', 'reverse'),
    ('standstill', False),
    ('vEgo', 0.31),
    ('vEgo', float('nan')),
    ('canValid', False),
    ('canTimeout', True),
    ('steerFaultTemporary', True),
    ('steerFaultPermanent', True),
    ('accFaulted', True),
    ('doorOpen', True),
    ('gasPressed', True),
    ('steeringDisengage', True),
  ],
)
def test_vehicle_faults_cannot_be_classified_as_park_readiness(field, value):
  sd, cs, intent, _, _ = setup()
  assert sd.cold_gm_park_readiness(cs, intent, NOW)
  setattr(cs, field, value)
  assert not sd.cold_gm_park_readiness(cs, intent, NOW)


@pytest.mark.parametrize('field', ['requestedLateral', 'requestedLongitudinal', 'lateralAllowed', 'longitudinalAllowed'])
def test_idle_ack_never_accepts_a_native_axis(field):
  sd, cs, intent, native, publish = setup()
  publish(replace(native, **{field: True}))
  assert not sd.cold_gm_park_readiness(cs, intent, NOW)


@pytest.mark.parametrize(
  'event',
  [
    EventName.commIssue,
    EventName.commIssueAvgFreq,
    EventName.locationdTemporaryError,
    EventName.controlsMismatch,
    EventName.processNotRunning,
    EventName.selfdrivedLagging,
  ],
)
def test_only_known_startup_host_events_can_wait(event):
  sd, cs, intent, _, _ = setup()
  sd.events.add(event)
  assert sd.cold_gm_park_readiness(cs, intent, NOW) == (event in (EventName.commIssue, EventName.commIssueAvgFreq, EventName.locationdTemporaryError))


@pytest.mark.parametrize('denial', [None, 'producer', 'epoch', 'source', 'expired', 'events', 'native', 'failed', 'active'])
def test_card_requires_same_identity_fresh_idle_proof(denial):
  sd, cs, _, native, publish = setup()
  sm = sd.sm
  feedback = TransportPauseFeedback(sd.CP)

  def axis(sequence=1):
    event = messaging.new_message('aolAxisState', valid=True, logMonoTime=NOW)
    a = event.aolAxisState
    a.qualified = a.nativeAcknowledged = True
    a.sessionId = a.faultSessionId = 'session'
    a.intentProducerSessionId = 'producer'
    a.sequence, a.observedMonoTime, a.validUntilMonoTime = sequence, NOW, NOW + 30_000_000
    a.sourceCarStateMonoTime = a.faultEventMonoTime = NOW
    a.faultReason = 'none'
    return event

  a = axis()
  sm.update_msgs(NOW / 1e9 + (sm.frame + 1) * 0.000001, [a.as_reader()])
  assert not feedback.observe(sm, sd.CP, cs, NOW, latched=True, intent_producer_session='producer')
  assert feedback.temporary_host_qualified
  a = axis(2)
  if denial == 'producer':
    a.aolAxisState.intentProducerSessionId = 'other'
  if denial == 'epoch':
    a.aolAxisState.intentIntegrityEpoch = 1
  if denial == 'source':
    a.aolAxisState.sourceCarStateMonoTime = NOW - 30_000_001
  if denial == 'expired':
    a.aolAxisState.validUntilMonoTime = NOW - 1
  if denial == 'native':
    publish(replace(native, requestedLateral=True, lateralAllowed=True))
  if denial == 'failed':
    feedback.failed = True
  if denial == 'active':
    feedback.ever_lateral_active = True
  sm.update_msgs(NOW / 1e9 + (sm.frame + 1) * 0.000001, [a.as_reader()])
  if denial == 'events':
    sm.valid['onroadEvents'] = False
  feedback.observe(sm, sd.CP, cs, NOW, latched=True, intent_producer_session='producer')
  assert feedback.temporary_host_qualified == (denial is None)


@pytest.mark.parametrize(
  'name,soft,expected',
  [
    ('wrongGear', True, True),
    ('wrongGear', False, False),
    ('seatbeltNotLatched', True, False),
    ('buttonCancel', False, False),
  ],
)
def test_only_known_park_gear_user_disable_can_keep_idle_intent(name, soft, expected):
  from openpilot.starpilot.car.gm.aol import cold_park_readiness

  sd, cs, _, native, _ = setup()
  event = messaging.new_message('onroadEvents', 1).onroadEvents[0]
  event.name, event.userDisable, event.softDisable, event.noEntry = name, True, soft, True
  assert cold_park_readiness(sd.CP, cs, native, [event]) == expected
