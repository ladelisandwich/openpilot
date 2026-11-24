"""Actual caller and native receipt guards for independent lateral intent."""
from contextlib import ExitStack
from dataclasses import replace
from types import SimpleNamespace

import pytest
from openpilot.cereal import messaging
from opendbc.car.structs import car
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.runtime import current_native
from openpilot.starpilot.aol.wire import encode_safety
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


@pytest.fixture
def actual():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, _, drive, card, receive = case._ioniq_availability_callers(full_op=True)
    # Retain real10Hz PandaStates publication cadence. The route's native
    # withdrawal was already present in latestPanda before its SD decision.
    for tick in range(2, 10):
      warm_axis, warm_state, warm_command = drive(tick)
      assert warm_state.enabled and warm_axis.lateralActive and warm_axis.longitudinalActive
      assert warm_command.latActive and warm_command.longActive
    axis, _, command = drive(10, native_request_lat=True, allowed=True,
      native_request_long=True, long_allowed=False, panda_controls_allowed=False)
    assert axis.lateralActive and not axis.longitudinalActive
    assert command.latActive and not command.longActive
    now = 10_100_000_000
    assert int(sd.sm.logMonoTime['pandaStates']) == now
    assert all(not panda.controlsAllowed for panda in sd.sm['pandaStates'])
    assert receive(now, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    cs = fixture.car_state()
    cs.cruiseState.available = True
    value = SimpleNamespace(sd=sd, drive=drive, card=card, receive=receive,
      feedback=card.aol_transport_feedback, sm=card.sm, cp=card.CP, cs=cs,
      axis=card.sm['aolAxisState'].as_builder(), now=now, latched=True,
      producer=card.slc_producer_session)
    assert invoke(value) is not None
    yield value


def invoke(value):
  return value.feedback._native_long_withdrawal_lateral_current(
    value.sm, value.cp, value.cs, value.axis, value.now,
    latched=value.latched, producer=value.producer)



def assert_early_long_retention_never_grants_long(value):
  before = dict(vars(value.feedback))
  admitted = invoke(value)
  assert admitted is not None and admitted.requestedLateral and admitted.lateralAllowed
  assert admitted.requestedLongitudinal and not admitted.longitudinalAllowed
  assert vars(value.feedback) == before
  axis, _, command = value.drive(11, native_request_lat=True, allowed=True,
    native_request_long=True, long_allowed=False, panda_controls_allowed=False)
  assert axis.lateralActive and command.latActive
  assert not axis.longitudinalActive and not command.longActive


def wire(value, native):
  msg = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=native.observedMonoTime)
  msg.aolSafetyWire = encode_safety(native)
  value.sm.update_msgs(value.now / 1e9, [msg.as_reader()])


def native(value):
  result = current_native(value.sm, value.cp, now_ns=value.now,
    axis_session_id=value.feedback.session)
  assert result is not None
  return result


def test_genuine_lateral_proof_changes_no_combined_health_or_permission(actual):
  feedback = actual.feedback
  before = dict(vars(feedback))
  deadline = actual.axis.validUntilMonoTime
  assert invoke(actual) is not None
  assert vars(feedback) == before  # Helper is pure; caller records received LAT.
  _, _, command = actual.drive(11, native_request_lat=True, allowed=True,
    native_request_long=True, long_allowed=False, panda_controls_allowed=False)
  assert command.latActive and not command.longActive
  assert actual.receive(10_110_000_000, axis=actual.sd.pm.messages['aolAxisState'],
    event=actual.sd.pm.messages['onroadEvents'])
  after = vars(feedback)
  permitted = {'lateral_baseline_ns', 'lateral_source_ns', 'lateral_native_ns', 'sequence', 'signature'}
  assert {key: value for key, value in after.items() if key not in permitted} == {
    key: value for key, value in before.items() if key not in permitted}
  assert not feedback.failed
  assert actual.axis.validUntilMonoTime == deadline  # Prior Cereal proof untouched.


@pytest.mark.parametrize('field,value', (
  ('failed', True), ('paused', True), ('pending_event', 1), ('retention_started_ns', 1),
  ('session', 'foreign'), ('availability_producer', 'foreign'), ('availability_epoch', -1),
  ('lateral_inventory', None), ('lateral_baseline_ns', 0),
  ('lateral_source_ns', 10_100_000_001), ('lateral_native_ns', 10_100_000_001),
))
def test_established_proof_state_is_mandatory(actual, field, value):
  setattr(actual.feedback, field, value)
  assert invoke(actual) is None


@pytest.mark.parametrize('field,value', (
  ('sessionId', 'foreign'), ('intentProducerSessionId', 'foreign'), ('intentIntegrityEpoch', 999),
  ('faultReason', 'critical'), ('mode', 'off'), ('nativeAcknowledged', False),
  ('desiredLateral', False), ('lateralActive', False), ('longitudinalActive', True),
  ('sourceCarStateMonoTime', 0), ('sourceCarStateMonoTime', 10_100_000_001),
  ('observedMonoTime', 10_100_000_001),
))
def test_received_axis_proof_guards(actual, field, value):
  setattr(actual.axis, field, value)
  if field in ('mode', 'longitudinalActive'):
    # The qualified early-LONG owner reads native permission independently of
    # these historical combined-axis metadata fields. It grants no LONG output.
    assert_early_long_retention_never_grants_long(actual)
  else:
    assert invoke(actual) is None


@pytest.mark.parametrize('field', ('canTimeout', 'steerFaultTemporary', 'steerFaultPermanent',
                                  'accFaulted', 'steeringDisengage'))
def test_vehicle_faults_reject(actual, field):
  setattr(actual.cs, field, True)
  assert invoke(actual) is None


def test_unlatched_wrong_producer_invalid_can_and_physical_cancel_reject(actual):
  actual.latched = False
  assert invoke(actual) is None
  actual.latched = True
  actual.producer = 'foreign'
  assert invoke(actual) is None
  actual.producer = actual.card.slc_producer_session
  actual.cs.canValid = False
  assert invoke(actual) is None
  actual.cs.canValid = True
  actual.cs.buttonEvents = [car.CarState.ButtonEvent(type=car.CarState.ButtonEvent.Type.cancel, pressed=False)]
  assert invoke(actual) is None


@pytest.mark.parametrize('change', ('lat_request', 'lat_grant', 'long_request', 'long_grant',
                                   'inventory', 'owner_controls', 'serial', 'native_rx',
                                   'native_heartbeat', 'native_fault', 'native_param'))
def test_actual_native_receipt_must_prove_exact_independent_lateral_only_health(actual, change):
  original = native(actual)
  owners = [index for index, config in enumerate(actual.cp.safetyConfigs)
    if str(config.safetyModel) not in ('silent', 'noOutput')]
  assert len(owners) == 1
  owner = owners[0]
  updates = {'lat_request': {'requestedLateral': False}, 'lat_grant': {'lateralAllowed': False},
    'long_request': {'requestedLongitudinal': False}, 'long_grant': {'longitudinalAllowed': True},
    'inventory': {'pandaInventory': (), 'sourcePandaStatesMonoTime': 0}, 'serial': {'pandaSerial': 'foreign'}}
  if change in updates:
    changed = replace(original, **updates[change])
  else:
    fields = {'owner_controls': {'controlsAllowed': True}, 'native_rx': {'safetyRxChecksInvalid': True},
      'native_heartbeat': {'heartbeatLost': True}, 'native_fault': {'faults': 1},
      'native_param': {'safetyParam': original.pandaInventory[owner].safetyParam ^ 1}}
    slots = list(original.pandaInventory)
    slots[owner] = replace(slots[owner], **fields[change])
    changed = replace(original, pandaInventory=tuple(slots))
  wire(actual, changed)
  if change == 'owner_controls':
    assert_early_long_retention_never_grants_long(actual)
  else:
    assert invoke(actual) is None


@pytest.mark.parametrize('change', ('controls', 'rx', 'heartbeat', 'fault', 'param'))
def test_latest_panda_veto_stays_independent(actual, change):
  message = messaging.new_message('pandaStates', len(actual.sm['pandaStates']), valid=True,
    logMonoTime=actual.sm.logMonoTime['pandaStates'])
  message.pandaStates = actual.sm['pandaStates']
  owner = next(index for index, config in enumerate(actual.cp.safetyConfigs)
    if str(config.safetyModel) not in ('silent', 'noOutput'))
  field, value = {'controls': ('controlsAllowed', True), 'rx': ('safetyRxChecksInvalid', True),
    'heartbeat': ('heartbeatLost', True), 'fault': ('faults', ['relayMalfunction']),
    'param': ('safetyParam', int(message.pandaStates[owner].safetyParam) ^ 1)}[change]
  setattr(message.pandaStates[owner], field, value)
  actual.sm.update_msgs(actual.now / 1e9, [message.as_reader()])
  if change == 'controls':
    assert_early_long_retention_never_grants_long(actual)
  else:
    assert invoke(actual) is None


@pytest.mark.parametrize('kind', ('immediateDisable', 'softDisable', 'userDisable'))
def test_no_control_event_is_exempted(actual, kind):
  # A real new envelope has a later receive clock. Reusing the exact prior
  # receive timestamp would exercise FrequencyTracker's zero-period fixture.
  actual.now += 1
  message = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=actual.now)
  message.onroadEvents[0].name = 'controlsMismatch'
  setattr(message.onroadEvents[0], kind, True)
  actual.sm.update_msgs(actual.now / 1e9, [message.as_reader()])
  assert invoke(actual) is None


@pytest.mark.parametrize('field', ('seen', 'valid', 'alive'))
def test_event_and_native_services_must_be_current(actual, field):
  getattr(actual.sm, field)['onroadEvents'] = False
  assert invoke(actual) is None


def test_native_lease_and_source_age_cannot_be_extended(actual):
  original = native(actual)
  actual.now = original.validUntilMonoTime + 1
  assert invoke(actual) is None


def test_changed_current_cp_cannot_reuse_native_inventory(actual):
  actual.cp = car.CarParams(**actual.cp.to_dict())
  actual.cp.alternativeExperience ^= 1
  assert invoke(actual) is None


@pytest.mark.parametrize('field', ('seen', 'valid', 'alive'))
def test_missing_native_service_cannot_reuse_lateral_health(actual, field):
  getattr(actual.sm, field)['aolSafetyWire'] = False
  assert invoke(actual) is None
