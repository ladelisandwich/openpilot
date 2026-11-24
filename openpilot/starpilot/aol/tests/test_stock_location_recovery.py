"""Stock location withdrawal retains intent, with explicit native/source vetoes.

Native wire vectors here supplement the separately qualified real CAN callers.
"""

from dataclasses import replace

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface as ChryslerInterface
from opendbc.car.chrysler.values import CAR as Chrysler
from opendbc.car.ford.interface import CarInterface as FordInterface
from opendbc.car.ford.values import CAR as Ford
from opendbc.car.hyundai.interface import CarInterface as HyundaiInterface
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.hyundai.values import CAR as Hyundai
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.runtime import current_native
from openpilot.starpilot.aol.vehicle import policy_for
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample


@pytest.fixture(params=(Chrysler.CHRYSLER_PACIFICA_2018, Chrysler.RAM_1500_5TH_GEN,
                        Ford.FORD_MUSTANG_MACH_E_MK1, Ford.FORD_ESCAPE_MK4, Hyundai.HYUNDAI_SONATA_HYBRID))
def established_stock(request):
  interface = FordInterface if request.param in (Ford.FORD_MUSTANG_MACH_E_MK1, Ford.FORD_ESCAPE_MK4) else ChryslerInterface
  fp = gen_empty_fingerprint()
  fp[0][0x5A] = 8
  if request.param == Hyundai.HYUNDAI_SONATA_HYBRID:
    interface = HyundaiInterface
    fp[0].update({0x391: 8, 0x38D: 8})
    fp[2][0x485] = 4
    fp[1][RADAR_START_ADDR] = 8
  with OpenpilotPrefix():
    cp = interface.get_params(request.param, fp, [], False, False, False)
  # Use the same admitted factory policy additions as Card. Wire vectors
  # remain controlled; the separate actual CAN caller qualifies the native owner.
  policy = policy_for(cp)
  cp.safetyConfigs[0].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  assert cp.alternativeExperience == 32
  assert cp.pcmCruise and not cp.openpilotLongitudinalControl
  state = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive, vEgo=15.)
  state.cruiseState.available = True
  feedback = TransportPauseFeedback(cp)
  sm, _ = sample(cp, NOW, NOW, 1)
  assert not feedback.observe(sm, cp, state, NOW, latched=True, intent_producer_session=PRODUCER)
  assert feedback.lateral_only_baseline
  return cp, state, feedback


def location_pause(cp):
  now = NOW + 10_000_000
  sm, native = sample(cp, now, now, 2, active=False, requested=False, allowed=False, event_stamp=now)
  change(sm, 'aolAxisState', lambda axis: setattr(axis, 'faultReason', 'critical'))
  event = sm.messages['onroadEvents'].as_builder()
  event.onroadEvents = [{'name': 'pcmDisable', 'userDisable': True},
                        {'name': 'locationdTemporaryError', 'softDisable': True}]
  sm.messages['onroadEvents'] = event.as_reader()
  return sm, native, now


def test_location_pause_retains_only_existing_intent_and_requires_fresh_recovery(established_stock):
  cp, state, feedback = established_stock
  sm, native, now = location_pause(cp)
  before = (feedback.healthy_feedback_ns, feedback.healthy_native_ns, feedback.lateral_baseline_ns,
            feedback.lateral_source_ns, feedback.lateral_native_ns)
  assert not feedback.observe(sm, cp, state, now, latched=True, intent_producer_session=PRODUCER)
  assert feedback.temporary_host_qualified and not feedback.failed
  assert before == (feedback.healthy_feedback_ns, feedback.healthy_native_ns, feedback.lateral_baseline_ns,
                    feedback.lateral_source_ns, feedback.lateral_native_ns)
  assert current_native(sm, cp, now_ns=now, axis_session_id=feedback.session) == native
  assert not native.lateralAllowed and not native.longitudinalAllowed
  fresh, recovered = sample(cp, now + 10_000_000, now + 10_000_000, 3, event_stamp=now + 10_000_000)
  assert not feedback.observe(fresh, cp, state, now + 10_000_000, latched=True, intent_producer_session=PRODUCER)
  assert recovered.lateralAllowed and not recovered.longitudinalAllowed


@pytest.mark.parametrize('fault', ('can', 'timeout', 'permanent_eps', 'temporary_eps', 'main_off', 'cancel',
  'brake', 'regen', 'gas', 'door', 'seatbelt', 'reverse', 'unknown_gear', 'native_denial', 'native_invalid',
  'native_serial', 'native_fault', 'native_heartbeat', 'hard_event', 'extra_user',
  'wrong_epoch', 'wrong_producer', 'wrong_session', 'expired', 'future_source', 'no_baseline', 'unlatched'))
def test_location_pause_keeps_independent_fault_and_identity_vetoes(established_stock, fault):
  cp, state, feedback = established_stock
  sm, native, now = location_pause(cp)
  physical = {'can': ('canValid', False), 'timeout': ('canTimeout', True),
    'permanent_eps': ('steerFaultPermanent', True), 'temporary_eps': ('steerFaultTemporary', True),
    'brake': ('brakePressed', True), 'regen': ('regenBraking', True), 'gas': ('gasPressed', True),
    'door': ('doorOpen', True), 'seatbelt': ('seatbeltUnlatched', True)}
  producer = PRODUCER
  if fault in physical:
    setattr(state, *physical[fault])
  elif fault == 'main_off':
    state.cruiseState.available = False
  elif fault == 'cancel':
    state.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
  elif fault in ('reverse', 'unknown_gear'):
    state.gearShifter = 'reverse' if fault == 'reverse' else 'unknown'
  elif fault == 'native_denial':
    native_change(sm, native, requestedLateral=True)
  elif fault == 'native_invalid':
    sm.valid['aolSafetyWire'] = False
  elif fault == 'native_serial':
    native_change(sm, native, pandaSerial='other', pandaInventory=(replace(native.pandaInventory[0], hardwareSerial='other'),))
  elif fault in ('native_fault', 'native_heartbeat', 'native_controls'):
    field = {'native_fault': 'faults', 'native_heartbeat': 'heartbeatLost', 'native_controls': 'controlsAllowed'}[fault]
    native_change(sm, native, pandaInventory=(replace(native.pandaInventory[0], **{field: 1 if field == 'faults' else True}),))
  elif fault == 'hard_event':
    change(sm, 'onroadEvents', lambda events: setattr(events[1], 'immediateDisable', True))
  elif fault == 'extra_user':
    change(sm, 'onroadEvents', lambda events: setattr(events[0], 'name', log.OnroadEvent.EventName.buttonCancel))
  elif fault == 'wrong_epoch':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch', 2))
  elif fault == 'wrong_producer':
    producer = 'other'
  elif fault == 'wrong_session':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'sessionId', 'other'))
  elif fault == 'expired':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'validUntilMonoTime', now - 1))
  elif fault == 'future_source':
    change(sm, 'aolAxisState', lambda axis: setattr(axis, 'sourceCarStateMonoTime', now + 1))
  elif fault == 'no_baseline':
    feedback.lateral_baseline_ns = 0
  assert feedback.observe(sm, cp, state, now, latched=fault != 'unlatched', intent_producer_session=producer)
  assert not feedback.temporary_host_qualified
