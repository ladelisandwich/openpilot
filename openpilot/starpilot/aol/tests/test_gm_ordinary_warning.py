"""Parsed SDGM sources and serialized peers; native responses are controlled fixtures."""

from dataclasses import replace

import pytest
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.starpilot.aol.tests.test_gm_held_native import NOW, Peers, peers, _sdgm_pedal_state
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback


@pytest.mark.parametrize('c9', (False, True))
@pytest.mark.parametrize('warning', ('calibrationInvalid', 'lowMemory', 'processNotRunning', 'radarTempUnavailable'))
@pytest.mark.parametrize('native_state', ('held_full', 'withdrawn_long', 'all_off'))
def test_sdgm_warning_withdrawal_keeps_only_intent(c9, warning, native_state):
  from openpilot.starpilot.aol.tests.test_gm import TestGmAol as GmFixture

  with OpenpilotPrefix():
    saved = Params()
    saved.put_bool('AlwaysOnLateral', True, block=True)
    card = GmFixture.card(sdgm_params(alpha=True, brake_c9=c9), saved)
    try:
      cp = card.CP
      feedback = TransportPauseFeedback(cp)
      cs = _sdgm_pedal_state(card, NOW, 0)
      card.aol_card_intent.update(cs, fault_active=False, now_ns=NOW)
      sm, native = peers(cp, card.slc_producer_session, NOW)
      assert not feedback.observe(sm, cp, cs, NOW, latched=True, intent_producer_session=card.slc_producer_session)
      assert feedback.lateral_baseline_ns == NOW
      for tick in range(1, 6):
        now = NOW + tick * 10_000_000
        cs = _sdgm_pedal_state(card, now, tick)
        controls = native_state == 'held_full'
        current_native = replace(
          native,
          longitudinalAllowed=controls,
          lateralAllowed=native_state != 'all_off',
          requestedLateral=native_state != 'all_off',
          requestedLongitudinal=native_state != 'all_off',
          pandaInventory=tuple(replace(slot, controlsAllowed=controls) for slot in native.pandaInventory),
        )
        sm, _ = peers(cp, card.slc_producer_session, now, native=current_native, controls=controls, sequence=tick + 1, long_active=False)
        axis = sm.messages['aolAxisState'].as_builder()
        axis.aolAxisState.faultReason = 'availabilityPause'
        axis.aolAxisState.faultEventMonoTime = now
        axis.aolAxisState.desiredLateral = False
        axis.aolAxisState.lateralActive = False
        axis.aolAxisState.nativeAcknowledged = False
        events = Events()
        events.add(getattr(EventName, warning))
        event = messaging.new_message('onroadEvents', len(events), valid=True, logMonoTime=now)
        event.onroadEvents = events.to_msg()
        sm = Peers([sm.messages['aolSafetyWire'], sm.messages['pandaStates'], axis.as_reader(), event.as_reader()])
        if not c9 and warning == 'lowMemory' and native_state == 'all_off' and tick == 1:
          _check_warning_guards(feedback, sm, cp, cs, now, card.slc_producer_session)
        assert not feedback.observe(sm, cp, cs, now, latched=True, intent_producer_session=card.slc_producer_session), (native_state, feedback.__dict__)
        assert feedback.ordinary_warning_qualified and not feedback.failed
        card.aol_card_intent.update(cs, fault_active=False, now_ns=now)
        assert card.aol_card_intent.allowed_latch
        # Retention does not write back or manufacture native authority.
        assert current_native == replace(
          native,
          longitudinalAllowed=controls,
          lateralAllowed=native_state != 'all_off',
          requestedLateral=native_state != 'all_off',
          requestedLongitudinal=native_state != 'all_off',
          pandaInventory=tuple(replace(slot, controlsAllowed=controls) for slot in native.pandaInventory),
        )
    finally:
      card.vehicle_startup.close()


def _check_warning_guards(feedback, sm, cp, cs, now, producer):
  from copy import copy
  from openpilot.starpilot.aol.wire import decode_safety, encode_safety

  for fault in (
    'unarmed',
    'producer',
    'epoch',
    'session',
    'source_stale',
    'source_future',
    'can_invalid',
    'can_timeout',
    'permanent_eps',
    'acc_fault',
    'door',
    'steering_off',
    'main_off',
    'gear_unknown',
    'native_expired',
    'native_future',
    'inventory',
    'native_rx',
    'heartbeat',
    'panda_rx',
    'panda_heartbeat',
    'immediate',
    'event_expired',
    'event_future',
  ):
    tested = copy(feedback)
    messages = {name: event.as_builder() for name, event in sm.messages.items()}
    state = cs.as_reader().as_builder()
    axis = messages['aolAxisState'].aolAxisState
    native = decode_safety(messages['aolSafetyWire'].aolSafetyWire)
    latched = fault != 'unarmed'
    if fault == 'producer':
      axis.intentProducerSessionId = 'different'
    if fault == 'epoch':
      axis.intentIntegrityEpoch += 1
    if fault == 'session':
      axis.sessionId = 'different'
    if fault == 'source_stale':
      axis.sourceCarStateMonoTime = now - 30_000_001
    if fault == 'source_future':
      axis.sourceCarStateMonoTime = now + 1
    if fault == 'can_invalid':
      state.canValid = False
    if fault == 'can_timeout':
      state.canTimeout = True
    if fault == 'permanent_eps':
      state.steerFaultPermanent = True
    if fault == 'acc_fault':
      state.accFaulted = True
    if fault == 'door':
      state.doorOpen = True
    if fault == 'steering_off':
      state.steeringDisengage = True
    if fault == 'main_off':
      state.cruiseState.available = False
    if fault == 'gear_unknown':
      state.gearShifter = 'unknown'
    if fault == 'native_expired':
      native = replace(native, validUntilMonoTime=now - 1)
    if fault == 'native_future':
      native = replace(native, observedMonoTime=now + 1, validUntilMonoTime=now + 200_000_001)
      messages['aolSafetyWire'].logMonoTime = now + 1
    if fault == 'inventory':
      native = replace(native, pandaInventory=tuple(replace(slot, hardwareSerial='other') for slot in native.pandaInventory))
    if fault == 'native_rx':
      native = replace(native, pandaInventory=tuple(replace(slot, safetyRxChecksInvalid=True) for slot in native.pandaInventory))
    if fault == 'heartbeat':
      native = replace(native, pandaInventory=tuple(replace(slot, heartbeatLost=True) for slot in native.pandaInventory))
    if fault == 'panda_rx':
      messages['pandaStates'].pandaStates[0].safetyRxChecksInvalid = True
    if fault == 'panda_heartbeat':
      messages['pandaStates'].pandaStates[0].heartbeatLost = True
    if fault == 'immediate':
      messages['onroadEvents'].onroadEvents[0].immediateDisable = True
    if fault == 'event_expired':
      messages['onroadEvents'].logMonoTime = now - 1_500_000_001
    if fault == 'event_future':
      messages['onroadEvents'].logMonoTime = now + 1
    messages['aolSafetyWire'].aolSafetyWire = encode_safety(native)
    shadow = Peers([event.as_reader() for event in messages.values()])
    assert not tested._ordinary_warning_intent_current(shadow, cp, state, axis, now, now, latched=latched, producer=producer), fault
