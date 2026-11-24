"""Route 18e regressions through actual parser, Card, SelfdriveD and Controls."""
from contextlib import ExitStack
from unittest.mock import patch
import time

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.structs import car
from opendbc.car.gm.bolt_cc import button_bytes
from opendbc.car.gm.tests.test_bolt_pedal import params
from opendbc.car.gm.values import CAR, CruiseButtons, DBC
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests import test_gm as gm_fixtures
from openpilot.starpilot.aol.tests.test_gm_held_native import NOW, parse as physical_packets
from openpilot.starpilot.aol.tests.test_seatbelt_cold_start import deliver_card as deliver
from openpilot.selfdrive.selfdrived.events import EventName


def deliver_card(card, sd, controls, cs, tick):
  # Both clocks are explicit fixture inputs, including on macOS hosts.
  with patch.object(time, 'CLOCK_BOOTTIME', getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC), create=True), \
       patch.object(time, 'clock_gettime_ns', return_value=NOW + tick * 10_000_000):
    deliver(card, sd, controls, cs, tick)


def parse(card, now, counter=0, *, packets_only=False):
  rows = physical_packets(card, now, counter=counter, packets_only=True)
  packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])
  rows.append(packer.make_can_msg('ASCMActiveCruiseControlStatus', 2, {'ACCCruiseState': 0}))
  if packets_only:
    return rows
  if now == NOW:
    card.CI.update([(now - 1_000_000, rows)])
  cs = card.CI.update([(now, rows)])
  assert cs.canValid and not cs.canTimeout
  return cs


@pytest.mark.parametrize('event', [EventName.commIssue, EventName.commIssueAvgFreq, EventName.locationdTemporaryError,
                                  EventName.controlsMismatch, EventName.processNotRunning])
def test_cold_main_waits_for_host_readiness_but_keeps_real_faults(event):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    saved = Params()
    saved.put_bool('AlwaysOnLateral', True, block=True)
    card = gm_fixtures.TestGmAol.card(params(CAR.CHEVROLET_BOLT_CC_2018_2021, pedal=True, camera=True), saved)
    cleanup.callback(card.vehicle_startup.close)
    case = fixture.IoniqAvailabilityTests(methodName='runTest')
    cleanup.callback(case.doCleanups)
    sd, controls, drive = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True)
    card.slc_producer_session = 'authority-card'
    cs = parse(card, NOW)
    card.aol_card_intent.update(cs, fault_active=False, now_ns=NOW)
    original_events = sd.update_events
    axis, state, command = drive(0, observed_state=cs, native_request_lat=False, native_request_long=False,
                                allowed=False, long_allowed=False, critical_event=event)
    temporary = event != EventName.controlsMismatch
    assert any(e.name == event and (e.softDisable if temporary else e.immediateDisable) for e in sd.events.to_msg())
    assert str(axis.faultReason) == ('availabilityPause' if temporary else 'critical')
    assert not command.latActive and not command.longActive and not state.enabled
    deliver_card(card, sd, controls, cs, 0)
    assert card.aol_card_intent.allowed_latch == temporary
    if temporary:
      # The source's original 30ms output deadline still expires during startup.
      sd._expire_axis_source(cs, NOW + 31_000_000)
      assert sd.aol_car_state_log_ns == 0 and str(sd.aol_fault_reason) == 'availabilityPause'
      assert EventName.controlsMismatch not in sd.events.names
      sd.update_events = original_events
      axis, _, command = drive(4, native_request_long=False, native_request_lat=False, allowed=False)
      assert axis.desiredLateral and not axis.lateralActive and not command.latActive and not command.longActive
      deliver_card(card, sd, controls, parse(card, NOW + 40_000_000, counter=1), 4)
      assert card.aol_card_intent.allowed_latch
      axis, _, command = drive(5, native_request_long=False)
      assert axis.lateralActive and command.latActive and not command.longActive
      deliver_card(card, sd, controls, parse(card, NOW + 50_000_000, counter=2), 5)
      assert card.aol_card_intent.allowed_latch


@pytest.mark.parametrize('candidate', [CAR.CHEVROLET_BOLT_CC_2017, CAR.CHEVROLET_BOLT_CC_2018_2021,
                                      CAR.CHEVROLET_BOLT_CC_2022_2023, CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL])
def test_gm_resume_press_survives_neutral_samples_before_control_ack(candidate):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    card = gm_fixtures.TestGmAol.card(params(candidate, pedal=True, camera=True), Params())
    cleanup.callback(card.vehicle_startup.close)
    card.v_cruise_helper = VCruiseHelper(card.CP)
    card.v_cruise_helper.v_cruise_kph = 92.8
    card.CC_prev = car.CarControl(enabled=False)
    card.CS_prev = parse(card, NOW)
    for tick in range(3):
      now = NOW + (tick + 1) * 10_000_000
      packets = parse(card, now, counter=tick % 4, packets_only=True)
      packets = [packet for packet in packets if packet[0] != 0x1E1]
      packets.append((0x1E1, button_bytes(CruiseButtons.RES_ACCEL, tick % 4), 0))
      cs = card.CI.update([(now, packets)])
      assert cs.buttonEnable == (tick == 0)
      control = messaging.new_message('carControl', valid=True, logMonoTime=now)
      control.carControl.enabled = tick == 2
      card.sm.update_msgs(now / 1e9, [control.as_reader()])
      with patch('openpilot.selfdrive.car.cruise.monotonic_ns', return_value=now):
        card.update_cruise_speed(cs)
      card.CS_prev = cs
    assert card.v_cruise_helper.v_cruise_kph == 92.8


@pytest.mark.parametrize('bad', ['native_denied', 'expired_native', 'rx_fault', 'producer', 'epoch',
                                'source_zero', 'expired_wait', 'immediate', 'process', 'main_off', 'door'])
def test_cold_intent_retention_never_bypasses_provenance_or_faults(bad):
  from dataclasses import replace
  from openpilot.starpilot.aol.tests.test_gm_held_native import baseline, peers, Peers
  from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
  from openpilot.starpilot.aol.wire import encode_safety
  from openpilot.selfdrive.selfdrived.events import Events
  with OpenpilotPrefix(), ExitStack() as cleanup:
    card, _, prior = baseline()
    cleanup.callback(card.vehicle_startup.close)
    feedback = TransportPauseFeedback(card.CP)

    def sample(now, *, recovery=False, mutate=False):
      cs = parse(card, now, counter=1 if recovery else 0)
      native = replace(prior, observedMonoTime=now, validUntilMonoTime=now + 200_000_000,
        requestedLateral=False, lateralAllowed=False, requestedLongitudinal=False, longitudinalAllowed=False,
        pandaInventory=(replace(prior.pandaInventory[0], controlsAllowed=False),))
      incoming, _ = peers(card.CP, card.slc_producer_session, now, controls=False, native=native,
                           sequence=2 if recovery else 1, long_active=False)
      axis = incoming.messages['aolAxisState'].as_builder()
      a = axis.aolAxisState
      a.lateralActive = a.longitudinalActive = a.desiredLongitudinal = False
      a.desiredLateral = recovery
      a.nativeAcknowledged = recovery
      a.faultReason = 'none' if recovery else 'availabilityPause'
      a.faultEventMonoTime = now
      events = Events()
      if not recovery:
        events.add(EventName.commIssue)
      if mutate:
        if bad == 'native_denied':
          native = replace(native, requestedLateral=True)
        elif bad == 'expired_native':
          native = replace(native, observedMonoTime=now - 200_000_001, validUntilMonoTime=now - 1)
        elif bad == 'rx_fault':
          native = replace(native, pandaInventory=(replace(native.pandaInventory[0], safetyRxChecksInvalid=True),))
        elif bad == 'producer':
          a.intentProducerSessionId = 'foreign'
        elif bad == 'epoch':
          a.intentIntegrityEpoch += 1
        elif bad == 'source_zero':
          a.sourceCarStateMonoTime = 0
        elif bad == 'immediate':
          events.add(EventName.controlsMismatch)
        elif bad == 'process':
          events.add(EventName.processNotRunning)
        elif bad == 'main_off':
          cs.cruiseState.available = False
        elif bad == 'door':
          cs.doorOpen = True
      event = messaging.new_message('onroadEvents', len(events), valid=True, logMonoTime=now)
      event.onroadEvents = events.to_msg()
      wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=native.observedMonoTime)
      wire.aolSafetyWire = encode_safety(native)
      sm = Peers([axis.as_reader(), event.as_reader(), wire.as_reader(), incoming.messages['pandaStates']])
      return feedback.observe(sm, card.CP, cs, now, latched=True, intent_producer_session=card.slc_producer_session)

    assert not sample(NOW)
    assert feedback.cold_main_wait_ns == NOW
    assert sample(NOW + (210_000_000 if bad == 'expired_wait' else 10_000_000), recovery=True, mutate=True)
    assert feedback.failed
