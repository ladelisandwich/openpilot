"""Parsed Volt brake/regen during lag retains intent, never command authority."""
from contextlib import ExitStack
from dataclasses import replace
import os
from unittest import mock

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.gm.bolt_cc import button_bytes
from opendbc.car.gm.values import DBC, CruiseButtons, CAR, is_bolt_pedal_profile
from opendbc.car.gm.tests.test_bolt_pedal import params as bolt_params
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests import test_gm as gm_fixture
from openpilot.starpilot.aol.tests.test_gm_startup_main import parsed_packets
from openpilot.starpilot.aol.tests.test_gm_route18e_recovery import deliver_card
from openpilot.selfdrive.selfdrived.events import Events, EventName
from openpilot.starpilot.aol.wire import decode_safety, encode_safety

BASE = 10_000_000_000


def callers(cleanup, cp=None):
  params = Params()
  cleanup.callback(params._finalizer)
  params.put_bool('AlwaysOnLateral', True, block=True)
  card = gm_fixture.TestGmAol.card(sdgm_params() if cp is None else cp, params)
  cleanup.callback(card.vehicle_startup.close)
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  cleanup.callback(case.doCleanups)
  sd, controls, step = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True)
  card.slc_producer_session = 'authority-card'
  packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])

  def state(tick, pedal='none', main=True):
    now = BASE + tick * 10_000_000
    if is_bolt_pedal_profile(card.CP):
      from openpilot.starpilot.aol.tests.test_gm_held_native import parse
      packets = parse(card, now, gesture={'regen': 'paddle'}.get(pedal, pedal), counter=tick % 4, packets_only=True)
      packets = [packet for packet in packets if packet[0] != 0xC9]
      packets.append(packer.make_can_msg('ASCMActiveCruiseControlStatus', 2, {'ACCCruiseState': 0}))
      packets.append(packer.make_can_msg('ECMEngineStatus', 0, {'CruiseMainOn': int(main), 'BrakePressed': int(pedal == 'brake')}))
      if tick == 0:
        card.CI.update([(now - 1_000_000, packets)])
      cs = card.CI.update([(now, packets)])
      assert cs.canValid and not cs.canTimeout
      return cs
    packets = parsed_packets(card.CP, packer, now, tick, 20., main)
    if card.CP.pcmCruise:
      # Factory adaptive standby; state zero reports non-adaptive cruise.
      address = packer.make_can_msg('ASCMActiveCruiseControlStatus', 2, {})[0]
      packets = [row for row in packets if not (row[0] == address and row[2] == 2)]
      packets.append(packer.make_can_msg('ASCMActiveCruiseControlStatus', 2, {'ACCCruiseState': 2}))
    addresses = {packer.make_can_msg(name, 0, {})[0] for name in ('ECMAcceleratorPos', 'EBCMRegenPaddle')}
    packets = [p for p in packets if p[0] not in addresses]
    packets += [packer.make_can_msg('ECMAcceleratorPos', 0, {'BrakePedalPos': 12 if pedal == 'brake' else 0}),
                packer.make_can_msg('EBCMRegenPaddle', 0, {'RegenPaddle': int(pedal == 'regen')})]
    if pedal == 'cancel':
      packets = [p for p in packets if p[0] != 0x1E1]
      packets.append((0x1E1, button_bytes(CruiseButtons.CANCEL, tick % 4), 0))
    cs = card.CI.update([(now, packets)])
    assert cs.canValid and not cs.canTimeout
    assert cs.brakePressed == (pedal == 'brake') and cs.regenBraking == (pedal == 'regen')
    return cs

  def drive(tick, cs, **kwargs):
    source = messaging.new_message('carState', valid=True, logMonoTime=BASE + tick * 10_000_000)
    source.carState = cs
    return step(tick, car_state_message=source, latched=card.aol_card_intent.allowed_latch,
                native_request_long=False, **kwargs)

  cs = state(0)
  card.aol_card_intent.update(cs, now_ns=BASE)
  assert card.aol_card_intent.allowed_latch
  for tick in (0, 1):
    cs = state(tick)
    axis, _, command = drive(tick, cs)
    assert axis.lateralActive and command.latActive and not command.longActive
    deliver_card(card, sd, controls, cs, tick)
    assert card.aol_card_intent.allowed_latch
  return card, sd, controls, state, drive


@pytest.mark.parametrize('profile', [None, *[(candidate, removed) for candidate in
  (CAR.CHEVROLET_BOLT_CC_2017, CAR.CHEVROLET_BOLT_CC_2018_2021, CAR.CHEVROLET_BOLT_CC_2022_2023,
   CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL) for removed in (False, True)]])
@pytest.mark.parametrize('pedal', ['brake', 'regen'])
@pytest.mark.parametrize('order', ['together', 'axis_first', 'event_first'])
def test_parsed_pedal_lag_keeps_intent_without_renewing_authority(pedal, order, profile):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cp = None if profile is None else bolt_params(profile[0], pedal=True, camera=not profile[1], removed=profile[1])
    card, sd, controls, parse, drive = callers(cleanup, cp)
    feedback = card.aol_transport_feedback
    baseline = (feedback.lateral_baseline_ns, feedback.lateral_source_ns, feedback.lateral_native_ns,
                feedback.healthy_feedback_ns, feedback.healthy_native_ns)
    cs = parse(2, pedal)
    with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=.02):
      axis, _, command = drive(2, cs)
    assert not command.latActive and not command.longActive and str(axis.faultReason) == 'availabilityPause'
    assert not axis.lateralActive and not axis.longitudinalActive and not axis.nativeAcknowledged
    assert axis.faultSessionId == sd.aol_session_id and axis.faultEventMonoTime == BASE + 20_000_000
    assert sd.aol_published_fault_proof is None
    assert not axis.desiredLateral and not axis.desiredLongitudinal
    proof, event = sd.pm.messages['aolAxisState'], sd.pm.messages['onroadEvents']
    if order != 'together':
      # Deliver one new envelope first, retaining the previously received peer.
      key, held = ('onroadEvents', event) if order == 'axis_first' else ('aolAxisState', proof)
      sd.pm.messages.pop(key)
      old = card.sm[key]
      replacement = messaging.new_message(key, len(old) if key == 'onroadEvents' else None,
                                          valid=True, logMonoTime=card.sm.logMonoTime[key])
      setattr(replacement, key, old)
      sd.pm.messages[key] = replacement.as_reader()
      deliver_card(card, sd, controls, cs, 2)
      assert card.aol_card_intent.allowed_latch
      sd.pm.messages[key] = held
    deliver_card(card, sd, controls, cs, 2)
    assert card.aol_card_intent.allowed_latch and not feedback.failed
    assert baseline == (feedback.lateral_baseline_ns, feedback.lateral_source_ns, feedback.lateral_native_ns,
                        feedback.healthy_feedback_ns, feedback.healthy_native_ns)
    cs = parse(3)
    axis, _, command = drive(3, cs, allowed=False, native_request_lat=False)
    assert not axis.lateralActive and not command.latActive and not command.longActive
    assert axis.desiredLateral and str(axis.faultReason) == 'none'
    deliver_card(card, sd, controls, cs, 3)
    assert card.aol_card_intent.allowed_latch and not feedback.failed
    cs = parse(4)
    axis, _, command = drive(4, cs)
    assert axis.lateralActive and command.latActive and not command.longActive
    # A real MAIN-off packet still clears the retained intent.
    cs = parse(5, main=False)
    drive(5, cs)
    deliver_card(card, sd, controls, cs, 5)
    assert not card.aol_card_intent.allowed_latch


@pytest.mark.parametrize('bad', ['source_zero', 'source_stale', 'producer', 'epoch', 'session', 'config',
  'commIssue', 'controlsMismatch', 'processNotRunning', 'buttonCancel', 'native_long', 'host_long',
  'cancel', 'main_off', 'settings_off', 'can_invalid', 'permanent_eps', 'door', 'belt', 'no_pedal', 'low_only'])
def test_pedal_overlap_never_bypasses_other_rejection(bad):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    card, sd, controls, parse, drive = callers(cleanup)
    cs = parse(2, 'brake')
    with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=.02):
      _, _, command = drive(2, cs)
    assert not command.latActive and not command.longActive
    proof = sd.pm.messages['aolAxisState'].as_builder()
    if bad == 'source_zero':
      proof.aolAxisState.sourceCarStateMonoTime = 0
    elif bad == 'source_stale':
      proof.aolAxisState.sourceCarStateMonoTime -= 30_000_001
    elif bad == 'producer':
      proof.aolAxisState.intentProducerSessionId = 'other'
    elif bad == 'epoch':
      proof.aolAxisState.intentIntegrityEpoch += 1
    elif bad == 'session':
      proof.aolAxisState.sessionId = 'other'
    elif bad == 'config':
      card.CP.safetyConfigs[0].safetyParam ^= 0x400
    elif bad in ('commIssue', 'controlsMismatch', 'processNotRunning', 'buttonCancel'):
      events = Events(sd.events.event_rules)
      events.add_from_msg(sd.pm.messages['onroadEvents'].onroadEvents)
      events.add(getattr(EventName, bad))
      message = sd.pm.messages['onroadEvents'].as_builder()
      message.onroadEvents = events.to_msg()
      sd.pm.messages['onroadEvents'] = message.as_reader()
    elif bad == 'host_long':
      proof.aolAxisState.desiredLongitudinal = True
    elif bad == 'native_long':
      wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=BASE + 20_000_000)
      state = decode_safety(bytes(sd.sm['aolSafetyWire']))
      wire.aolSafetyWire = encode_safety(replace(state, requestedLongitudinal=True, longitudinalAllowed=True))
      sd.sm.update_msgs((BASE + 20_000_000) / 1e9, [wire.as_reader()])
    elif bad in ('cancel', 'main_off', 'no_pedal', 'low_only'):
      cs = parse(2, 'cancel' if bad == 'cancel' else 'none', main=bad != 'main_off')
      if bad == 'low_only':
        from opendbc.car.structs import car
        cs.gearShifter = car.CarState.GearShifter.low
    elif bad == 'settings_off':
      card.aol_card_intent.settings = replace(card.aol_card_intent.settings, enabled=False)
    else:
      setattr(cs, {'can_invalid': 'canValid', 'permanent_eps': 'steerFaultPermanent',
                   'door': 'doorOpen', 'belt': 'seatbeltUnlatched'}[bad], bad != 'can_invalid')
    sd.pm.messages['aolAxisState'] = proof.as_reader()
    baseline = (card.aol_transport_feedback.lateral_baseline_ns, card.aol_transport_feedback.lateral_source_ns,
                card.aol_transport_feedback.lateral_native_ns)
    deliver_card(card, sd, controls, cs, 2)
    intent_only = bad in ('commIssue', 'processNotRunning', 'native_long', 'belt')
    assert card.aol_card_intent.allowed_latch == intent_only
    if intent_only:
      # Qualified SOFT withdrawal retains intent, never an output permission.
      assert str(proof.aolAxisState.faultReason) == 'availabilityPause'
      assert not proof.aolAxisState.lateralActive and not proof.aolAxisState.longitudinalActive
      assert not proof.aolAxisState.nativeAcknowledged
      assert not card.sm['carControl'].latActive and not card.sm['carControl'].longActive
      assert baseline == (card.aol_transport_feedback.lateral_baseline_ns, card.aol_transport_feedback.lateral_source_ns,
                          card.aol_transport_feedback.lateral_native_ns)


@pytest.mark.parametrize('pedal', ['brake', 'regen'])
def test_later_physical_cancel_after_retained_lag(pedal):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    card, sd, controls, parse, drive = callers(cleanup)
    cs = parse(2, pedal)
    with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=.02):
      drive(2, cs)
    deliver_card(card, sd, controls, cs, 2)
    assert card.aol_card_intent.allowed_latch
    cs = parse(3, 'cancel')
    drive(3, cs)
    deliver_card(card, sd, controls, cs, 3)
    # Cancel remains governed by the existing GM mapping and withdraws LONG.
    assert not controls.pm.messages['carControl'].carControl.longActive


@pytest.mark.parametrize('pause_speed', [0., 100.])
def test_configured_brake_pause_still_inhibits_output(pause_speed):
  from openpilot.starpilot.aol.intent import read_settings
  with OpenpilotPrefix(), ExitStack() as cleanup:
    card, sd, _, parse, drive = callers(cleanup)
    settings = replace(read_settings(sd.params), pause_brake_mps=pause_speed)
    sd.aol_settings = card.aol_card_intent.settings = settings
    axis, _, command = drive(2, parse(2, 'brake'))
    assert axis.lateralActive == command.latActive == (pause_speed == 0.)
    assert not axis.longitudinalActive and not command.longActive


@pytest.mark.parametrize('simulation', [None, '1'])
def test_fixture_restores_outer_prefix_and_simulation(monkeypatch, simulation):
  if simulation is None:
    monkeypatch.delenv('SIMULATION', raising=False)
  else:
    monkeypatch.setenv('SIMULATION', simulation)
  with OpenpilotPrefix():
    outer = os.environ.get('OPENPILOT_PREFIX')
    with OpenpilotPrefix(), ExitStack() as cleanup:
      cleanup.enter_context(mock.patch.dict(os.environ, {'SIMULATION': '0'}))
      callers(cleanup)
    assert os.environ.get('OPENPILOT_PREFIX') == outer
    assert os.environ.get('SIMULATION') == simulation
