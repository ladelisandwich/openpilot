"""Recorded F1 auto discovery through actual Card; fresh native permission uses actual CAN.

Synthetic payloads match the declared F1 physical topology; no ECU I/O or seeded permission.
"""

import json
import os
import pytest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car import car_helpers, Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.can import CANPacker
from opendbc.car.gm.tests.test_volt_camera_automatic import recorded_fingerprint
from opendbc.car.gm.tests.test_volt_cc_f1 import params, tick
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, DBC, GMFlags
from opendbc.safety.tests.libsafety import libsafety_py as native
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.vehicle_selection import encode
from openpilot.starpilot.aol.vehicle import policy_for


def test_actual_automatic_f1_card_has_aol_owner():
  packets = [CanData(a, bytes(n), b) for b, frames in recorded_fingerprint().items() for a, n in frames.items()]

  def receive(wait_for_one=False):
    return [packets] if wait_for_one else []

  def identify(*args, **kwargs):
    return car_helpers.get_car(receive, *args[1:], **kwargs)

  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.enter_context(patch.dict(os.environ, {'SKIP_FW_QUERY': '1', 'FINGERPRINT': '', 'SIMULATION': '1', 'REPLAY': '1'}))
    settings = Params()
    cleanup.callback(settings._finalizer)
    settings.put_bool('OpenpilotEnabledToggle', True, block=True)
    settings.put_bool('AlwaysOnLateral', True, block=True)
    settings.put_bool('IsReleaseBranch', False, block=True)
    settings.put('VehicleSelection', json.loads(encode(None)), block=True)
    with (
      patch('openpilot.selfdrive.car.card.messaging.recv_one_retry', return_value=SimpleNamespace(can=[1])),
      patch('openpilot.selfdrive.car.card.get_car', side_effect=identify),
    ):
      card = Car()
    cleanup.callback(card.vehicle_startup.close)
    cleanup.callback(card.params._finalizer)
    assert card.CP.carFingerprint == CAR.CHEVROLET_VOLT_CC
    assert card.CP.safetyConfigs[0].safetyParam == 0xC163
    assert str(card.CP.fingerprintSource) == 'can'
    assert policy_for(card.CP).intent_supported
    assert card.aol_card_intent is not None
    assert card.CP.alternativeExperience & 32


def test_native_f1_has_independent_lat_with_cruise_disengaged():
  cp = params()
  cp.alternativeExperience = 32
  ci = car_helpers.interfaces[cp.carFingerprint](cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  safety = native.libsafety
  safety.init_tests()
  safety.set_alternative_experience(32)
  assert safety.set_safety_hooks(4, 0xC163) == 0
  for i in range(120):
    now = 10_000_000_000 + i * 10_000_000
    safety.set_timer(now // 1000)
    safety.set_aol_test_heartbeat(True)
    cs, frames = tick(ci, packer, now, counter=i % 4, active=False)
    for a, data, bus in frames:
      assert safety.safety_rx_hook(native.make_CANPacket(a, bus, data))
    safety.safety_tick()
    safety.aol_set_host_request(1)
  assert cs.canValid and not cs.canTimeout and not cs.cruiseState.enabled
  assert not safety.get_controls_allowed()
  assert safety.safety_config_valid()
  assert safety.aol_get_request_mask() == 1
  assert safety.aol_get_permission_mask() == 1


@pytest.mark.parametrize(
  'case', ['healthy', 'main_off', 'park', 'reverse', 'neutral', 'foot', 'analog', 'paddle', 'gas', 'stale', 'heartbeat', 'expired_request', 'relay_fault']
)
def test_f1_native_preserves_physical_and_freshness_gates(case):
  cp = params()
  cp.alternativeExperience = 32
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  safety = native.libsafety
  safety.init_tests()
  safety.set_alternative_experience(32)
  assert safety.set_safety_hooks(4, 0xC163) == 0
  for i in range(120):
    now = 10_000_000_000 + i * 10_000_000
    safety.set_timer(now // 1000)
    safety.set_aol_test_heartbeat(True)
    cs, frames = tick(
      ci,
      packer,
      now,
      counter=i % 4,
      active=case in ('foot', 'analog', 'paddle', 'gas'),
      brake=case == 'foot',
      analog=6 if case == 'analog' else 0,
      regen=case == 'paddle',
      gas=case == 'gas',
    )
    gear = {'park': 1, 'reverse': 2, 'neutral': 3}.get(case, 4)
    engine = packer.make_can_msg('ECMEngineStatus', 0, {'CruiseMainOn': int(case != 'main_off'), 'BrakePressed': int(case == 'foot')})
    prndl = packer.make_can_msg('ECMPRDNL2', 0, {'PRNDL2': gear})
    frames = [f for f in frames if f[0] not in (engine[0], prndl[0])] + [engine, prndl]
    for a, data, bus in frames:
      assert safety.safety_rx_hook(native.make_CANPacket(a, bus, data))
    safety.safety_tick()
    neutral = structs.CarControl(enabled=False, latActive=False, longActive=False)
    _, sent = ci.apply(neutral.as_reader(), now)
    for a, data, bus in sent:
      assert safety.safety_tx_hook(native.make_CANPacket(a, bus, data))
    safety.aol_set_host_request(3)
  assert safety.safety_config_valid()
  if case == 'stale':
    safety.set_timer(now // 1000 + 300_001)
    safety.safety_tick()
  if case == 'heartbeat':
    safety.set_aol_test_heartbeat(False)
  if case == 'expired_request':
    safety.set_timer(now // 1000 + 200_001)
  if case == 'relay_fault':
    safety.set_relay_malfunction(True)
  mask = safety.aol_get_permission_mask()
  if case in ('stale', 'heartbeat', 'expired_request', 'relay_fault', 'main_off'):
    assert mask == 0
  elif case in ('foot', 'analog', 'paddle'):
    assert mask == 1 and not safety.get_controls_allowed()
  elif case == 'gas':
    assert mask == 3
  else:
    assert mask == 1
  command = structs.CarControl(enabled=False, latActive=True)
  command.actuators.torque = 0.01
  _, frames = ci.apply(command.as_reader(), now + 1)
  steering = [frame for frame in frames if frame[0] == 0x180]
  assert steering
  for a, data, bus in steering:
    allowed = bool(safety.safety_tx_hook(native.make_CANPacket(a, bus, data)))
    assert allowed == (case in ('healthy', 'foot', 'analog', 'paddle', 'gas'))
  for a, size, bus in [(0x2CB, 8, 0), (0x315, 5, 0), (0x200, 6, 0), (0x184, 8, 2), (0x370, 6, 0)]:
    assert not safety.safety_tx_hook(native.make_CANPacket(a, bus, bytes(size)))


@pytest.mark.parametrize(
  'field,value',
  [
    ('passive', True),
    ('notCar', True),
    ('dashcamOnly', True),
    ('flags', int(GMFlags.CC_LONG | GMFlags.PEDAL_LONG)),
    ('radarUnavailable', False),
    ('pcmCruise', True),
  ],
)
def test_f1_changed_configuration_does_not_get_host_aol(field, value):
  cp = params()
  cp.alternativeExperience = 32
  assert policy_for(cp).intent_supported
  setattr(cp, field, value)
  assert not policy_for(cp).intent_supported


def test_f1_release_profile_and_native_remain_unavailable():
  cp = params(release=True)
  assert cp.dashcamOnly and str(cp.safetyConfigs[0].safetyModel) == 'noOutput'
  assert not policy_for(cp).intent_supported
  release = native.ffi.dlopen(native._build_libsafety(release=True))
  release.init_tests()
  release.set_alternative_experience(32)
  assert release.set_safety_hooks(4, 0xC163) == 0
  release.set_aol_test_heartbeat(True)
  release.aol_set_host_request(3)
  assert release.aol_get_request_mask() == release.aol_get_permission_mask() == 0
  for address, size in [(0x180, 4), (0x1E1, 7), (0x315, 5), (0x2CB, 8)]:
    assert not release.safety_tx_hook(native.make_CANPacket(address, 0, bytes(size)))


@pytest.mark.parametrize('experience', [0, 33, 64, 96])
def test_f1_does_not_acquire_unsupported_alternative_axis_policy(experience):
  cp = params()
  cp.alternativeExperience = experience
  from opendbc.car.gm.cancel_remap import aol_enabled

  assert not aol_enabled(cp)
  safety = native.libsafety
  safety.init_tests()
  safety.set_alternative_experience(experience)
  assert safety.set_safety_hooks(4, 0xC163) == 0
  safety.set_aol_test_heartbeat(True)
  safety.aol_set_host_request(3)
  assert safety.aol_get_request_mask() == safety.aol_get_permission_mask() == 0


@pytest.mark.parametrize('gesture', ['foot', 'analog', 'paddle', 'cancel'])
@pytest.mark.parametrize(
  'case', ['healthy', 'missing_receipt', 'old_receipt', 'wrong_state', 'no_physical', 'native_denied', 'native_fault', 'producer', 'epoch', 'critical']
)
def test_f1_physical_withdrawal_keeps_only_current_granted_lateral(gesture, case):
  from dataclasses import replace
  from openpilot.common.params import Params
  from openpilot.selfdrive.selfdrived.events import EventName
  from openpilot.starpilot.aol.tests.test_gm import TestGmAol
  from openpilot.starpilot.aol.tests.test_gm_held_native import NOW, peers, Peers
  from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
  from openpilot.starpilot.car.gm.aol import physical_longitudinal_withdrawal_source

  with OpenpilotPrefix():
    settings = Params()
    settings.put_bool('AlwaysOnLateral', True, block=True)
    card = TestGmAol.card(params(), settings)
    try:
      cp = card.CP
      packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
      for i in range(110):
        now = NOW - 1_100_000_000 + i * 10_000_000
        cs, _ = tick(card.CI, packer, now, counter=i % 4, active=True)
      baseline, prior = peers(cp, 'f1-card', now, sequence=1)
      feedback = TransportPauseFeedback(cp)
      assert not feedback.observe(baseline, cp, cs, now, latched=True, intent_producer_session='f1-card')
      now += 10_000_000
      physical = case != 'no_physical'
      cs, frames = tick(
        card.CI,
        packer,
        now,
        counter=2,
        active=False,
        brake=physical and gesture == 'foot',
        analog=6 if physical and gesture == 'analog' else 0,
        regen=physical and gesture == 'paddle',
      )
      if gesture == 'cancel' and physical:
        from opendbc.car.gm.bolt_cc import button_bytes
        from opendbc.car.gm.values import CruiseButtons

        frames = [f for f in frames if f[0] != 0x1E1] + [(0x1E1, button_bytes(CruiseButtons.CANCEL, 2), 0)]
        cs = card.CI.update([(now, frames)])
      observation = (cs, now, now, now)
      if case == 'missing_receipt':
        observation = None
      if case == 'old_receipt':
        observation = (cs, now - 30_000_001, now, now)
      if case == 'wrong_state':
        observation = (cs.as_reader().as_builder(), now, now, now)
      source = physical_longitudinal_withdrawal_source(cp, cs, card.CI.can_parsers[Bus.pt], observation, now)
      assert bool(source) == (case not in ('missing_receipt', 'old_receipt', 'wrong_state', 'no_physical'))
      native = replace(
        prior,
        observedMonoTime=now,
        validUntilMonoTime=now + 200_000_000,
        sourcePandaStatesMonoTime=now,
        longitudinalAllowed=False,
        pandaInventory=(replace(prior.pandaInventory[0], controlsAllowed=False),),
      )
      if case == 'native_denied':
        native = replace(native, lateralAllowed=False)
      if case == 'native_fault':
        native = replace(native, pandaInventory=(replace(native.pandaInventory[0], faults=1),))
      current, _ = peers(cp, 'f1-card', now, native=native, sequence=2, long_active=False)
      rows = []
      for name, message in current.messages.items():
        row = message.as_builder()
        if name == 'pandaStates':
          row.pandaStates[0].controlsAllowed = False
        if name == 'aolAxisState' and case == 'producer':
          row.aolAxisState.intentProducerSessionId = 'other'
        if name == 'aolAxisState' and case == 'epoch':
          row.aolAxisState.intentIntegrityEpoch = 2
        if name == 'onroadEvents':
          from openpilot.selfdrive.selfdrived.events import Events

          events = Events()
          events.add(EventName.controlsMismatch if case == 'critical' else EventName.buttonCancel if gesture == 'cancel' else EventName.pedalPressed)
          row.onroadEvents = events.to_msg()
        rows.append(row.as_reader())
      current = Peers(rows)
      original = bytes(current['aolSafetyWire'])
      fault = feedback.observe(current, cp, cs, now, latched=True, gm_withdrawal_source_ns=source, intent_producer_session='f1-card')
      assert fault == (case != 'healthy')
      assert bytes(current['aolSafetyWire']) == original
      assert not current['aolAxisState'].longitudinalActive and not native.longitudinalAllowed
    finally:
      card.vehicle_startup.close()


@pytest.mark.parametrize('gesture', ['foot', 'analog', 'paddle'])
def test_f1_actual_state_update_binds_physical_receipt(gesture):
  from openpilot.cereal import messaging
  from openpilot.starpilot.aol.tests.test_gm import TestGmAol as GmCardFixture
  now = 10_020_000_000
  with OpenpilotPrefix(), ExitStack() as cleanup:
    settings = Params()
    settings.put_bool('AlwaysOnLateral', True, block=True)
    card = GmCardFixture.card(params(), settings)
    cleanup.callback(card.vehicle_startup.close)
    packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])
    for i in range(100):
      cs, _ = tick(card.CI, packer, now - 1_000_000_000 + i * 10_000_000, counter=i % 4, active=False)

    class Capture:
      def update(self, rows):
        self.rows = rows

    capture = Capture()
    _, frames = tick(capture, packer, now, counter=0, active=False, brake=gesture == 'foot', analog=6 if gesture == 'analog' else 0, regen=gesture == 'paddle')
    event = messaging.new_message('can', len(frames), valid=True, logMonoTime=now)
    for dst, (address, data, bus) in zip(event.can, frames, strict=True):
      dst.address, dst.dat, dst.src = address, data, bus
    # Only CAN transport and clocks are fixtures; state_update parses the real bytes.
    with (
      patch('openpilot.selfdrive.car.card.messaging.drain_sock_raw', return_value=[event.to_bytes()]),
      patch('openpilot.selfdrive.car.card.time.monotonic_ns', return_value=now),
      patch('openpilot.selfdrive.car.card.time.clock_gettime_ns', return_value=now),
    ):
      cs, _ = card.state_update(defer_radar=True)
    assert cs.canValid and not cs.canTimeout
    assert cs.brakePressed == (gesture in ('foot', 'analog')) and cs.regenBraking == (gesture == 'paddle')
    observed = card._aol_can_observation
    assert observed[0] is cs and observed[1:] == (now, now, now)
    assert card.aol_gm_withdrawal_source(cs, now) == now
