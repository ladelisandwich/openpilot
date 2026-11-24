"""Real Card intent publication to SelfdriveD/Controls and current native TX.
Clock/acquisition/perception are finite fixtures; no permissions/CS/CP are forged.
"""

from unittest.mock import patch

import pytest
from opendbc.car.nissan.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import messaging
from openpilot.starpilot.aol.tests import test_ioniq_availability as availability
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety, decode_intent
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.car.nissan.tests.test_aol import Frames, factory


@pytest.mark.parametrize('identity', list(CAR))
def test_actual_card_intent_selfdrived_controls_and_native(identity):
  with factory(identity, manual=identity == CAR.NISSAN_ROGUE) as (card, _):
    assert card.aol_qualified
    case = availability.IoniqAvailabilityTests()
    try:
      cp = card.CP
      with patch.dict('os.environ', {'SIMULATION': '0'}):
        sd, controls, step = case._authority_callers(False, cp=cp.as_reader().as_builder(), genuine_events=True)
      card.slc_producer_session = 'authority-card'
      safety = libsafety_py.libsafety
      safety.set_alternative_experience(32)
      safety.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam)
      safety.init_tests()
      safety.set_alternative_experience(32)
      frames = Frames(card.CI)
      previous = {}
      published = {}
      real_new = messaging.new_message
      for tick in range(1, 131):
        now = 10_000_000_000 + tick * 10_000_000
        reverse = 65 <= tick < 75
        temporary = 85 <= tick < 95
        brake = 50 <= tick < 60
        main = tick < 120
        batch = frames.make(tick, main=main, cruise=tick < 50, brake=brake, gear=2 if reverse else 4, status=9 if temporary else 5)
        safety.set_timer(tick * 10000)
        for addr, data, bus in batch:
          safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))
        safety.set_aol_test_heartbeat(True)
        request = 0
        if previous.get('aolAxisState') is not None:
          previous_axis = previous['aolAxisState'].aolAxisState
          request = int(previous_axis.desiredLateral) | (int(previous_axis.desiredLongitudinal) << 1)
        safety.aol_set_host_request(request)
        permission = safety.aol_get_permission_mask()
        native = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=now)
        native.aolSafetyWire = encode_safety(
          SafetyState(
            1,
            True,
            now,
            now + 200_000_000,
            14,
            cp.safetyConfigs[0].safetyParam,
            bool(permission & 1),
            bool(permission & 2),
            bool(request & 1),
            bool(request & 2),
            'authority-panda',
            sd.aol_session_id,
            now,
            (PandaSlot(0, 'authority-panda', 14, cp.safetyConfigs[0].safetyParam, 32, bool(safety.get_controls_allowed()), False, False, 0),),
          )
        )
        events = []
        if previous.get('onroadEvents') is not None:
          events.append(previous['onroadEvents'])
        else:
          events.append(messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=now).as_reader())
        pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now)
        pandas.pandaStates[0].safetyModel = 'nissan'
        pandas.pandaStates[0].safetyParam = cp.safetyConfigs[0].safetyParam
        pandas.pandaStates[0].alternativeExperience = 32
        pandas.pandaStates[0].controlsAllowed = safety.get_controls_allowed()
        pandas.pandaStates[0].safetyRxChecksInvalid = not safety.safety_config_valid()
        peers = [native.as_reader(), pandas.as_reader(), *events]
        peers.extend(message for name, message in previous.items() if name in ('aolAxisState', 'selfdriveState'))
        # One receipt per source per actor. Native is the actual RX/TX hook result.
        card.sm.update_msgs(now / 1e9, peers)
        canevent = messaging.log_from_bytes(card_module.can_list_to_can_capnp(batch)).as_builder()
        canevent.logMonoTime = now

        def capture(service, event):
          published[service] = messaging.log_from_bytes(event if isinstance(event, bytes) else event.to_bytes())

        def stamped(service=None, size=None, _stamp=now, **kw):
          return real_new(service, size, **({'logMonoTime': _stamp} | kw))

        with (
          patch.object(card_module, 'REPLAY', False),
          patch.object(card_module.time, 'monotonic_ns', return_value=now),
          patch.object(card_module.time, 'monotonic', return_value=now / 1e9),
          patch.object(card_module.time, 'clock_gettime_ns', return_value=now + 2000),
          patch.object(card_module.messaging, 'drain_sock_raw', return_value=[canevent.to_bytes()]),
          patch.object(card_module.messaging, 'new_message', side_effect=stamped),
          patch.object(card.sm, 'update'),
          patch.object(card.pm, 'send', side_effect=capture),
        ):
          cs, rd = card.state_update()
          card.state_publish(cs, rd)
        intent = decode_intent(bytes(published['aolIntentWire'].aolIntentWire))
        assert intent is not None
        axis, _, command = step(
          tick,
          car_state_message=published['carState'].as_builder(),
          safety_message=native,
          panda_state_message=pandas,
          allowed=bool(permission & 1),
          long_allowed=bool(permission & 2),
          native_request_lat=bool(request & 1),
          native_request_long=bool(request & 2),
          panda_controls_allowed=safety.get_controls_allowed(),
          intent_message=published['aolIntentWire'].as_builder(),
        )
        previous = dict(sd.pm.messages)
        previous['carControl'] = controls.pm.messages['carControl']
        # Real onroad events are needed for Card's classified fault feedback.
        if 'onroadEvents' not in previous:
          ev = messaging.new_message('onroadEvents', len(sd.events.to_msg()), valid=True, logMonoTime=now)
          ev.onroadEvents = sd.events.to_msg()
          previous['onroadEvents'] = ev.as_reader()
        assert not command.longActive
        card.sm.update_msgs(now / 1e9, [controls.pm.messages['carControl']])
        sent = []
        with (patch.object(card_module.time, 'monotonic_ns', return_value=now),
              patch.object(card_module.time, 'monotonic', return_value=now / 1e9),
          patch.object(card_module.time, 'clock_gettime_ns', return_value=now + 2000),
              patch.object(card, 'publish_sendcan', side_effect=lambda frames, valid=True, output=sent: output.extend(frames))):
          card.controls_update(cs, card.sm['carControl'])
        sends = sent
        steer = next(frame for frame in sends if frame[0] == 0x169)
        assert safety.safety_tx_hook(libsafety_py.make_CANPacket(steer[0], steer[2], steer[1]))
        if tick in (45, 62, 80, 105):
          assert intent.lateralArmed and command.latActive, (identity, tick, intent, axis.to_dict(), sd.events.names)
        if reverse or temporary or not main:
          assert not command.latActive
      assert not card.aol_card_intent.allowed_latch
    finally:
      case.doCleanups()
