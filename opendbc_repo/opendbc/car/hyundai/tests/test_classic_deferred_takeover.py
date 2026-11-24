"""Keep factory SCC alive through the recorded G90 host-startup gap."""
import os
from unittest.mock import patch

import pytest

from opendbc.car import structs
from opendbc.car.hyundai.classic_long_aol import CLASSIC_LONG_IDS, qualified
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.tests.test_g90_continuity import ManualKeeper
from opendbc.car.hyundai.tests.test_startup_owner_current_admission import classic_frames
from opendbc.car.hyundai.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.tests.test_classic_long_profiles import params as classic_params
from openpilot.starpilot.vehicle_startup import VehicleStartupOwner

CARS = (CAR.GENESIS_G90, CAR.HYUNDAI_SONATA, CAR.HYUNDAI_SONATA_HYBRID, CAR.GENESIS_G80)


@pytest.fixture(scope='module', autouse=True)
def native_classic_positive_path():
  # Positive diagnostic takeover is a DEBUG-only registry capability. The
  # paired test_classic_startup_release suite retains RELEASE denial checks.
  previous = libsafety_py.__dict__.get('libpath')
  libsafety_py.load(libsafety_py._build_libsafety(release=False))
  try:
    yield
  finally:
    if previous is not None:
      libsafety_py.load(previous)


@pytest.mark.parametrize('identity', CARS)
@pytest.mark.parametrize('scenario', (
  'healthy', 'stale_control', 'active_control', 'missing_source', 'native_mode',
  'cruise_fault', 'stock_engaged', 'disable_negative', 'restore_failure', 'publisher_failure'))
def test_actual_card_defers_classic_disable_until_inactive_producers_are_ready(identity, scenario):
  native = libsafety_py.libsafety
  assert native.set_safety_hooks(int(structs.CarParams.SafetyModel.elm327), 1) == 0
  native.init_tests()
  now = [100_000_000_000]
  requests, emitted = [], []
  muted = [False]

  def admitted():
    return native.get_current_safety_mode() == int(structs.CarParams.SafetyModel.elm327) and not native.get_controls_allowed()

  def send(frames):
    for a, d, b in frames:
      assert native.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d))
    emitted.extend((now[0], frame) for frame in frames)
    if scenario == 'publisher_failure' and any(d[:4] == b'\x03\x28\x83\x01' for _, d, _ in frames):
      muted[0] = True
      raise OSError('diagnostic publication failed after possible ECU delivery')

  def query(sender, receiver, bus, addresses, commands, responses):
    command = commands[0]
    requests.append((now[0], command))

    class Query:
      def get_data(self, timeout):
        sender([(0x7D0, bytes([len(command)]) + command + bytes(7 - len(command)), 0)])
        if command == b'\x28\x83\x01':
          muted[0] = True
          return {(0x7D0, None): b'\x7f\x28\x22'} if scenario in ('disable_negative', 'restore_failure') else {}
        if command == b'\x28\x00\x01':
          if scenario == 'restore_failure':
            return {}
          muted[0] = False
        return {(0x7D0, None): b''}
    return Query()

  def sleep(seconds):
    now[0] += round(seconds * 1e9)

  def update_sm(host, command, *, stale=False):
    msg = messaging.new_message('carControl', valid=True, logMonoTime=now[0] - (151_000_000 if stale else 0))
    msg.carControl = command
    panda = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=now[0])
    ps = panda.pandaStates[0]
    ps.safetyModel, ps.safetyParam = native.get_current_safety_mode(), native.get_current_safety_param()
    ps.alternativeExperience = host.CP.alternativeExperience
    ps.ignitionLine = True
    host.sm.update_msgs(now[0] / 1e9, [msg.as_reader(), panda.as_reader()])

  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}), \
       patch('time.clock_gettime_ns', side_effect=lambda clock: now[0]), \
       patch('time.monotonic_ns', side_effect=lambda: now[0]), \
       patch('time.monotonic', side_effect=lambda: now[0] / 1e9), \
       patch('time.sleep', side_effect=sleep), \
       patch('opendbc.car.hyundai.g90_startup.IsoTpParallelQuery', side_effect=query):
    saved = Params()
    for key in ('OpenpilotEnabledToggle', 'AlwaysOnLateral', 'AlphaLongitudinalEnabled'):
      saved.put_bool(key, True, block=True)
    holder = VehicleStartupOwner()
    prepared = holder.prepare(classic_params(identity, lda=True), CarInterface, (list, send), requested=True, admission=admitted, defer_until_controls=True)
    owner = holder.owner
    assert owner is not None
    owner.clock, owner.keeper_factory = lambda: now[0] / 1e9, ManualKeeper
    try:
      assert not requests and not owner.session_attempted and not muted[0], 'FACTORY_SCC_DISABLED_BEFORE_CONTROL_READY'
      assert owner.outcome is Outcome.PENDING
      assert not owner.before_control(configured=True, sources_current=True, control_current=True)
      card = Car(CI=CarInterface(prepared), RI=RadarInterface(prepared), startup_owner=owner)
      card.params.put_bool('IsOffroad', False, block=True)
      before = card.CP.to_dict()
      # The original route left SCC absent for 3.97 seconds during startup.
      for tick in range(1, 401):
        now[0] += 10_000_000
        state = card.CI.update([(now[0], classic_frames(card.CI, tick))])
      assert state.canValid and not state.canTimeout and not state.accFaulted
      assert not requests and not muted[0] and owner.keeper is None
      command = structs.CarControl(enabled=scenario == 'active_control', latActive=scenario == 'active_control').as_reader()
      if scenario == 'missing_source':
        parser = next(iter(card.CI.can_parsers.values()))
        parser.message_states[parser.dbc.name_to_msg['TCS13'].address].timestamps.clear()
      if scenario == 'cruise_fault':
        state.accFaulted = True
      if scenario == 'stock_engaged':
        state.cruiseState.enabled = True
      config = card.CP.safetyConfigs[-1]
      if scenario == 'native_mode':
        native.set_alternative_experience(card.CP.alternativeExperience)
        assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
      update_sm(card, command, stale=scenario == 'stale_control')
      if scenario in ('disable_negative', 'restore_failure', 'publisher_failure'):
        with pytest.raises(RuntimeError, match='pending'):
          card.controls_update(state, command, initialize_only=True)
        assert owner.outcome is Outcome.ABORT_UNCERTAIN and not card.ci_initialized
        assert card.CP.to_dict() == before and not owner.handed_off
        assert muted[0] == (scenario == 'restore_failure')
        assert owner.restored == (scenario != 'restore_failure')
        return
      card.controls_update(state, command, initialize_only=True)
      if scenario != 'healthy':
        assert not requests and not muted[0] and not card.ci_initialized
        assert owner.outcome is Outcome.PENDING and card.CP.to_dict() == before
        return
      assert [x[1] for x in requests] == [b'\x10\x03', b'\x28\x83\x01']
      assert muted[0] and card.ci_initialized and owner.outcome is Outcome.SENT_UNCONFIRMED
      assert owner.keeper is not None and not owner.handed_off
      disabled_at = next(t for t, command in requests if command == b'\x28\x83\x01')
      native.set_alternative_experience(card.CP.alternativeExperience)
      assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
      native.init_tests()
      for tick in range(401, 411):
        now[0] += 10_000_000
        frames = classic_frames(card.CI, tick)
        state = card.CI.update([(now[0], frames)])
        native.set_timer(tick * 10_000)
        for a, d, b in frames:
          native.safety_rx_hook(libsafety_py.make_CANPacket(a, b, d))
      update_sm(card, command)
      captured = []

      def publish(frames, *, valid):
        if valid:
          send(frames)
          card.vehicle_startup.sent(frames, valid=valid)
          captured.extend(frames)

      with patch.object(card, 'publish_sendcan', side_effect=publish):
        card.controls_update(state, command, initialize_only=True)
      assert {0x420, 0x421} <= {a for a, _, b in captured if b == 0}
      assert owner.handed_off and now[0] - disabled_at < 300_000_000
      assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
      assert card.CP.to_dict() == before
    finally:
      holder.close()


@pytest.mark.parametrize('identity', sorted(CLASSIC_LONG_IDS, key=str))
def test_shared_classic_factory_never_suppresses_scc_during_host_preparation(identity):
  cp = classic_params(identity, lda=True)
  if not qualified(cp):
    return
  before = cp.to_dict()
  sent = []
  holder = VehicleStartupOwner()
  result = holder.prepare(cp, CarInterface, (list, sent.extend), requested=True, admission=lambda: True, defer_until_controls=True)
  try:
    assert holder.owner is not None and holder.owner.outcome is Outcome.PENDING
    assert not sent and not holder.owner.session_attempted and not holder.owner.disable_attempted
    assert result.to_dict() == before
  finally:
    holder.close()
  assert not sent
