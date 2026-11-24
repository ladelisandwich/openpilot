from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.hyundai.ecu_startup import Outcome, stock_copy
from opendbc.car.hyundai.gv60_startup import GV60Startup, qualified
from opendbc.car.hyundai.hyundaicanfd import create_gv60_radar_heartbeat, hkg_can_fd_checksum
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_generic_hda2_startup_lifecycle import Keeper, physical_frames
from opendbc.car.hyundai.tests.test_radar_hda2_startup import params
from opendbc.car.hyundai.values import CAR, Buttons
from opendbc.safety.tests.libsafety import libsafety_py


def physical(ci, tick):
  frames = physical_frames(ci, tick)
  packer = CANPacker(ci.can_parsers[Bus.pt].dbc.name)
  frames = [(address, packer.make_can_msg('MDPS', bus, {'COUNTER': raw[2], 'MDPS_StrTqSnsrVal': 0, 'MDPS_OutTqVal': 0})[1], bus)
            if address == 0xEA and bus == 1 else (address, raw, bus) for address, raw, bus in frames]
  parser = ci.can_parsers[Bus.pt]
  packer = CANPacker(parser.dbc.name)
  name = ci.CS.cruise_btns_msg_canfd
  address = parser.dbc.name_to_msg[name].address
  frames = [frame for frame in frames if frame[0] != address]
  if tick % 2 == 0:
    frames.append(packer.make_can_msg(name, parser.bus, {'COUNTER': tick // 2, 'CRUISE_BUTTONS': 0, 'LDA_BTN': 0}))
  return frames


@pytest.mark.parametrize('alt', [False, True])
def test_factory_gv60_profile_and_stock_params_exact(alt):
  cp = params(CAR.GENESIS_GV60_EV_1ST_GEN, alt=alt)
  stock = params(CAR.GENESIS_GV60_EV_1ST_GEN, alpha=False, alt=alt)
  assert cp.safetyConfigs[0].safetyParam == (0x9095 if alt else 0x9015)
  assert qualified(cp) and stock_copy(cp).to_dict() == stock.to_dict()
  before = cp.to_dict()
  owner = CarInterface.startup_owner(cp, (list, list), requested=True)
  assert isinstance(owner, GV60Startup) and owner.address == 0x730 and owner.bus == 1
  assert owner.communication_control_request() == b'\x28\x01\x01'
  assert owner.stock_cp.to_dict() == stock.to_dict() and cp.to_dict() == before
  assert CarInterface.startup_required(cp)
  assert CarInterface.startup_owner(cp, (list, list), requested=False) is None


@pytest.mark.parametrize('alt', [False, True])
@pytest.mark.parametrize('reply,accepted', [(b'\x68\x01', True), (None, True), (b'\x68\x03', False), (b'\x7f\x28\x22', False)])
def test_unsuppressed_response_and_verified_restore(alt, reply, accepted):
  cp = params(CAR.GENESIS_GV60_EV_1ST_GEN, alt=alt)
  sent, requests, waits = [], [], []
  owner = GV60Startup(cp, (list, sent.extend), keeper_factory=Keeper)

  def query(sender, receiver, bus, addresses, req, responses):
    assert bus == 1 and addresses == [(0x730, None)] and len(req) == 1
    command = req[0]
    requests.append(command)

    def receive(timeout):
      sender([(0x730, bytes([len(command)]) + command + bytes(7 - len(command)), 1)])
      if command == b'\x28\x01\x01':
        assert responses == [b''] and timeout == 0.1
        return {} if reply is None else {(0x730, None): reply}
      return {(0x730, None): b''}

    return SimpleNamespace(get_data=receive)

  with patch.object(owner, '_query', side_effect=query), patch('opendbc.car.hyundai.ecu_startup.time.sleep', side_effect=waits.append):
    result = owner.prepare(admission=lambda: True)
  assert owner.session_attempted and owner.disable_attempted
  assert requests[:2] == [b'\x10\x03', b'\x28\x01\x01'] and 0.05 in waits
  if accepted:
    assert owner.outcome is Outcome.SENT_UNCONFIRMED and qualified(result)
    assert owner.keeper is not None and owner.keeper.running
    assert owner.disable_response == ('reply_absent_sent_unconfirmed' if reply is None else 'positive_communication_control')
  else:
    assert requests[2:] == [b'\x10\x03', b'\x28\x00\x01', b'\x10\x01']
    assert owner.outcome is Outcome.STOCK_RESTORED and owner.restored
    assert result.to_dict() == params(CAR.GENESIS_GV60_EV_1ST_GEN, alpha=False, alt=alt).to_dict()
  with patch.object(owner, '_query', side_effect=query), patch('opendbc.car.hyundai.ecu_startup.time.sleep', side_effect=waits.append):
    owner.close()


def initialize_native(word):
  native = libsafety_py.libsafety
  native.set_alternative_experience(0)
  debug = native.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) == 0
  native.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, word)
  native.init_tests()
  return native, debug


@pytest.mark.parametrize('alt', [False, True])
def test_actual_controller_heartbeat_current_native_acceptance(alt, monkeypatch):
  cp = params(CAR.GENESIS_GV60_EV_1ST_GEN, alt=alt)
  native, debug = initialize_native(cp.safetyConfigs[0].safetyParam)
  ci = CarInterface(cp)
  ci.update([])
  now = [1_000_000_000]
  monkeypatch.setattr('time.clock_gettime_ns', lambda clock: now[0])
  monkeypatch.setattr('time.monotonic_ns', lambda: now[0])
  reached = []
  for tick in range(1, 121):
    now[0] += 10_000_000
    native.set_timer(now[0] // 1000)
    frames = physical(ci, tick)
    for address, raw, bus in frames:
      native.safety_fwd_hook(bus, address)
      native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, raw))
    ci.update([(now[0], frames)])
    if tick <= 20:
      continue
    assert ci.CS.out.canValid and native.safety_config_valid() == debug
    _, emitted = ci.apply(structs.CarControl.new_message().as_reader(), now[0])
    for address, raw, bus in emitted:
      assert bool(native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))) == debug, hex(address)
    heartbeat = [(address, raw, bus) for address, raw, bus in emitted if address == 0x100]
    for _address, raw, bus in heartbeat:
      assert len(raw) == 24 and bus == 0
      reached.append(raw)
  assert len(reached) == 25 and len({raw[2] for raw in reached}) == 25


@pytest.mark.parametrize('alt', [False, True])
@pytest.mark.parametrize('mutation', ['crc', 'counter', 'body', 'brake', 'gas', 'bus', 'length', 'stale', 'relay'])
def test_exact_heartbeat_refuses_mutations(alt, mutation):
  cp = params(CAR.GENESIS_GV60_EV_1ST_GEN, alt=alt)
  native, debug = initialize_native(cp.safetyConfigs[0].safetyParam)
  ci = CarInterface(cp)
  ci.update([])
  for tick in range(1, 21):
    native.set_timer(1_000_000 + tick * 10_000)
    for address, raw, bus in physical(ci, tick):
      native.safety_fwd_hook(bus, address)
      native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, raw))
  address, body, bus = create_gv60_radar_heartbeat(1, False, False)
  accepted = bool(native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, body)))
  assert accepted == debug
  raw = bytearray(create_gv60_radar_heartbeat(2, False, False)[1])
  if mutation == 'counter':
    raw = bytearray(body)
  elif mutation == 'crc':
    raw[0] ^= 1
  elif mutation in ('body', 'brake', 'gas'):
    raw[{'body': 3, 'brake': 4, 'gas': 22}[mutation]] ^= 1
    raw[:2] = hkg_can_fd_checksum(address, None, raw).to_bytes(2, 'little')
  elif mutation == 'bus':
    bus = 1
  elif mutation == 'length':
    raw = raw[:16]
  elif mutation == 'stale':
    native.set_timer(3_000_000)
    native.safety_tick()
  elif mutation == 'relay':
    native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, raw))
    assert native.get_relay_malfunction() or not debug
  assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))


@pytest.mark.parametrize('word', [0x15, 0x95, 0x1000, 0x1011, 0x1014, 0x1094, 0x1815, 0x9014, 0x1015, 0x1095, 0x1017, 0x1097])
def test_generic_and_unknown_profiles_do_not_gain_heartbeat(word):
  native, _ = initialize_native(word)
  address, raw, bus = create_gv60_radar_heartbeat(1, False, False)
  assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))


@pytest.mark.parametrize('word', [0x9015, 0x9095])
def test_gv60_does_not_admit_aol_alternative_experience(word):
  native = libsafety_py.libsafety
  native.set_alternative_experience(32)
  native.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, word)
  assert not native.safety_config_valid()
  native.set_alternative_experience(0)


@pytest.mark.parametrize('alt', [False, True])
@pytest.mark.parametrize('byte_index', range(3, 24))
def test_every_reserved_heartbeat_byte_is_checked(alt, byte_index):
  cp = params(CAR.GENESIS_GV60_EV_1ST_GEN, alt=alt)
  native, debug = initialize_native(cp.safetyConfigs[0].safetyParam)
  ci = CarInterface(cp)
  ci.update([])
  for tick in range(1, 21):
    native.set_timer(1_000_000 + tick * 10_000)
    for address, raw, bus in physical(ci, tick):
      native.safety_fwd_hook(bus, address)
      native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, raw))
  address, body, bus = create_gv60_radar_heartbeat(1, False, False)
  accepted = bool(native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, body)))
  assert accepted == debug
  raw = bytearray(create_gv60_radar_heartbeat(2, False, False)[1])
  raw[byte_index] ^= 2
  raw[:2] = hkg_can_fd_checksum(address, None, raw).to_bytes(2, 'little')
  assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))


@pytest.mark.parametrize('alt', [False, True])
@pytest.mark.parametrize('brake,gas', [(False, False), (True, False), (False, True), (True, True)])
def test_heartbeat_reports_only_current_physical_pedals(alt, brake, gas):
  cp = params(CAR.GENESIS_GV60_EV_1ST_GEN, alt=alt)
  native, debug = initialize_native(cp.safetyConfigs[0].safetyParam)
  ci = CarInterface(cp)
  ci.update([])
  for tick in range(1, 21):
    native.set_timer(1_000_000 + tick * 10_000)
    for address, source, bus in physical(ci, tick):
      raw = bytearray(source)
      if address == 0x175 and bus == 1:
        raw[10] = (raw[10] & ~2) | (int(brake) << 1)
        raw[:2] = hkg_can_fd_checksum(address, None, raw).to_bytes(2, 'little')
      if address == 0x35 and bus == 1:
        raw[5] = int(gas)
        raw[:2] = hkg_can_fd_checksum(address, None, raw).to_bytes(2, 'little')
      native.safety_fwd_hook(bus, address)
      native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, raw))
  address, raw, bus = create_gv60_radar_heartbeat(1, brake, gas)
  assert bool(native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))) == debug
  address, raw, bus = create_gv60_radar_heartbeat(2, not brake, gas)
  assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))
  address, raw, bus = create_gv60_radar_heartbeat(3, brake, not gas)
  assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))


@pytest.mark.parametrize('alt', [False, True])
def test_gv60_ordinary_set_and_cancel_drive_current_native_controller(alt, monkeypatch):
  cp = params(CAR.GENESIS_GV60_EV_1ST_GEN, alt=alt)
  native, debug = initialize_native(cp.safetyConfigs[0].safetyParam)
  if not debug:
    assert not native.safety_config_valid()
    address, raw, bus = create_gv60_radar_heartbeat(1, False, False)
    assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw))
    return
  ci = CarInterface(cp)
  ci.update([])
  now = [1_000_000_000]
  monkeypatch.setattr('time.clock_gettime_ns', lambda clock: now[0])
  monkeypatch.setattr('time.monotonic_ns', lambda: now[0])
  active_commands = 0
  for tick in range(1, 121):
    now[0] += 10_000_000
    native.set_timer(now[0] // 1000)
    frames = []
    for address, source, bus in physical(ci, tick):
      raw = bytearray(source)
      if address == 0x1CF and bus == 1:
        button = Buttons.SET_DECEL if tick in (22, 24) else Buttons.CANCEL if tick in (100, 102) else Buttons.NONE
        raw[2] = (raw[2] & ~7) | int(button)
      native.safety_fwd_hook(bus, address)
      assert native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, raw))
      frames.append((address, bytes(raw), bus))
    state = ci.update([(now[0], frames)])
    active = 30 <= tick < 100
    if tick % 10 == 0:
      native.set_aol_test_heartbeat(active)
    if tick <= 20:
      continue
    assert state.canValid and native.safety_config_valid() and abs(state.steeringTorque) < 1.
    if tick >= 30:
      assert native.get_controls_allowed() == active
    command = structs.CarControl.new_message()
    command.enabled, command.latActive, command.longActive = active, active, active
    command.actuators.torque = 0.1 if active else 0.0
    command.actuators.accel = 0.2 if active else 0.0
    command.actuators.longControlState = 'pid' if active else 'off'
    applied, emitted = ci.apply(command.as_reader(), now[0])
    for address, raw, bus in emitted:
      assert native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, raw)), (hex(address), tick)
    active_commands += int(active and applied.torque != 0.0)
  assert active_commands > 0 and not native.get_controls_allowed()
