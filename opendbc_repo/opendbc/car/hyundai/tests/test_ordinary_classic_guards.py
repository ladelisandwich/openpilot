from types import SimpleNamespace
from unittest.mock import patch

import pytest
from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
from opendbc.car.hyundai.ordinary_classic_startup import OrdinaryClassicStartup
from opendbc.car.hyundai.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py


def owner_and_clock(*, silence=False, warm_failure=None):
  native = libsafety_py.libsafety
  native.set_alternative_experience(0)
  assert native.set_safety_hooks(structs.CarParams.SafetyModel.elm327, 1) == 0
  native.init_tests()
  now = [10_000_000_000]
  commands, owners = [], []
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x391] = 8
  cp = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], True, False, False)

  def sender(frames):
    assert native.get_current_safety_mode() == int(structs.CarParams.SafetyModel.elm327)
    assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
    for address, payload, bus in frames:
      assert native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, payload))

  def receiver(*args, **kwargs):
    now[0] += 100_000_000
    frames = []
    for parser in owners[0].ci.can_parsers.values():
      packer = CANPacker(parser.dbc.name)
      for address in parser.addresses:
        name = parser.dbc.addr_to_msg[address].name
        frame = CanData(*packer.make_can_msg(name, parser.bus, {}))
        if warm_failure == 'wrong_bus' and name in ('SCC11', 'SCC12'):
          frame = CanData(frame.address, frame.dat, 1)
        frames.append(frame)
    return [TimestampedCanPacket(frames, 1 if warm_failure == 'stale' else now[0])]

  owner = CarInterface.startup_owner(cp, (receiver, sender), requested=True)
  assert isinstance(owner, OrdinaryClassicStartup)
  owners.append(owner)

  def query(send, recv, bus, addresses, requests, responses):
    command = requests[0]
    commands.append(command)
    assert bus == 0 and addresses == [(0x7D0, None)]

    def data(timeout):
      send([(0x7D0, bytes([len(command)]) + command + bytes(7 - len(command)), 0)])
      if command == b'\x28\x83\x01':
        return {} if silence else {(0x7D0, None): b'\x7f\x28\x22'}
      return {(0x7D0, None): b''}

    return SimpleNamespace(get_data=data)

  owner._query = query
  return owner, now, commands


@pytest.mark.parametrize('failure', ['wrong_bus', 'stale'])
def test_restored_stock_cannot_publish_without_fresh_matching_scc(failure):
  owner, now, commands = owner_and_clock(warm_failure=failure)

  def advance(seconds):
    now[0] += round(seconds * 1e9)

  with (
    patch('time.clock_gettime_ns', side_effect=lambda clock: now[0]),
    patch('time.monotonic', side_effect=lambda: now[0] / 1e9),
    patch('time.sleep', side_effect=advance),
  ):
    try:
      stock = owner.prepare(admission=lambda: True)
      assert owner.restored and owner.outcome is Outcome.STOCK_RESTORED
      ci = CarInterface(stock)
      with pytest.raises(RuntimeError, match='stock SCC sources did not become fresh'):
        owner.configure(ci)
      assert owner.outcome is Outcome.ABORT_UNCERTAIN and not owner.published
      with pytest.raises(RuntimeError):
        owner.seal_publication()
      assert commands == [b'\x10\x03', b'\x28\x83\x01', b'\x10\x03', b'\x28\x00\x01', b'\x10\x01']
    finally:
      owner.close()
  assert owner.closed and owner.restored


def test_actual_keeper_thread_failure_restores_and_prevents_publication():
  owner, now, commands = owner_and_clock(silence=True)

  def advance(seconds):
    now[0] += round(seconds * 1e9)

  with (
    patch('time.clock_gettime_ns', side_effect=lambda clock: now[0]),
    patch('time.sleep', side_effect=advance),
    patch('threading.Thread.start', side_effect=RuntimeError('actual thread startup failed')) as start,
  ):
    with pytest.raises(RuntimeError, match='actual thread startup failed'):
      owner.prepare(admission=lambda: True)
    start.assert_called_once()
  assert owner.closed and owner.restored and not owner.published
  assert owner.keeper.thread is None and owner.keeper.event.is_set()
  assert commands == [b'\x10\x03', b'\x28\x83\x01', b'\x10\x03', b'\x28\x00\x01', b'\x10\x01']
  with pytest.raises(RuntimeError):
    owner.seal_publication()


def test_classic_scc_counter_corruption_remains_a_native_veto():
  from opendbc.safety.tests.test_hyundai import checksum
  from opendbc.car.hyundai.tests.test_ordinary_classic_stock_fallback import stock_native_frames
  from openpilot.starpilot.aol.vehicle import policy_for

  native = libsafety_py.libsafety
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x391] = 8
  cp = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
  policy = policy_for(cp)
  cp.safetyConfigs[0].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  ci = CarInterface(cp)
  ci.update([])
  config = cp.safetyConfigs[0]
  native.set_alternative_experience(cp.alternativeExperience)
  assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
  native.init_tests()
  native.set_alternative_experience(cp.alternativeExperience)
  for tick in range(1, 121):
    native.set_timer(tick * 10_000)
    for address, payload, bus in stock_native_frames(ci, tick):
      assert native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, payload))
    if tick % 100 == 0:
      native.safety_tick()
  assert native.safety_config_valid() and not native.safety_test_toyota_rx_summary_invalid()
  packer = CANPacker('hyundai_can_generated')
  address, payload, bus = checksum(packer.make_can_msg('SCC12', 0, {'ACCMode': 0, 'CR_VSM_Alive': 12}))
  rejected = False
  for tick in range(121, 129):
    native.set_timer(tick * 10_000)
    rejected |= not native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, bytes(payload)))
  native.safety_tick()
  assert rejected and not native.safety_config_valid() and native.safety_test_toyota_rx_summary_invalid()
  assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
