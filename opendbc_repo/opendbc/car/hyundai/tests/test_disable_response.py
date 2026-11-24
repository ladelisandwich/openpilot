from unittest.mock import patch

import pytest

from opendbc.car import disable_ecu as generic_disable
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.ev6_startup import EV6Startup
from opendbc.car.hyundai.g90_startup import G90Startup
from opendbc.car.hyundai.gv70_startup import GV70Startup
from opendbc.car.hyundai.legacy_long_startup import LegacyLongStartup
from opendbc.car.hyundai.values import HyundaiSafetyFlags


OWNERS = (G90Startup, LegacyLongStartup, EV6Startup, GV70Startup)
RESPONSES = (None, b'\x68\x03', b'\x7f\x28\x12', b'\x7f\x28\x13', b'\x7f\x28\x22',
             b'\x7f\x28\x31', b'\x7f\x28\x33', b'\x68\x01', b'\x7f', 'exception')


@pytest.mark.parametrize('owner_type', OWNERS)
@pytest.mark.parametrize('response', RESPONSES)
def test_inherited_disable_observes_response_before_acceptance(owner_type, response):
  owner = object.__new__(owner_type)
  owner.address = 0x7d0 if owner_type in (G90Startup, LegacyLongStartup) else 0x730
  owner.bus = 0 if owner.address == 0x7d0 else 1
  owner.session_attempted = owner.disable_attempted = False
  sent, observations = [], []
  owner.callbacks = (lambda **kwargs: [], sent.extend)

  def query(send, recv, bus, addresses, requests, responses):
    assert bus == owner.bus and addresses == [(owner.address, None)]
    request = requests[0]
    assert requests in ([b'\x10\x03'], [b'\x28\x83\x01'])
    assert responses == ([b'\x50\x03'] if request == b'\x10\x03' else [b''])

    class Query:
      def get_data(self, timeout):
        observations.append((request, timeout))
        send([(owner.address, bytes([len(request)]) + request + bytes(7-len(request)), owner.bus)])
        if request == b'\x10\x03':
          return {(owner.address, None): b''}
        if response == 'exception':
          raise OSError('Transport failed after command send')
        return {} if response is None else {(owner.address, None): response}
    return Query()

  owner._query = query
  with patch.object(generic_disable, 'IsoTpParallelQuery', side_effect=query), patch('time.sleep') as sleep:
    result = owner._disable(owner.callbacks[0], owner._send, bus=owner.bus, addr=owner.address, com_cont_req=b'\x28\x83\x01')
  assert result == (response in (None, b'\x68\x03'))
  assert observations == [(b'\x10\x03', .1), (b'\x28\x83\x01', .1)]
  assert sleep.call_args_list == [((.05,), {})]
  assert owner.session_attempted and owner.disable_attempted
  assert len(sent) == 2


@pytest.mark.parametrize('restore_verified', (True, False))
def test_g90_explicit_rejection_cannot_publish_longitudinal(restore_verified):
  import time
  from opendbc.can import CANPacker
  from opendbc.car import Bus, gen_empty_fingerprint, structs, car_helpers
  from opendbc.car.can_definitions import CanData
  from opendbc.car.hyundai import g90_startup
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
  from opendbc.car.hyundai.values import CAR, DBC
  from openpilot.starpilot.vehicle_startup import VehicleStartupOwner

  holder = VehicleStartupOwner()
  interfaces, sent, requests = [], [], []
  packer = CANPacker(DBC[CAR.GENESIS_G90][Bus.pt])

  def recv(wait_for_one=False):
    if not interfaces:
      return []
    frames = []
    for parser in interfaces[0].can_parsers.values():
      for address in parser.addresses:
        frames.append(CanData(*packer.make_can_msg(parser.dbc.addr_to_msg[address].name, parser.bus, {})))
    return [TimestampedCanPacket(frames, time.clock_gettime_ns(time.CLOCK_BOOTTIME))]

  def query(send, receive, bus, addresses, request, response):
    payload = request[0]
    requests.append(payload)

    class Query:
      def get_data(self, timeout):
        send([(addresses[0][0], bytes([len(payload)]) + payload + bytes(7-len(payload)), bus)])
        if payload == b'\x10\x03':
          return {(addresses[0][0], None): b''}
        if payload == b'\x28\x83\x01':
          return {(addresses[0][0], None): b'\x7f\x28\x22'}
        if payload in (b'\x28\x00\x01', b'\x10\x01'):
          return {(addresses[0][0], None): b''} if restore_verified else {}
        raise AssertionError(payload)
    return Query()

  def prepare(cp, candidate, fingerprints, firmware):
    return holder.prepare(cp, CarInterface, (recv, sent.extend), requested=True, admission=lambda: True)

  fp = gen_empty_fingerprint()
  original = CarInterface.get_params(CAR.GENESIS_G90, fp, [], True, False, False)
  fingerprint = (CAR.GENESIS_G90, fp, '0'*17, [], original.fingerprintSource, True)
  with patch.object(time, 'CLOCK_BOOTTIME', getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC), create=True), \
       patch.object(car_helpers, 'fingerprint', return_value=fingerprint), \
       patch.object(generic_disable, 'IsoTpParallelQuery', side_effect=query), \
       patch.object(g90_startup, 'IsoTpParallelQuery', side_effect=query), patch('time.sleep'):
    if not restore_verified:
      with pytest.raises(RuntimeError, match='restoration unverified'):
        car_helpers.get_car(recv, sent.extend, lambda *args: None, True, False, pre_create_hook=prepare)
      return
    ci = car_helpers.get_car(recv, sent.extend, lambda *args: None, True, False, pre_create_hook=prepare)
    interfaces.append(ci)
    holder.configure(ci)
    holder.seal_publication()
    with structs.CarParams.from_bytes(ci.CP.to_bytes()) as reader:
      published = reader.as_builder()
  assert holder.owner.outcome is Outcome.STOCK_RESTORED
  assert not published.openpilotLongitudinalControl and published.pcmCruise
  assert not published.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.LONG.value
  assert b'\x28\x00\x01' in requests and b'\x10\x01' in requests
  assert original.openpilotLongitudinalControl


@pytest.mark.parametrize('owner_type', OWNERS)
@pytest.mark.parametrize('response', (None, b'\x68\x03', b'\x7f\x28\x22'))
def test_real_isotp_response_reaches_shared_owner(owner_type, response):
  from opendbc.car.can_definitions import CanData
  from opendbc.car.isotp_parallel_query import IsoTpParallelQuery

  owner = object.__new__(owner_type)
  owner.address = 0x7d0 if owner_type in (G90Startup, LegacyLongStartup) else 0x730
  owner.bus = 0 if owner.address == 0x7d0 else 1
  owner.session_attempted = owner.disable_attempted = False
  pending, sent = [], []

  def recv(wait_for_one=False):
    frames = list(pending)
    pending.clear()
    return [frames] if frames else []

  def send(frames):
    for address, data, bus in frames:
      sent.append((address, data, bus))
      assert address == owner.address and bus == owner.bus
      request = data[1:1 + data[0]]
      if request == b'\x10\x03':
        payload = b'\x50\x03\x00\x14\x01\xf4'
      elif request == b'\x28\x83\x01':
        payload = response
      else:
        raise AssertionError(request)
      if payload is not None:
        pending.append(CanData(address + 8, bytes([len(payload)]) + payload + bytes(7-len(payload)), bus))

  owner.callbacks = recv, send
  owner._query = IsoTpParallelQuery
  with patch.object(generic_disable, 'IsoTpParallelQuery', IsoTpParallelQuery), patch('time.sleep') as sleep:
    result = owner._disable(recv, owner._send, bus=owner.bus, addr=owner.address, com_cont_req=b'\x28\x83\x01')
  assert result == (response in (None, b'\x68\x03'))
  assert owner.session_attempted and owner.disable_attempted
  assert len(sent) == 2
  assert sleep.call_args_list == [((.05,), {})]


@pytest.mark.parametrize('owner_type', OWNERS)
def test_session_timeout_never_sends_communication_control(owner_type):
  owner = object.__new__(owner_type)
  owner.address, owner.bus = 0x7d0, 0
  owner.session_attempted = owner.disable_attempted = False
  sent, observations = [], []
  owner.callbacks = (lambda **kwargs: [], sent.extend)

  def query(send, recv, bus, addresses, requests, responses):
    assert requests == [b'\x10\x03']

    class Query:
      def get_data(self, timeout):
        observations.append(timeout)
        send([(owner.address, b'\x02\x10\x03\x00\x00\x00\x00\x00', owner.bus)])
        return {}
    return Query()

  owner._query = query
  with patch.object(generic_disable, 'IsoTpParallelQuery', side_effect=query), patch('time.sleep'):
    result = owner._disable(owner.callbacks[0], owner._send, bus=owner.bus, addr=owner.address, com_cont_req=b'\x28\x83\x01')
  assert result is False
  assert observations == [.1] * 10
  assert len(sent) == 10 and owner.session_attempted and not owner.disable_attempted
