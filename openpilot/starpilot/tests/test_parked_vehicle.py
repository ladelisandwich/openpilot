"""Receive-only native CAN parser proof, bounded input and physical freshness."""

from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot import parked_vehicle as vehicle
from openpilot.starpilot.ui.tests.test_ioniq6_alpha_request import stock_ioniq
from opendbc.can.packer import CANPacker


class CANFixture:
  def __init__(self, monkeypatch):
    self.now = 10_000_000_000
    self.queue = deque()
    monkeypatch.setattr(vehicle.time, 'monotonic_ns', lambda: self.now)
    monkeypatch.setattr(vehicle.time, 'clock_gettime_ns', lambda _: self.now + 2_000_000_000)
    monkeypatch.setattr(messaging, 'sub_sock', Mock())
    monkeypatch.setattr(messaging, 'recv_one_or_none', lambda _: self.queue.popleft() if self.queue else None)
    self.reader = vehicle.ParkedVehicleReader(stock_ioniq().as_reader())
    self.packers = {bus: CANPacker(parser.dbc_name) for bus, parser in self.reader.parsers.items()}
    # Normal CarState lazy subscriptions become explicit before generating data.
    self.reader.state.update(self.reader.parsers)
    self.park = next(value for value, text in self.reader.state.shifter_values.items() if text in ('P', 'PARK'))
    self.drive = next(value for value, text in self.reader.state.shifter_values.items() if text in ('D', 'DRIVE'))

  def event(self, *, gear=None, speed=0, button=0, omit=()):
    frames = []
    for bus, parser in self.reader.parsers.items():
      for state in tuple(parser.message_states.values()):
        if state.name in omit:
          continue
        values = {}
        if state.name == self.reader.state.gear_msg_canfd:
          values['GEAR'] = self.park if gear is None else gear
        if state.name == 'WHEEL_SPEEDS':
          values.update(dict.fromkeys(('WHL_SpdFLVal', 'WHL_SpdFRVal', 'WHL_SpdRLVal', 'WHL_SpdRRVal'), speed))
        if state.name == 'CRUISE_BUTTONS':
          values['CRUISE_BUTTONS'] = button
        address, data, src = self.packers[bus].make_can_msg(state.name, parser.bus, values)
        frames.append({'address': address, 'dat': data, 'src': src})
    event = messaging.new_message('can', len(frames), valid=True)
    event.logMonoTime = self.now + 2_000_000_000
    event.can = frames
    return event

  def tick(self, **kwargs):
    self.now += 10_000_000
    self.queue.append(self.event(**kwargs).as_reader())
    return self.reader.poll()

  def warm(self):
    for _ in range(150):
      self.tick()
    assert self.reader.ready(self.now, self.now + 2_000_000_000)


@pytest.fixture
def can(monkeypatch):
  fixture = CANFixture(monkeypatch)
  yield fixture
  fixture.reader.close()


def test_actual_carstate_and_canparser_park_buttons_and_cleanup(can):
  can.warm()
  assert vehicle.stationary(can.reader.state.out)
  found = []
  for pressed in (0, 1, 1, 0, 2, 0):
    found.extend(event for sample in can.tick(button=pressed) for event in sample.buttons)
  assert ('accelCruise', True) in found and ('accelCruise', False) in found
  assert ('decelCruise', True) in found and ('decelCruise', False) in found
  can.reader.close()
  assert can.reader.socket is None and not can.reader.parsers
  assert not can.reader.ready(can.now, can.now + 2_000_000_000)


@pytest.mark.parametrize('fault', ['drive', 'speed', 'stale_gear', 'stale_wheels', 'source', 'receiver', 'future', 'invalid', 'order', 'backlog'])
def test_actual_decoder_refuses_changed_or_stale_physical_state(can, fault):
  can.warm()
  if fault in ('drive', 'speed'):
    can.tick(**({'gear': can.drive} if fault == 'drive' else {'speed': 1.}))
  elif fault in ('stale_gear', 'stale_wheels'):
    name = can.reader.state.gear_msg_canfd if fault == 'stale_gear' else 'WHEEL_SPEEDS'
    for _ in range(27):
      can.tick(omit=(name,))
  elif fault in ('source', 'receiver'):
    now, boot = can.now, can.now + 2_000_000_000
    assert not can.reader.ready(now + (260_000_000 if fault == 'receiver' else 0), boot + (260_000_000 if fault == 'source' else 0))
    return
  else:
    can.now += 10_000_000
    event = can.event()
    if fault == 'future':
      event.logMonoTime += 1
    elif fault == 'invalid':
      event.valid = False
    elif fault == 'order':
      event.logMonoTime = can.reader.last_source_ns
    if fault == 'backlog':
      for i in range(vehicle.MAX_PACKETS):
        message = can.event()
        message.logMonoTime = can.now + 2_000_000_000 - vehicle.MAX_PACKETS + i
        can.queue.append(message.as_reader())
    else:
      can.queue.append(event.as_reader())
    with pytest.raises(ValueError):
      can.reader.poll()
    return
  assert not can.reader.ready(can.now, can.now + 2_000_000_000)


def test_receipt_clock_is_sampled_after_recv_not_before(can, monkeypatch):
  can.warm()
  def received(_):
    if not can.queue:
      return None
    can.now += 1_000_000
    event = can.queue.popleft().as_builder()
    event.logMonoTime = can.now + 2_000_000_000
    return event.as_reader()
  monkeypatch.setattr(messaging, 'recv_one_or_none', received)
  can.tick()
  assert can.reader.ready(can.now, can.now + 2_000_000_000)


def test_constructor_never_creates_interface_controller_or_publisher(monkeypatch):
  from opendbc.car.hyundai.interface import CarInterface
  monkeypatch.setattr(CarInterface, '__init__', Mock(side_effect=AssertionError('No control interface in game')))
  monkeypatch.setattr(messaging, 'PubMaster', Mock(side_effect=AssertionError('No publishing in game')))
  monkeypatch.setattr(messaging, 'pub_sock', Mock(side_effect=AssertionError('No publishing in game')))
  reader = vehicle.ParkedVehicleReader(stock_ioniq().as_reader())
  reader.close()
  with pytest.raises(ValueError):
    vehicle.ParkedVehicleReader(NS(carFingerprint='unknown', notCar=False, brand='hyundai'))
