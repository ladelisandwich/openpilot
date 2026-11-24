"""Native standstill source credit, driver forwarding, and receiver delivery."""

import unittest
from opendbc.can import CANParser
from opendbc.car.subaru.sng_sender import replacement
from opendbc.safety.tests import test_subaru_stock_aol as stock
from opendbc.safety.tests.libsafety import libsafety_py


class TestSubaruSng(unittest.TestCase):
  def setup(self, word=8, experience=0):
    self.base = stock.TestSubaruStockAol()
    self.base.configure(word, experience)
    self.safety = libsafety_py.libsafety
    self.receiver = CANParser('subaru_global_2017_generated', [('Throttle', 100), ('Brake_Pedal', 50)], 2)
    self.distance = 4.0
    self.throttle = 0
    self.brake = 0
    for _ in range(15):
      self.feed(cruise=False)
    self.feed(cruise=True)
    if word in (8, 520):
      self.assertTrue(self.safety.get_controls_allowed())

  def feed(self, *, cruise=True, main=True, driver_gas=0, driver_brake=0, distance=4.0, lead=True, moving=False, gear=121, temporary=False):
    self.base.now += 10000
    self.safety.set_timer(self.base.now)
    samples = [
      ('Throttle', {'Throttle_Pedal': driver_gas, 'Signal1': 9, 'Engine_RPM': 1234, 'Neutral': 1, 'Signal3': 7, 'Off_Accel': 3}, 0),
      ('Brake_Pedal', {'Brake_Pedal': driver_brake, 'Speed': 0, 'Signal1': 7, 'Signal2': 31, 'Signal3': 1, 'Signal4': 0xABCD}, 0),
      ('Wheel_Speeds', dict.fromkeys(('FL', 'FR', 'RL', 'RR'), 54 if moving else 0), 0),
      ('Steering_Torque', {'Steer_Warning': int(temporary)}, 0),
      ('Brake_Status', {'Brake': int(bool(driver_brake))}, 0),
      ('CruiseControl', {'Cruise_On': int(main), 'Cruise_Activated': int(cruise)}, 0),
      ('Transmission', {'Gear': gear}, 0),
      ('ES_Distance', {'Car_Follow': int(lead), 'Close_Distance': distance}, 2),
    ]
    result = []
    for name, values, bus in samples:
      msg = self.base.frame(name, values, bus)
      self.safety.safety_rx_hook(msg)
      result.append((msg.addr, bytes(msg.data[0:8]), bus))
    self.safety.safety_tick()
    for address, data, bus in result:
      if bus == 0 and address in (0x40, 0x139) and self.safety.safety_fwd_hook(bus, address) == 2:
        self.receiver.update([(self.base.now * 1000, [(address, data, 2)])])
    return result

  def send(self, source, throttle=False, speed=False):
    address, data, _bus = source
    frame = replacement(address, data, throttle=throttle, speed=speed)
    accepted = self.safety.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))
    if accepted:
      self.receiver.update([(self.base.now * 1000 + 1000, [frame])])
    return bool(accepted)

  def test_passthrough_delivery_then_bounded_forwarding_lease_and_expiry(self):
    for experience, word in ((0, 8), (32, 8), (0, 520), (32, 520)):
      self.setup(experience=experience, word=word)
      for _ in range(50):
        sources = self.feed()
        self.assertTrue(self.send(sources[0]))
        self.assertTrue(self.send(sources[1]))
        self.assertFalse(self.send(sources[0]))
        self.assertFalse(self.send(sources[1]))
        self.assertEqual(self.safety.safety_fwd_hook(0, 0x40), -1)
        self.assertEqual(self.safety.safety_fwd_hook(0, 0x139), -1)
      self.assertTrue(self.receiver.can_valid)
      self.assertLessEqual(self.receiver.message_states[0x40].counter_fail, 1)
      self.assertLessEqual(self.receiver.message_states[0x139].counter_fail, 1)
      self.safety.set_timer(self.base.now + 40000)
      self.assertEqual(self.safety.safety_fwd_hook(0, 0x40), 2)
      self.assertEqual(self.safety.safety_fwd_hook(0, 0x139), 2)
      self.base.now += 40000
      for _ in range(4):
        self.feed()
        self.assertTrue(self.receiver.can_valid)
        self.assertLessEqual(self.receiver.message_states[0x40].counter_fail, 1)
        self.assertLessEqual(self.receiver.message_states[0x139].counter_fail, 1)
      self.assertEqual(self.receiver.message_states[0x40].counter_fail, 0)
      self.assertEqual(self.receiver.message_states[0x139].counter_fail, 0)

  def test_resume_requires_departure_and_fifteen_copy_bound(self):
    self.setup()
    sources = self.feed()
    self.assertFalse(self.send(sources[0], throttle=True))
    sources = self.feed(distance=4.02)
    self.assertTrue(self.send(sources[0], throttle=True))
    for _ in range(14):
      sources = self.feed(distance=4.02)
      self.assertTrue(self.send(sources[0], throttle=True))
    sources = self.feed(distance=4.02)
    self.assertFalse(self.send(sources[0], throttle=True))
    self.assertTrue(self.send(sources[0]))

  def test_driver_pedals_forward_immediately_and_clear_pending_departure(self):
    for gas, brake in ((5, 0), (0, 512)):
      self.setup()
      sources = self.feed(distance=4.02)
      self.assertTrue(self.send(sources[0], throttle=True))
      self.assertTrue(self.send(sources[1]))
      sources = self.feed(driver_gas=gas, driver_brake=brake)
      self.assertEqual(self.safety.safety_fwd_hook(0, 0x40 if gas else 0x139), 2)
      # Physical gas five is already an exact unmodified pass-through.
      self.assertEqual(self.send(sources[0], throttle=True), bool(gas))
      self.assertFalse(self.send(sources[1], speed=True))
      sources = self.feed()
      self.assertFalse(self.send(sources[0], throttle=True))

  def test_hold_waits_for_standstill_and_preserves_brake_bytes(self):
    self.setup()
    sources = self.feed()
    self.assertFalse(self.send(sources[1], speed=True))
    for _ in range(52):
      sources = self.feed()
    self.assertTrue(self.send(sources[1], speed=True))
    raw = sources[1][1]
    delivered = self.receiver.vl['Brake_Pedal']
    self.assertEqual(delivered['Brake_Pedal'], 0)
    self.assertEqual(delivered['Signal4'], 0xABCD)
    self.assertEqual(raw[4:], replacement(0x139, raw, speed=True)[1][4:])

  def test_wrong_configuration_and_physical_vetoes(self):
    for word in (0, 1, 2, 9, 16, 512, 521, 522):
      self.setup(word=word)
      sources = self.feed(distance=4.02)
      self.assertFalse(self.send(sources[0], throttle=True))
      self.assertFalse(self.send(sources[1], speed=True))
    for fields in ({'cruise': False}, {'main': False}, {'lead': False}, {'moving': True}, {'gear': 3}, {'temporary': True}):
      self.setup()
      sources = self.feed(distance=4.02, **fields)
      self.assertFalse(self.send(sources[0], throttle=True))
      self.assertFalse(self.send(sources[1], speed=True))

  def test_stale_source_bad_checksum_and_opaque_mutation_reject(self):
    self.setup()
    sources = self.feed()
    address, data, bus = replacement(sources[0][0], sources[0][1])
    bad = bytearray(data)
    bad[3] ^= 0x20
    bad[0] = ((address & 255) + (address >> 8) + sum(bad[1:])) & 255
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, bytes(bad))))
    self.safety.set_timer(self.base.now + 20001)
    self.assertFalse(self.send(sources[0]))
    self.setup()
    sources = self.feed()
    bad = bytearray(sources[1][1])
    bad[0] ^= 1
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x139, 0, bytes(bad)))
    self.assertFalse(self.send(sources[1]))
