"""Actual registered mode39 bounds; no forced controlsAllowed or native callback bypass."""
import struct
import unittest

import pytest
from opendbc.car.tesla.tests.preap_stream import PreAPStream
from opendbc.car.tesla.preap.pedal_contract import PedalCalibration, PedalCodec
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.tesla.tests.pedal_fixture import configure_native, feedback_frame, native_words


def warm(bus, word, *, factor=1., zero=0., speed=80.):
  safety = libsafety_py.libsafety
  calibration = PedalCalibration(factor, zero, bus)
  configure_native(safety, calibration, word, b'actual-registered-profile-test')
  stream = PreAPStream()
  priming = PedalCodec(calibration)
  now = 0
  for tick in range(1, 91):
    now = 1_000_000 + tick * 10000
    safety.set_timer(now)
    safety.set_aol_test_heartbeat(tick >= 65)
    frames = stream.frames(tick, lever=2 if tick in (65, 75) else 0, cruise=1, speed=speed)
    if tick % 2 == 0:
      frames.append(feedback_frame(calibration, tick // 2))
    for address, data, source in frames:
      assert safety.safety_rx_hook(libsafety_py.make_CANPacket(address, source, data))
    safety.safety_tick()
    safety.aol_set_host_request(3 if tick >= 65 else 0)
    if tick % 2 == 0:
      assert transmit(safety, priming.command(enabled=False))
    if 65 <= tick < 75:
      assert not (safety.aol_get_permission_mask() & 2), 'one physical pull is lateral only'
  assert safety.safety_config_valid() and safety.get_controls_allowed()
  assert safety.aol_get_permission_mask() == 3
  return safety, calibration, now


def transmit(safety, frame):
  address, data, bus = frame
  return bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)))


@pytest.mark.parametrize('bus', [0, 2])
@pytest.mark.parametrize('word', [1, 3, 7])
def test_actual_registered_priming_rate_absolute_bus_and_counter_bounds(bus, word):
  safety, calibration, now = warm(bus, word)
  codec = PedalCodec(calibration)
  assert transmit(safety, codec.command(enabled=False))
  for di in range(0, 73, 4):
    assert transmit(safety, codec.command(float(di), enabled=True))
  valid = codec.command(74., enabled=True)
  # At80kph original speed-profile max is75.78DI; this77DI request is
  # within+5 rate from72 but outside absolute native demand.
  too_high = PedalCodec(calibration)
  too_high.counter = valid[1][4] & 15
  assert not transmit(safety, too_high.command(77., enabled=True))
  assert transmit(safety, valid), 'rejected TX must not corrupt accepted counter/rate state'
  replay = valid
  assert not transmit(safety, replay)
  address, data, source = codec.command(74., enabled=True)
  assert not transmit(safety, (address, data, 2 - source))
  broken = bytearray(data)
  broken[5] ^= 1
  assert not transmit(safety, (address, bytes(broken), source))
  assert transmit(safety, (address, data, source))
  safety.set_timer(now + 500001)
  assert not transmit(safety, codec.command(74., enabled=True))
  assert transmit(safety, codec.command(enabled=False)), 'zero/disabled must remain available to withdraw/resynchronize'


@pytest.mark.parametrize('fault', ['state', 'channels', 'counter', 'short'])
def test_actual_feedback_fault_withdraws_then_needs_fresh_physical_rearm(fault):
  safety, calibration, now = warm(2, 1)
  codec = PedalCodec(calibration)
  assert transmit(safety, codec.command(enabled=False))
  assert transmit(safety, codec.command(0., enabled=True))
  counter = 45 if fault == 'counter' else 46
  frame = feedback_frame(calibration, counter, state=1 if fault == 'state' else 0,
                         bad_channel=fault == 'channels')
  if fault == 'short':
    frame = frame[0], frame[1][:-1], frame[2]
  safety.set_timer(now + 1000)
  address, data, bus = frame
  safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
  assert not safety.get_controls_allowed()
  assert not (safety.aol_get_permission_mask() & 2)
  assert not transmit(safety, codec.command(0., enabled=True))
  assert transmit(safety, codec.command(enabled=False))


class TestTeslaPreapPedalStartup(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_safety_hooks(0, 0)
    self.safety.set_aol_test_heartbeat(False)
    self.safety.set_alternative_experience(32)

  def read(self):
    output = libsafety_py.ffi.new('uint8_t[48]')
    length = self.safety.tesla_preap_test_config_read(output)
    return bytes(libsafety_py.ffi.buffer(output, length))

  def stage(self, words):
    for index, value in enumerate(words):
      self.assertTrue(self.safety.tesla_preap_test_config_word(index, value))
    return bool(self.safety.tesla_preap_test_config_word(0xFFFF, 0xC051))

  def test_startup_order_commit_ack_and_consume_once(self):
    words = native_words(PedalCalibration(1., 0., 0), 1, b'startup-envelope')
    self.assertEqual(self.read(), b'')
    self.assertFalse(self.safety.tesla_preap_test_config_word(2, words[2]))
    self.assertFalse(self.safety.tesla_preap_test_config_word(0xFFFF, 0xC051))
    self.assertTrue(self.safety.tesla_preap_test_config_word(0, words[0]))
    self.assertFalse(self.safety.tesla_preap_test_config_word(24, 0))
    self.assertTrue(self.stage(words))
    self.assertEqual(self.read(), struct.pack('<24H', *words))
    self.assertFalse(self.safety.tesla_preap_test_config_word(0xFFFF, 0))
    self.assertEqual(self.read(), b'')
    self.assertTrue(self.stage(words))
    self.assertNotEqual(self.safety.set_safety_hooks(39, 3), 0)
    self.assertEqual(self.safety.get_current_safety_mode(), 19)  # rejected profile falls back to NoOutput
    self.assertFalse(transmit(self.safety, PedalCodec(PedalCalibration(1., 0., 0)).command(enabled=False)))
    self.assertEqual(self.read(), struct.pack('<24H', *words), 'a rejected profile cannot consume the matching capsule')
    self.assertEqual(self.safety.set_safety_hooks(39, 1), 0)
    self.assertTrue(transmit(self.safety, PedalCodec(PedalCalibration(1., 0., 0)).command(enabled=False)))
    self.assertEqual(self.read(), struct.pack('<24H', *words))
    self.assertFalse(self.safety.tesla_preap_test_config_word(0, 1))
    self.safety.set_safety_hooks(39, 1)
    self.assertFalse(transmit(self.safety, PedalCodec(PedalCalibration(1., 0., 0)).command(enabled=False)),
                     'startup capsule is consumed once')
    self.safety.set_safety_hooks(0, 0)
    self.assertEqual(self.read(), b'')

  def test_invalid_wire_calibration_never_commits(self):
    valid = native_words(PedalCalibration(1., 0., 2), 7, b'admission-negatives')
    cases = []
    for index, value in ((0, 2), (1, 1), (2, 0), (3, 0)):
      words = list(valid)
      words[index] = value
      cases.append(words)
    for factor, zero in ((0., 0.), (1e-8, 0.), (float('nan'), 0.),
                         (float('inf'), 0.), (1., float('nan')), (1., float('inf')), (1., 1e6)):
      cases.append(native_words(PedalCalibration(factor, zero, 2), 7, b'admission-negatives'))
    cases.append([*valid[:8], *([0] * 16)])
    for words in cases:
      with self.subTest(words=words[:8]):
        self.assertFalse(self.stage(words))
        self.assertEqual(self.read(), b'')

  def test_negative_calibration_and_gas_override_preserve_native_guards(self):
    safety, calibration, now = warm(2, 7, factor=-1., zero=100., speed=180.)
    codec = PedalCodec(calibration)
    self.assertTrue(transmit(safety, codec.command(enabled=False)))
    self.assertTrue(transmit(safety, codec.command(4., enabled=True)))
    unsafe = PedalCodec(calibration)
    unsafe.counter = codec.counter
    self.assertFalse(transmit(safety, unsafe.command(12., enabled=True)), 'rate limit applies to negative calibration')
    safety.set_timer(now + 20_000)
    address, data, bus = feedback_frame(calibration, 46, di=8.)
    self.assertTrue(safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data)))
    self.assertFalse(safety.aol_get_permission_mask() & 2)
    self.assertFalse(transmit(safety, codec.command(4., enabled=True)))
    self.assertTrue(transmit(safety, codec.command(enabled=False)))
