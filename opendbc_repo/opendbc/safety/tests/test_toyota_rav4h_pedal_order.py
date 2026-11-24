"""Actual interleaved physical producers and required/optional ingress boundary."""

import pytest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.toyota import pedal
from opendbc.car.toyota.toyotacan import toyota_checksum
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers

source_matched_native = helpers.source_matched_native


class PhysicalSources:
  def __init__(self, word, experience):
    self.word = word
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_alternative_experience(experience)
    assert self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, word) == 0
    self.safety.set_aol_test_heartbeat(True)
    self.packer = CANPacker('toyota_nodsu_pt_generated' if word == 0x2049 else 'toyota_tnga_k_pt_generated')
    self.counter = 0
    self.now = 1_000_000
    for _ in range(25):
      self.tick()

  def rx(self, frame):
    a, d, b = frame
    return bool(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(a, b, d)))

  def sensor(self, gas=0):
    a, d, b = self.packer.make_can_msg('GAS_SENSOR', 0, {'INTERCEPTOR_GAS': gas, 'INTERCEPTOR_GAS2': gas, 'COUNTER_PEDAL': self.counter, 'STATE': 0})
    self.counter = (self.counter + 1) & 15
    d = bytearray(d)
    d[-1] = pedal.checksum(d)
    return a, bytes(d), b

  def pcm(self, gas=False):
    return self.packer.make_can_msg('PCM_CRUISE', 0, {'CRUISE_ACTIVE': 1, 'GAS_RELEASED': not gas, 'CRUISE_STATE': 8})

  def tick(self, cruise=True):
    self.now += 10_000
    self.safety.set_timer(self.now)
    frames = helpers.physical(self.packer, cruise=cruise)
    if self.word == 0x2049:
      frames = [frame for frame in frames if frame[0] != 0x2FF]
    for frame in frames:
      assert self.rx(frame)
    assert self.rx(self.sensor())
    self.safety.safety_tick()
    self.safety.aol_set_host_request(3)

  def tx(self, value):
    a, d, b = pedal.command(self.packer, value, 0)
    return bool(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d)))

  def close(self):
    self.safety.set_alternative_experience(0)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)


@pytest.mark.parametrize('word', (0xB049, 0xB249, 0x2049))
@pytest.mark.parametrize('experience', (0, 32))
def test_interleaved_producer_cannot_clear_other_gas_veto(word, experience):
  source = PhysicalSources(word, experience)
  try:
    assert source.safety.safety_config_valid()
    assert source.rx(source.sensor(806))
    assert source.safety.get_gas_pressed_prev() and not source.tx(0.1)
    # No pedal update occurs here. A released PCM source must retain pedal gas.
    assert source.rx(source.pcm(gas=False))
    assert source.safety.get_gas_pressed_prev() and not source.tx(0.1)
    assert source.rx(source.sensor())
    source.tick(cruise=False)
    source.tick(cruise=True)
    assert source.rx(source.pcm(gas=True))
    assert source.rx(source.sensor())
    # Existing standalone pedal keeps its original pedal-owned gas semantics.
    # The composite must retain the independent PCM veto between packets.
    assert bool(source.safety.get_gas_pressed_prev()) == (word != 0x2049)
    if word != 0x2049:
      assert not source.tx(0.1)
    assert source.tx(0)
  finally:
    source.close()


@pytest.mark.parametrize('word', (0xB049, 0xB249))
@pytest.mark.parametrize('experience', (0, 32))
@pytest.mark.parametrize('address', (0x262, 0x1D3))
def test_wrong_length_optional_ingress_cannot_retain_required_main_or_eps(word, experience, address):
  source = PhysicalSources(word, experience)
  try:
    assert source.tx(0.1) == (word == 0xB049)
    data = bytearray(8 if address == 0x262 else 7)
    data[-1] = toyota_checksum(address, None, data)
    # The generic ingress accepts an unmatched frame; its mode-owned observer
    # must invalidate source authority without refreshing the mandatory row.
    assert source.rx((address, bytes(data), 0))
    assert source.safety.aol_get_permission_mask() == 0
    assert not source.tx(0.1) and source.tx(0)
    source.tick(cruise=False)
    source.tick(cruise=True)
    assert source.tx(0.1) == (word == 0xB049)
  finally:
    source.close()
