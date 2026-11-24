"""Native source-credit checks for Dom's stock Resume/Set transport."""
import unittest
from opendbc.safety.tests import test_subaru_stock_aol as stock
from opendbc.safety.tests.libsafety import libsafety_py


class TestSubaruSoftwareCruise(unittest.TestCase):
  def setup(self, experience=0, word=512):
    self.base = stock.TestSubaruStockAol()
    self.base.configure(word, experience)
    self.safety = libsafety_py.libsafety
    for _ in range(12):
      self.base.feed(cruise=False)
    self.base.feed(cruise=True)
    self.assertTrue(self.safety.get_controls_allowed())
    self.source()
    self.source()

  def source(self, **fields):
    values = dict(Signal1=0x12345, Signal2=0x12345, **fields)
    msg = self.base.frame('Cruise_Buttons', values)
    self.counter = msg.data[1] & 15
    self.safety.safety_rx_hook(msg)
    return msg

  def button(self, copy=1, direction='Resume', **fields):
    values = {'Signal1': 0x12345, 'Signal2': 0x12345, 'COUNTER': (self.counter + copy) % 16}
    values[direction] = 1
    values.update(fields)
    return self.base.frame('Cruise_Buttons', values)

  def test_exact_stock_pair_and_no_direct_pedal_permission(self):
    for experience, word in ((0, 512), (32, 512), (0, 520), (32, 520)):
      self.setup(experience, word=word)
      first, second = self.button(1), self.button(2)
      self.assertTrue(self.safety.safety_tx_hook(first))
      self.assertTrue(self.safety.safety_tx_hook(second))
      self.assertFalse(self.safety.safety_tx_hook(first))
      self.assertFalse(self.safety.safety_tx_hook(second))
      self.assertFalse(self.safety.safety_tx_hook(self.button(3)))
      self.assertFalse(self.safety.safety_tx_hook(self.base.frame('Throttle', {'Throttle_Pedal': 5}, 2)))
      self.assertFalse(self.safety.safety_tx_hook(self.base.frame('ES_Brake', {'Brake_Pressure': 1}, 0)))

  def test_set_direction_and_opaque_payload_are_bound(self):
    self.setup()
    self.assertFalse(self.safety.safety_tx_hook(self.button(Main=1)))
    self.assertFalse(self.safety.safety_tx_hook(self.button(Signal1=0)))
    self.assertFalse(self.safety.safety_tx_hook(self.button(Signal2=0)))
    self.assertFalse(self.safety.safety_tx_hook(self.button(copy=2)))
    self.assertTrue(self.safety.safety_tx_hook(self.button(direction='Set')))
    self.assertFalse(self.safety.safety_tx_hook(self.button(copy=2, direction='Resume')))
    self.assertTrue(self.safety.safety_tx_hook(self.button(copy=2, direction='Set')))

  def test_stock_off_brake_reverse_fault_main_and_wrong_configuration_veto(self):
    changes = ({'cruise': False}, {'cruise': True, 'brake': True}, {'cruise': True, 'gear': 3},
               {'cruise': True, 'temporary': True}, {'cruise': True, 'permanent': True}, {'cruise': True, 'main': False})
    for change in changes:
      self.setup()
      self.base.feed(**change)
      self.assertFalse(self.safety.safety_tx_hook(self.button()))
    for word in (0, 1, 17, 145, 177, 2, 513, 514, 521, 522):
      self.setup(word=word)
      self.assertFalse(self.safety.safety_tx_hook(self.button()))

  def test_physical_held_buttons_checksum_stale_and_burst_veto(self):
    for held in ('Main', 'Set', 'Resume'):
      self.setup()
      self.source(**{held: 1})
      self.assertFalse(self.safety.safety_tx_hook(self.button()))
    self.setup()
    bad = self.source()
    bad.data[0] ^= 1
    self.safety.safety_rx_hook(bad)
    self.assertFalse(self.safety.safety_tx_hook(self.button()))
    self.setup()
    self.safety.set_timer(self.base.now + 100_001)
    self.assertFalse(self.safety.safety_tx_hook(self.button()))
    self.setup()
    self.assertTrue(self.safety.safety_tx_hook(self.button()))
    self.assertTrue(self.safety.safety_tx_hook(self.button(copy=2)))
    self.source()
    self.assertFalse(self.safety.safety_tx_hook(self.button()))
    self.safety.set_timer(self.base.now + 1)
    self.assertFalse(self.safety.safety_tx_hook(self.button(copy=2)))
