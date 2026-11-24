"""CRUISE_BUTTONS rollover observed on EV6 route 00000007--9ca3e15d3c."""
import unittest

from opendbc.safety.tests import test_hyundai_hda2_long_aol as helpers


class TestHyundaiButtonRollover(unittest.TestCase):
  setUp = helpers.TestHyundaiHda2LongAol.setUp
  tearDown = helpers.TestHyundaiHda2LongAol.tearDown
  mode = helpers.TestHyundaiHda2LongAol.mode
  packet = helpers.TestHyundaiHda2LongAol.packet
  rx = helpers.TestHyundaiHda2LongAol.rx
  healthy = helpers.TestHyundaiHda2LongAol.healthy

  def start(self, word, optional=False):
    self.mode(word)
    if self.release:
      self.skipTest('Independent torque LONG authority is DEBUG only')
    self.healthy()
    self.safety.set_aol_test_heartbeat(True)
    self.safety.aol_set_host_request(5 if optional else 0)
    self.now = 1_000_000

  def button(self, counter, button=0, advance=20_000):
    self.now += advance
    self.safety.set_timer(self.now)
    self.rx('CRUISE_BUTTONS', {'COUNTER': counter, 'CRUISE_BUTTONS': button})

  def claim_and_withdraw(self, end, optional, brake=False):
    self.safety.aol_set_host_request(7 if optional else 3)
    self.assertEqual(self.safety.aol_get_permission_mask(), 3)
    if brake:
      self.rx('TCS', {'DriverBraking': 1})
    else:
      self.button(end, 4)
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.aol_set_host_request(5 if optional else 1)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_full_engagement_retains_lateral_across_14_to_zero(self):
    for word in (0x815, 0x895, 0x2815, 0x2895):
      for button in (1, 2):
        for wrap in ('press', 'held', 'release'):
          for brake in (False, True):
            with self.subTest(word=word, button=button, wrap=wrap, brake=brake):
              self.start(word)
              samples = {'press': ((14, 0), (0, button), (1, 0)),
                         'held': ((13, 0), (14, button), (0, button), (1, 0)),
                         'release': ((13, 0), (14, button), (0, 0))}[wrap]
              for count, value in samples:
                self.button(count, value)
              self.claim_and_withdraw(samples[-1][0] + 1, False, brake)

  def test_optional_release_retains_lateral_across_14_to_zero(self):
    for word in (0x815, 0x895, 0x2815, 0x2895):
      for button in (1, 2):
        for wrap in ('press', 'held', 'release'):
          with self.subTest(word=word, button=button, wrap=wrap):
            self.start(word, optional=True)
            samples = {'press': ((14, 0), (0, button), (1, 0)),
                       'held': ((13, 0), (14, button), (0, button), (1, 0)),
                       'release': ((13, 0), (14, button), (0, 0))}[wrap]
            for count, value in samples:
              self.button(count, value)
            self.claim_and_withdraw(samples[-1][0] + 1, True)

  def test_existing_15_to_zero_rollover_still_authorizes(self):
    for optional in (False, True):
      for word in (0x815, 0x895, 0x2815, 0x2895):
        with self.subTest(optional=optional, word=word):
          self.start(word, optional)
          self.button(14)
          self.button(15, 2)
          self.button(0)
          self.claim_and_withdraw(1, optional)

  def test_duplicates_gaps_and_expired_samples_do_not_authorize(self):
    for optional in (False, True):
      for first, pressed, released, delay in ((5, 5, 6, 20_000), (5, 7, 8, 20_000),
                                              (14, 1, 2, 20_000), (13, 14, 0, 100_001)):
        with self.subTest(optional=optional, first=first, pressed=pressed, delay=delay):
          self.start(0x815, optional)
          self.button(first)
          self.button(pressed, 2, delay)
          self.button(released)
          self.safety.aol_set_host_request(7 if optional else 3)
          self.button((released + 1) & 15, 4)
          self.safety.aol_set_host_request(5 if optional else 1)
          self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_captured_ev6_set_sequence_retains_lateral_after_cancel(self):
    self.start(0x815)
    from opendbc.safety.tests.libsafety import libsafety_py
    for payload in (
      '90d0002000000000',
      'cae0022000000000',
      'da00022000000000',
      '6310022000000000',
      'b520022000000000',
      '0c30022000000000',
      '0440022000000000',
      'bd50022000000000',
      '6b60022000000000',
      '5e70002000000000',
      'f780002000000000',
    ):
      self.now += 20_000
      self.safety.set_timer(self.now)
      self.assertTrue(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x1CF, 1, bytes.fromhex(payload))))
      self.healthy(omit='CRUISE_BUTTONS')
    self.claim_and_withdraw(9, False)
