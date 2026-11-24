"""Native-only independent cruise gesture regression; run in root DEBUG/RELEASE."""
import unittest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai.canfd_stock_aol import STOCK_AOL_WORDS
from opendbc.safety.tests.libsafety import libsafety_py


class TestHyundaiIndependentCruiseButtons(unittest.TestCase):
  def reset(self, word):
    self.word = word
    self.safety = libsafety_py.libsafety
    self.safety.set_alternative_experience(0)
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, word), 0)
    self.safety.init_tests()
    self.packer = CANPacker('hyundai_canfd_generated')
    self.counters = {}
    self.now = 1_000_000
    self.safety.set_timer(self.now)

  def packet(self, name, bus, values):
    frame = self.packer.make_can_msg(name, bus, values)
    return libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])

  def feed(self, *, main=False, lkas=False, cancel=False, acc=False):
    self.safety.set_timer(self.now)
    pt = 1 if self.word & 0x10 else 0
    fuel = {0: 'ACCELERATOR_BRAKE_ALT', 1: 'ACCELERATOR', 2: 'ACCELERATOR_ALT'}[self.word & 3]
    buttons = 'CRUISE_BUTTONS_ALT' if self.word & 0x20 else 'CRUISE_BUTTONS'
    for name in (fuel, 'TCS', 'WHEEL_SPEEDS', 'MDPS', buttons, 'SCC_CONTROL'):
      self.counters[name] = self.counters.get(name, 0) + 1
      maximum = 16 if name == 'CRUISE_BUTTONS' else 256
      values = {'COUNTER': self.counters[name] % maximum}
      if name == 'MDPS':
        values['MDPS_StrTqSnsrVal'] = 0
      elif name == buttons:
        values.update(ADAPTIVE_CRUISE_MAIN_BTN=int(main), LDA_BTN=int(lkas), CRUISE_BUTTONS=4 if cancel else 0)
      elif name == 'SCC_CONTROL':
        values.update(MainMode_ACC=1, ACCMode=int(acc))
      bus = 2 if name == 'SCC_CONTROL' and self.word & 8 else pt
      self.assertTrue(self.safety.safety_rx_hook(self.packet(name, bus, values)), name)
    self.now += 10_000

  def warmup(self):
    for _ in range(6):
      self.feed()

  def claim(self):
    self.feed(lkas=True)
    self.feed()
    self.safety.set_aol_test_heartbeat(True)
    self.safety.aol_set_host_request(1)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_cancel_and_main_keep_only_previously_claimed_lateral(self):
    # This is existing admitted inventory, not a new safety word allocation.
    for word in sorted(STOCK_AOL_WORDS):
      with self.subTest(word=hex(word)):
        self.reset(word)
        self.warmup()
        self.claim()
        self.feed(cancel=True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)
        self.feed()
        self.feed(main=True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)
        self.feed()
        self.safety.aol_set_host_request(0)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        pt = 1 if word & 0x10 else 0
        self.assertFalse(self.safety.safety_tx_hook(self.packet('SCC_CONTROL', pt, {'ACCMode': 1})))

  def test_cancel_without_claimed_authority_cannot_arm_or_keep_a_pending_gesture(self):
    for word in (0x800, 0x811, 0x891, 0x821, 0x829):
      for pending in (False, True):
        with self.subTest(word=hex(word), pending=pending):
          self.reset(word)
          self.warmup()
          if pending:
            self.feed(lkas=True)
            self.feed()
          self.feed(cancel=True)
          self.safety.set_aol_test_heartbeat(True)
          self.safety.aol_set_host_request(1)
          self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_main_does_not_renew_an_unclaimed_gesture_deadline(self):
    for word in (0x811, 0x891, 0x821, 0x829):
      with self.subTest(word=hex(word)):
        self.reset(word)
        self.warmup()
        original = self.now
        self.feed(lkas=True)
        self.feed()
        # Continue healthy source packets so RX expiry cannot explain refusal.
        while self.now <= original + 200_000:
          self.feed()
        self.feed(main=True)
        self.feed()
        while self.now <= original + 300_000:
          self.feed()
        self.safety.set_timer(self.now)
        self.safety.set_aol_test_heartbeat(True)
        self.safety.aol_set_host_request(1)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_cancel_and_main_cannot_recover_faults_or_host_loss(self):
    for fault in ('heartbeat', 'relay', 'rx', 'host_expiry'):
      with self.subTest(fault=fault):
        self.reset(0x811)
        self.warmup()
        self.claim()
        if fault == 'heartbeat':
          self.safety.set_aol_test_heartbeat(False)
        elif fault == 'relay':
          self.safety.set_relay_malfunction(True)
        elif fault == 'rx':
          self.now += 2_000_000
          self.safety.set_timer(self.now)
        else:
          # Keep sources healthy but do not renew the original host request.
          for _ in range(32):
            self.feed()
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.feed(cancel=True)
        self.feed(main=True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
