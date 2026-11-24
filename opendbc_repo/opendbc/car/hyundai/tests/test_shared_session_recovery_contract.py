import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.hyundai.ecu_startup import HyundaiECUStartup, Outcome
from opendbc.car.hyundai.ev9_longitudinal import candidate
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR


class TestSharedSessionRecoveryContract(unittest.TestCase):
  def owner(self, *, default_ack=True, default_payload=b''):
    stock = params(candidate=CAR.KIA_EV9)
    cp = candidate(stock, enabled=True, is_release=False)
    self.assertIsNotNone(cp)
    sent, requests = [], []
    owner = HyundaiECUStartup(cp, (lambda **kwargs: [], sent.extend), address=0x730, bus=1, label='session-contract')

    def query(send, recv, bus, addresses, req, response):
      request = req[0]
      requests.append(request)
      self.assertEqual((bus, addresses), (1, [(0x730, None)]))

      def data(timeout):
        send([(0x730, bytes((len(request),)) + request + bytes(7-len(request)), 1)])
        if request == b'\x10\x01' and not default_ack:
          return {}
        return {(0x730, None): default_payload if request == b'\x10\x01' else b''}

      return SimpleNamespace(get_data=data)

    owner._query = query
    return owner, sent, requests

  @patch.object(time, 'CLOCK_BOOTTIME', getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC), create=True)
  def test_silent_session_only_attempt_requires_explicit_default_session_before_stock(self):
    owner, sent, requests = self.owner()

    def session_only(recv, send, **kwargs):
      send([(0x730, b'\x02\x10\x03'+bytes(5), 1)])
      return False

    owner._disable = session_only
    owner.prepare(admission=lambda: True)
    self.assertFalse(owner.disable_attempted)
    self.assertTrue(owner.restored)
    self.assertEqual(owner.outcome, Outcome.STOCK_RESTORED)
    self.assertIn(b'\x10\x01', requests)
    self.assertNotIn(b'\x28\x00\x01', requests)
    self.assertTrue(sent)

  @patch.object(time, 'CLOCK_BOOTTIME', getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC), create=True)
  def test_failed_default_session_after_session_only_attempt_cannot_publish_stock(self):
    owner, _, _ = self.owner(default_ack=False)

    def session_only(recv, send, **kwargs):
      send([(0x730, b'\x02\x10\x03'+bytes(5), 1)])
      return False

    owner._disable = session_only
    with self.assertRaises(RuntimeError):
      owner.prepare(admission=lambda: True)
    self.assertEqual(owner.outcome, Outcome.ABORT_UNCERTAIN)
    self.assertFalse(owner.restored)

  @patch.object(time, 'CLOCK_BOOTTIME', getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC), create=True)
  def test_communication_restore_ack_without_default_session_ack_is_unverified(self):
    owner, _, requests = self.owner(default_ack=False)
    owner.admission = lambda: True
    owner._send([(0x730, b'\x03\x28\x83\x01'+bytes(4), 1)])
    self.assertTrue(owner.disable_attempted)
    self.assertFalse(owner._restore())
    self.assertIn(b'\x10\x01', requests)
    self.assertFalse(owner.restored)

  @patch.object(time, 'CLOCK_BOOTTIME', getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC), create=True)
  def test_malformed_default_session_payload_cannot_confirm_recovery(self):
    for payload in (b'\x00', bytes(2), bytes(3), bytes(5)):
      with self.subTest(payload=payload):
        owner, _, _ = self.owner(default_payload=payload)
        owner.admission = lambda: True
        owner._send([(0x730, b'\x03\x28\x83\x01'+bytes(4), 1)])
        self.assertFalse(owner._restore())
        self.assertFalse(owner.restored)
    for payload in (b'', bytes(4)):
      with self.subTest(valid_payload=payload):
        owner, _, _ = self.owner(default_payload=payload)
        owner.admission = lambda: True
        owner._send([(0x730, b'\x03\x28\x83\x01'+bytes(4), 1)])
        self.assertTrue(owner._restore())
        self.assertTrue(owner.restored)

  @patch.object(time, 'CLOCK_BOOTTIME', getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC), create=True)
  def test_session_transport_exception_requires_recovery_and_failed_close_stays_failed(self):
    for ack in (True, False):
      with self.subTest(default_ack=ack):
        owner, _, requests = self.owner(default_ack=ack)

        def fail_send(frames):
          if any(data[:3] == b'\x02\x10\x03' for _, data, _ in frames):
            raise OSError('session send outcome unknown')
        owner.callbacks = (lambda **kwargs: [], fail_send)

        def attempt(recv, send, **kwargs):
          send([(0x730, b'\x02\x10\x03' + bytes(5), 1)])
        owner._disable = attempt
        if ack:
          owner.prepare(admission=lambda: True)
          self.assertIs(owner.outcome, Outcome.STOCK_RESTORED)
        else:
          with self.assertRaises(RuntimeError):
            owner.prepare(admission=lambda: True)
          for _ in range(2):
            with self.assertRaises(RuntimeError):
              owner.close()
          self.assertIs(owner.outcome, Outcome.ABORT_UNCERTAIN)
        self.assertTrue(owner.session_attempted)
        self.assertFalse(owner.disable_attempted)
        self.assertIn(b'\x10\x01', requests)
        self.assertNotIn(b'\x28\x00\x01', requests)

  def test_foreign_endpoint_and_other_session_requests_do_not_create_attempts(self):
    owner, sent, _ = self.owner()
    for address, data, bus in ((0x731, b'\x02\x10\x03' + bytes(5), 1),
                               (0x730, b'\x02\x10\x03' + bytes(5), 0),
                               (0x730, b'\x02\x10\x01' + bytes(5), 1)):
      owner._send([(address, data, bus)])
      self.assertFalse(owner.session_attempted)
      self.assertFalse(owner.disable_attempted)
    self.assertEqual(len(sent), 3)
