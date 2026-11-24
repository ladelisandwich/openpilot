import unittest
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import CanData
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.ev9_startup import EV9Startup
from opendbc.car.hyundai.ev9_longitudinal import candidate
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR


class TestEV9ReadyEvidence(unittest.TestCase):
  def owner(self, defect=None, *, ready=False):
    stock = params(candidate=CAR.KIA_EV9)
    ticks = [0]
    seconds = [1.]
    boot = [1_000_000_000]
    packer = CANPacker('hyundai_canfd_generated')

    def recv(*, wait_for_one=False):
      self.assertFalse(wait_for_one)
      ticks[0] += 1
      seconds[0] += .01
      boot[0] += 10_000_000
      if defect == 'empty':
        return []
      counter = 1 if defect == 'counter' else ticks[0] % 256
      address, raw, bus = packer.make_can_msg('ACCELERATOR', 1, {'COUNTER': counter})
      data = bytearray(raw)
      data[3] |= int(ready) << 6
      data[:2] = hkg_can_fd_checksum(address, None, data).to_bytes(2, 'little')
      if defect == 'crc':
        data[-1] ^= 1
      if defect == 'length':
        data = data[:4]
      if defect == 'bus':
        bus = 0
      stamp = 0 if defect == 'zero_stamp' else boot[0] + 1 if defect == 'future' else 1 if defect == 'stale' else boot[0]
      return [TimestampedCanPacket([CanData(address, bytes(data), bus)], stamp)]

    owner = EV9Startup(candidate(stock, enabled=True, is_release=False), (recv, lambda frames: None),
                       stock_cp=stock, clock=lambda: seconds[0])
    owner.boot_clock = lambda: boot[0]
    return owner, stock

  def test_only_current_crc_counter_and_timestamp_qualified_false_allows_not_ready(self):
    for defect in ('empty', 'counter', 'crc', 'length', 'bus', 'zero_stamp', 'future', 'stale'):
      with self.subTest(defect=defect):
        owner, _ = self.owner(defect)
        with patch('opendbc.car.hyundai.ev9_startup.time.sleep'):
          self.assertIsNone(owner._ready())
    for ready in (False, True):
      owner, _ = self.owner(ready=ready)
      with patch('opendbc.car.hyundai.ev9_startup.time.sleep'):
        self.assertIs(owner._ready(), ready)

  def test_unknown_ready_never_attempts_session_or_disable_and_retains_exact_stock(self):
    owner, stock = self.owner('empty')
    with patch.object(owner, '_ready', return_value=None), patch.object(owner, '_query') as query:
      cp = owner.prepare(admission=lambda: True)
    query.assert_not_called()
    self.assertIs(owner.outcome, Outcome.STOCK_UNTOUCHED)
    self.assertFalse(owner.session_attempted)
    self.assertFalse(owner.disable_attempted)
    self.assertEqual(cp.to_dict(), stock.to_dict())
