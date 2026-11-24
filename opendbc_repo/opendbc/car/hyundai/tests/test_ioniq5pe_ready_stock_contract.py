from copy import deepcopy
from unittest.mock import patch

import pytest

from opendbc.car import CanData
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.ioniq5pe_startup import Phase
from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
from opendbc.car.hyundai.tests.ioniq5pe_route18_ready_capture import PACKETS, READY_BEFORE_FINGERPRINT_NS
from opendbc.car.hyundai.tests.test_ioniq5pe_2027 import params


@pytest.mark.parametrize('requested', [False, True])
def test_recorded_ready_parked_start_keeps_exact_stock_without_inventing_ecu_acceptance(requested):
  stock = params()
  before = deepcopy(stock.to_dict())
  floor = 1_000_000_000
  now, cursor = [floor], [0]
  sent = []
  assert READY_BEFORE_FINGERPRINT_NS == 3_163_390_599

  def receive(**kwargs):
    if cursor[0] == len(PACKETS):
      now[0] += 10_000_000
      return []
    offset, frames = PACKETS[cursor[0]]
    cursor[0] += 1
    now[0] = floor + offset
    return [TimestampedCanPacket([CanData(address, bytes.fromhex(raw), bus) for address, bus, raw in frames], now[0])]

  with (patch('time.clock_gettime_ns', side_effect=lambda clock: now[0]),
        patch('time.monotonic', side_effect=lambda: now[0] / 1e9)):
    owner = CarInterface.startup_owner(stock, (receive, sent.extend), requested=requested)
    if not requested:
      assert owner is None and not cursor[0]
    else:
      # Diagnostic admission is a controlled peer; no accepted ECU response is supplied.
      prepared = owner.prepare(admission=lambda: True)
      try:
        ci = CarInterface(prepared)
        owner.configure(ci)
        owner.finalize_aol_configuration(ci)
        owner.seal_publication()
        assert cursor[0] >= 2
        assert prepared.to_dict() == before
        assert owner.owner.phase is Phase.STOCK and not owner.pending_target
        assert not owner.session_attempted and not owner.takeover_attempted and owner.thread is None
      finally:
        owner.close()
    assert not sent and stock.to_dict() == before
