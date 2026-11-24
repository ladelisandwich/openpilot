from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint, make_tester_present_msg
from opendbc.car.hyundai import g90_startup
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.ev9_keeper import EV9Keeper
from opendbc.car.hyundai.g90_startup import G90Startup
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR


class ManualKeeper(EV9Keeper):
  def start(self):
    self.tick()


def prepared(*, aol=False, send_error=False, keeper_factory=ManualKeeper, holders=None):
  now = [8.637416955]
  sent = []
  cp = CarInterface.get_params(CAR.GENESIS_G90, gen_empty_fingerprint(), [], alpha_long=True, is_release=False, docs=False)
  assert cp.openpilotLongitudinalControl
  if aol:
    cp.safetyConfigs[0].safetyParam |= 0x400
    cp.alternativeExperience = 32

  def send(frames):
    if send_error and make_tester_present_msg(0x7d0, 0, suppress_response=True) in frames:
      raise OSError('Actual sender rejected tester')
    sent.extend((now[0], frame) for frame in frames)

  def query(can_send, can_recv, bus, addresses, requests, responses):
    request = requests[0]

    class Query:
      def get_data(self, timeout):
        can_send([(0x7d0, bytes([len(request)]) + request + bytes(7 - len(request)), 0)])
        return {} if request == b"\x28\x83\x01" else {(0x7d0, None): b''}
    return Query()

  owner = G90Startup(cp, (list, send))
  if holders is not None:
    holders.append(owner)
  owner.clock = lambda: now[0]
  owner.keeper_factory = keeper_factory
  with patch.object(g90_startup, 'IsoTpParallelQuery', side_effect=query), patch.object(g90_startup.time, 'sleep'):
    owner.prepare(admission=lambda: True)
  return owner, now, sent


@pytest.mark.parametrize('aol', [False, True])
def test_recorded_disable_to_first_controller_interval_has_continuous_tester(aol):
  owner, now, sent = prepared(aol=aol)
  tester = make_tester_present_msg(0x7d0, 0, suppress_response=True)
  try:
    assert any(frame == tester for _, frame in sent), 'No tester starts after successful ECU disable'
    for second in range(1, 7):
      now[0] = 8.637416955 + second
      owner.keeper.tick()
    now[0] = 15.046147807
    times = [stamp for stamp, frame in sent if frame == tester]
    assert len(times) == 7
    assert max(right - left for left, right in zip(times, times[1:], strict=False)) <= 1.000001
    assert now[0] - times[-1] < 1.0
    assert owner.outcome is Outcome.SENT_UNCONFIRMED
    assert all(frame == tester or frame[1][:3] == b"\x02\x10\x03" or frame[1][:4] == b"\x03\x28\x83\x01" for _, frame in sent)
  finally:
    owner.published = True
    owner.close()


@pytest.mark.parametrize('kind', ['invalid', 'empty', 'wrong_bus', 'wrong_payload', 'unpublished', 'valid'])
def test_only_actual_valid_controller_tester_receipt_ends_continuity(kind):
  owner, now, sent = prepared()
  try:
    tester = make_tester_present_msg(0x7d0, 0, suppress_response=True)
    owner.published = kind != 'unpublished'
    frames = [tester]
    if kind == 'empty':
      frames = []
    elif kind == 'wrong_bus':
      frames = [(tester[0], tester[1], 1)]
    elif kind == 'wrong_payload':
      frames = [(tester[0], b"\x02\x3e\x00" + bytes(5), tester[2])]
    owner.sent(frames, valid=kind != 'invalid')
    assert owner.handed_off == (kind == 'valid')
    assert owner.keeper.event.is_set() == (kind == 'valid')
    count = len(sent)
    now[0] += 1.0
    owner.keeper.tick()
    assert len(sent) == count + (kind != 'valid')
  finally:
    owner.published = True
    owner.close()


@pytest.mark.parametrize('fault', ['deadline', 'clock_reversal', 'sender'])
def test_failed_continuity_cannot_be_acknowledged_or_rearmed(fault):
  owner, now, sent = prepared(send_error=fault == 'sender')
  owner.published = True
  try:
    if fault == 'deadline':
      now[0] += 60.0
    elif fault == 'clock_reversal':
      now[0] -= 1.0
    owner.keeper.tick()
    with pytest.raises(RuntimeError, match='diagnostic continuity failed'):
      owner.maintain(configured=True)
    assert owner.outcome is Outcome.ABORT_UNCERTAIN
    assert not owner.handed_off
    count = len(sent)
    now[0] += 1.0
    assert not owner.keeper.tick()
    assert len(sent) == count
  finally:
    owner.close()


def test_close_stops_tester_before_unpublished_stock_restoration():
  owner, now, sent = prepared()
  requests = []

  def query(*args, **kwargs):
    assert owner.keeper.event.is_set()
    request = args[4][0]
    requests.append(request)

    class Query:
      def get_data(self, timeout):
        return {(0x7d0, None): b''}
    return Query()

  with patch.object(g90_startup, 'IsoTpParallelQuery', side_effect=query):
    owner.close()
  assert requests == [b"\x10\x03", b"\x28\x00\x01", b"\x10\x01"]
  assert owner.closed and owner.restored
  count = len(sent)
  now[0] += 1.0
  assert not owner.keeper.tick()
  assert len(sent) == count


def test_actual_thread_start_failure_restores_stock_and_preserves_original_error():
  holders = []
  with patch('threading.Thread.start', side_effect=OSError('thread launch failed')):
    with pytest.raises(OSError, match='thread launch failed'):
      prepared(keeper_factory=EV9Keeper, holders=holders)
  owner = holders[0]
  assert owner.closed and owner.restored
  assert not owner.published
  assert owner.keeper.event.is_set()
  assert owner.keeper.thread is None
