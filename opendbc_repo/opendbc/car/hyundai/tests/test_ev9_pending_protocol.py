"""EV9 source UDS distinctions through the real ISO-TP decoder."""
from unittest.mock import patch

import pytest

from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.ev9_longitudinal import candidate
from opendbc.car.hyundai.ev9_pending_startup import EV9PendingStartup
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR


@pytest.mark.parametrize('reply,accepted,outcome', [
  (b'\x68\x01', True, 'positive_communication_control'),
  (None, True, 'no_reply_sent_unconfirmed'),
  (b'\x7f\x28\x22', False, 'negative_or_malformed_reply'),
  (b'\x7f\x28\x78', False, 'negative_or_malformed_reply'),
  (b'\x68\x03', False, 'negative_or_malformed_reply'),
  (b'\x68\x01\x00', False, 'negative_or_malformed_reply'),
])
def test_real_decoder_does_not_turn_silence_or_nrc_into_positive_ack(reply, accepted, outcome):
  stock = params(candidate=CAR.KIA_EV9)
  cp = candidate(stock, enabled=True, is_release=False)
  sent, pending = [], []

  def receive(**kwargs):
    frames = pending[:]
    pending.clear()
    return [frames] if frames else []

  def send(frames):
    sent.extend(frames)
    if reply is not None:
      pending.append(CanData(0x738, bytes((len(reply),)) + reply + bytes(7 - len(reply)), 1))

  owner = EV9PendingStartup(cp, (receive, send), stock_cp=stock)
  assert owner._ev9_exchange(receive, send, b'\x28\x01\x01', b'\x68\x01') is accepted
  assert owner.disable_response == outcome
  assert sent == [CanData(0x730, b'\x03\x28\x01\x01\x00\x00\x00\x00', 1)]
  assert not owner.owner.active()
  assert not owner.handed_off


def test_session_success_retains_source_50ms_settle():
  stock = params(candidate=CAR.KIA_EV9)
  owner = EV9PendingStartup(candidate(stock, enabled=True, is_release=False), (lambda **kw: [], lambda frames: None), stock_cp=stock)
  with patch('opendbc.car.hyundai.ev9_pending_startup.IsoTpParallelQuery') as query, \
       patch('opendbc.car.hyundai.ev9_pending_startup.time.sleep') as sleep:
    query.return_value.get_data.return_value = {(0x730, None): b'\x50\x03\x00\x32\x01\xf4'}
    assert owner._ev9_exchange(lambda **kw: [], lambda frames: None, b'\x10\x03', b'\x50\x03')
    query.return_value.get_data.assert_called_once_with(.1, total_timeout=.1)
    sleep.assert_called_once_with(.05)


def test_missing_session_retains_ten_bounded_source_attempts_without_disable():
  stock = params(candidate=CAR.KIA_EV9)
  owner = EV9PendingStartup(candidate(stock, enabled=True, is_release=False), (lambda **kw: [], lambda frames: None), stock_cp=stock)
  with patch('opendbc.car.hyundai.ev9_pending_startup.IsoTpParallelQuery') as query, \
       patch('opendbc.car.hyundai.ev9_pending_startup.time.sleep') as sleep:
    query.return_value.get_data.return_value = {}
    assert not owner._ev9_exchange(lambda **kw: [], lambda frames: None, b'\x10\x03', b'\x50\x03')
    assert query.call_count == sleep.call_count == 10
    assert all(call.args[4] == [b'\x10\x03'] for call in query.call_args_list)
    assert owner.disable_response is None
