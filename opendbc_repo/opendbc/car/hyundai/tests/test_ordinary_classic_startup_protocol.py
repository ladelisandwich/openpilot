from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint, make_tester_present_msg
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_g90_continuity import ManualKeeper
from opendbc.car.hyundai.values import CAR


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_SONATA, CAR.HYUNDAI_SONATA_HYBRID])
@pytest.mark.parametrize('mode', ['silence', 'positive', 'negative', 'malformed', 'send_exception', 'denied', 'restore_failure'])
def test_actual_factory_protocol_preserves_stock_or_aborts_before_publication(identity, mode):
  fp = gen_empty_fingerprint()
  fp[0][0x391] = 8
  desired = CarInterface.get_params(identity, fp, [], True, False, False)
  stock = CarInterface.get_params(identity, fp, [], False, False, False)
  sent, requests, waits = [], [], []
  now = [5.0]
  failed_send = [False]

  def send(frames):
    sent.extend(frames)
    if mode == 'send_exception' and not failed_send[0] and any(frame[1][:4] == b"\x03\x28\x83\x01" for frame in frames):
      failed_send[0] = True
      raise OSError('transport could have reached ECU')

  owner = CarInterface.startup_owner(desired, (list, send), requested=True)
  assert owner is not None and not sent
  owner.clock = lambda: now[0]
  owner.keeper_factory = ManualKeeper

  def query(can_send, can_recv, bus, addresses, commands, responses):
    command = commands[0]
    requests.append(command)
    assert bus == 0 and addresses == [(0x7D0, None)]

    class Query:
      def get_data(self, timeout):
        waits.append((command.hex(), timeout))
        can_send([(0x7D0, bytes([len(command)]) + command + bytes(7 - len(command)), 0)])
        if command == b"\x28\x83\x01":
          if mode == 'silence':
            return {}
          if mode in ('negative', 'restore_failure'):
            return {(0x7D0, None): b"\x7f\x28\x22"}
          if mode == 'malformed':
            return {(0x7D0, None): b"\x68\x03\x00"}
          return {(0x7D0, None): b"\x68\x03"}
        if mode == 'restore_failure' and command == b"\x28\x00\x01":
          return {}
        return {(0x7D0, None): b''}

    return Query()

  with patch.object(owner, '_query', side_effect=query), patch('time.sleep') as sleep:
    if mode == 'restore_failure':
      with pytest.raises(RuntimeError, match='restoration unverified'):
        owner.prepare(admission=lambda: True)
      assert owner.outcome is Outcome.ABORT_UNCERTAIN and not owner.published
      with pytest.raises(RuntimeError, match='cleanup could not confirm'):
        owner.close()
      assert owner.closed and not owner.restored
      return
    try:
      result = owner.prepare(admission=lambda: mode != 'denied')
      assert not owner.published
      if mode in ('silence', 'positive'):
        assert owner.outcome is Outcome.SENT_UNCONFIRMED
        assert result.to_dict() == desired.to_dict()
        assert owner.keeper is not None
        tester = make_tester_present_msg(0x7D0, 0, suppress_response=True)
        assert tester in sent
        for second in range(1, 8):
          now[0] = 5.0 + second
          assert owner.keeper.tick()
        assert not owner.handed_off
        assert not owner.before_control(configured=False, sources_current=True, control_current=True)
        assert not owner.before_control(configured=True, sources_current=False, control_current=True)
        assert not owner.before_control(configured=True, sources_current=True, control_current=False)
      else:
        assert result.to_dict() == stock.to_dict()
        assert owner.keeper is None
        if mode == 'denied':
          assert owner.outcome is Outcome.STOCK_UNTOUCHED and not requests and not sent
        else:
          assert owner.outcome is Outcome.STOCK_RESTORED and owner.restored
          assert requests[-3:] == [b"\x10\x03", b"\x28\x00\x01", b"\x10\x01"]
      if mode != 'denied':
        assert (b"\x28\x83\x01".hex(), 0.1) in waits
        assert any(call.args == (0.05,) for call in sleep.call_args_list)
    finally:
      owner.close()
  assert not owner.published and owner.closed
