from typing import Any
from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_ev9_pending_startup import pending_startup_fixture
from opendbc.car.hyundai.values import CAR
from openpilot.starpilot.tests.test_ev6_startup import params as captured_params


CASES = ['g90', 'legacy', 'ev6', 'gv70', 'ev9', 'ioniq5', 'kona2']


def make_owner(kind, send):
  if kind in ('g90', 'legacy'):
    identity = CAR.GENESIS_G90 if kind == 'g90' else CAR.GENESIS_G80
    cp = CarInterface.get_params(identity, gen_empty_fingerprint(), [], True, False, False)
  elif kind in ('ev6', 'gv70'):
    cp = captured_params(car=CAR.KIA_EV6 if kind == 'ev6' else CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, radar=True)
  else:
    fp = gen_empty_fingerprint()
    fp[2].update({0x50: 16, 0x2A4: 24})
    fp[1].update({0x1CF: 8, 0x1AA: 16, 0x35: 32, 0x175: 24, 0xA0: 24,
                  0xEA: 24, 0x1BA: 24, 0x1E5: 16, 0x36A: 16})
    cp = CarInterface.get_params(CAR.HYUNDAI_IONIQ_5 if kind == 'ioniq5' else CAR.HYUNDAI_KONA_EV_2ND_GEN,
                                 fp, [], False, False, False)
  owner: Any = CarInterface.startup_owner(cp, (list, send), requested=True)
  assert owner is not None
  if kind in ('ev6', 'gv70'):
    owner._capture = lambda: object()
  return owner


@pytest.mark.parametrize('kind', CASES)
@pytest.mark.parametrize('reject_restore_session', [False, True])
def test_explicit_disable_rejection_waits_for_restoration_session_before_enable(kind, reject_restore_session):
  if kind == 'ev9':
    check_pending_ev9_restoration_settle(reject_restore_session)
    return
  sent, requests, sleeps = [], [], []
  now = [0.]
  session_responses = []
  owner = make_owner(kind, sent.extend)

  def query(send, recv, bus, addresses, commands, responses):
    command = commands[0]
    requests.append((command, now[0]))
    assert (bus, addresses) == (owner.bus, [(owner.address, None)])

    class Query:
      def get_data(self, timeout):
        send([(owner.address, bytes([len(command)]) + command + bytes(7 - len(command)), owner.bus)])
        if command == b"\x10\x03":
          session_responses.append(now[0])
          if reject_restore_session and len(session_responses) > 1:
            return {}
          return {(owner.address, None): b''}
        if command in (b"\x28\x83\x01", b"\x28\x01\x01"):
          return {(owner.address, None): b"\x7f\x28\x22"}
        if command == b"\x28\x00\x01":
          return {(owner.address, None): b''} if now[0] - session_responses[-1] >= .049999 else {}
        return {(owner.address, None): b''}
    return Query()

  def sleep(seconds):
    sleeps.append(seconds)
    now[0] += seconds

  with patch.object(owner, '_query', side_effect=query), patch('time.sleep', side_effect=sleep):
    if reject_restore_session:
      with pytest.raises(RuntimeError, match='restoration unverified'):
        owner.prepare(admission=lambda: True)
      assert owner.outcome is Outcome.ABORT_UNCERTAIN and not owner.published
      assert all(command != b"\x28\x00\x01" for command, _ in requests)
      assert sleeps == [.05]
      with pytest.raises(RuntimeError, match='cleanup could not confirm'):
        owner.close()
      assert not owner.restored and not owner.published
      return
    try:
      result = owner.prepare(admission=lambda: True)
    except RuntimeError as exc:
      pytest.fail('RESTORE_SESSION_SETTLE_MISSING ' + str((kind, requests, sleeps, exc)))
    try:
      assert owner.outcome is Outcome.STOCK_RESTORED and owner.restored
      assert not result.openpilotLongitudinalControl and result.pcmCruise and not owner.published
      assert requests[-3:][0][0] == b"\x10\x03"
      assert requests[-2][0] == b"\x28\x00\x01" and requests[-1][0] == b"\x10\x01"
      assert sleeps == [.05, .05]
      assert requests[-2][1] - session_responses[-1] >= .049999
    finally:
      owner.close()


def check_pending_ev9_restoration_settle(reject_restore_session):
  with pending_startup_fixture() as (fixture, owner, ci, packer):
    requests, sleeps, sessions = [], [], []
    before = ci.CP.to_dict()
    fixture.warm(owner, ci, packer)

    def query(send, recv, bus, addresses, commands, responses):
      command = commands[0]
      requests.append((command, fixture.seconds))
      assert (bus, addresses, responses) == (1, [0x730], [b""])

      class Query:
        def get_data(self, timeout, *, total_timeout):
          assert timeout == total_timeout == .1
          send([(0x730, bytes([len(command)]) + command + bytes(7 - len(command)), 1)])
          if command == b"\x10\x03":
            sessions.append(fixture.seconds)
            if reject_restore_session and len(sessions) > 1:
              return {(0x730, None): b"\x7f\x10\x22"}
            return {(0x730, None): b"\x50\x03"}
          if command == b"\x28\x01\x01":
            return {(0x730, None): b"\x7f\x28\x22"}
          if command == b"\x28\x00\x01":
            return {(0x730, None): b"\x68\x00"} if fixture.seconds - sessions[-1] >= .049999 else {}
          assert command == b"\x10\x01"
          return {(0x730, None): b"\x50\x01"}
      return Query()

    def sleep(seconds):
      sleeps.append(seconds)
      fixture.seconds += seconds

    owner.exchange = owner._ev9_exchange
    with patch('opendbc.car.hyundai.ev9_pending_startup.IsoTpParallelQuery', side_effect=query), patch('time.sleep', side_effect=sleep):
      with pytest.raises(RuntimeError, match='communication control'):
        fixture.commit(owner, ci)
      assert owner.owner.published and not owner.can_emit(ci.CC)
      assert owner.thread is None and not owner.handed_off
      assert ci.CP.to_dict() == before  # Published target is never relabeled as stock.
      if reject_restore_session:
        assert not owner.restored
        assert all(command != b"\x28\x00\x01" for command, _ in requests)
        assert sleeps == [.05] + [.1] * 10
        with pytest.raises(RuntimeError, match='restoration remains unverified'):
          owner.close()
        assert not owner.restored and not owner.can_emit(ci.CC)
      else:
        assert owner.restored
        assert [command for command, _ in requests[-3:]] == [b"\x10\x03", b"\x28\x00\x01", b"\x10\x01"]
        assert sleeps == [.05, .05]
        assert requests[-2][1] - sessions[-1] >= .049999
        owner.close()
