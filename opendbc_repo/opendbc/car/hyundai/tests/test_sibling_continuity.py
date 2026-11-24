from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint, make_tester_present_msg, structs
from opendbc.car.hyundai import ecu_startup, g90_startup
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.ev9_keeper import EV9Keeper
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_g90_continuity import ManualKeeper
from opendbc.car.hyundai.values import CAR
from openpilot.starpilot.tests.test_ev6_startup import params


CASES = [(CAR.GENESIS_G80, 1), (CAR.KIA_XCEED_PHEV, 1),
         (CAR.KIA_EV6, 1), (CAR.KIA_EV6, 2),
         (CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, 1), (CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, 2)]


def make_owner(identity, pandas, *, send_error=False, mode='sent', capture=True, real_keeper=False):
  now, sent, requests = [10.0], [], []
  if identity in (CAR.GENESIS_G80, CAR.KIA_XCEED_PHEV):
    cp = CarInterface.get_params(identity, gen_empty_fingerprint(), [], True, False, False)
  else:
    cp = params(car=identity, radar=True)
    if pandas == 2:
      owner_config = cp.safetyConfigs[-1].to_dict()
      cp.safetyConfigs = [structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.noOutput), owner_config]

  def send(frames):
    if send_error and any(frame[1][:3] == b"\x02\x3e\x80" for frame in frames):
      raise OSError('tester failed')
    sent.extend((now[0], frame) for frame in frames)
  owner = CarInterface.startup_owner(cp, (list, send), requested=True)
  assert owner is not None
  owner.clock = lambda: now[0]
  owner.keeper_factory = EV9Keeper if real_keeper else ManualKeeper
  if hasattr(owner, '_capture'):
    owner._capture = lambda: object() if capture else None

  def query(can_send, can_recv, bus, addresses, commands, responses):
    request = commands[0]
    assert (bus, addresses) == (owner.bus, [(owner.address, None)])
    requests.append(request)

    class Query:
      def get_data(self, timeout):
        can_send([(owner.address, bytes([len(request)]) + request + bytes(7 - len(request)), owner.bus)])
        if request == b"\x28\x83\x01":
          return {} if mode == 'sent' else {(owner.address, None): b"\x7f\x28\x22"}
        return {(owner.address, None): b''}
    return Query()
  return owner, now, sent, requests, query


@pytest.mark.parametrize('identity,pandas', CASES)
def test_factory_startup_continuity_until_real_controller_receipt(identity, pandas):
  owner, now, sent, requests, query = make_owner(identity, pandas)
  tester = make_tester_present_msg(owner.address, owner.bus, suppress_response=True)
  with (patch.object(ecu_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(g90_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(ecu_startup.time, 'sleep')):
    try:
      owner.prepare(admission=lambda: True)
      assert owner.outcome is Outcome.SENT_UNCONFIRMED
      assert any(frame == tester for _, frame in sent), 'SIBLING_DIAGNOSTIC_CONTINUITY_MISSING'
      for tick in range(1, 8):
        now[0] = 10.0 + tick
        owner.keeper.tick()
      assert [stamp for stamp, frame in sent if frame == tester] == list(range(10, 18))
      assert owner.bus == (0 if identity in (CAR.GENESIS_G80, CAR.KIA_XCEED_PHEV) else 4 * (pandas - 1) + 1)
      owner.published = True
      for frames, valid in (([], True), ([tester], False), ([(tester[0], tester[1], (tester[2] + 1) % 8)], True)):
        owner.sent(frames, valid=valid)
        assert not owner.handed_off and not owner.keeper.event.is_set()
      owner.sent([tester], valid=True)
      assert owner.handed_off and owner.keeper.event.is_set()
      count = len(sent)
      now[0] += 1
      assert not owner.keeper.tick() and len(sent) == count
    finally:
      owner.close()


@pytest.mark.parametrize('identity,pandas', CASES)
@pytest.mark.parametrize('fault', ['deadline', 'clock', 'sender'])
def test_sibling_continuity_fault_keeps_startup_unavailable(identity, pandas, fault):
  owner, now, sent, requests, query = make_owner(identity, pandas, send_error=fault == 'sender')
  with (patch.object(ecu_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(g90_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(ecu_startup.time, 'sleep')):
    try:
      owner.prepare(admission=lambda: True)
      assert owner.keeper is not None
      if fault == 'deadline':
        now[0] += 60
      elif fault == 'clock':
        now[0] -= 1
      owner.keeper.tick()
      with pytest.raises(RuntimeError, match='diagnostic continuity failed'):
        owner.maintain(configured=True)
      assert owner.outcome is Outcome.ABORT_UNCERTAIN
      assert not owner.handed_off
    finally:
      owner.close()


@pytest.mark.parametrize('identity,pandas', CASES)
@pytest.mark.parametrize('mode', ['restored', 'denied', 'thread_failure'])
def test_unpublished_sibling_restores_and_reaps_before_cleanup(identity, pandas, mode):
  owner, now, sent, requests, query = make_owner(identity, pandas, mode='restored' if mode == 'restored' else 'sent', real_keeper=mode == 'thread_failure')
  with (patch.object(ecu_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(g90_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(ecu_startup.time, 'sleep')):
    if mode == 'thread_failure':
      with patch('threading.Thread.start', side_effect=OSError('launch failed')):
        with pytest.raises(OSError, match='launch failed'):
          owner.prepare(admission=lambda: True)
      assert owner.closed and owner.restored
      assert owner.keeper.thread is None and owner.keeper.event.is_set()
    else:
      owner.prepare(admission=lambda: mode != 'denied')
      assert owner.outcome is (Outcome.STOCK_UNTOUCHED if mode == 'denied' else Outcome.STOCK_RESTORED)
      assert owner.keeper is None
      assert not owner.cp.openpilotLongitudinalControl
      if mode == 'denied':
        assert not requests and not sent
      else:
        assert requests[-3:] == [b"\x10\x03", b"\x28\x00\x01", b"\x10\x01"]
      owner.close()


def test_keeper_actual_thread_launch_failure_can_be_stopped_without_masking_original():
  keeper = EV9Keeper(lambda: None)
  with patch('threading.Thread.start', side_effect=OSError('launch failed')):
    with pytest.raises(OSError, match='launch failed'):
      keeper.start()
  assert keeper.thread is None and keeper.event.is_set()
  keeper.stop()


@pytest.mark.parametrize('identity,pandas', CASES)
@pytest.mark.parametrize('receipt', ['valid', 'invalid', 'exception'])
def test_actual_card_publisher_keeps_sibling_diagnostics_until_sent(identity, pandas, receipt):
  from types import SimpleNamespace
  from openpilot.selfdrive.car.card import Car
  from openpilot.starpilot.vehicle_startup import VehicleStartupOwner

  owner, now, sent, requests, query = make_owner(identity, pandas)
  with (patch.object(ecu_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(g90_startup, 'IsoTpParallelQuery', side_effect=query),
        patch.object(ecu_startup.time, 'sleep')):
    owner.prepare(admission=lambda: True)
    owner.published = True
    card = Car.__new__(Car)
    card.CI = CarInterface(owner.cp)
    card.CP = owner.cp
    card.vehicle_startup = VehicleStartupOwner(owner)
    delivered = []

    def send(service, packet):
      assert service == 'sendcan' and not owner.handed_off
      if receipt == 'exception':
        raise OSError('publisher failed')
      delivered.append(packet)
    try:
      with patch.object(card, 'pm', SimpleNamespace(send=send), create=True), patch.object(card, 'volt_startup_keepalive', return_value=False):
        tester = make_tester_present_msg(owner.address, owner.bus, suppress_response=True)
        if receipt == 'exception':
          with pytest.raises(OSError, match='publisher failed'):
            card.publish_sendcan([tester])
        else:
          card.publish_sendcan([tester], valid=receipt == 'valid')
        assert bool(delivered) == (receipt != 'exception')
        assert owner.handed_off == (receipt == 'valid')
        count = len(sent)
        now[0] += 1
        owner.keeper.tick()
        assert len(sent) == count + (receipt != 'valid')
    finally:
      owner.close()


@pytest.mark.parametrize('identity', [CAR.KIA_EV9, CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_KONA_EV_2ND_GEN])
def test_existing_keeper_owners_close_after_real_thread_launch_failure(identity):
  from openpilot.starpilot.vehicle_startup import VehicleStartupOwner

  if identity == CAR.KIA_EV9:
    import threading
    from opendbc.car.hyundai.tests.test_ev9_pending_startup import pending_startup_fixture

    with pending_startup_fixture() as (fixture, owner, ci, packer):
      fixture.warm(owner, ci, packer)
      published = ci.CP.to_dict()
      owner.thread_factory = threading.Thread
      with patch('threading.Thread.start', side_effect=OSError('launch failed')):
        with pytest.raises(OSError, match='launch failed'):
          try:
            fixture.commit(owner, ci)
          finally:
            owner.close()
      assert owner.closed and owner.restored and owner.close_completed
      assert owner.thread is None and owner.stop_event.is_set()
      assert owner.fault and not owner.owner.active() and not owner.can_emit(ci.CC)
      assert owner.owner.published and ci.CP.to_dict() == published
      assert [request for request, _ in fixture.queries[-3:]] == [b'\x10\x03', b'\x28\x00\x01', b'\x10\x01']
    return
  else:
    fp = gen_empty_fingerprint()
    fp[2].update({0x50: 16, 0x2A4: 24})
    fp[1].update({0x1CF: 8, 0x1AA: 16, 0x35: 32, 0x175: 24, 0xA0: 24,
                  0xEA: 24, 0x1BA: 24, 0x1E5: 16, 0x36A: 16})
    cp = CarInterface.get_params(identity, fp, [], False, False, False)
  sent = []
  owner = CarInterface.startup_owner(cp, (list, sent.extend), requested=True)
  assert owner is not None
  requests = []

  def query(send, recv, bus, addresses, commands, responses):
    command = commands[0]
    requests.append(command)

    class Query:
      def get_data(self, timeout):
        send([(owner.address, bytes([len(command)]) + command + bytes(7 - len(command)), owner.bus)])
        return {} if command in (b"\x28\x83\x01", b"\x28\x01\x01") else {(owner.address, None): b''}
    return Query()
  holder = VehicleStartupOwner()
  with (patch.object(CarInterface, 'startup_owner', return_value=owner),
        patch.object(owner, '_query', side_effect=query),
        patch.object(ecu_startup.time, 'sleep')):
    with patch('threading.Thread.start', side_effect=OSError('launch failed')):
      with pytest.raises(OSError, match='launch failed'):
        try:
          holder.prepare(cp, CarInterface, (list, sent.extend), requested=True, admission=lambda: True)
        finally:
          holder.close()
  assert owner.closed and owner.restored and not owner.published
  assert owner.keeper.thread is None and owner.keeper.event.is_set()
  assert requests[-3:] == [b"\x10\x03", b"\x28\x00\x01", b"\x10\x01"]


@pytest.mark.parametrize('identity,pandas', [case for case in CASES if case[0] in (CAR.KIA_EV6, CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN)])
@pytest.mark.parametrize('capture', [True, False])
def test_real_current_session_capture_precedes_continuity(identity, pandas, capture):
  import time
  from opendbc.car import CanData
  from opendbc.car.hyundai.captured_adrv_startup import CapturedADRVStartup
  from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
  from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket

  owner, now, sent, requests, query = make_owner(identity, pandas)
  counter = [0]

  def recv():
    counter[0] += 1
    data = bytearray(32)
    data[2], data[4] = counter[0] % 256, 7
    data[:2] = hkg_can_fd_checksum(0x51, None, data).to_bytes(2, 'little')
    return [TimestampedCanPacket([CanData(0x51, bytes(data), owner.template_bus)], time.clock_gettime_ns(time.CLOCK_BOOTTIME))]
  owner.callbacks = (recv, owner.callbacks[1])
  if capture:
    owner._capture = CapturedADRVStartup._capture.__get__(owner, type(owner))
  else:
    owner._capture = lambda: None
  with patch.object(ecu_startup, 'IsoTpParallelQuery', side_effect=query), patch.object(ecu_startup.time, 'sleep'):
    try:
      owner.prepare(admission=lambda: True)
      assert (owner.template is not None) == capture
      if capture:
        assert counter[0] >= 2 and owner.outcome is Outcome.SENT_UNCONFIRMED
        assert owner.keeper is not None
        assert owner.template.counter == counter[0] % 256
        assert owner.template_bus == 4 * (pandas - 1)
        assert any(frame == make_tester_present_msg(0x730, 4 * (pandas - 1) + 1, suppress_response=True) for _, frame in sent)
      else:
        assert not requests and not sent
        assert owner.outcome is Outcome.STOCK_UNTOUCHED and owner.keeper is None
    finally:
      owner.close()
