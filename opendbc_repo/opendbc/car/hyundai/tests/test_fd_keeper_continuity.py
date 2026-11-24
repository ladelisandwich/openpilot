from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, structs
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_torque_ev_startup import physical_frames, stock as torque_stock
from opendbc.car.hyundai.tests.test_ev9_pending_startup import pending_startup_fixture
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.vehicle_startup import VehicleStartupOwner

PROFILES = [(CAR.KIA_EV9, True), *[(car, alt) for car in (CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_KONA_EV_2ND_GEN) for alt in (False, True)]]


class Keeper:
  def __init__(self, send, **kwargs):
    self.send = send
    self.abort_reason = None
    self.started = False

  def start(self):
    self.started = True

  def stop(self):
    self.started = False


@pytest.mark.parametrize("profile", PROFILES)
@pytest.mark.parametrize("publication", ["success", "invalid", "exception"])
def test_actual_controller_tester_publish_commits_fd_startup_handoff(profile, publication, monkeypatch):
  car_id, _ = profile
  if car_id == CAR.KIA_EV9:
    check_pending_ev9_publication(publication, monkeypatch)
    return
  stock = torque_stock.__wrapped__(SimpleNamespace(param=profile))
  requests = []
  owner = CarInterface.startup_owner(stock, (lambda **kwargs: [], lambda frames: None), requested=True)
  assert owner is not None
  owner.keeper_factory = Keeper

  def query(send, recv, bus, addresses, request, response):
    payload = request[0]
    requests.append(payload)

    def data(timeout):
      send([(0x730, bytes([len(payload)]) + payload + bytes(7 - len(payload)), 1)])
      return {} if payload[0] == 0x28 else {(0x730, None): b""}

    return SimpleNamespace(get_data=data)

  with patch.object(owner, "_query", side_effect=query), patch("time.sleep"):
    cp = owner.prepare(admission=lambda: True)
  assert owner.outcome is Outcome.SENT_UNCONFIRMED
  assert requests == [b"\x10\x03", b"\x28\x83\x01"]
  ci = CarInterface(cp)
  ci.update([])
  owner.configure(ci)
  owner.seal_publication()
  holder = VehicleStartupOwner(owner)
  packer = CANPacker(DBC[car_id][Bus.pt])
  stamp = owner.floor_ns
  for tick in range(1, 121):
    frames = physical_frames(cp, packer, tick)
    stamp = owner.floor_ns + tick * 10_000_000
    state = ci.update([(stamp, frames)])
  assert state.canValid and not state.canTimeout and owner.sources_current(ci, stamp)
  for configured, sources, current in ((False, True, True), (True, False, True), (True, True, False)):
    assert not owner.before_control(configured=configured, sources_current=sources, control_current=current)
    assert owner.keeper.started and not owner.handed_off
  assert owner.before_control(configured=True, sources_current=True, control_current=True)
  assert owner.keeper.started and not owner.handed_off
  published = []
  should_fail = [False]

  def publish(service, packet):
    assert service == "sendcan" and not holder.send_lock.acquire(blocking=False)
    if should_fail[0]:
      raise OSError("publisher failure before successful send")
    published.append(packet)

  card = object.__new__(Car)
  card.CP, card.CI, card.vehicle_startup = cp, ci, holder
  monkeypatch.setattr(card, "pm", SimpleNamespace(send=publish), raising=False)
  command = structs.CarControl().as_reader()
  tester = make_tester_present_msg(0x730, 1, suppress_response=True)
  config = cp.safetyConfigs[0]
  native = libsafety_py.libsafety
  native.set_alternative_experience(cp.alternativeExperience)
  assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
  native.init_tests()
  assert native.safety_tx_hook(libsafety_py.make_CANPacket(tester[0], tester[2], tester[1]))
  try:
    ci.CC.frame = 1
    _, frames = ci.apply(command, stamp)
    assert tester not in frames
    card.publish_sendcan(frames, valid=True)
    assert owner.keeper.started and not owner.handed_off
    ci.CC.frame = 100
    _, frames = ci.apply(command, stamp + 10_000_000)
    assert tester in frames
    if publication == "exception":
      should_fail[0] = True
      with pytest.raises(OSError, match="publisher failure"):
        card.publish_sendcan(frames, valid=True)
      should_fail[0] = False
    elif publication == "invalid":
      card.publish_sendcan(frames, valid=False)
    if publication != "success":
      assert owner.keeper.started and not owner.handed_off
    card.publish_sendcan(frames, valid=True)
    assert published and not owner.keeper.started and owner.handed_off
    assert holder.send_lock.acquire(blocking=False)
    holder.send_lock.release()
    assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
  finally:
    owner.close()


def check_pending_ev9_publication(publication, monkeypatch):
  with pending_startup_fixture() as (fixture, owner, ci, packer):
    fixture.warm(owner, ci, packer)
    fixture.commit(owner, ci)
    assert owner.owner.active() and owner.thread.started and not owner.handed_off
    assert [request for request, _ in fixture.queries] == [b"\x10\x03", b"\x28\x01\x01"]
    fixture.warm(owner, ci, packer)
    for configured, sources, current in ((False, True, True), (True, False, True), (True, True, False)):
      fixture.acknowledge(owner, ci)
      assert not owner.before_control(configured=configured, sources_current=sources, control_current=current)
      assert owner.thread.started and not owner.thread.joined and not owner.handed_off
    fixture.acknowledge(owner, ci)
    assert owner.before_control(configured=True, sources_current=True, control_current=True)
    holder = VehicleStartupOwner(owner)
    published, should_fail = [], [False]

    def publish(service, packet):
      assert service == "sendcan" and not holder.send_lock.acquire(blocking=False)
      if should_fail[0]:
        raise OSError("publisher failure before successful send")
      published.append(packet)

    card = object.__new__(Car)
    card.CP, card.CI, card.vehicle_startup = ci.CP, ci, holder
    monkeypatch.setattr(card, "pm", SimpleNamespace(send=publish), raising=False)
    tester = make_tester_present_msg(0x730, 1, suppress_response=True)
    native = libsafety_py.libsafety
    config = ci.CP.safetyConfigs[0]
    native.set_alternative_experience(ci.CP.alternativeExperience)
    assert native.set_safety_hooks(int(config.safetyModel.raw), int(config.safetyParam)) == 0
    native.init_tests()
    assert native.safety_tx_hook(libsafety_py.make_CANPacket(tester[0], tester[2], tester[1]))
    try:
      card.publish_sendcan([tester], valid=True)
      assert not owner.handed_off and not owner.thread.joined
      _, frames = ci.apply(structs.CarControl().as_reader(), fixture.boot)
      assert {(0x1A0, 1), (0x100, 0), (0xCB, 1), (0x730, 1)}.issubset({(a, b) for a, _, b in frames})
      by_id = {(address, bus): data for address, data, bus in frames}
      scc, angle = by_id[(0x1A0, 1)], by_id[(0xCB, 1)]
      assert (((scc[17] & 7) << 8) | scc[16]) - 1023 == 0
      assert ((scc[18] << 4) | (scc[17] >> 4)) - 1023 == 0
      assert ((scc[8] >> 4) & 7) == 0 and ((angle[3] >> 4) & 15) == 1
      # Each neutral replacement is required, not merely the diagnostic tester.
      for missing in (0x1A0, 0x100, 0xCB, 0x730):
        card.publish_sendcan([frame for frame in frames if frame[0] != missing], valid=True)
        assert not owner.handed_off and not owner.thread.joined
      if publication == "exception":
        should_fail[0] = True
        with pytest.raises(OSError, match="publisher failure"):
          card.publish_sendcan(frames, valid=True)
        should_fail[0] = False
      elif publication == "invalid":
        card.publish_sendcan(frames, valid=False)
      assert not owner.handed_off and not owner.thread.joined
      card.publish_sendcan(frames, valid=True)
      assert published and owner.handed_off and owner.thread.joined
      assert holder.send_lock.acquire(blocking=False)
      holder.send_lock.release()
      assert not native.get_controls_allowed() and native.aol_get_permission_mask() == 0
    finally:
      fixture.admitted = True
      owner.close()
    assert owner.restored
