from functools import partial
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.ev9_keeper import EV9Keeper
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
from opendbc.car.hyundai.radar_hda2_startup import CARS, RadarHDA2Startup, qualified, required
from opendbc.car.hyundai.tests.test_radar_hda2_startup_lifecycle import fingerprint, physical_frames, firmware
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def make_owner():
  cp = CarInterface.get_params(CAR.GENESIS_GV70_1ST_GEN, fingerprint(), firmware(), True, False, False)
  owner = CarInterface.startup_owner(cp, (list, list), requested=True)

  def query(sender, receiver, bus, addresses, requests, responses):
    command = requests[0]

    def data(timeout):
      sender([(0x730, bytes([len(command)]) + command + bytes(7 - len(command)), 1)])
      return {} if command == b"\x28\x83\x01" else {(0x730, None): b""}

    return SimpleNamespace(get_data=data)

  owner._query = query
  return cp, owner


@pytest.mark.parametrize("fault", ["wrong_bus", "stale", "missing_scc", "missing_camera", "missing_buttons", "missing_gear"])
def test_stock_fallback_requires_real_fresh_sources_on_owned_bus(fault):
  cp, owner = make_owner()
  now = [10_000_000_000]
  stock = owner.stock_cp
  owner.outcome, owner.restored, owner.floor_ns = Outcome.STOCK_RESTORED, True, now[0]
  owner.cp, owner.prepared_cp = stock, stock.to_dict()
  ci = CarInterface(stock)
  tick = [0]

  def recv():
    tick[0] += 1
    now[0] += 10_000_000
    frames = physical_frames(ci, tick[0])
    if fault == "wrong_bus":
      frames = [CanData(address, data, 0 if bus == 1 else bus) for address, data, bus in frames]
    if fault == "missing_scc":
      frames = [frame for frame in frames if frame[0] != 0x1A0]
    if fault == "missing_camera":
      frames = [frame for frame in frames if frame[0] != 0x2A4]
    if fault == "missing_buttons":
      frames = [frame for frame in frames if frame[0] != 0x1CF]
    if fault == "missing_gear":
      frames = [frame for frame in frames if frame[0] != 0x130]
    return [TimestampedCanPacket(frames, owner.floor_ns if fault == "stale" else now[0])]

  owner.callbacks = (recv, list)
  owner.clock = lambda: now[0] / 1e9

  def sleep(seconds):
    now[0] += round(seconds * 1e9)

  with patch("time.clock_gettime_ns", side_effect=lambda clock: now[0]), patch("time.sleep", side_effect=sleep):
    with pytest.raises(RuntimeError, match="stock SCC sources did not resume"):
      owner.configure(ci)
  assert owner.outcome is Outcome.ABORT_UNCERTAIN and not owner.ready and not owner.published


def test_actual_keeper_thread_start_failure_restores_before_publication():
  cp, owner = make_owner()

  class Thread:
    ident = None

    def __init__(self, **kwargs):
      pass

    def start(self):
      raise OSError("thread start failed")

  owner.keeper_factory = partial(EV9Keeper, thread_factory=Thread)
  with patch("time.sleep"):
    with pytest.raises(OSError, match="thread start failed"):
      owner.prepare(admission=lambda: True)
  assert owner.closed and owner.restored and not owner.published
  assert owner.keeper.thread is None and owner.keeper.event.is_set()


@pytest.mark.parametrize("mutation", ["passive", "dashcamOnly", "notCar", "brand", "word", "experience", "lka", "angle", "camera", "ccnc", "alt_buttons"])
def test_factory_owner_refuses_unqualified_radar_profiles(mutation):
  cp, owner = make_owner()
  if mutation in ("passive", "dashcamOnly", "notCar"):
    setattr(cp, mutation, True)
  elif mutation == "brand":
    cp.brand = "other"
  elif mutation == "word":
    cp.safetyConfigs[0].safetyParam = 0x804
  elif mutation == "experience":
    cp.alternativeExperience = 32
  elif mutation == "lka":
    cp.flags = int(cp.flags) & ~int(HyundaiFlags.CANFD_LKA_STEER_MSG)
  else:
    cp.flags = int(cp.flags) | int(
      {"lka": HyundaiFlags.CANFD_LKA_STEER_MSG, "angle": HyundaiFlags.CANFD_ANGLE_STEERING, "camera": HyundaiFlags.CANFD_CAMERA_SCC,
       "ccnc": HyundaiFlags.CCNC, "alt_buttons": HyundaiFlags.CANFD_ALT_BUTTONS}[mutation]
    )
  assert not qualified(cp)
  with pytest.raises(ValueError, match="Exact unmarked radar HDA-II"):
    RadarHDA2Startup(cp, (list, list))


@pytest.mark.parametrize("identity", sorted(CARS))
@pytest.mark.parametrize("hybrid", [False, True])
@pytest.mark.parametrize("alpha", [False, True])
@pytest.mark.parametrize("alternate", [False, True])
def test_two_panda_layout_retains_existing_unowned_factory_disposition(identity, hybrid, alpha, alternate):
  original = fingerprint(hybrid, alternate)
  fp = gen_empty_fingerprint()
  fp[5].update(original[1])
  fp[6].update(original[2])
  cp = CarInterface.get_params(identity, fp, firmware(), alpha, False, False)
  assert len(cp.safetyConfigs) == 2
  before = cp.to_dict()
  assert CarInterface.startup_owner(cp, (list, list), requested=alpha) is None
  assert not CarInterface.startup_required(cp)
  assert cp.to_dict() == before


@pytest.mark.parametrize("identity", sorted(CARS))
@pytest.mark.parametrize("hybrid", [False, True])
@pytest.mark.parametrize("alternate", [False, True])
def test_missing_discovered_adas_keeps_existing_alpha_availability_gate(identity, hybrid, alternate):
  cp = CarInterface.get_params(identity, fingerprint(hybrid, alternate), [], True, False, False)
  before = cp.to_dict()
  assert not cp.alphaLongitudinalAvailable and not cp.openpilotLongitudinalControl and cp.pcmCruise
  assert not required(cp) and CarInterface.startup_owner(cp, (list, list), requested=True) is None
  assert cp.to_dict() == before


@pytest.mark.parametrize("identity", [CAR.KIA_EV6, CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, CAR.KIA_EV9,
                                     CAR.HYUNDAI_IONIQ_6, CAR.HYUNDAI_IONIQ_5_PE, CAR.HYUNDAI_IONIQ_5_N])
def test_new_owner_never_substitutes_for_captured_angle_or_ccnc_contract(identity):
  cp = CarInterface.get_params(identity, fingerprint(), firmware(), True, False, False)
  assert not required(cp)
