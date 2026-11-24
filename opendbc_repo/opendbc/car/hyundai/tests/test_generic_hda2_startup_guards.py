from unittest.mock import patch

import pytest

from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.ioniq6_handoff import TimestampedCanPacket
from opendbc.car.hyundai.tests.test_generic_hda2_startup_lifecycle import physical_frames
from opendbc.car.hyundai.generic_hda2_startup import CARS, EXISTING_OWNERS_OR_DISTINCT_RECEIVERS, GenericHDA2Startup, qualified, required
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_radar_hda2_startup_lifecycle import fingerprint, firmware
from opendbc.car.hyundai.values import CAR, HyundaiFlags


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_STARIA_4TH_GEN, CAR.KIA_CARNIVAL_2025, CAR.KIA_CARNIVAL_HEV_4TH_GEN,
                                     CAR.KIA_NIRO_EV_2ND_GEN, CAR.HYUNDAI_IONIQ_5_N])
@pytest.mark.parametrize('alt', [False, True])
@pytest.mark.parametrize('mutation', ['passive', 'dashcamOnly', 'notCar', 'brand', 'word', 'experience', 'angle',
                                      'hybrid', 'camera', 'ccnc', 'alt_buttons', 'gas_flag'])
def test_scoped_generic_ev_owner_refuses_other_profiles(identity, alt, mutation):
  cp = CarInterface.get_params(identity, fingerprint(False, alt), firmware(), True, False, False)
  assert qualified(cp)
  if mutation in ('passive', 'dashcamOnly', 'notCar'):
    setattr(cp, mutation, True)
  elif mutation == 'brand':
    cp.brand = 'other'
  elif mutation == 'word':
    cp.safetyConfigs[0].safetyParam = 0x815
  elif mutation == 'experience':
    cp.alternativeExperience = 32
  elif mutation == 'ccnc':
    cp.flags ^= int(HyundaiFlags.CCNC)
  elif mutation == 'hybrid':
    cp.flags ^= int(HyundaiFlags.HYBRID)
  elif mutation == 'gas_flag':
    cp.flags ^= int(HyundaiFlags.EV)
  else:
    cp.flags |= int({'angle': HyundaiFlags.CANFD_ANGLE_STEERING, 'hybrid': HyundaiFlags.HYBRID,
                     'camera': HyundaiFlags.CANFD_CAMERA_SCC, 'alt_buttons': HyundaiFlags.CANFD_ALT_BUTTONS}[mutation])
  assert not qualified(cp)
  with pytest.raises(ValueError, match='Exact source-selected ordinary HDA-II'):
    GenericHDA2Startup(cp, (list, list))


@pytest.mark.parametrize('identity', sorted(CARS))
@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('alt', [False, True])
def test_two_panda_disposition_is_preserved_and_not_claimed_owned(identity, alpha, alt):
  original = fingerprint(False, alt)
  fp = gen_empty_fingerprint()
  fp[5].update(original[1])
  fp[6].update(original[2])
  cp = CarInterface.get_params(identity, fp, firmware(), alpha, False, False)
  before = cp.to_dict()
  assert len(cp.safetyConfigs) == 2
  assert not required(cp) and CarInterface.startup_owner(cp, (list, list), requested=alpha) is None
  assert cp.to_dict() == before


@pytest.mark.parametrize('identity', sorted(CARS))
@pytest.mark.parametrize('alt', [False, True])
def test_missing_observed_adas_keeps_original_alpha_gate(identity, alt):
  cp = CarInterface.get_params(identity, fingerprint(False, alt), [], True, False, False)
  before = cp.to_dict()
  assert not cp.alphaLongitudinalAvailable and not cp.openpilotLongitudinalControl and cp.pcmCruise
  assert CarInterface.startup_owner(cp, (list, list), requested=True) is None
  assert cp.to_dict() == before


@pytest.mark.parametrize('identity', sorted(EXISTING_OWNERS_OR_DISTINCT_RECEIVERS))
def test_new_owner_never_substitutes_for_distinct_existing_receiver_recipe(identity):
  cp = CarInterface.get_params(identity, fingerprint(), firmware(), True, False, False)
  assert not required(cp)


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_STARIA_4TH_GEN, CAR.KIA_CARNIVAL_2025, CAR.KIA_CARNIVAL_HEV_4TH_GEN,
                                     CAR.KIA_NIRO_EV_2ND_GEN, CAR.HYUNDAI_IONIQ_5_N])
@pytest.mark.parametrize('fault', ['wrong_bus', 'stale', 'missing_scc', 'missing_camera', 'missing_buttons', 'missing_accelerator'])
def test_stock_fallback_needs_independent_ev_buttons_camera_and_scc_sources(identity, fault):
  cp = CarInterface.get_params(identity, fingerprint(), firmware(), True, False, False)
  owner = CarInterface.startup_owner(cp, (list, list), requested=True)
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
    if fault == 'wrong_bus':
      frames = [CanData(address, raw, 0 if bus == 1 else bus) for address, raw, bus in frames]
    elif fault != 'stale':
      omit = (ci.can_parsers[Bus.pt].dbc.name_to_msg[ci.CS.accelerator_msg_canfd].address if fault == 'missing_accelerator' else
              {'missing_scc': 0x1A0, 'missing_camera': 0x2A4, 'missing_buttons': 0x1CF}[fault])
      frames = [frame for frame in frames if frame[0] != omit]
    return [TimestampedCanPacket(frames, owner.floor_ns if fault == 'stale' else now[0])]

  owner.callbacks = (recv, list)
  owner.clock = lambda: now[0] / 1e9

  def sleep(seconds):
    now[0] += round(seconds * 1e9)

  with patch('time.clock_gettime_ns', side_effect=lambda clock: now[0]), patch('time.sleep', side_effect=sleep):
    with pytest.raises(RuntimeError, match='stock SCC sources did not resume'):
      owner.configure(ci)
  assert owner.outcome is Outcome.ABORT_UNCERTAIN and not owner.ready and not owner.published


@pytest.mark.parametrize('lka', [False, True])
def test_future_unreviewed_disable_profile_cannot_publish_without_a_source_contract(lka):
  cp = CarInterface.get_params(CAR.KIA_NIRO_EV_2ND_GEN, fingerprint(), firmware(), True, False, False)
  cp.carFingerprint = 'FUTURE_UNREVIEWED_HYUNDAI'
  if not lka:
    cp.flags &= ~int(HyundaiFlags.CANFD_LKA_STEER_MSG)
    cp.flags |= int(HyundaiFlags.CANFD_RADAR_SCC)
    cp.safetyConfigs[0].safetyParam = 0x5
  assert required(cp) and not qualified(cp)
  sent = []
  with pytest.raises(ValueError, match='Exact source-selected ordinary HDA-II'):
    CarInterface.startup_owner(cp, (list, sent.extend), requested=True)
  assert not sent
