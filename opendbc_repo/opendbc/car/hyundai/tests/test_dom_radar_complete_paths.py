from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import CAR
from openpilot.cereal import messaging
from openpilot.selfdrive.car.card import Car

CASES = [('HYUNDAI_IONIQ', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_IONIQ_EV_LTD', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_KONA_EV_2022', 'hyundai_mrrevo14f_radar_generated', 1, 1538, 16, 8, False),
 ('HYUNDAI_KONA_EV_2ND_GEN', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, False),
 ('HYUNDAI_SANTA_FE', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_SANTA_FE_2022', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_SANTA_FE_HEV_2022', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_SANTA_FE_PHEV_2022', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_SONATA', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_PALISADE', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_SONATA_HYBRID', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('HYUNDAI_IONIQ_5', 'hyundai_mrr30_radar_generated', 0, 528, 16, 32, False),
 ('HYUNDAI_IONIQ_5_PE', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, False),
 ('HYUNDAI_IONIQ_5_N', 'hyundai_mrr30_radar_generated', 0, 528, 16, 32, False),
 ('HYUNDAI_IONIQ_6', 'hyundai_mrr35_radar_generated', 1, 933, 32, 24, False),
 ('HYUNDAI_IONIQ_9', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, False),
 ('KIA_K5_HEV_2020', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('KIA_NIRO_EV', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('KIA_NIRO_PHEV', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('KIA_NIRO_PHEV_2022', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('KIA_XCEED_PHEV', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('KIA_EV6', 'hyundai_mrr30_radar_generated', 0, 528, 16, 32, False),
 ('KIA_EV6_2025', 'hyundai_mrr30_radar_generated', 0, 528, 16, 32, False),
 ('KIA_EV9', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, False),
 ('GENESIS_GV60_EV_1ST_GEN', 'hyundai_mrr30_radar_generated', 0, 528, 16, 32, False),
 ('GENESIS_G70_2020', 'hyundai_kia_mando_front_radar_generated', 1, 1280, 32, 8, False),
 ('GENESIS_GV70_ELECTRIFIED_2ND_GEN', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, False),
 ('GENESIS_GV70_2026', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, False),
 ('GENESIS_G90', 'hyundai_genesis_g90_radar_generated', 1, 1280, 64, 8, False),
 ('GENESIS_GV80_2025', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, False),
 ('HYUNDAI_KONA_NON_SCC', 'hyundai_mrrevo14f_radar_generated', 1, 1538, 16, 8, False),
 ('HYUNDAI_IONIQ_6', 'hyundai_mrr35_radar_generated', 0, 933, 32, 24, True),
 ('HYUNDAI_PALISADE_2023', 'hyundai_radar_210_21f_generated', 0, 528, 16, 32, True),
 ('GENESIS_GV70_ELECTRIFIED_1ST_GEN', 'hyundai_radar_210_21f_generated', 0, 528, 16, 32, False)]
FROZEN_LAST_TARGETS = {'hyundai_genesis_g90_radar_generated': '006001a900003f38',
 'hyundai_kia_mando_front_radar_generated': '006001a900003f38',
 'hyundai_mrr30_radar_generated': '0000000000000000000000000000000000000000000030005223fe383f000000',
 'hyundai_mrr35_radar_generated': '0000000000003000a921fe383f0000000000000000000000',
 'hyundai_mrrevo14f_radar_generated': 'ff030000559c871f',
 'hyundai_radar_210_21f_generated': '0000000000000000000000000000000000000000000030005223fe383f000000'}


def factory(name, bus, start, count, length, lka, *, absent=False, wrong_bus=False):
  fingerprint = gen_empty_fingerprint()
  if lka:
    fingerprint[2][0x50] = 16
  if not absent:
    fingerprint[1 - bus if wrong_bus else bus].update(dict.fromkeys(range(start, start + count), length))
  return CarInterface.get_params(getattr(CAR, name), fingerprint, [], False, False, False)


def bank(dbc, bus, start, count, active=True, counter=0):
  packer = CANPacker(dbc)
  result = []
  for address in range(start, start + count):

    target = active and address == start + count - 1
    if 'mrrevo14f' in dbc:
      values = {'1_DISTANCE':255.75, '2_DISTANCE':42.5 if target else 255.75,
                '2_LATERAL':-1.5, '2_SPEED':-2.}
    elif 'mrr30' in dbc or '210_21f' in dbc:
      values = {'1_STATE':0, '2_STATE':3 if target else 0,
                '2_LONG_DIST':42.5, '2_LAT_DIST':-1.5, '2_REL_SPEED':-2.}
    elif 'mrr35' in dbc:
      values = {'STATE':3 if target else 0, 'LONG_DIST':42.5, 'LAT_DIST':-1.5, 'REL_SPEED':-2.}
    else:
      values = {'STATE':3 if target else 0, 'LONG_DIST':42.5, 'AZIMUTH':0., 'REL_SPEED':-2.}
    name = f'RADAR_TRACK_{address:x}'
    message = packer.dbc.name_to_msg[name]
    if 'COUNTER' in message.sigs:
      values['COUNTER'] = counter % (1 << message.sigs['COUNTER'].size)
    result.append(packer.make_can_msg(name, bus, values))
  return result


@pytest.mark.parametrize('name,dbc,bus,start,count,length,lka', CASES)
def test_original_vehicle_factory_reaches_expected_radar_bank(name, dbc, bus, start, count, length, lka):
  assert factory(name, bus, start, count, length, lka, absent=True).radarUnavailable
  assert factory(name, bus, start, count, length, lka, wrong_bus=True).radarUnavailable
  cp = factory(name, bus, start, count, length, lka)
  assert not cp.radarUnavailable
  radar = RadarInterface(cp)
  assert radar.rcp.dbc_name == dbc
  assert radar.rcp.bus == bus
  assert (radar.start_addr, radar.msg_count) == (start, count)


@pytest.mark.parametrize('name,dbc,bus,start,count,length,lka', CASES)
def test_original_vehicle_last_target_reaches_card_and_is_retired(name, dbc, bus, start, count, length, lka):
  cp = factory(name, bus, start, count, length, lka)
  radar = RadarInterface(cp)
  frames = bank(dbc, bus, start, count)
  assert all(len(data) == length for _, data, _ in frames)
  expected_payload = bytes.fromhex(FROZEN_LAST_TARGETS[dbc])
  assert frames[-1][1] == expected_payload
  frames[-1] = (frames[-1][0], expected_payload, frames[-1][2])
  wrong_bus_frames = [(address, data, 1 - bus) for address, data, _ in frames]
  assert radar.update([(1_000_000_000, wrong_bus_frames)]) is None
  result = radar.update([(1_050_000_000, frames)])
  assert result is not None and not result.errors.canError
  assert len(result.points) == 1
  point = result.points[0]
  assert point.dRel == pytest.approx(42.5)
  expected_y = -1.5 if any(part in dbc for part in ('mrrevo14f', 'mrr30', 'mrr35', '210_21f')) else 0.
  assert point.yRel == pytest.approx(expected_y, abs=.02)
  assert point.vRel == pytest.approx(-2.)
  publications = []
  caller = SimpleNamespace(pm=SimpleNamespace(send=lambda service, event: publications.append((service, event.to_bytes()))))
  Car.radar_publish(caller, result)
  assert len(publications) == 1 and publications[0][0] == 'radarTracks'
  event = messaging.log_from_bytes(publications[0][1])
  assert event.valid and len(event.radarTracks.points) == 1
  wire_point = event.radarTracks.points[0]
  assert wire_point.trackId == point.trackId
  assert (wire_point.dRel, wire_point.yRel, wire_point.vRel) == pytest.approx((point.dRel, point.yRel, point.vRel))
  repeated = radar.update([(1_100_000_000, bank(dbc, bus, start, count, counter=1))])
  assert repeated is not None and repeated.points[0].trackId == point.trackId
  removed = radar.update([(1_150_000_000, bank(dbc, bus, start, count, active=False, counter=2))])
  assert removed is not None and not removed.points
