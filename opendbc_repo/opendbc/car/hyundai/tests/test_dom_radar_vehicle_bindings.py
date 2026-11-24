from types import SimpleNamespace
import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface, get_radar_can_parser
from opendbc.car.hyundai.values import CAR, DBC
from openpilot.cereal import messaging
from openpilot.selfdrive.car.card import Car

CASES = [
  (CAR.HYUNDAI_IONIQ, 'hyundai_kia_mando_front_radar_generated', 1, 0x500, 32, 8),
  (CAR.HYUNDAI_SANTA_FE_2022, 'hyundai_kia_mando_front_radar_generated', 1, 0x500, 32, 8),
  (CAR.HYUNDAI_SANTA_FE_HEV_2022, 'hyundai_kia_mando_front_radar_generated', 1, 0x500, 32, 8),
  (CAR.HYUNDAI_SANTA_FE_PHEV_2022, 'hyundai_kia_mando_front_radar_generated', 1, 0x500, 32, 8),
  (CAR.HYUNDAI_KONA_EV_2022, 'hyundai_mrrevo14f_radar_generated', 1, 0x602, 16, 8),
  (CAR.HYUNDAI_KONA_EV_2ND_GEN, 'hyundai_mrr35_radar_generated', 0, 0x3A5, 32, 24),
  (CAR.HYUNDAI_IONIQ_5, 'hyundai_mrr30_radar_generated', 0, 0x210, 16, 32),
  (CAR.HYUNDAI_IONIQ_5_N, 'hyundai_mrr30_radar_generated', 0, 0x210, 16, 32),
  (CAR.KIA_EV6, 'hyundai_mrr30_radar_generated', 0, 0x210, 16, 32),
  (CAR.GENESIS_GV60_EV_1ST_GEN, 'hyundai_mrr30_radar_generated', 0, 0x210, 16, 32),
]


def params(identity, bank, bus):
  fp = gen_empty_fingerprint()
  fp[bus].update(bank)
  return CarInterface.get_params(identity, fp, [], False, False, False)


@pytest.mark.parametrize('identity,dbc,bus,start,count,length', CASES)
def test_every_dom_binding_factory_selection_and_unchanged_control_authority(identity, dbc, bus, start, count, length):
  absent = params(identity, {}, bus)
  cp = params(identity, {start:length}, bus)
  assert absent.radarUnavailable
  assert not cp.radarUnavailable
  assert DBC[identity][Bus.radar] == dbc
  radar = RadarInterface(cp)
  assert (radar.rcp.bus, radar.start_addr, radar.msg_count) == (bus,start,count)
  assert get_radar_can_parser(cp) is not None
  before, after = absent.to_dict(), cp.to_dict()
  before.pop('radarUnavailable')
  after.pop('radarUnavailable')
  assert before == after
  assert params(identity, {start:length}, 1-bus).radarUnavailable
  if dbc != 'hyundai_kia_mando_front_radar_generated':
    assert params(identity, {start:length-1}, bus).radarUnavailable


def frames(dbc,bus,start,count,active=True,counter=0):
  packer=CANPacker(dbc)
  result=[]
  for address in range(start,start+count):
    target=active and address==start
    if 'mrrevo14f' in dbc:
      values={'1_DISTANCE':42.5 if target else 255.75,'1_LATERAL':-1.5,'1_SPEED':-2.,'2_DISTANCE':255.75}
    elif 'mrr30' in dbc:
      values={'1_STATE':3 if target else 0,'1_LONG_DIST':42.5,'1_LAT_DIST':-1.5,'1_REL_SPEED':-2.,'2_STATE':0}
    elif 'mrr35' in dbc:
      values={'STATE':3 if target else 0,'LONG_DIST':42.5,'LAT_DIST':-1.5,'REL_SPEED':-2.}
    else:
      values={'STATE':3 if target else 0,'LONG_DIST':42.5,'AZIMUTH':0.,'REL_SPEED':-2.}
    message=packer.dbc.name_to_msg[f'RADAR_TRACK_{address:x}']
    if 'COUNTER' in message.sigs:
      values['COUNTER']=counter % (1 << message.sigs['COUNTER'].size)
    result.append(packer.make_can_msg(f'RADAR_TRACK_{address:x}',bus,values))
  return result


@pytest.mark.parametrize('identity,dbc,bus,start,count,length', CASES)
def test_restored_vehicle_actual_track_bank_and_card_wire_publication(identity,dbc,bus,start,count,length):
  cp=params(identity,{start:length},bus)
  radar=RadarInterface(cp)
  bank=frames(dbc,bus,start,count)
  assert all(len(data)==length for _,data,_ in bank)
  assert radar.update((1_000_000_000,[(address,data,1-bus) for address,data,_ in bank])) is None
  result=radar.update((1_050_000_000,bank))
  assert result is not None and not result.errors.canError
  assert len(result.points)==1
  point=result.points[0]
  assert point.dRel==pytest.approx(42.5)
  assert point.yRel==pytest.approx(0. if 'mando' in dbc else -1.5,abs=.02)
  assert point.vRel==pytest.approx(-2.)
  published=[]
  caller=SimpleNamespace(pm=SimpleNamespace(send=lambda service,event:published.append((service,event.to_bytes()))))
  Car.radar_publish(caller,result)
  assert len(published)==1 and published[0][0]=='radarTracks'
  event=messaging.log_from_bytes(published[0][1])
  assert event.valid and len(event.radarTracks.points)==1
  assert event.radarTracks.points[0].dRel==pytest.approx(42.5)
  next_result=radar.update((1_100_000_000,frames(dbc,bus,start,count,counter=1)))
  assert next_result.points[0].trackId==point.trackId
  empty=radar.update((1_150_000_000,frames(dbc,bus,start,count,False,counter=2)))
  assert empty is not None and len(empty.points)==0
