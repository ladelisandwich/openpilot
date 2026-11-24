from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.tesla.tests.test_preap_radar import frames, radar_params, radar_interface
from openpilot.cereal import messaging
from openpilot.selfdrive.controls.radard import RadarD


def source_tick(sm, tick, rr=None, speed=0.0, probability=0.0, distance=10.0, standstill=False, stamp=None, radar_event=None):
  stamp = 1_000_000_000 + tick * 50_000_000 if stamp is None else stamp
  model_event = messaging.new_message('modelV2', valid=True)
  model_event.logMonoTime = stamp
  model = model_event.modelV2
  model.velocity.x = [speed]
  leads = model.init('leadsV3', 2)
  for lead in leads:
    lead.x = [distance + 1.52]
    lead.y = [0.0]
    lead.v = [speed]
    lead.a = [0.0]
    lead.xStd = lead.yStd = lead.vStd = [1.0]
    lead.prob = probability
  car_event = messaging.new_message('carState', valid=True)
  car_event.logMonoTime = stamp
  car_event.carState.vEgo = speed
  car_event.carState.standstill = standstill
  events = [model_event.as_reader(), car_event.as_reader()]
  if radar_event is not None:
    events.append(radar_event.as_reader())
  elif rr is not None:
    radar_event = messaging.new_message('radarTracks', valid=True)
    radar_event.logMonoTime = stamp
    radar_event.radarTracks = rr
    events.append(radar_event.as_reader())
  sm.update_msgs(stamp / 1e9, events)


def test_actual_preap_decoder_to_radard_tracks_errors_retirement_and_recovery():
  cp = radar_params(0.25)
  decoder, packer = radar_interface(cp, 0.25), CANPacker("tesla_radar_bosch_generated")
  radar = RadarD(cp.radarDelay)
  sm = messaging.SubMaster(["modelV2", "carState", "radarTracks"])
  decoder.update([])
  last_cycle = 0
  identifier = None
  for tick in range(1, 31):
    now = 1_000_000_000 + tick * 50_000_000
    cycle = (now - 1_000_000_000) // 125_000_000
    rr = None
    if cycle > last_cycle:
      radar_time = 1_000_000_000 + cycle * 125_000_000
      rr = decoder.update([(radar_time, frames(packer, measured=False))])
      last_cycle = cycle
    if rr is not None:
      with structs.RadarData.from_bytes(rr.to_bytes()) as decoded:
        source_tick(sm, tick, decoded, speed=10.0, probability=1.0, distance=30.0)
    else:
      source_tick(sm, tick, speed=10.0, probability=1.0, distance=30.0)
    radar.update(sm, sm["radarTracks"])
    if radar.tracks:
      identifier = next(iter(radar.tracks))
  assert identifier is not None
  count = radar.tracks[identifier].cnt
  assert radar.kalman_params.A[0][1] == 0.05
  source_tick(sm, 31, speed=10.0, probability=1.0, distance=30.0)
  radar.update(sm, sm["radarTracks"])
  assert radar.tracks[identifier].cnt == count + 1
  assert radar.radar_state.leadOne.present and radar.radar_state.leadOne.radar
  trigger = packer.make_can_msg("RadarPoint31_B", 1, {"Index2": 0})
  for step in range(5):
    rr = decoder.update([(4_000_000_000 + step * 125_000_000, [trigger])])
  with structs.RadarData.from_bytes(rr.to_bytes()) as decoded:
    source_tick(sm, 70, decoded)
  radar.update(sm, sm["radarTracks"])
  assert radar.radar_state.radarErrors.canError
  rr = decoder.update([(4_625_000_000, frames(packer, fault=True, index=1))])
  with structs.RadarData.from_bytes(rr.to_bytes()) as decoded:
    source_tick(sm, 73, decoded)
  radar.update(sm, sm["radarTracks"])
  assert radar.radar_state.radarErrors.radarFault and not radar.tracks
  rr = decoder.update([(4_750_000_000, frames(packer))])
  with structs.RadarData.from_bytes(rr.to_bytes()) as decoded:
    source_tick(sm, 75, decoded)
  radar.update(sm, sm["radarTracks"])
  assert len(radar.tracks) == 1 and identifier not in radar.tracks
