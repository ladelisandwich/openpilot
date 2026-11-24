import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.ioniq5pe_long_profile import candidate, qualified
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import CAR
from openpilot.cereal import messaging
from openpilot.selfdrive.car.card import Car

START, COUNT = 0x3A5, 32
DBC = 'hyundai_mrr35_radar_generated'

LAST_PAYLOAD = bytes.fromhex('0000000000003000a921fe383f0000000000000000000000')


def factory(present=True, bus=0):
  fp = gen_empty_fingerprint()
  fp[1].update({0x35: 32, 0x175: 24, 0xA0: 24, 0xEA: 24, 0x1CF: 8, 0x1A0: 32})
  fp[2].update({0x110: 32, 0x362: 32})
  if present:
    fp[bus].update(dict.fromkeys(range(START, START + COUNT), 24))
  return CarInterface.get_params(CAR.HYUNDAI_IONIQ_5_PE, fp, [], False, False, False)


def bank(active=True, counter=0):
  packer = CANPacker(DBC)
  frames = []
  for address in range(START, START + COUNT):
    name = f'RADAR_TRACK_{address:x}'
    fields = {'STATE': 3 if active and address == START + COUNT - 1 else 0,
              'LONG_DIST': 42.5, 'LAT_DIST': -1.5, 'REL_SPEED': -2.}
    sig = packer.dbc.name_to_msg[name].sigs.get('COUNTER')
    if sig is not None:
      fields['COUNTER'] = counter % (1 << sig.size)
    frames.append(packer.make_can_msg(name, 0, fields))
  return frames


class TestPELongRadar(unittest.TestCase):
  def test_long_candidate_preserves_actual_factory_presence_and_bus_contract(self):
    for present, bus, unavailable in ((False, 0, True), (True, 1, True), (True, 0, False)):
      stock = factory(present, bus)
      self.assertEqual(stock.radarUnavailable, unavailable)
      cp = candidate(stock, enabled=True, is_release=False)
      self.assertTrue(qualified(cp))
      self.assertEqual(cp.radarUnavailable, stock.radarUnavailable)
      self.assertTrue(cp.openpilotLongitudinalControl)

  def test_actual_long_parser_last_frozen_target_reaches_serialized_card_and_retires(self):
    cp = candidate(factory(), enabled=True, is_release=False)
    radar = RadarInterface(cp)
    self.assertFalse(radar.radar_off_can)
    self.assertEqual((radar.rcp.dbc_name, radar.rcp.bus, radar.start_addr, radar.msg_count), (DBC, 0, START, COUNT))
    frames = bank()
    self.assertEqual(frames[-1][1], LAST_PAYLOAD)
    frames[-1] = (frames[-1][0], LAST_PAYLOAD, 0)
    self.assertIsNone(radar.update([(1_000_000_000, [(a, d, 1) for a, d, _ in frames])]))
    result = radar.update([(1_050_000_000, frames)])
    self.assertIsNotNone(result)
    self.assertFalse(result.errors.canError)
    self.assertEqual(len(result.points), 1)
    point = result.points[0]
    self.assertAlmostEqual(point.dRel, 42.5)
    self.assertAlmostEqual(point.yRel, -1.5, delta=.02)
    self.assertAlmostEqual(point.vRel, -2.)
    publications = []
    caller = SimpleNamespace(pm=SimpleNamespace(send=lambda name, event: publications.append((name, event.to_bytes()))))
    Car.radar_publish(caller, result)
    self.assertEqual(len(publications), 1)
    self.assertEqual(publications[0][0], 'radarTracks')
    event = messaging.log_from_bytes(publications[0][1])
    self.assertTrue(event.valid)
    self.assertEqual(len(event.radarTracks.points), 1)
    self.assertEqual(event.radarTracks.points[0].trackId, point.trackId)
    repeated = radar.update([(1_100_000_000, bank(counter=1))])
    self.assertEqual(repeated.points[0].trackId, point.trackId)
    retired = radar.update([(1_150_000_000, bank(False, 2))])
    self.assertEqual(len(retired.points), 0)
