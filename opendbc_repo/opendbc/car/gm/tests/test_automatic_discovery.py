"""Automatic pedal discovery must not move healthy normal CAN onto OBD pins."""
from dataclasses import replace
from unittest import mock

import pytest

from opendbc.car import car_helpers, fw_versions
from opendbc.car.can_definitions import CanData
from opendbc.car.ford.discovery import normal_bus_query_brand
from opendbc.car.gm.discovery import pedal_normal_bus0_topology
from opendbc.car.gm.values import CAR
from opendbc.car.hyundai.discovery import NORMAL_BUS_LAYOUT, normal_bus_topology

# Original pre-query samples from a8c036520b19999c/00000004--7e3abadb55.
# Query family only: live VIN/FW and ordinary CAN discovery still own identity.
RECORDED = ((0, 211, '2d1c'),
 (2, 211, '2d1c'),
 (0, 513, '047b025258c6'),
 (2, 513, '047b025258c6'),
 (0, 190, '0402620001fd'),
 (2, 190, '0402620001fd'),
 (0, 201, '8000002749001000'),
 (2, 201, '8000002749001000'),
 (0, 452, '1362c1ee004901b7'),
 (2, 452, '1362c1ee004901b7'),
 (0, 241, '1c020041fbfe'),
 (2, 241, '1c020041fbfe'),
 (0, 481, '00000000031cc0'),
 (2, 481, '00000000031cc0'))


def recorded_frames():
  return [CanData(address, bytes.fromhex(data), bus) for bus, address, data in RECORDED]


def brand(frames):
  return normal_bus_query_brand(lambda **kwargs: [frames], normal_bus_topology,
                               gm_probe=pedal_normal_bus0_topology)


def test_recorded_pedal_layout_scopes_queries_to_gm():
  assert brand(recorded_frames()) == 'gm'


@pytest.mark.parametrize('missing', range(len(RECORDED)))
def test_incomplete_layout_does_not_scope_gm(missing):
  frames = recorded_frames()
  del frames[missing]
  assert brand(frames) is None


@pytest.mark.parametrize('bad', range(len(RECORDED)))
def test_wrong_bus_or_dlc_does_not_scope_gm(bad):
  for bus_delta, size_delta in ((1, 0), (128, 0), (0, -1), (0, 1)):
    frames = recorded_frames()
    original = frames[bad]
    frames[bad] = CanData(original.address, bytes(len(original.dat) + size_delta), original.src + bus_delta)
    assert brand(frames) is None


def test_conflicting_dlc_cannot_be_hidden_by_later_valid_frame():
  frames = recorded_frames()
  bad = CanData(frames[0].address, bytes(len(frames[0].dat) + 1), frames[0].src)
  assert brand([bad, *frames]) is None


def test_other_bus_activity_and_unrecognized_layout_keep_generic_discovery():
  assert brand([*recorded_frames(), CanData(123, bytes(8), 1)]) is None
  assert brand([]) is None
  assert brand([CanData(123, bytes(8), 0)]) is None


def test_hyundai_path_and_existing_read_budget_are_preserved():
  frames = [CanData(address, bytes(size), bus) for (bus, address), size in NORMAL_BUS_LAYOUT.items()]
  assert brand(frames) == 'hyundai'
  calls = []

  def recv(**kwargs):
    calls.append(kwargs)
    return []
  assert normal_bus_query_brand(recv, lambda recv: normal_bus_topology(recv, clock=lambda: 1.),
                               gm_probe=pedal_normal_bus0_topology) is None
  assert len(calls) == 24


@pytest.mark.parametrize('registry', ['configured', 'effective'])
@pytest.mark.parametrize('change', ['bus1', 'empty'])
def test_changed_gm_query_topology_does_not_scope_automatic_queries(registry, change):
  requests = fw_versions.FW_QUERY_CONFIGS['gm'].requests
  if registry == 'configured':
    target, name = fw_versions.FW_QUERY_CONFIGS['gm'], 'requests'
    changed = [replace(r, bus=1) for r in requests] if change == 'bus1' else []
  else:
    target, name = fw_versions, 'REQUESTS'
    changed = [(b, c, replace(r, bus=1) if b == 'gm' else r) for b, c, r in fw_versions.REQUESTS] if change == 'bus1' else [
      item for item in fw_versions.REQUESTS if item[0] != 'gm']
  with mock.patch.object(target, name, changed):
    assert not car_helpers._gm_fw_uses_normal_bus0()
    with mock.patch.object(car_helpers, 'normal_bus_query_brand', return_value=None) as probe, \
         mock.patch.object(car_helpers, 'get_vin', return_value=(0, 0, '00000000000000000')), \
         mock.patch.object(car_helpers, 'get_present_ecus', return_value=set()), \
         mock.patch.object(car_helpers, 'get_fw_versions_ordered', return_value=[]), \
         mock.patch.object(car_helpers, 'can_fingerprint', return_value=(None, {})), \
         mock.patch.dict(car_helpers.os.environ, {'FINGERPRINT': '', 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': ''}):
      car_helpers.fingerprint(lambda **kw: [], mock.Mock(), mock.Mock(), None)
    assert probe.call_args.kwargs['gm_probe'] is None


# Complete observed pre-query address/DLC set; payloads are not identity evidence.
RECORDED_LAYOUT = {188: 8,
 320: 3,
 211: 2,
 390: 7,
 352: 5,
 513: 6,
 485: 8,
 170: 8,
 190: 6,
 201: 8,
 417: 7,
 493: 8,
 419: 1,
 495: 4,
 451: 8,
 499: 3,
 452: 8,
 503: 1,
 453: 6,
 193: 8,
 197: 8,
 209: 7,
 454: 8,
 241: 6,
 456: 8,
 463: 3,
 508: 8,
 500: 6,
 532: 6,
 554: 3,
 568: 1,
 647: 3,
 717: 5,
 707: 8,
 872: 1,
 967: 4,
 298: 8,
 977: 8,
 979: 7,
 985: 5,
 988: 6,
 1001: 8,
 1221: 5,
 869: 4,
 189: 7,
 407: 7,
 479: 3,
 501: 8,
 1257: 6,
 546: 7,
 550: 8,
 558: 8,
 384: 4,
 489: 8,
 560: 8,
 562: 8,
 711: 6,
 309: 8,
 481: 7,
 1322: 6,
 381: 6,
 840: 5,
 842: 5,
 566: 6,
 989: 8,
 386: 8,
 311: 8,
 308: 4,
 1249: 8,
 1922: 7,
 761: 7,
 866: 4,
 313: 8,
 1300: 8,
 995: 7,
 528: 5,
 844: 8,
 1907: 7,
 810: 8,
 353: 3,
 564: 5,
 608: 8,
 609: 6,
 610: 6,
 611: 6,
 612: 8,
 1017: 8,
 613: 8,
 1019: 2,
 1328: 4,
 1020: 8,
 961: 8,
 322: 7,
 753: 5,
 497: 8,
 388: 8,
 1105: 6,
 969: 8,
 800: 6,
 1005: 6,
 1022: 1,
 1013: 3,
 567: 5,
 1009: 8,
 2020: 8,
 2028: 8,
 1217: 8,
 1227: 4,
 1223: 3,
 1233: 8,
 1275: 3,
 1243: 3,
 1225: 7,
 1913: 7,
 1187: 4,
 1927: 7,
 1265: 8,
 1906: 7,
 1905: 7,
 1904: 7,
 328: 1,
 1912: 7,
 573: 1,
 1280: 4,
 304: 1,
 565: 5,
 563: 5}


VIN = "1G1RC6S50HU000001"


class RecordedIsoTpEcu:
  """Actual query/UDS framing, inert CAN peer. No real bus or firmware claim."""
  def __init__(self, response_bus=0):
    self.clock = 0.
    self.frames = []
    self.pending = {}
    self.sent = []
    self.response_bus = response_bus

  def send(self, messages):
    self.sent.extend(messages)
    for message in messages:
      payload = message.dat
      if payload[0] >> 4 == 3:
        self.frames.extend(self.pending.pop(message.address, []))
        continue
      if payload[0] >> 4 != 0:
        continue
      request = payload[1:1 + payload[0]]
      if message.address in (0x7df, 0x7e0) and request == b'\x22\xf1\x90':
        self.reply(0x7e0, 0x7e8, b'\x62\xf1\x90' + VIN.encode())
      elif request == b'\x3e' and message.address < 0x700:
        self.reply(message.address, message.address + 0x400, b'\x7e')
      elif request[:1] == b'\x1a' and message.address < 0x700:
        self.reply(message.address, message.address + 0x400, b'\x5a' + request[1:2] + b'FW')

  def reply(self, tx, rx, value):
    if len(value) < 8:
      self.frames.append(CanData(rx, bytes([len(value)]) + value + bytes(7 - len(value)), self.response_bus))
    else:
      self.frames.append(CanData(rx, bytes([0x10 | len(value) >> 8, len(value) & 0xff]) + value[:6], self.response_bus))
      self.pending[tx] = [CanData(rx, bytes([0x20 | (i + 1) % 16]) + value[start:start + 7] + bytes(max(0, 7 - len(value[start:start + 7]))), self.response_bus)
                          for i, start in enumerate(range(6, len(value), 7))]

  def recv(self, wait_for_one=False):
    self.clock += .01
    values, self.frames = self.frames, []
    background = [CanData(address, bytes(size), bus) for bus in (0, 2) for address, size in RECORDED_LAYOUT.items()]
    return [values + background]


def test_actual_automatic_vin_firmware_and_can_fingerprint_stay_normal_bus0():
  peer = RecordedIsoTpEcu()
  callbacks = []
  with mock.patch.dict(car_helpers.os.environ, {'FINGERPRINT': '', 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': ''}), \
       mock.patch.object(car_helpers.time, 'monotonic', side_effect=lambda: peer.clock), \
       mock.patch.object(car_helpers.time, 'sleep', side_effect=lambda seconds: setattr(peer, 'clock', peer.clock + seconds)):
    result = car_helpers.fingerprint(peer.recv, peer.send, callbacks.append, None)
  assert result[0] == CAR.CHEVROLET_BOLT_CC_2017
  assert result[2] == VIN
  assert result[3] and all(fw.brand == 'gm' and fw.bus == 0 for fw in result[3])
  assert result[4] == car_helpers.CarParams.FingerprintSource.can
  assert callbacks and set(callbacks) == {False}
  assert peer.sent and all(frame.src == 0 for frame in peer.sent)
  assert any(frame.dat[0] >> 4 == 3 for frame in peer.sent)


def test_automatic_query_scope_preserves_actual_pedal_factory_safety_configuration():
  peer = RecordedIsoTpEcu()
  with mock.patch.dict(car_helpers.os.environ, {'FINGERPRINT': '', 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': ''}), \
       mock.patch.object(car_helpers.time, 'monotonic', side_effect=lambda: peer.clock), \
       mock.patch.object(car_helpers.time, 'sleep', side_effect=lambda seconds: setattr(peer, 'clock', peer.clock + seconds)):
    ci = car_helpers.get_car(peer.recv, peer.send, mock.Mock(), True, False)
  assert ci.CP.carFingerprint == CAR.CHEVROLET_BOLT_CC_2017
  assert ci.CP.openpilotLongitudinalControl
  assert ci.CP.safetyConfigs[0].safetyModel == car_helpers.CarParams.SafetyModel.gm
  assert ci.CP.safetyConfigs[0].safetyParam == 189
  # AOL is added by card's pre-create hook, not by diagnostic query selection.
  direct = car_helpers.interfaces[CAR.CHEVROLET_BOLT_CC_2017].get_params(
    CAR.CHEVROLET_BOLT_CC_2017, {0: RECORDED_LAYOUT, 1: {}, 2: RECORDED_LAYOUT}, [], True, False, docs=False)
  assert ci.CP.alternativeExperience == direct.alternativeExperience
  assert ci.CP.flags == direct.flags
  assert ci.CP.lateralTuning.to_dict() == direct.lateralTuning.to_dict()
