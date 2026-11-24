"""Actual VIN/ISO-TP/FW/fingerprint/factory with deterministic CAN transport."""
from dataclasses import replace
from unittest import mock

import pytest

from opendbc.car import car_helpers, fw_versions, structs, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.ford.discovery import normal_bus0_queries
from opendbc.car.ford.values import CAR, FordFlags
from opendbc.car.hyundai.values import CAR as HYUNDAI
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.vehicle_selection import VehicleSelectionOwner, startup_candidate

CANFD_CARS = tuple(car for car in CAR if car.config.flags & FordFlags.CANFD and not car.config.flags & FordFlags.LKA_STEERING)
VIN = '3FMTK1SS0MMA00001'


class FordEcus:
  """Transport only: the real ISO-TP layer assembles VIN and firmware replies."""
  def __init__(self, *, firmware=None, vin=VIN, background=None, offset=0):
    self.offset = offset
    self.clock = 100.
    self.vin = vin
    self.versions = firmware if firmware is not None else {(0x706, b'\x22\xf1\x88'): b'RJ6T-14H102-BAE',
      (0x730, b'\x22\xf1\x88'): b'LJ9C-14D003-GA', (0x760, b'\x22\xf1\x88'): b'LK9C-2D053-CN',
      (0x7E0, b'\x22\xf1\x88'): b'MJ98-14C204-BYG'}
    self.background = background or [CanData(0x5A, bytes(8), offset), CanData(0x3D6, bytes(8), offset + 2),
                                     CanData(0x186, bytes(8), offset + 2)]
    self.queued = []
    self.pending = {}
    self.mux_calls = []
    self.sent = []

  def sleep(self, seconds):
    self.clock += seconds

  def mux(self, enabled):
    self.mux_calls.append(enabled)
    assert not enabled, 'Ford discovery switched CAN2 onto the OBD network'

  def reply(self, address, payload, *, bus=None):
    bus = self.offset if bus is None else bus
    if len(payload) < 8:
      frames = [bytes([len(payload)]) + payload]
    else:
      frames = [bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6]]
      frames += [bytes([0x20 | (index & 15)]) + payload[offset:offset + 7]
                 for index, offset in enumerate(range(6, len(payload), 7), 1)]
    self.queued.append(CanData(address + 8, frames[0].ljust(8, b'\0'), bus))
    self.pending[address, bus] = [CanData(address + 8, data.ljust(8, b'\0'), bus) for data in frames[1:]]

  def send(self, frames):
    self.sent.extend(frames)
    for address, data, bus in frames:
      if data[0] >> 4 == 3:
        self.queued.extend(self.pending.pop((address, bus), []))
      elif data[0] >> 4 == 0:
        request = bytes(data[1:1 + data[0]])
        if bus == 0 and address == 0x7DF and request == b'\x22\xf1\x90':
          self.reply(0x7E0, b'\x62\xf1\x90' + self.vin.encode(), bus=0)
        elif bus == self.offset and request == b'\x3e\x00' and any(tx == address for tx, _ in self.versions):
          self.reply(address, b'\x7e\x00')
        elif bus == self.offset and (address, request) in self.versions:
          self.reply(address, bytes([request[0] + 0x40]) + request[1:] + self.versions[address, request])

  def recv(self, wait_for_one=False):
    self.clock += .01
    queued, self.queued = self.queued, []
    return [self.background + queued] if wait_for_one else []


def discover(transport, candidate, *, cached=None):
  with OpenpilotPrefix():
    saved = Params()
    selection = VehicleSelectionOwner(saved, lambda: True)
    assert selection.choose(selection.snapshot().raw, str(candidate)).verified
    with mock.patch.dict(car_helpers.os.environ, {'FINGERPRINT': '', 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': ''}), \
         mock.patch.object(car_helpers.time, 'monotonic', side_effect=lambda: transport.clock), \
         mock.patch.object(car_helpers.time, 'sleep', side_effect=transport.sleep):
      ci = car_helpers.get_car(transport.recv, transport.send, transport.mux, False, False,
                               cached_params=cached, forced_candidate=startup_candidate(saved))
    return ci


@pytest.mark.parametrize('candidate', CANFD_CARS)
@pytest.mark.parametrize('offset', (0, 4))
def test_registered_ford_uses_real_normal_bus_vin_fw_can_and_factory(candidate, offset):
  transport = FordEcus(offset=offset)
  ci = discover(transport, candidate)
  assert transport.mux_calls and set(transport.mux_calls) == {False}
  assert ci.CP.carFingerprint == candidate and ci.CP.carVin == VIN
  assert ci.CP.fingerprintSource == structs.CarParams.FingerprintSource.fixed
  assert ci.CP.flags & FordFlags.CANFD
  assert ci.CP.safetyConfigs[-1].safetyModel == structs.CarParams.SafetyModel.ford
  assert len(ci.CP.carFw) == len(transport.versions)
  for fw in ci.CP.carFw:
    assert fw.brand == 'ford' and fw.bus == offset
    assert fw.fwVersion == transport.versions[fw.address, b'\x22\xf1\x88']
  assert all(frame.src in (0, 4) for frame in transport.sent)
  assert len(ci.CP.safetyConfigs) == (2 if offset else 1)
  if offset:
    assert ci.CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput
    assert ci.CP.safetyConfigs[0].safetyParam == 0
  assert any(frame.dat[0] >> 4 == 3 for frame in transport.sent), 'Real ISO-TP flow control was not exercised'


def test_matching_cache_still_uses_live_can_without_new_diagnostic_queries():
  firmware = structs.CarParams.CarFw(brand='ford', bus=0, address=0x706, fwVersion=b'RJ6T-14H102-BAE')
  cached = structs.CarParams(brand='ford', carFingerprint=CAR.FORD_MUSTANG_MACH_E_MK1, carVin=VIN, carFw=[firmware])
  transport = FordEcus()
  ci = discover(transport, CAR.FORD_MUSTANG_MACH_E_MK1, cached=cached)
  assert ci.CP.carVin == VIN and not transport.sent and transport.mux_calls == [False]
  assert ci.CP.flags & FordFlags.CANFD


def test_wrong_brand_cache_cannot_replace_live_ford_responses():
  cached = structs.CarParams(brand='mazda', carFingerprint=CAR.FORD_MUSTANG_MACH_E_MK1, carVin=VIN,
                            carFw=[structs.CarParams.CarFw(brand='mazda', fwVersion=b'wrong')])
  transport = FordEcus()
  ci = discover(transport, CAR.FORD_MUSTANG_MACH_E_MK1, cached=cached)
  assert transport.sent and ci.CP.carFw and all(f.brand == 'ford' for f in ci.CP.carFw)


@pytest.mark.parametrize('drift', [
  'essential_bus1', 'unique_logging_bus1', 'empty', 'effective_only', 'foreign_auxiliary',
  'unique_auxiliary', 'missing_auxiliary', 'effective_missing_auxiliary',
])
def test_query_topology_drift_refuses_direct_scoped_discovery_before_io(drift):
  original = fw_versions.FW_QUERY_CONFIGS['ford']
  changed = list(original.requests)
  if drift == 'essential_bus1':
    changed = [replace(request, bus=1) if not request.logging else request for request in changed]
  elif drift == 'unique_logging_bus1':
    changed.append(replace(changed[0], request=[changed[0].request[0], b'\x22\xf1\xff']))
  elif drift == 'empty':
    changed = []
  elif drift == 'missing_auxiliary':
    changed = [request for request in changed if request.bus != 4]
  elif drift == 'foreign_auxiliary':
    changed.append(replace(changed[1], bus=8))
  elif drift == 'unique_auxiliary':
    changed.append(replace(changed[1], bus=4, request=[changed[1].request[0], b'\x22\xf1\xff']))
  effective = [('ford', original, replace(original.requests[1], bus=1))] if drift == 'effective_only' else fw_versions.REQUESTS
  if drift == 'effective_missing_auxiliary':
    effective = [(brand, owner, request) for brand, owner, request in effective if brand != 'ford' or request.bus != 4]
  config = replace(original, requests=changed, extra_ecus=[])
  with mock.patch.dict(fw_versions.FW_QUERY_CONFIGS, {'ford': config}), mock.patch.object(fw_versions, 'REQUESTS', effective):
    assert not normal_bus0_queries()
    transport = FordEcus()
    with pytest.raises(ValueError, match='topology'):
      car_helpers.fingerprint(transport.recv, transport.send, transport.mux, None, known_ford_bus0=True)
    assert not transport.mux_calls and not transport.sent


@pytest.mark.parametrize('candidate', [None, CAR.FORD_ESCAPE_MK4, HYUNDAI.HYUNDAI_IONIQ_6])
def test_automatic_classic_and_other_brand_discovery_are_unchanged(candidate):
  layout = gen_empty_fingerprint()
  layout[0][0x5A] = 8
  layout[2][0x3D6] = 8
  result = (CAR.FORD_MUSTANG_MACH_E_MK1, layout, VIN, [], structs.CarParams.FingerprintSource.can, True)
  with OpenpilotPrefix(), mock.patch.object(car_helpers, 'fingerprint', return_value=result) as fp:
    car_helpers.get_car(mock.Mock(), mock.Mock(), mock.Mock(), False, False, forced_candidate=candidate)
  assert fp.call_args.kwargs == {}
