"""Real serialized car fixes must reach the standard, singly owned GPS channel."""

from types import SimpleNamespace
from unittest.mock import patch

import pytest

from openpilot.cereal import messaging, log
from openpilot.starpilot.gps.car_external import CarGpsExternal
from openpilot.starpilot.gps.source import GPS_MAX_AGE_NS

NOW = 100_000_000_000


class Messages:
  def __init__(self, *, stamp=NOW - 100_000_000):
    self.event = messaging.new_message('starpilotCarState', valid=True, logMonoTime=NOW)
    self.event.starpilotCarState.gps = {
      'sourceMonoTime': stamp,
      'latitude': 41.0,
      'longitude': -90.0,
      'altitude': 200.0,
      'speed': 12.0,
      'bearingDeg': 90.0,
      'horizontalAccuracy': 5.0,
      'verticalAccuracy': 10.0,
      'bearingAccuracyDeg': 10.0,
      'speedAccuracy': 0.5,
      'unixTimestampMillis': 1_700_000_000_000,
      'hasFix': True,
      'vNED': [0.0, 12.0, 0.0],
    }
    self.valid = {'starpilotCarState': True}
    self.logMonoTime = {'starpilotCarState': NOW}

  def __getitem__(self, service):
    assert service == 'starpilotCarState'
    return self.event.starpilotCarState

  def update(self, timeout):
    assert timeout == 0


def native(*, fixed=False):
  event = messaging.new_message('gpsLocationExternal', valid=True, logMonoTime=NOW)
  event.gpsLocationExternal = {'source': 'ublox', 'hasFix': fixed, 'latitude': 40.0, 'longitude': -80.0, 'speed': 3.0}
  return event


def test_car_fallback_keeps_true_source_units_fix_time_and_standard_wire():
  sm, owner = Messages(), CarGpsExternal()
  event = owner.select(sm, NOW, native())
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    assert decoded.valid and decoded.logMonoTime == sm['starpilotCarState'].gps.sourceMonoTime
    gps = decoded.gpsLocationExternal
    assert str(gps.source) == 'car' and gps.hasFix and gps.satelliteCount == 0
    for name in (
      'latitude',
      'longitude',
      'altitude',
      'speed',
      'bearingDeg',
      'horizontalAccuracy',
      'verticalAccuracy',
      'bearingAccuracyDeg',
      'speedAccuracy',
      'unixTimestampMillis',
    ):
      assert getattr(gps, name) == getattr(sm['starpilotCarState'].gps, name)
    assert list(gps.vNED) == [0.0, 12.0, 0.0]
  assert owner.select(sm, NOW + 1) is None
  assert owner.select(sm, NOW + 1, native()) is None
  repeated = owner.select(sm, NOW + 100_000_000)
  assert repeated.logMonoTime == event.logMonoTime
  # Polling cannot rejuvenate the old observation.
  assert owner.select(sm, NOW + GPS_MAX_AGE_NS) is None
  assert not owner.select(sm, NOW + GPS_MAX_AGE_NS, native()).gpsLocationExternal.hasFix


def test_native_fix_priority_loss_recovery_and_stale_native_fallback():
  sm, owner, good = Messages(), CarGpsExternal(), native(fixed=True)
  assert owner.select(sm, NOW, good) is good
  assert owner.select(sm, NOW + 1) is None
  assert str(owner.select(sm, NOW + 1, native()).gpsLocationExternal.source) == 'car'
  assert owner.select(sm, NOW + 2, good) is good
  sm.event.starpilotCarState.gps.sourceMonoTime = NOW + GPS_MAX_AGE_NS
  sm.logMonoTime['starpilotCarState'] = NOW + GPS_MAX_AGE_NS
  assert str(owner.select(sm, NOW + GPS_MAX_AGE_NS + 1).gpsLocationExternal.source) == 'car'


@pytest.mark.parametrize('fault', ['invalid', 'noFix', 'future', 'stale', 'accuracy', 'nanMotion', 'badVelocity'])
def test_bad_car_fix_never_replaces_a_native_observation(fault):
  sm, owner, absent = Messages(), CarGpsExternal(), native()
  gps = sm.event.starpilotCarState.gps
  if fault == 'invalid':
    sm.valid['starpilotCarState'] = False
  elif fault == 'noFix':
    gps.hasFix = False
  elif fault == 'future':
    gps.sourceMonoTime = NOW + 1
  elif fault == 'stale':
    gps.sourceMonoTime = NOW - GPS_MAX_AGE_NS - 1
  elif fault == 'accuracy':
    gps.horizontalAccuracy = 500.0
  elif fault == 'nanMotion':
    gps.speed = float('nan')
  else:
    gps.vNED = [0.0, float('inf'), 0.0]
  assert owner.select(sm, NOW, absent) is absent
  assert owner.select(sm, NOW) is None


def test_actual_ubloxd_loop_publishes_car_gps_even_without_ubx_bytes():
  from openpilot.system.ubloxd import ubloxd

  sm, sent, calls = Messages(), [], [0]

  class StopLoop(Exception):
    pass

  def receive(_sock):
    calls[0] += 1
    if sent or calls[0] > 1:
      raise StopLoop
    return None

  publisher = SimpleNamespace(send=lambda service, event: sent.append((service, event if isinstance(event, bytes) else event.to_bytes())))
  with (
    patch.object(ubloxd.messaging, 'SubMaster', return_value=sm),
    patch.object(ubloxd.messaging, 'PubMaster', return_value=publisher) as pm,
    patch.object(ubloxd.messaging, 'sub_sock'),
    patch.object(ubloxd.messaging, 'recv_one', side_effect=receive),
    patch.object(ubloxd, 'Params'),
    patch.object(ubloxd.time, 'monotonic_ns', return_value=NOW),
  ):
    with pytest.raises(StopLoop):
      ubloxd.main()
  pm.assert_called_once_with(['ubloxGnss', 'gpsLocationExternal'])
  assert len(sent) == 1 and sent[0][0] == 'gpsLocationExternal'
  with log.Event.from_bytes(sent[0][1]) as event:
    assert event.gpsLocationExternal.hasFix and str(event.gpsLocationExternal.source) == 'car'
