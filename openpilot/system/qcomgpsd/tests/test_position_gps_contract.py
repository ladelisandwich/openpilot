"""Real binary position decoder -> publisher branch -> serialized GPS -> consumers.

Root-only runtime candidate. AST extraction omits modem I/O, not the producer
assignments; the real schema supplies defaults and round-trips every GPS field.
"""
import ast
import datetime
import math
import struct
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from openpilot.cereal import messaging, log
from openpilot.system.qcomgpsd.structs import position_report, parse_struct, dict_unpacker
from openpilot.starpilot.gps.source import GPS_SOURCES, select_location
from openpilot.starpilot.gps.acquisition import GpsAcquisition
from openpilot.starpilot.navigation.runtime import location
from openpilot.starpilot.navigation.status import NavigationStatusSource
from openpilot.starpilot.ui.onroad_map import MapFeed


def binary_position(axis, *, source=2, week=2400, vdop=2.):
  fmt, names = parse_struct(position_report)
  values: dict[str, int | float] = dict.fromkeys(names, 0)
  values.update({'u_PosSource': source, 'w_GpsWeekNumber': week, 'q_GpsFixTimeMs': 10000,
                 't_DblFinalPosLatLon[0]': math.radians(40), 't_DblFinalPosLatLon[1]': math.radians(-90),
                 'q_FltFinalPosAlt': 200., 'q_FltHeadingRad': 1., 'q_FltHeadingUncRad': .1,
                 'q_FltVdop': vdop, 'q_FltEllipseSemimajorAxis': axis, 'q_FltEllipseSemiminorAxis': 2.,
                 'u_EllipseConfidence': 68, 'u_HorizontalReliability': 4,
                 'q_FltVelEnuMps[0]': 2., 'q_FltVelEnuMps[1]': 3.})
  return struct.pack(fmt, *[values[name] for name in names])


def publish_position(data):
  path = Path(__file__).parents[1] / 'qcomgpsd.py'
  branch = next(node for node in ast.walk(ast.parse(path.read_text())) if isinstance(node, ast.If) and
                ast.unparse(node.test) == 'log_type == LOG_GNSS_POSITION_REPORT')
  unpacker, _ = dict_unpacker(position_report)
  sent = []
  class Publisher:
    def send(self, service, event):
      assert service == 'gpsLocation'
      sent.append(event.to_bytes())
  namespace = {'math': math, 'datetime': datetime, 'unpack_position': unpacker, 'log_payload': data,
               'messaging': messaging, 'log': log, 'pm': Publisher()}
  loop = ast.For(target=ast.Name(id='_once', ctx=ast.Store()),
                 iter=ast.Tuple(elts=[ast.Constant(0)], ctx=ast.Load()), body=branch.body, orelse=[])
  exec(compile(ast.fix_missing_locations(ast.Module(body=[loop], type_ignores=[])), str(path), 'exec'), namespace)
  return sent


class Messages:
  def __init__(self, gps, stamp):
    services = [*GPS_SOURCES, 'starpilotNavigation', 'deviceState']
    self.valid = dict.fromkeys(services, False)
    self.seen = dict.fromkeys(services, False)
    self.alive = dict.fromkeys(services, False)
    self.logMonoTime = dict.fromkeys(services, 0)
    self.recv_time = dict.fromkeys(services, 0.)
    self.valid['gpsLocation'] = self.seen['gpsLocation'] = self.alive['gpsLocation'] = True
    self.logMonoTime['gpsLocation'] = stamp
    self.recv_time['gpsLocation'] = stamp / 1e9
    self.gps = gps
  def __getitem__(self, service):
    if service != 'gpsLocation':
      raise KeyError(service)
    return self.gps
  def update(self, timeout):
    pass


@pytest.mark.parametrize('axis,valid', [(4.25, True), (25., True), (0., False), (-1., False),
                                        (25.125, False), (float('nan'), False), (float('inf'), False)])
def test_modem_horizontal_metric_reaches_real_wire_and_all_shared_consumers(axis, valid):
  wire = publish_position(binary_position(axis))
  assert len(wire) == 1
  with log.Event.from_bytes(wire[0]) as event:
    gps = event.gpsLocation
    assert str(gps.source) == 'qcomdiag' and gps.hasFix
    if math.isfinite(axis):
      assert gps.horizontalAccuracy == pytest.approx(axis)
    sm = Messages(gps, event.logMonoTime)
    now = event.logMonoTime + 10_000_000
    assert (select_location(sm, now) is not None) == valid
    assert (location(sm, now) is not None) == valid
    empty_acquisition = SimpleNamespace(update=lambda timeout: None,
      updated={'gpsLocationExternal': False, 'qcomGnss': False}, valid={})
    feed = MapFeed(sm, acquisition=GpsAcquisition(sm_factory=lambda: empty_acquisition))
    with patch('openpilot.starpilot.ui.onroad_map.time.monotonic_ns', return_value=now), \
         patch('openpilot.starpilot.ui.onroad_map.time.monotonic', return_value=now / 1e9):
      assert (feed.read().fix is not None) == valid
    clock = [event.logMonoTime - 100_000_000]
    with patch('openpilot.cereal.messaging.SubMaster', return_value=sm):
      status = NavigationStatusSource(mono_clock=lambda: clock[0], boot_clock=lambda: clock[0])
    clock[0] = now
    assert (status.search_position() is not None) == valid
    # A consumer cannot use stale, future or wrong-source GPS.
    assert select_location(sm, event.logMonoTime + 2_500_000_001) is None
    assert select_location(sm, event.logMonoTime - 1) is None
    wrong_source = event.as_builder()
    wrong_source.gpsLocation.source = 'ublox'
    with log.Event.from_bytes(wrong_source.to_bytes()) as mismatched:
      assert select_location(Messages(mismatched.gpsLocation, mismatched.logMonoTime), now) is None


def test_report_rejections_and_modem_no_fix_sentinel_are_preserved():
  assert not publish_position(binary_position(4.25, source=0))
  assert not publish_position(binary_position(4.25, week=0xFFFF))
  wire = publish_position(binary_position(4.25, vdop=500.))
  with log.Event.from_bytes(wire[0]) as event:
    assert not event.gpsLocation.hasFix
    assert select_location(Messages(event.gpsLocation, event.logMonoTime), event.logMonoTime) is None
  unpacker, _ = dict_unpacker(position_report)
  with pytest.raises(struct.error):
    unpacker(binary_position(4.25)[:-1])


def test_valid_modem_fix_is_not_lost_due_to_default_accuracy():
  """Stable root before/after entry point; fail for the known producer omission."""
  wire = publish_position(binary_position(4.25))
  assert len(wire) == 1
  with log.Event.from_bytes(wire[0]) as event:
    assert event.gpsLocation.horizontalAccuracy > 0, 'qcomdiag omitted horizontalAccuracy'
  test_modem_horizontal_metric_reaches_real_wire_and_all_shared_consumers(4.25, True)
