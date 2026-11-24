"""Finite actual parser/receiver negatives; never an upstream struct override."""
import pytest

from opendbc.can import CANPacker
from opendbc.car.tesla.tests.test_preap_radar import frames, radar_params, radar_interface
from openpilot.cereal import log, messaging


@pytest.mark.parametrize('mount', (False, True))
@pytest.mark.parametrize('fault', ('wrong_bus', 0, 1, 7, 9, 12, 16, 64))
def test_bad_receiver_frames_do_not_refresh_bosch_cycle_or_hide_expiry(mount, fault):
  cp = radar_params(mount=mount)
  ri = radar_interface(cp, 0.25)
  packer = CANPacker('tesla_radar_bosch_generated')
  ri.update([])
  for cycle in range(1, 5):
    rr = ri.update([(cycle * 125_000_000, frames(packer))])
  assert len(rr.points) == 1 and rr.points[0].yRel == 1.25
  before = {a: dict(ri.preap.rcp.ts_nanos[a]) for a in ri.preap.radar_message_addresses}
  bad = [(a, d, 2) if fault == 'wrong_bus' else (a, d[:fault] if fault < 8 else d + bytes(fault - 8), b)
         for a, d, b in frames(packer)]
  for cycle in range(5, 17):
    assert ri.update([(cycle * 125_000_000, bad)]) is None
  # Original decoder publishes only on the final B trigger. Do not invent an
  # immediate error publication or immediate cached-point retirement.
  trigger = packer.make_can_msg('RadarPoint31_B', 1, {'Index2': 0})
  assert {a: dict(ri.preap.rcp.ts_nanos[a]) for a in ri.preap.radar_message_addresses} == before
  # Honor the current parser's five-invalid-read debounce.
  for step in range(5):
    rr = ri.update([(2_125_000_000 + step * 125_000_000, [trigger])])
  assert rr is not None and rr.errors.canError
  event = messaging.new_message('radarTracks', valid=False)
  event.radarTracks = rr
  with log.Event.from_bytes(event.to_bytes()) as received:
    assert received.which() == 'radarTracks' and not received.valid
    assert received.radarTracks.errors.canError


@pytest.mark.parametrize('mount', (False, True))
def test_genuine_eight_byte_receiver_and_unrelated_frames_are_preserved(mount, monkeypatch):
  ri = radar_interface(radar_params(mount=mount), 0.25)
  packer = CANPacker('tesla_radar_bosch_generated')
  actual_parser = ri.preap.rcp.update
  observed = []

  def inspect_then_parse(packets):
    observed.extend(packets)
    return actual_parser(packets)

  monkeypatch.setattr(ri.preap.rcp, 'update', inspect_then_parse)
  data = frames(packer) + [(0x310, bytes(12), 2), (0x700, bytes(64), 1), (0x301, bytes(3), 0)]
  for cycle in range(1, 5):
    packet = (cycle * 125_000_000, data)
    rr = ri.update([packet])
    assert observed[-1] == packet
    assert all(before[1] is after[1] for before, after in zip(data, observed[-1][1], strict=True))
  assert not rr.errors.canError and len(rr.points) == 1 and rr.points[0].yRel == 1.25


@pytest.mark.parametrize('mount', (False, True))
@pytest.mark.parametrize('missing', ('a', 'b', 'both', 'short_a', 'short_b', 'trigger_only'))
def test_current_pair_is_required_before_republishing_a_point(mount, missing):
  ri = radar_interface(radar_params(mount=mount), 0.25)
  packer = CANPacker('tesla_radar_bosch_generated')
  for cycle in range(1, 5):
    rr = ri.update([(cycle * 125_000_000, frames(packer))])
  assert len(rr.points) == 1
  previous_id = rr.points[0].trackId
  batch = frames(packer)
  omitted = {'a': (784,), 'b': (785,), 'both': (784, 785),
             'short_a': (784,), 'short_b': (785,)}.get(missing, ())
  if missing == 'trigger_only':
    batch = [batch[-1]]
  elif missing.startswith('short'):
    batch = [(a, d[:-1] if a in omitted else d, b) for a, d, b in batch]
  else:
    batch = [frame for frame in batch if frame[0] not in omitted]
  rr = ri.update([(625_000_000, batch)])
  assert not rr.points
  rr = ri.update([(750_000_000, frames(packer))])
  assert len(rr.points) == 1 and rr.points[0].trackId != previous_id
