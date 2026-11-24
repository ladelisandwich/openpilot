"""Captured packet batching must preserve READY evidence without weakening freshness."""
import json
from pathlib import Path
from unittest.mock import patch

import pytest

from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.ioniq6_handoff import (
  TimestampedCanPacket, _qualified_ioniq6_ready, inspect_ioniq6_long_sources,
)


def capture():
  return json.loads((Path(__file__).parent / 'testdata' / 'ioniq6_preflight_batched.json').read_text())


def samples():
  rows = []
  for packet in capture()['batches']:
    for address, payload, bus in packet['frames']:
      if address == 0x35 and bus == 1:
        data = bytes.fromhex(payload)
        valid = int.from_bytes(data[:2], 'little') == hkg_can_fd_checksum(address, None, bytearray(data))
        rows.append((packet['ns'], packet['ns'] / 1e9, data[2], bool(data[3] & 0x40), valid, packet['ns']))
  assert len(rows) >= 20
  assert any(a[0] == b[0] and (b[2] - a[2]) % 256 == 1 for a, b in zip(rows, rows[1:], strict=False))
  return rows


@pytest.mark.parametrize('ready', [False, True])
def test_captured_batched_packet_times_preserve_ready(ready):
  rows = [(stamp, receipt, counter, ready, crc, boot) for stamp, receipt, counter, _, crc, boot in samples()]
  assert _qualified_ioniq6_ready(rows, rows[-1][1]) is ready


def test_captured_batched_packet_counter_wrap_preserves_ready():
  rows = [(stamp, receipt, (250 + i) % 256, ready, crc, boot)
          for i, (stamp, receipt, _, ready, crc, boot) in enumerate(samples())]
  assert _qualified_ioniq6_ready(rows, rows[-1][1]) is False


def test_actual_preflight_retains_batched_not_ready_evidence():
  packets = capture()['batches']
  clock = [packets[0]['ns'] / 1e9]
  pending = iter(packets)

  def receive(wait_for_one=False):
    packet = next(pending, None)
    if packet is None:
      clock[0] += .001
      return []
    clock[0] = packet['ns'] / 1e9
    return [TimestampedCanPacket([CanData(addr, bytes.fromhex(data), bus) for addr, data, bus in packet['frames']], packet['ns'])]

  with patch('opendbc.car.hyundai.ioniq6_handoff._ioniq6_boot_ns', side_effect=lambda: round(clock[0] * 1e9)), \
       patch('opendbc.car.hyundai.ioniq6_handoff.time.sleep', return_value=None):
    result = inspect_ioniq6_long_sources(receive, lambda: clock[0])
  assert result.source_period is not None
  assert result.vehicle_ready is False


@pytest.mark.parametrize('corruption', ['crc', 'duplicate_counter', 'skipped_counter', 'mixed_ready', 'future', 'stale',
                                      'backwards', 'same_stamp_different_receipt', 'queued_burst', 'short_span', 'short_count'])
def test_batched_ready_keeps_independent_refusals(corruption):
  rows = [list(row) for row in samples()]
  index = next(i for i in range(1, len(rows)) if rows[i][0] == rows[i - 1][0])
  if corruption == 'crc':
    rows[index][4] = False
  elif corruption == 'duplicate_counter':
    rows[index][2] = rows[index - 1][2]
  elif corruption == 'skipped_counter':
    rows[index][2] = (rows[index - 1][2] + 2) % 256
  elif corruption == 'mixed_ready':
    rows[index][3] = True
  elif corruption == 'future':
    rows[index][5] = rows[index][0] - 1
  elif corruption == 'stale':
    rows[index][5] = rows[index][0] + 50_000_001
  elif corruption == 'backwards':
    rows[index][0] = rows[index - 1][0] - 1
  elif corruption == 'same_stamp_different_receipt':
    rows[index][1] += .001
  elif corruption == 'queued_burst':
    for row in rows:
      row[1] = rows[0][1]
  elif corruption == 'short_span':
    rows = rows[:6]
  elif corruption == 'short_count':
    rows = rows[:3]
  assert _qualified_ioniq6_ready(rows, rows[-1][1]) is None


def ready_frames():
  return [(packet['ns'], [(addr, bytes.fromhex(data), bus) for addr, data, bus in packet['frames'] if addr == 0x35 and bus == 1])
          for packet in capture()['batches'] if any(addr == 0x35 and bus == 1 for addr, _, bus in packet['frames'])]


def test_shared_pe_ev9_observer_accepts_consecutive_frames_in_one_captured_packet():
  from opendbc.car.hyundai.ioniq5pe_ready import PEReadySource

  packets = ready_frames()
  index = next(i for i, (_, frames) in enumerate(packets) if len(frames) > 1)
  observer = PEReadySource()
  for stamp, frames in packets[:index + 1]:
    observer.update([(stamp, frames)], stamp)
  stamp, _ = packets[index]
  assert observer.current(stamp, packets[0][0] - 1, 100_000_000) is False


@pytest.mark.parametrize('corruption', ['replay_packet', 'future_packet', 'bad_crc', 'duplicate_counter', 'stale_current'])
def test_shared_pe_ev9_observer_keeps_independent_refusals(corruption):
  from opendbc.car.hyundai.ioniq5pe_ready import PEReadySource

  packets = ready_frames()
  observer = PEReadySource()
  for stamp, frames in packets:
    observer.update([(stamp, frames)], stamp)
  stamp, frames = packets[-1]
  assert observer.current(stamp, packets[0][0] - 1, 100_000_000) is False
  if corruption == 'replay_packet':
    observer.update([(stamp, frames)], stamp + 1)
  elif corruption == 'future_packet':
    observer.update([(stamp + 10, frames)], stamp)
  elif corruption == 'stale_current':
    assert observer.current(stamp + 100_000_001, packets[0][0] - 1, 100_000_000) is None
    return
  else:
    address, payload, bus = frames[-1]
    data = bytearray(payload)
    if corruption == 'bad_crc':
      data[0] ^= 1
    else:
      counter = observer.last_counter
      assert isinstance(counter, int)
      data[2] = counter
      data[:2] = hkg_can_fd_checksum(address, None, data).to_bytes(2, 'little')
    observer.update([(stamp + 1, [(address, bytes(data), bus)])], stamp + 1)
  assert observer.current(stamp + 1, packets[0][0] - 1, 100_000_000) is None
