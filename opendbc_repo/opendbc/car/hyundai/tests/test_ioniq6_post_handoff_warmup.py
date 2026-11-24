"""Recorded parser grace must not complete a new I6 ECU ownership warmup."""
from collections import deque
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.ioniq6_handoff import build_ioniq6_hda2_long_candidate
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.card import Car


def recorded_owner():
  fp = gen_empty_fingerprint()
  fp[0][0x3A5] = 24
  fp[2].update({0x110: 32, 0x362: 32})
  fp[1].update({0x130: 16, 0x1CF: 8, 0x35: 32, 0x175: 24, 0xA0: 24, 0xEA: 24, 0x1BA: 24, 0x1E5: 16, 0x36A: 16})
  stock = CarInterface.get_params(CAR.HYUNDAI_IONIQ_6, fp, [], False, False, False)
  cp = build_ioniq6_hda2_long_candidate(stock, fp)
  assert cp is not None
  cp.safetyConfigs[-1].safetyParam |= 0x0800
  assert cp.flags == 1059329 and cp.safetyConfigs[-1].safetyParam == 0x8895
  owner = Car.__new__(Car)
  owner.CP, owner.CI = cp, CarInterface(cp)
  owner.ioniq6_long_selected = owner.ioniq6_long_prearmed = True
  owner.ioniq6_long_lost = owner.ioniq6_host_warmed = False
  owner.ioniq6_keepalive = SimpleNamespace(stop=Mock())
  return owner


def test_recorded_c8_actual_parser_grace_waits_for_post_handoff_sources():
  fixture = json.loads((Path(__file__).parent / "testdata/ioniq6_post_handoff_warmup.json").read_text())
  owner = recorded_owner()
  floor = fixture["handoffCompletedNs"]
  owner.ioniq6_handoff_source_floor_ns = floor
  now = [0]
  history = []
  with patch("time.clock_gettime_ns", side_effect=lambda clock: now[0]):
    for packet in fixture["packets"]:
      ns = packet["ns"]
      if fixture["lastCardSourceBeforeTransactionNs"] < ns <= floor:
        continue  # The real UDS transaction consumed this Card cursor interval.
      now[0] = ns
      frames = [(a, bytes.fromhex(data), bus) for a, data, bus in packet["frames"]]
      state = owner.CI.update([(ns, frames)])
      if ns <= floor:
        continue
      parser_valid = state.canValid
      owner.observe_ioniq6_long_authority([(ns, frames)], state)
      history.append((ns, parser_valid, owner.ioniq6_host_warmed, owner.ioniq6_long_lost))
  assert history[0][1] is True, "Recorded parser initially tolerates stale sources"
  assert history[0][2] is False, "STALE_PARSER_GRACE_MUST_NOT_FINISH_HANDOFF_WARMUP"
  assert any(not row[1] for row in history), "Real parser grace eventually expires"
  assert history[-1][1:3] == (True, True)
  assert not any(row[3] for row in history)
  owner.ioniq6_keepalive.stop.assert_not_called()
  # A genuinely invalid input after completed warmup retains the old refusal.
  invalid = SimpleNamespace(canValid=False)
  owner.observe_ioniq6_long_authority([], invalid)
  assert owner.ioniq6_long_lost and not invalid.canValid
  owner.ioniq6_keepalive.stop.assert_called_once()


@pytest.mark.parametrize("fault", ("no_floor", "empty_inventory", "pre_handoff", "future", "expired", "bad_counter", "missing"))
def test_warmup_proof_rejects_missing_or_invalid_source_receipts(fault):
  owner = recorded_owner()
  owner.ioniq6_handoff_source_floor_ns = 1_000_000_000
  now = 1_100_000_000
  required = [source for parser in owner.CI.can_parsers.values()
              for source in parser.message_states.values() if not source.ignore_alive]
  assert len(required) >= 6
  for source in required:
    source.timestamps = deque([now])
  source = required[-1]
  if fault == "no_floor":
    owner.ioniq6_handoff_source_floor_ns = 0
  elif fault == "empty_inventory":
    owner.CI.can_parsers = {}
  elif fault == "pre_handoff":
    source.timestamps = deque([owner.ioniq6_handoff_source_floor_ns])
  elif fault == "future":
    source.timestamps = deque([now + 1])
  elif fault == "expired":
    source.timeout_threshold = 10_000_000
    source.timestamps = deque([now - 10_000_001])
  elif fault == "bad_counter":
    source.counter_fail = 5
  elif fault == "missing":
    source.timestamps.clear()
  with patch("time.clock_gettime_ns", return_value=now):
    owner.observe_ioniq6_long_authority([], SimpleNamespace(canValid=True))
  assert not owner.ioniq6_host_warmed and not owner.ioniq6_long_lost


@pytest.mark.parametrize("address,bus", ((0x1A0, 1), (0x1BA, 1), (0x1E5, 1), (0x100, 0)))
def test_factory_source_return_still_revokes_during_warmup(address, bus):
  owner = recorded_owner()
  owner.ioniq6_handoff_source_floor_ns = 1_000_000_000
  state = SimpleNamespace(canValid=True)
  owner.observe_ioniq6_long_authority([(1_100_000_000, [(address, bytes(32), bus)])], state)
  assert owner.ioniq6_long_lost and not state.canValid
  owner.ioniq6_keepalive.stop.assert_called_once()
