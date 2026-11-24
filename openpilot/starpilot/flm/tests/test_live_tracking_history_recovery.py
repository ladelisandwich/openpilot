"""Actual historical CAN maintenance never grants a live learning permission."""
from typing import Any

from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from openpilot.starpilot.flm.live_tracking_actor import LiveTrackingActor
from openpilot.starpilot.flm.tests.test_event_controller_prefix import fixture


@pytest.mark.parametrize('current_native', [False, True])
@pytest.mark.parametrize('source_available', [False, True])
def test_stalled_live_collection_still_decodes_original_native_history(current_native, source_available):
  replay = fixture('starpilot', two_windows=True).engine
  history = replay.emissions
  assert tuple(history.decode_queue) == tuple(history.events)
  assert history.events and history.decoder is None
  assert 0 < len(history.decode_queue) <= 64
  # Unscored bookkeeping sentinel only; no new physical/native evidence.
  last = replay.records[-1]
  replay.records.append(replace(last, control_ns=last.control_ns + 6_000_000_000))
  old_size = len(replay.records)
  cutoff = replay.records[-1].control_ns - 4_000_000_000
  stamp, _ = replay.checkpoints[0]
  actor: Any = LiveTrackingActor(source_tracking=True)
  actor.ingress = SimpleNamespace(interrupt=Mock())
  actor.certificate = SimpleNamespace(binding=SimpleNamespace(identity=replay.identity, baseline_revision=replay.baseline_revision))
  collector = SimpleNamespace(adapter=SimpleNamespace(replay=replay), ready=[])
  source = object() if source_available else None
  now = replay.records[-1].control_ns
  assert actor.tick(collector, source, now_ns=now, current_native=current_native) is None
  assert not history.decode_queue
  assert history.decoder is not None
  assert not history.decode_errors
  assert len(replay.records) < old_size
  assert replay.records[-1].control_ns == now and cutoff > stamp
  assert actor.application is None and actor.seed is None
  actor.ingress.interrupt.assert_called_once_with('current_source_or_native_permission_unavailable', preserve_completed=False)


def test_stalled_legacy_bootstrap_keeps_original_history_policy():
  replay = fixture('starpilot', two_windows=True).engine
  history = replay.emissions
  assert tuple(history.decode_queue) == tuple(history.events)
  actor: Any = LiveTrackingActor(source_tracking=False)
  collector = SimpleNamespace(adapter=SimpleNamespace(replay=replay), ready=[])
  before = tuple(history.decode_queue), tuple(replay.records)
  assert actor.tick(collector, None, now_ns=replay.records[-1].control_ns, current_native=False) is None
  assert (tuple(history.decode_queue), tuple(replay.records)) == before
  assert history.decoder is None
