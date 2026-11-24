"""Actual optional publisher must send a freshly observed settings receipt."""
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.learning_wire import read_source_receipt
from openpilot.starpilot.flm.runtime_adapter_registry import SourceState

BASE = 1_000_000_000
CP = 'b' * 64


def publication(*, fresh, changes=None, snapshot_cost=0):
  now = [BASE + 330_000_000]  # Actual route: published source age reached330ms.
  old = SourceState(True, False, '0', 'a' * 64, BASE, BASE + 500_000_000, CP, (0., 0., 0.), 'c' * 32)
  sm, publisher, fitter = mock.Mock(updated={'deviceMotion': False}, logMonoTime={'deviceMotion': 0}), mock.Mock(), mock.Mock(child=None)
  fitter.poll.return_value = None
  sent = []

  def refresh(stamp):
    result = replace(old, observed_ns=stamp, expires_ns=stamp + 500_000_000, **(changes or {}))
    now[0] += snapshot_cost
    return result

  def send(service, message):
    assert service == 'autoSteeringLearning'
    sent.append(messaging.log_from_bytes(message.to_bytes()))
    raise StopIteration

  publisher.send.side_effect = send
  with mock.patch.object(steerlearnerd, 'drain_pair', return_value=None):
    with pytest.raises(StopIteration):
      steerlearnerd.run(SimpleNamespace(), sm, object(), object(), publisher,
        mock.Mock(), mock.Mock(), mock.Mock(return_value=old), mock.Mock(), mock.Mock(),
        mock.Mock(), mock.Mock(), fitter, mock.Mock(), clock=lambda: now[0],
        publication_source_reader=refresh if fresh else None)
  fitter.close.assert_called_once_with('learner_parent_exit')
  return sent[0], now[0], fitter


def test_actual_parent_old_receipt_expires_before_next_normal_publication():
  old, _, _ = publication(fresh=False)
  fresh, now, _ = publication(fresh=True)
  next_publication = now + 300_000_000  # Route median281ms, p95325ms.
  assert read_source_receipt(old, now_ns=next_publication, cp_sha256=CP) is None
  current = read_source_receipt(fresh, now_ns=next_publication, cp_sha256=CP)
  assert current is not None and current.observed_ns == now
  assert current.expires_ns == current.observed_ns + 500_000_000
  assert fresh.logMonoTime >= current.observed_ns
  assert fresh.autoSteeringLearning.learning.runtimeProfile.version == 0


@pytest.mark.parametrize('changes', [
  {'enabled': False, 'source_epoch': 'd' * 64},
  {'manual': True, 'source_epoch': 'd' * 64},
  {'reset': '1', 'source_epoch': 'd' * 64},
  {'calibration_generation': 'e' * 32, 'source_epoch': 'd' * 64},
  {'cp_sha256': 'f' * 64},
])
def test_publication_refresh_preserves_real_identity_manual_reset_rejections(changes):
  message, _, fitter = publication(fresh=True, changes=changes)
  learning = message.autoSteeringLearning.learning
  assert learning.status == 'unavailable'
  assert learning.reason == 'publication_source_changed_or_unavailable'
  assert learning.runtimeProfile.version == 0
  fitter.cancel.assert_called()


def test_actual_settings_read_cannot_extend_a_receipt_it_outlives():
  message, now, _ = publication(fresh=True, snapshot_cost=500_000_001)
  assert message.autoSteeringLearning.learning.status == 'unavailable'
  assert read_source_receipt(message, now_ns=now, cp_sha256=CP) is None
  source = message.autoSteeringLearning.learning.sourceReceipt
  assert source.validUntilMonoTime == source.observedMonoTime + 500_000_000
