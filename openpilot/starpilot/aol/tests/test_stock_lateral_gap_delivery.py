"""Characterize separate socket delivery after an accepted stock critical lag."""

from unittest import mock
import pytest
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE
from openpilot.starpilot.aol.tests.test_ordinary_source_availability import settings
from openpilot.starpilot.aol.tests.legacy_lag_test_helpers import legacy_stock_family as stock_family
from openpilot.starpilot.aol.tests.test_stock_lateral_pcm import receive_pair


@pytest.mark.parametrize('family', ['toyota', 'hkg_classic', 'hkg_canfd', 'honda'])
@pytest.mark.parametrize('order', ['proof_first', 'event_first', 'together'])
def test_matched_stock_lag_gap_socket_order_retains_only_intent(family, order):
  with stock_family(family) as (sd, controls, drive, card, receive):
    old, _, _ = drive(2, state_stamp=BASE, **settings('lateral_only', BASE, 3))
    assert old.lateralActive and (
      receive(BASE) if family == 'toyota' else receive(BASE, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    )
    feedback = card.aol_transport_feedback
    first, now = BASE + 10_000_000, BASE + 45_399_000
    with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=0.02):
      axis, _, _ = drive(3, state_stamp=first, **settings('lateral_only', first, 4))
      assert str(axis.faultReason) == 'critical' and (
        receive(first) if family == 'toyota' else receive(first, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
      )
      assert not feedback.failed and not feedback.pending_event and feedback.lateral_only_baseline
      assert feedback.signature[5] == 'critical' and not any(feedback.signature[9:14])
      baseline = (feedback.lateral_source_ns, feedback.lateral_baseline_ns)
      paused, state, command = drive(6, timeout=True, state_stamp=first, offset_ns=5_399_000, **settings('lateral_only', first, 4))
      assert str(paused.faultReason) == 'availabilityPause' and paused.sourceCarStateMonoTime == 0
      assert not state.enabled and not command.latActive and not command.longActive
      proof, event = sd.pm.messages['aolAxisState'], sd.pm.messages['onroadEvents']

      def receipt(at, **kwargs):
        return receive_pair(sd, controls, card, at, **kwargs) if family == 'toyota' else receive(at, **kwargs)

      if order == 'proof_first':
        assert receipt(now, axis=proof)
        assert not feedback.failed and feedback.retention_pending
        assert receipt(now + 1_000_000, event=event)
      elif order == 'event_first':
        assert receipt(now, event=event)
        assert not feedback.failed and feedback.retention_pending
        assert receipt(now + 1_000_000, axis=proof)
      else:
        assert receipt(now, axis=proof, event=event)
      assert not feedback.failed and feedback.retention_verified and not feedback.retention_started_ns
      assert (feedback.lateral_source_ns, feedback.lateral_baseline_ns) == baseline
      assert old.validUntilMonoTime == BASE + 30_000_000 and not sd.events.contains('enable')
      with mock.patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=old.validUntilMonoTime + 1):
        expired, _ = controls.state_control()
      assert not expired.latActive and not expired.longActive


@pytest.mark.parametrize('order', ['proof_first', 'event_first', 'together'])
def test_current_soft_stock_lag_and_source_gap_preserve_only_intent(order):
  from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import stock_family as current_stock_family

  with current_stock_family('toyota') as (sd, controls, drive, card, receive):
    old, _, command = drive(2, state_stamp=BASE, **settings('lateral_only', BASE, 3))
    assert old.lateralActive and command.latActive and receive(BASE)
    first, gap_time = BASE + 10_000_000, BASE + 45_399_000
    with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=0.02):
      soft, state, command = drive(3, state_stamp=first, **settings('lateral_only', first, 4))
      assert str(soft.faultReason) == 'availabilityPause'
      assert soft.sourceCarStateMonoTime == first and not state.enabled
      assert not soft.lateralActive and not soft.longitudinalActive
      assert not command.latActive and not command.longActive
      assert sd.aol_published_fault_proof is None and sd.aol_published_stock_lateral_proof is None
      assert receive(first) and not card.aol_transport_feedback.failed
      gap, _, command = drive(6, timeout=True, state_stamp=first, offset_ns=5_399_000,
                              **settings('lateral_only', first, 4))
      assert str(gap.faultReason) == 'availabilityPause' and gap.sourceCarStateMonoTime == 0
      assert not gap.lateralActive and not gap.longitudinalActive
      assert not command.latActive and not command.longActive
      proof, event = sd.pm.messages['aolAxisState'], sd.pm.messages['onroadEvents']
      if order == 'proof_first':
        assert receive_pair(sd, controls, card, gap_time, axis=proof)
        assert receive_pair(sd, controls, card, gap_time + 1_000_000, event=event)
      elif order == 'event_first':
        assert receive_pair(sd, controls, card, gap_time, event=event)
        assert receive_pair(sd, controls, card, gap_time + 1_000_000, axis=proof)
      else:
        assert receive_pair(sd, controls, card, gap_time, axis=proof, event=event)
      assert card.aol_card_intent.allowed_latch and not card.aol_transport_feedback.failed
    assert old.validUntilMonoTime == BASE + 30_000_000
    with mock.patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=old.validUntilMonoTime + 1):
      expired, _ = controls.state_control()
    assert not expired.latActive and not expired.longActive
    fresh = BASE + 50_000_000
    recovered, state, command = drive(7, state_stamp=fresh, **settings('lateral_only', fresh, 5))
    assert recovered.lateralActive and command.latActive and not command.longActive
    assert recovered.sourceCarStateMonoTime == fresh
    assert not state.enabled and not recovered.desiredLongitudinal and not sd.events.contains('enable')
