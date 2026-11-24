"""Numerical state-machine regressions; no source/native/road authority."""

from dataclasses import replace
import unittest
from openpilot.starpilot.flm.tests.test_tracking_source_context import owner, refresh
from openpilot.starpilot.flm.tests.test_live_tracking import episode
from openpilot.starpilot.flm.live_tracking import _demand


def pending_recovery():
  o = owner()
  old = replace(episode(o, 0, 'ff_gain_left'), normalization_accel=0.5)
  o.selector_hint = {
    'group': 'ff.left',
    'name': 'ff_gain_left',
    'delta': 0.005,
    'demand': _demand(old),
    'binding': o.binding,
    'profile_sha256': o._hash(o.accepted),
    'model_revision': o.certificate.model_revision,
    'controller_domain': o.domain,
    'selected_ns': old.end_ns,
    'expires_ns': old.end_ns + 600_000_000_000,
    'source_episode_ids': (old.episode_id,),
    'quota': 1,
    'minimum_episodes': 1,
  }
  o.selector_pending = True
  o.selector_after_ns = old.end_ns
  return o, old


class SourceRecoveryProgress(unittest.TestCase):
  def test_old_left_hint_does_not_swallow_new_right_windows(self):
    o, old = pending_recovery()
    discovery = set()
    for i in range(2, 7):
      e = replace(episode(o, i, 'ff_gain_right'), normalization_accel=0.5)
      refresh(o, e.end_ns)
      o.observe_episode(e, now_ns=e.end_ns)
      if i < 4:
        discovery.add(e.episode_id)
    assert o.offer_state is not None and o.offer_state['names'] == ('ff_gain_right',)
    assert len(o.before) == 3 and discovery.isdisjoint(e.episode_id for e in o.before)
    assert not any(o.accepted.values())

  def test_missing_old_native_effect_does_not_hide_another_hook(self):
    o, old = pending_recovery()
    for i in range(2, 7):
      e = replace(episode(o, i, 'center_taper_max', kind='large'), normalization_accel=0.5)
      refresh(o, e.end_ns)
      o.observe_episode(e, now_ns=e.end_ns)
    assert o.offer_state is not None and o.offer_state['names'] == ('center_taper_max',)
    assert len(o.before) == 3 and not any(o.accepted.values())

  def test_missing_response_still_cannot_resume_or_launch(self):
    o, old = pending_recovery()
    e = replace(episode(o, 2, 'ff_gain_left'), normalization_accel=0.5)
    o.response_witness = None
    assert o._revalidate_selector(e, e.end_ns)
    assert o.discovery is None and o.offer_state is None
    assert o.reason == 'fresh_selector_response_required'


class ReceiptClockConversion(unittest.TestCase):
  def capture(self, publication, receipt, control):
    from types import SimpleNamespace as NS
    from openpilot.starpilot.flm.feedback_observation import capture_controls

    class SM(dict):
      pass

    output = NS(torque=0.0, to_dict=lambda: {'torque': 0.0})
    sm = SM(carOutput=NS(actuatorsOutput=output))
    sm.logMonoTime = {'carOutput': publication}
    sm.recv_frame = {'carOutput': 1}
    sm.recv_time = {'carOutput': receipt / 1e9}
    sm.seen = {'carOutput': True}
    sm.valid = {'carOutput': True}
    source = NS(enabled=True, manual=False, observed_ns=control - 1, expires_ns=control + 100_000_000, source_epoch='a' * 64, cp_sha256='b' * 64)
    obj = NS(sm=sm, CP=NS(steerControlType='torque'), global_learning=NS(eligible=True, source=source), steer_limited_by_safety=False)
    cc = NS(actuators=NS(torque=0.0), latActive=True)
    return capture_controls(obj, cc, control_ns=control, input_limited=False)

  def test_actual_capture_preserves_receipt_order_across_uptime_boundary(self):
    stamp = 1_024_000_000_052
    assert int(stamp / 1e9 * 1e9) == stamp - 1  # Demonstrated original producer defect.
    row = self.capture(stamp, stamp, stamp + 1_000_000)
    assert row.receipt_ns == stamp and row.publication_ns == stamp

  def test_stale_and_future_receipts_are_still_rejected(self):
    from openpilot.starpilot.flm.feedback_observation import MAX_AGE_NS

    stamp = 1_024_000_000_052
    with self.assertRaises(ValueError):
      self.capture(stamp, stamp, stamp + MAX_AGE_NS + 1)
    with self.assertRaises(ValueError):
      self.capture(stamp, stamp + 2_000_000, stamp + 1_000_000)

  def test_recovery_ack_barrier_does_not_hide_new_source_candidates(self):
    o, old = pending_recovery()
    o.selector_after_ns = 500_000_000_000
    for i in range(2, 7):
      e = replace(episode(o, i, 'ff_gain_right'), normalization_accel=0.5)
      refresh(o, e.end_ns)
      o.observe_episode(e, now_ns=e.end_ns)
    assert o.offer_state is not None and o.offer_state['names'] == ('ff_gain_right',)
    assert len(o.before) == 3 and not any(o.accepted.values())
