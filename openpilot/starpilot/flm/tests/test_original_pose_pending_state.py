"""A consumed state is usable before its command's independent CAN receipt."""
from dataclasses import replace

import pytest

from openpilot.starpilot.flm.original_source_capture import OriginalSourcePose, validate_pose
from openpilot.starpilot.flm.physical_observation import PendingPhysical
from openpilot.starpilot.flm.tests.test_original_pose_work import BASE, fixture, observe, packet, pair


def pending_right(collector, sm, **changes):
  stamp = BASE + 10_000_000
  sm.update_msgs(stamp / 1e9, [packet('carState', stamp).as_reader()])
  control, _ = pair(stamp, speed=22., angle=2.)
  control = replace(control, **changes)
  collector.controls[control.control_ns] = control
  return control


def test_pose_uses_original_pending_state_without_admitting_its_actuation():
  with fixture() as (collector, cp, sm):
    assert type(observe(collector, cp, sm, BASE)) is PendingPhysical
    original = tuple(collector.ready)
    control = pending_right(collector, sm)
    row = observe(collector, cp, sm, BASE + 12_000_000)
    assert type(row) is OriginalSourcePose
    validate_pose(row, BASE + 12_000_000)
    assert (row.speed_mps, row.steering_angle_deg) == (21., 1.)
    assert row.right_state_ns == control.state_ns
    assert (row.native_control_ns, row.native_apply_ns) == (original[-1][0].control_ns, original[-1][1].apply_ns)
    assert tuple(collector.ready) == original
    assert collector.controls[control.control_ns] is control
    assert (row.mono_ns, row.received_ns, row.observed_ns) == (BASE, BASE, BASE)


@pytest.mark.parametrize('changes', [
  {'identity': '9' * 64}, {'generation': 2}, {'source_epoch': '9' * 64},
  {'baseline_revision': 'other'}, {'controller_revision': 'other'}, {'native_identity': '9' * 64},
  {'enabled': False}, {'active': False}, {'valid': False}, {'manual': True},
  {'override': True}, {'fault': True}, {'saturated': True}, {'native_allowed': False},
  {'control_ns': BASE + 12_000_001}, {'control_ns': BASE + 41_000_000},
  {'native_expires_ns': BASE + 10_000_000}, {'native_required': False}, {'domain': 'steering_angle'},
])
def test_unqualified_pending_state_cannot_complete_pose(changes):
  with fixture() as (collector, cp, sm):
    assert type(observe(collector, cp, sm, BASE)) is PendingPhysical
    pending_right(collector, sm, **changes)
    assert type(observe(collector, cp, sm, BASE + 12_000_000)) is PendingPhysical


def test_pending_state_does_not_replace_current_native_permission():
  with fixture() as (collector, cp, sm):
    pending_right(collector, sm)
    collector._learning_current_native_allowed = False
    assert observe(collector, cp, sm, BASE + 12_000_000) is None


def test_conflicting_source_state_is_refused():
  with fixture() as (collector, cp, sm):
    pending_right(collector, sm)
    control, _ = pair(BASE - 10_000_000, speed=99.)
    collector.controls[control.control_ns] = control
    assert observe(collector, cp, sm, BASE + 12_000_000) is None
