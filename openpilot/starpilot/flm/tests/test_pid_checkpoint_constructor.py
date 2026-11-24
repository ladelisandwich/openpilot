"""Actual PID constructor timing/gains cannot be changed by recorded state."""
from dataclasses import replace

import pytest

from openpilot.starpilot.flm.controller_replay import capture_snapshot, restore_snapshot
from openpilot.starpilot.flm.tests.test_pid_replay import factory


@pytest.mark.parametrize('field', ('controller.dt', 'controller.sat_limit', 'controller.sat_check_min_speed',
                                  'pid.i_dt', 'pid._k_d.1.0'))
def test_pid_checkpoint_preserves_every_actual_constructor_owned_gain(field):
  controller, vm = factory().create()
  snapshot = capture_snapshot(controller, vm)
  assert field in dict(snapshot.values)
  changed = tuple((name, value + .01 if name == field else value) for name, value in snapshot.values)
  with pytest.raises(ValueError, match='immutable actual gains'):
    restore_snapshot(controller, vm, replace(snapshot, values=changed))
