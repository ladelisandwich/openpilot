"""Actual host/controller paths reject impossible original native receipts."""
from contextlib import contextmanager
import os
from pathlib import Path
from unittest import mock

import pytest

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
from openpilot.starpilot.lateral.controller_selection import DOCUMENT_KEY, ControllerMode, replace_mode
from openpilot.starpilot.lateral.tests.test_lane_runtime import feed, publish_corolla_params
from openpilot.starpilot.lateral.torque_runtime import TorqueHost, TorqueSettings
from openpilot.starpilot.lateral.torque_tuning import TorqueSource

BASE = 20_000_000_000


def native_packet(cp, stamp, *, use_params=True, valid=True, version=1, outer_valid=True):
  event = messaging.new_message('lateralTorqueParameters', valid=outer_valid, logMonoTime=stamp)
  row = event.lateralTorqueParameters
  row.useParams, row.valid, row.version = use_params, valid, version
  row.latAccelFactorFiltered = float(cp.lateralTuning.torque.latAccelFactor) * 1.1
  row.latAccelOffsetFiltered = .03
  row.frictionCoefficientFiltered = float(cp.lateralTuning.torque.friction) * 1.1
  return messaging.log_from_bytes(event.to_bytes())


@contextmanager
def real_host_controls():
  with OpenpilotPrefix(), mock.patch.dict(os.environ, {'REPLAY': '1', 'TORQUE_REPLAY_RUNTIME': '1',
        'AOL_REPLAY_RUNTIME': '0', 'LANE_CENTERING_REPLAY_RUNTIME': '0'}), \
       mock.patch('openpilot.selfdrive.controls.controlsd.messaging.PubMaster'):
    params = Params()
    cp = publish_corolla_params(params)
    Path(params.get_param_path(DOCUMENT_KEY)).write_bytes(replace_mode(None, cp, ControllerMode.STANDARD))
    controls = Controls()
    assert controls.torque_host is not None
    yield controls


@pytest.mark.parametrize('case', ('future_at_receipt', 'stale_source', 'invalid_inner', 'wrong_version', 'invalid_outer'))
def test_actual_controls_host_rejects_bad_native_before_cursor_and_controller_mutation(case):
  with real_host_controls() as controls:
    host = controls.torque_host
    original_tune = controls.LaC.torque_params.to_dict()
    for stamp in (BASE - 500_000_000, BASE - 250_000_000):
      controls.sm.update_msgs(stamp / 1e9, [native_packet(controls.CP, stamp, valid=False)])
    stamp = BASE + 1 if case == 'future_at_receipt' else BASE - 500_000_001 if case == 'stale_source' else BASE
    event = native_packet(controls.CP, stamp, valid=case != 'invalid_inner',
                          version=99 if case == 'wrong_version' else 1, outer_valid=case != 'invalid_outer')
    controls.sm.update_msgs(BASE / 1e9, [event])
    for tick in range(20):
      now = BASE + tick * 10_000_000
      feed(controls, now, tick)
      controls.state_control()
      assert host.selected == host.vehicle
      assert host.applied == host.vehicle
      assert controls.LaC.torque_params.to_dict() == original_tune
      assert host.last_application == host.vehicle


def test_actual_adopted_host_rejects_future_original_receipt_and_holds_previous_owner():
  with real_host_controls() as controls:
    cp = controls.CP
    factory = SelectedControllerFactory(cp, 'standard')
    controller, _ = factory.create()
    ci = factory.interface_class(cp)
    host = TorqueHost(controls.params, cp, allow_learning=True, automatic_interface=ci, automatic_controller='standard')
    host.adopt_linear_controller(controller)
    before = host.native_base
    assert before is not None
    original_tune = controller.torque_params.to_dict()
    for stamp in (BASE - 500_000_000, BASE - 250_000_000):
      controls.sm.update_msgs(stamp / 1e9, [native_packet(cp, stamp, valid=False)])
    controls.sm.update_msgs(BASE / 1e9, [native_packet(cp, BASE + 1)])
    for tick in range(20):
      now = BASE + tick * 10_000_000
      tune = host.sample(controls.sm, now_ns=now, lat_active=True)
      host.apply(controller, tune)
      assert host.native_base == before
      assert host.selected == before and host.applied == before
      assert controller.torque_params.to_dict() == original_tune


def test_force_auto_choice_still_accepts_current_valid_native_with_use_params_false():
  with real_host_controls() as controls:
    host = controls.torque_host
    for stamp in (BASE - 500_000_000, BASE - 250_000_000, BASE):
      controls.sm.update_msgs(stamp / 1e9, [native_packet(controls.CP, stamp, use_params=False)])
    host.settings = TorqueSettings(force_auto=True)
    selected = host._select(controls.sm, BASE)
    assert selected.source == TorqueSource.LEARNED
    assert selected.lat_accel_factor > host.vehicle.lat_accel_factor
    controls.sm.logMonoTime['lateralTorqueParameters'] = BASE + 1
    assert host._select(controls.sm, BASE + 10_000_000) == host.vehicle


def test_manual_factor_choice_still_wins_with_rejected_native_source():
  with real_host_controls() as controls:
    host = controls.torque_host
    wanted = host.vehicle.lat_accel_factor * 1.2
    host.settings = TorqueSettings(user_factor=wanted)
    for stamp in (BASE - 500_000_000, BASE - 250_000_000):
      controls.sm.update_msgs(stamp / 1e9, [native_packet(controls.CP, stamp, valid=False)])
    controls.sm.update_msgs(BASE / 1e9, [native_packet(controls.CP, BASE + 1)])
    selected = host._select(controls.sm, BASE + 10_000_000)
    assert selected.source == TorqueSource.USER
    assert selected.lat_accel_factor == wanted
    assert selected.lat_accel_offset == host.vehicle.lat_accel_offset
    assert selected.friction == host.vehicle.friction
