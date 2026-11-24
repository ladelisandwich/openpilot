"""Split policy with a declared native verifier fixture; no CAN or road claim."""
from dataclasses import replace
from types import SimpleNamespace
from typing import cast
from unittest.mock import Mock

import pytest
from opendbc.car.steering_projection import TorqueProjection
from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import BASE, SyntheticAdapter, snapshots
from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current
from openpilot.starpilot.flm.unchanged_output import redundant_late_output


def fixture():
  control, output, _ = snapshots(BASE)
  control = replace(control, domain='normalized_torque', native_required=True, native_allowed=True,
    native_identity='4' * 64, native_observed_ns=BASE - 50_000_000, native_expires_ns=BASE + 150_000_000)
  limits = tuple(zip(('STEER_DELTA_UP', 'STEER_DELTA_DOWN', 'STEER_DRIVER_ALLOWANCE',
    'STEER_DRIVER_MULTIPLIER', 'STEER_DRIVER_FACTOR'), (10., 10., 50., 2., 1.), strict=True))
  witness = TorqueProjection(1, 'hyundai', 'synthetic', 0, 'hyundai.LKAS11', 'round', 10,
    .2, .2, 200, 200, 1000., (), (), 'driver', 200., 0., limits, (), True, 832, 0, b'12345678', .2)
  previous = replace(output, domain='normalized_torque', state_ns=BASE + 19_000_000,
    apply_ns=BASE + 20_000_000, projection=witness)
  later = replace(previous, state_ns=BASE + 29_000_000, apply_ns=BASE + 31_000_000,
    projection=witness._replace(frame=11, can_data=b'87654321'))
  collector = FullrateCollector(SyntheticAdapter())
  collector.adapter.command_domain = 'normalized_torque'
  collector.adapter.replay = SimpleNamespace(applied_matches=Mock(return_value=True))
  collector.live_tracking = SimpleNamespace(episodes=SimpleNamespace(source_tracking=True), interrupt=Mock())
  collector._learning_current_native_allowed = True
  collector.ready.append((control, previous))
  collector.last_apply = previous.apply_ns
  collector.last_response = collector.last_qualified_response = BASE + 21_000_000
  return collector, control, previous, later


def test_verified_unchanged_repeat_does_not_erase_or_renew_existing_evidence():
  collector, control, previous, later = fixture()
  assert consumed_sources_current(control, previous)
  assert not consumed_sources_current(control, later)
  prior = (tuple(collector.ready), collector.last_apply, collector.last_response, collector.last_qualified_response)
  collector._join_emission(control, later)
  assert (tuple(collector.ready), collector.last_apply, collector.last_response, collector.last_qualified_response) == prior
  collector.live_tracking.interrupt.assert_not_called()
  collector.adapter.replay.applied_matches.assert_called_once_with(control, later)
  assert not collector.training_interruptions


@pytest.mark.parametrize('case', ('no_prior', 'different_control', 'plant', 'native', 'decode', 'changed_output',
  'limited', 'override', 'fault', 'inactive', 'invalid', 'card_age', 'source_age', 'expired_native',
  'old_prior', 'codec', 'requested', 'not_late', 'native_disabled'))
def test_repeat_exception_cannot_admit_changed_unqualified_or_unverified_input(case):
  collector, control, previous, later = fixture()
  if case == 'no_prior':
    collector.ready.clear()
  elif case == 'different_control':
    control = replace(control, generation=control.generation + 1)
  elif case == 'plant':
    collector.live_tracking.episodes.source_tracking = False
  elif case == 'native':
    collector._learning_current_native_allowed = False
  elif case == 'decode':
    collector.adapter.replay.applied_matches.return_value = False
  elif case == 'changed_output':
    later = replace(later, applied=.201, projection=cast(TorqueProjection, later.projection)._replace(final_raw=201, reported_torque=.201))
  elif case in ('limited', 'override', 'fault'):
    later = replace(later, **{case: True})
  elif case in ('inactive', 'invalid'):
    later = replace(later, **{'active' if case == 'inactive' else 'valid': False})
  elif case == 'card_age':
    later = replace(later, state_ns=BASE)
  elif case == 'source_age':
    control = replace(control, state_ns=BASE - 30_000_001)
    collector.ready[-1] = (control, previous)
  elif case == 'expired_native':
    control = replace(control, native_expires_ns=BASE + 30_000_000)
    collector.ready[-1] = (control, previous)
  elif case == 'old_prior':
    later = replace(later, apply_ns=BASE + 50_000_001, state_ns=BASE + 50_000_000)
  elif case == 'codec':
    later = replace(later, projection=cast(TorqueProjection, later.projection)._replace(can_bus=1))
  elif case == 'requested':
    later = replace(later, projection=cast(TorqueProjection, later.projection)._replace(requested_torque=.201))
  elif case == 'not_late':
    later = replace(later, apply_ns=BASE + 30_000_000)
  elif case == 'native_disabled':
    later = replace(later, native_allowed=False)
  assert not redundant_late_output(collector, control, later)


def test_failed_native_decode_keeps_existing_withdrawal():
  collector, control, _, later = fixture()
  collector.adapter.replay.applied_matches.return_value = False
  collector._join_emission(control, later)
  assert not collector.ready and collector.last_apply == collector.last_qualified_response == 0
  collector.live_tracking.interrupt.assert_called_once_with('consumed_source_mismatch', context_changed=False)


@pytest.mark.parametrize('tamper', (False, True))
def test_ioniq6_unchanged_repeat_uses_actual_can_decoder(tmp_path, tamper):
  # Physical state/current permission are declared test fixtures. CarInterface,
  # encoder, packets, emission history and decoder are the registered path.
  from opendbc.car.hyundai.values import CAR
  from openpilot.starpilot.flm.controller_replay import ControllerReplay
  from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
  from openpilot.starpilot.flm.tests.test_torque_emission_join import interface, cc, settle, emitted, BASE as ENCODER_BASE

  ci = interface(CAR.HYUNDAI_IONIQ_6)
  command = cc(0.)
  settle(ci, command)
  stamp = ENCODER_BASE + ci.CC.frame * 10_000_000
  source, first = emitted(ci, command, tmp_path, control_ns=stamp)
  collector, control, _, _ = fixture()
  control = replace(control, requested=0., control_ns=stamp, state_ns=stamp-1,
    identity=source.identity, source_epoch=source.source_epoch, baseline_revision=source.baseline_revision,
    native_required=False)
  history = EmissionHistory(ci.CP, identity=control.identity, baseline_revision=control.baseline_revision)
  rows = [first]
  for _ in range(4):
    _, output = emitted(ci, command, tmp_path, control_ns=stamp)
    rows.append(output)
  if tamper:
    witness = rows[-1].projection
    assert witness is not None
    data = bytes([witness.can_data[0] ^ 0xff]) + witness.can_data[1:]
    rows[-1] = replace(rows[-1], projection=witness._replace(can_data=data))
  for output in rows:
    assert history.record_carrier(output) and history.record(control, output)
  previous, later = rows[-2:]
  assert consumed_sources_current(control, previous) and not consumed_sources_current(control, later)
  replay = ControllerReplay.__new__(ControllerReplay)
  replay.emissions = history
  collector.adapter.replay = replay
  collector.ready.clear()
  collector.ready.append((control, previous))
  assert redundant_late_output(collector, control, later) is not tamper
