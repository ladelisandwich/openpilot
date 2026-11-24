"""Collector split-policy regressions; synthetic physical fixture, no fit claim.

History is sealed through the real collector methods using the existing explicit
synthetic transport fixture. Direct _join_emission calls isolate reset policy;
these tests do not claim registered controller/native/physical qualification.
"""
from dataclasses import replace

import pytest

from openpilot.starpilot.flm.adaptive_profile import Result
from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import BASE, SyntheticAdapter, snapshots


def history():
  collector = FullrateCollector(SyntheticAdapter())
  stamp = BASE
  for _ in range(281):
    stamp += 50_000_000
    control, output, physical = snapshots(stamp)
    assert collector.control(control, output.apply_ns)
    assert collector.output(output, output.apply_ns)
    collector.physical(physical, output.apply_ns)
  assert len(collector.blocks) == 1
  assert collector.ready and collector.last_qualified_response
  assert collector.fit_result is None  # Never manufacture a fitted result.
  collector.result = Result('unavailable', 'synthetic test state', None)
  return collector, control, output


def withdrawn(collector, now_ns):
  assert not collector.controls and not collector.applied
  assert not collector.recent_controls and not collector.recent_applied
  assert not collector.ready and not collector.points and not collector.raw_points
  assert collector.active_since == collector.last_apply == collector.last_response == 0
  assert collector.last_qualified_response == 0
  assert collector.result is None and collector.last_validation is None
  assert collector.revalidate_fit(now_ns) is None


@pytest.mark.parametrize('failure', ('card_source', 'dispatch', 'source_order'))
def test_normalized_clock_rejection_preserves_exact_sealed_history_only(failure):
  collector, control, output = history()
  saved = tuple(collector.blocks)
  samples = tuple(saved[0].samples)
  physical_context = collector.physical_context
  # The direct call tests only normalized join-reset dispatch, before any
  # encoder/native branch can run; the invalid pair must never be admitted.
  collector.adapter.command_domain = 'normalized_torque'
  control = replace(control, domain='normalized_torque')
  output = replace(output, domain='normalized_torque')
  if failure == 'card_source':
    output = replace(output, state_ns=control.state_ns, apply_ns=control.control_ns + 30_000_000)
  elif failure == 'dispatch':
    output = replace(output, state_ns=control.control_ns + 30_000_001,
                     apply_ns=control.control_ns + 30_000_001)
  else:
    output = replace(output, state_ns=control.state_ns - 1)
  collector._join_emission(control, output)
  assert collector.training_interruptions['consumed_source_mismatch'] == 1
  assert tuple(collector.blocks) == saved
  assert collector.blocks[0] is saved[0]
  assert all(a is b for a, b in zip(collector.blocks[0].samples, samples, strict=True))
  assert collector.physical_context == physical_context
  withdrawn(collector, output.apply_ns)


def test_wrong_output_domain_hard_resets_sealed_history():
  collector, control, output = history()
  collector._join_emission(control, replace(output, domain='wrong-domain'))
  assert collector.training_interruptions['consumed_source_mismatch'] == 1
  assert not collector.blocks and collector.physical_context is None
  assert collector.fit_result is collector.fit_validation is None
  withdrawn(collector, output.apply_ns)


@pytest.mark.parametrize('field,value', (('identity', '4' * 64), ('source_epoch', '5' * 64),
                                      ('baseline_revision', '6' * 64)))
def test_actual_control_context_change_still_hard_resets_history(field, value):
  collector, control, output = history()
  later, _, _ = snapshots(control.control_ns + 50_000_000)
  changed = replace(later, **{field: value})
  assert collector.control(changed, changed.control_ns)
  assert collector.training_interruptions['actual_controller_identity_changed'] == 2
  assert not collector.blocks and collector.physical_context is None
  assert collector.fit_result is collector.fit_validation is None
  assert collector.result is None and collector.last_validation is None


def test_non_normalized_clock_mismatch_keeps_original_hard_reset_behavior():
  collector, control, output = history()
  assert control.domain != 'normalized_torque'
  stale = replace(output, state_ns=control.state_ns + 30_000_001,
                  apply_ns=control.state_ns + 30_000_002)
  collector._join_emission(control, stale)
  assert collector.training_interruptions['consumed_source_mismatch'] == 1
  assert not collector.blocks and collector.physical_context is None
  withdrawn(collector, stale.apply_ns)
