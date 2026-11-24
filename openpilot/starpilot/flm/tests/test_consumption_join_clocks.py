"""Recorded route7c clocks and strict independent consumption boundaries.

No supplied physical response, native permission or fit is created here.
"""
from types import SimpleNamespace

import pytest
from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current


def pair():
  return (SimpleNamespace(state_ns=1681782046348, control_ns=1681789927746),
          SimpleNamespace(state_ns=1681812490090, apply_ns=1681814665097))


def test_recorded_route7c_individually_fresh_consumption_not_cross_source_age():
  control, output = pair()
  assert output.state_ns - control.state_ns == 30_443_742
  assert control.control_ns - control.state_ns == 7_881_398
  assert output.apply_ns - output.state_ns == 2_175_007
  assert output.apply_ns - control.control_ns == 24_737_351
  assert consumed_sources_current(control, output)


@pytest.mark.parametrize('boundary', ('control_state', 'card_state', 'dispatch'))
@pytest.mark.parametrize('excess', (0, 1))
def test_each_original_thirty_ms_budget_is_inclusive_but_never_extended(boundary, excess):
  end = 10_100_000_000
  control = SimpleNamespace(state_ns=end - 30_000_000, control_ns=end)
  output = SimpleNamespace(state_ns=end, apply_ns=end)
  if boundary == 'control_state':
    control.state_ns -= excess
  elif boundary == 'card_state':
    output.apply_ns = end + 30_000_000 + excess
  else:
    output.state_ns = output.apply_ns = end + 30_000_000 + excess
  assert consumed_sources_current(control, output) is (excess == 0)


@pytest.mark.parametrize('field,value', (('state_ns', 0), ('state_ns', -1), ('state_ns', True),
                                      ('control_ns', 1.0), ('control_ns', 0)))
def test_invalid_control_clocks_fail_closed(field, value):
  control, output = pair()
  setattr(control, field, value)
  assert not consumed_sources_current(control, output)


def test_card_state_precedes_control_source_or_follows_application_fails():
  control, output = pair()
  output.state_ns = control.state_ns - 1
  assert not consumed_sources_current(control, output)
  output.state_ns = output.apply_ns + 1
  assert not consumed_sources_current(control, output)


def test_recorded_clock_relationship_reaches_actual_registered_encoder_join(tmp_path):
  from dataclasses import replace
  from opendbc.car.toyota.values import CAR
  from openpilot.starpilot.flm.fullrate_collector import FullrateCollector
  from openpilot.starpilot.flm.runtime_adapter_registry import read_source
  from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
  from openpilot.starpilot.flm.tests.test_torque_emission_join import BASE, interface, settle, cc, emitted

  ci = interface(CAR.TOYOTA_COROLLA_TSS2)
  settle(ci, cc(0.01))
  dispatch = 24_737_351
  apply_ns = BASE + ci.CC.frame * 10_000_000
  original_control_ns = apply_ns - dispatch
  control, output = emitted(ci, cc(0.01), tmp_path, control_ns=original_control_ns)
  assert control.control_ns == original_control_ns and output.apply_ns == apply_ns
  # emitted/source constructs the source at the original command clock. Its
  # compact encoder fixture omits receipt metadata; obtain the actual typed
  # source with the identical fixture settings rather than inventing a stamp.
  actual_source = read_source(ci.CP.as_reader(),
    SimpleNamespace(get_param_path=lambda key: str(tmp_path / key)),
    now_ns=original_control_ns, calibration=(0.0, 0.0, 0.0), vehicle_session='fixture-drive')
  assert actual_source.source_epoch == control.source_epoch
  control.source_receipt_ns = actual_source.observed_ns
  assert 0 < control.source_receipt_ns <= control.control_ns <= actual_source.expires_ns
  assert control.control_ns <= control.source_receipt_ns + 500_000_000
  control.state_ns = control.control_ns - 7_881_398
  control.domain = 'normalized_torque'
  output = replace(output, control_ns=control.control_ns, state_ns=output.apply_ns - 2_175_007)
  history = EmissionHistory(ci.CP, identity=control.identity, baseline_revision=control.baseline_revision)
  assert history.record_carrier(output)
  collector = FullrateCollector(SimpleNamespace(command_domain='normalized_torque', record_actual_emission=history.record))
  collector._join_emission(control, output)
  assert list(collector.ready) == [(control, output)]
  history.verify(control, output)
  collector._join_emission(control, replace(output, state_ns=output.apply_ns - 30_000_001))
  assert not collector.ready
  assert collector.training_interruptions['consumed_source_mismatch'] == 1
