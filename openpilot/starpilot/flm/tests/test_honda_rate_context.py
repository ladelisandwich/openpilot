"""Actual normalized Honda rate clipping remains trace context, never a fit row."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence
from openpilot.starpilot.flm.torque_eligibility import bounded_context_output, exact_unclipped_pair
from openpilot.starpilot.flm.torque_emission_replay import Emission, EncoderBranch
from openpilot.starpilot.flm.tests.test_steering_projection import HONDA, apply, control, install_observer, make_interface, settle

BASE = 10_000_000_000


def captured(ci, requested, tick):
  cc = control(requested)
  actuators, packets, witness = apply(ci, cc)
  assert witness is not None
  assert (witness.can_address, witness.can_data, witness.can_bus) in packets
  clipped = witness.unlimited_raw is None
  stamp = BASE + tick * 10_000_000
  event = Emission(stamp, stamp, stamp + 1_000_000, witness, 'a' * 64, 'b' * 64, 'c' * 64,
    True, True, False, False, clipped, True, False, 0, 0, '')
  output = AppliedEvidence(stamp, stamp, stamp + 1_000_000, 'normalized_torque',
    float(actuators.torque), True, True, False, False, clipped, True, witness)
  return cc, event, output


def fixture():
  ci = make_interface(HONDA.HONDA_CIVIC)
  assert install_observer(ci)
  settle(ci, control(0.))
  return ci


def test_original_rate_clipped_honda_sequence_reproduces_real_normalized_state():
  ci, branch = fixture(), EncoderBranch()
  for tick in range(16):
    _, event, output = captured(ci, .8, tick)
    assert event.limited and event.witness.unlimited_raw is None
    assert bounded_context_output(output)
    assert branch.step(event, event.witness.requested_torque, actual=True) is not None
    assert branch.previous == ci.CC.last_torque
    # This output remains clipped even though the trace is now replayable.
    assert output.limited
    hint = SimpleNamespace(limited=True, domain='normalized_torque',
      stage_schema_revision='pid-selected-stage-snapshot-v2', saturated=False,
      replay_inputs=(('curvature_limited', 0.),))
    assert not exact_unclipped_pair(hint, output)


@pytest.mark.parametrize('candidate', [1., -.8])
def test_candidate_cannot_increase_or_reverse_original_normalized_clipping(candidate):
  ci = fixture()
  _, event, _ = captured(ci, .8, 0)
  with pytest.raises(ValueError, match='clipping regression'):
    EncoderBranch().step(event, candidate)


def test_smaller_same_direction_demand_retains_native_rate_limit():
  ci = fixture()
  _, event, _ = captured(ci, .8, 0)
  branch = EncoderBranch()
  result = branch.step(event, .4)
  assert result is not None
  assert branch.previous == ci.CC.last_torque


def test_lost_health_wrong_codec_and_unlimited_rows_are_not_clipped_context():
  ci = fixture()
  _, event, output = captured(ci, .8, 0)
  assert not bounded_context_output(replace(output, native_allowed=False))
  assert not bounded_context_output(replace(output, fault=True))
  assert not bounded_context_output(replace(output, limited=False))
  assert not bounded_context_output(replace(output, projection=event.witness._replace(mapping_kind='round')))
