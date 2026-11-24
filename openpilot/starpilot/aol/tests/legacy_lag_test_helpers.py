"""Historical critical/OFF receipt inputs, distinct from current SOFT production.

The current producer must first classify the fresh host-only fault as an
availability pause. Republish its historical all-OFF representation through
actual publication code to exercise the retained prior-critical guards.
"""
from contextlib import contextmanager
from unittest import mock

from openpilot.cereal import log
from openpilot.starpilot.aol.runtime import AxisDecision
from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import stock_family


@contextmanager
def legacy_lag_publications(sd):
  actual = sd.publish_axis_proof

  def publish(cs, **kwargs):
    if (sd.aol_fault_reason == 'availabilityPause' and sd.aol_car_state_log_ns > 0 and
        log.OnroadEvent.EventName.selfdrivedLagging in sd.events.names and not sd.enabled):
      assert not sd.aol_axis_decision.lateral_active and not sd.aol_axis_decision.longitudinal_active
      assert not sd.aol_authority_lost
      sd.aol_fault_reason = 'critical'
      sd.aol_axis_decision = AxisDecision()
    return actual(cs, **kwargs)

  with mock.patch.object(sd, 'publish_axis_proof', side_effect=publish):
    yield


@contextmanager
def legacy_stock_family(family):
  with stock_family(family) as callers, legacy_lag_publications(callers[0]):
    yield callers


@contextmanager
def source_availability_refusals(sd, *, historical=False):
  """Witness the actual source classifier rejecting the injected veto."""
  actual = sd.ordinary_source_availability
  verdicts = []

  def classify(*args, **kwargs):
    reason = sd.aol_fault_reason
    admitted = actual(*args, **kwargs)
    verdicts.append((reason, admitted))
    return admitted

  with mock.patch.object(sd, 'ordinary_source_availability', side_effect=classify):
    yield verdicts
  if historical:
    # Reject the forged prior-critical role at its actual decision boundary.
    # A later canonical SOFT publication may independently retain intent; it
    # cannot turn that historical refusal into output or an enable gesture.
    assert verdicts and not verdicts[0][1]
    assert all(not accepted or reason == 'availabilityPause' for reason, accepted in verdicts)
  else:
    assert not any(accepted for _, accepted in verdicts)
    assert verdicts or sd.aol_authority_lost or sd.events.contains('immediateDisable')
