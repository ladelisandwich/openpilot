"""Presentation distinguishes cold faults from a real acknowledged AOL pause."""
from contextlib import ExitStack

import pytest

from opendbc.car.hyundai.values import CAR
from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_canfd_stock_profiles import params


@pytest.mark.parametrize('context', ['cold_model', 'park', 'unarmed'])
def test_never_active_fault_does_not_claim_steering_was_paused(context):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(False, cp=params(CAR.KIA_EV6), genuine_events=True)
    if context == 'cold_model':
      sd.initialized = False
    observed = fixture.car_state()
    if context == 'park':
      observed.gearShifter = car.CarState.GearShifter.park
      observed.vEgo = 0.
      observed.standstill = True
    for tick in range(3):
      axis, state, command = step(tick, observed_state=observed, calibrated=context != 'cold_model',
        latched=context != 'unarmed', allowed=context != 'unarmed',
        critical_event=log.OnroadEvent.EventName.lowMemory)
      assert str(axis.faultReason) == 'availabilityPause'
      assert not axis.desiredLateral and not axis.lateralActive
      assert not state.enabled and not state.active and not command.latActive and not command.longActive
      assert state.alertType != 'aolAvailability/warning'
      assert 'aolAvailability/warning' not in sd.AM.alerts
      assert log.OnroadEvent.EventName.lowMemory in sd.events.names
      assert sd.events.contains(fixture.ET.SOFT_DISABLE)


def test_acknowledged_disabled_aol_pause_warns_then_cold_transition_retires_only_custom_entry():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step, _, _ = case._ioniq_availability_callers(full_op=False)
    axis, state, command = step(2, car_output_valid=False)
    assert axis.desiredLateral and axis.nativeAcknowledged
    assert not axis.lateralActive and not command.latActive
    assert not state.enabled and not state.active
    queued = sd.AM.alerts['aolAvailability/warning'].alert
    assert queued is not None
    assert (queued.alert_text_1, queued.alert_text_2) == ('Steering Paused', 'Communication Issue')
    # A genuine original communication takeover may outrank the custom notice.
    if state.alertType != 'aolAvailability/warning':
      assert state.alertType == 'commIssue/softDisable'
      assert state.alertStatus == log.SelfdriveState.AlertStatus.critical
    # The real fixture publishes calibration at its service cadence, not every
    # control tick. Wait for the actual delivered state without forging health.
    for tick in range(3, 103):
      axis, state, command = step(tick, calibrated=False, car_output_valid=False)
      assert not axis.lateralActive and not command.latActive and not command.longActive
      if str(sd.sm['extrinsicsCalibration'].calStatus) == 'uncalibrated':
        break
    else:
      raise AssertionError('Uncalibrated state was not genuinely delivered within one second')
    assert str(sd.sm['extrinsicsCalibration'].calStatus) == 'uncalibrated'
    assert not axis.desiredLateral and not axis.lateralActive and not command.latActive
    assert state.alertType != 'aolAvailability/warning'
    assert 'aolAvailability/warning' not in sd.AM.alerts
    assert not state.enabled and not state.active and not command.longActive
