"""AOL steering withdrawal must select the existing soft-fault takeover alert."""
from contextlib import ExitStack
import pytest
from opendbc.car.hyundai.values import CAR
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests.test_canfd_stock_profiles import params
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


@pytest.mark.parametrize('identity', [CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, CAR.KIA_EV6])
def test_independent_aol_low_memory_withdrawal_selects_takeover_alert(identity):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(False, cp=params(identity), genuine_events=True)
    for tick in range(20):
      axis, host, command = step(tick)
    assert axis.lateralActive and command.latActive and not host.enabled
    axis, host, command = step(20, critical_event=log.OnroadEvent.EventName.lowMemory)
    assert not axis.lateralActive and not command.latActive and not host.enabled
    assert sd.aol_fault_reason == 'availabilityPause'
    state = sd.pm.messages['selfdriveState'].selfdriveState
    assert state.alertType == 'lowMemory/softDisable', state.to_dict()
    assert state.alertStatus == log.SelfdriveState.AlertStatus.critical


@pytest.mark.parametrize('identity', [CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, CAR.KIA_EV6])
def test_temporary_eps_keeps_existing_warning_without_critical_takeover(identity):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(False, cp=params(identity), genuine_events=True)
    for tick in range(20):
      axis, host, command = step(tick)
    assert axis.lateralActive and command.latActive and not host.enabled
    observed = fixture.car_state()
    observed.steerFaultTemporary = True
    axis, host, command = step(20, observed_state=observed)
    assert not axis.lateralActive and not command.latActive and not host.enabled
    assert sd.aol_fault_reason != 'critical'
    assert not sd.aol_soft_takeover_alert
    state = sd.pm.messages['selfdriveState'].selfdriveState
    assert state.alertStatus != log.SelfdriveState.AlertStatus.critical
