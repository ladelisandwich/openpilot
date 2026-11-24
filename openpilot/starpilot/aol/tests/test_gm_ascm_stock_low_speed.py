"""Recorded route206ae7ed9f: ordinary ACC minimum must not revoke AOL."""
from contextlib import ExitStack
import pytest
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE, intent
from openpilot.starpilot.aol.intent import read_settings
from opendbc.car.gm.tests.test_bolt_volt_configurations import ordinary_params
from opendbc.car.gm.values import CAR
from openpilot.cereal import log


@pytest.mark.parametrize("c9", [False, True])
@pytest.mark.parametrize("native_granted", [False, True])
def test_ascm_stock_cruise_minimum_cannot_revoke_independent_main_intent(c9, native_granted):
  cp = ordinary_params(CAR.CHEVROLET_VOLT_ASCM, alpha=False, sascm=True, accelerator=not c9, radar=False)
  assert cp.pcmCruise and not cp.openpilotLongitudinalControl
  assert cp.safetyConfigs[0].safetyParam == (0x605 if c9 else 0x205)
  case = fixture.IoniqAvailabilityTests(methodName="runTest")
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(False, cp=cp, genuine_events=True)
    sd.aol_settings = read_settings(sd.params)
    cs = fixture.car_state()
    cs.vEgo = 4.
    axis, _, cc = step(2, observed_state=cs, state_stamp=BASE, intent_message=intent(BASE, 3), native_request_long=False)
    assert axis.lateralActive and cc.latActive
    cs = fixture.car_state(brake=True)
    cs.vEgo = 1.3862173557281494  # Recorded ASCM source at57.081039046s.
    cs.gearShifter = "low"
    stamp = BASE + 10_000_000
    axis, _, cc = step(3, observed_state=cs, state_stamp=stamp, intent_message=intent(stamp, 4), native_request_long=False, allowed=native_granted)
    assert log.OnroadEvent.EventName.belowEngageSpeed in sd.events.names
    assert not sd.enabled and not cc.longActive
    assert axis.desiredLateral, (axis.to_dict(), sd.events.names)
    assert bool(axis.lateralActive) == native_granted
    assert not cc.latActive  # Existing minSteerSpeed still vetoes actual steering.
    cs = fixture.car_state()
    cs.vEgo = 4.
    stamp += 10_000_000
    axis, _, cc = step(4, observed_state=cs, state_stamp=stamp, intent_message=intent(stamp, 5), native_request_long=False)
    assert axis.lateralActive and cc.latActive and not cc.longActive
