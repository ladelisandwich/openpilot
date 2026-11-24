"""Real I6 callers: existing upstream mismatch ownership, no denied LONG output."""
from contextlib import ExitStack

import pytest
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture


def test_native_brake_withdrawal_precedes_card_source_without_losing_lateral():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, _, drive, card, receive = case._ioniq_availability_callers(full_op=True)
    # PandaStates is a real10Hz service. Prime the ordinary healthy100Hz
    # callers until the next ORIGINAL10Hz publication is due; the installed
    # route likewise had latestPanda FALSE before the mismatch decision.
    for tick in range(2, 10):
      warm_axis, warm_state, warm_command = drive(tick)
      assert warm_state.enabled and warm_axis.lateralActive and warm_axis.longitudinalActive
      assert warm_command.latActive and warm_command.longActive
    axis, state, command = drive(10, native_request_lat=True, allowed=True, native_request_long=True,
      long_allowed=False, panda_controls_allowed=False)
    assert int(sd.sm.logMonoTime['pandaStates']) == 10_100_000_000
    assert all(not panda.controlsAllowed for panda in sd.sm['pandaStates'])
    # Native denial takes effect immediately. Healthy independently granted LAT
    # and its existing intent do not become a fabricated critical source fault.
    assert not axis.longitudinalActive and not command.longActive
    assert axis.lateralActive and command.latActive
    assert state.enabled and not sd.aol_authority_lost
    assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    assert sd.mismatch_counter > 0
    assert receive(10_100_000_000, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    assert card.aol_card_intent.allowed_latch
    axis, state, command = drive(11, brake=True, native_request_lat=True, allowed=True,
      native_request_long=True, long_allowed=False, panda_controls_allowed=False)
    assert not state.enabled and not state.active
    assert axis.lateralActive and command.latActive
    assert not axis.longitudinalActive and not command.longActive
    assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    assert receive(10_110_000_000, state=fixture.car_state(brake=True),
      axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    assert card.aol_card_intent.allowed_latch


def test_persistent_native_long_denial_still_hits_original_upstream_mismatch_counter():
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, _, drive, _, _ = case._ioniq_availability_callers(full_op=True)
    for tick in range(2, 10):
      warm_axis, warm_state, warm_command = drive(tick)
      assert warm_state.enabled and warm_axis.lateralActive and warm_axis.longitudinalActive
      assert warm_command.latActive and warm_command.longActive
    for tick in range(10, 214):
      axis, state, command = drive(tick, native_request_lat=True, allowed=True,
        native_request_long=True, long_allowed=False, panda_controls_allowed=False)
      if tick == 10:
        assert int(sd.sm.logMonoTime['pandaStates']) == 10_100_000_000
        assert all(not panda.controlsAllowed for panda in sd.sm['pandaStates'])
      assert not axis.longitudinalActive and not command.longActive
      if log.OnroadEvent.EventName.controlsMismatch in sd.events.names:
        break
      assert axis.lateralActive and command.latActive and state.enabled
    assert sd.mismatch_counter >= 200
    assert log.OnroadEvent.EventName.controlsMismatch in sd.events.names
    assert not state.enabled and not axis.lateralActive and not command.latActive


@pytest.mark.parametrize('fault', ['lat_denied', 'native_invalid', 'rx_fault', 'relay_fault', 'inconsistent_long_receipt'])
def test_other_native_denials_and_faults_remain_immediate(fault):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, _, drive, _, _ = case._ioniq_availability_callers(full_op=True)
    options = {'native_request_lat': True, 'allowed': True, 'native_request_long': True,
      'long_allowed': False, 'panda_controls_allowed': False}
    if fault == 'lat_denied':
      options['allowed'] = False
    elif fault == 'native_invalid':
      options['native_valid'] = False
    elif fault == 'rx_fault':
      options['panda_fault'] = True
    elif fault == 'relay_fault':
      options['relay_fault'] = True
    else:
      options['panda_controls_allowed'] = True
    axis, state, command = drive(2, **options)
    assert not axis.longitudinalActive and not command.longActive
    if fault == 'inconsistent_long_receipt':
      # controlsAllowed is an independent Panda observation. A healthy LAT
      # grant with denied LONG is not itself an immediate integrity fault.
      from openpilot.starpilot.aol.runtime import current_native
      native = current_native(sd.sm, sd.CP, now_ns=axis.observedMonoTime,
                              axis_session_id=sd.aol_session_id)
      assert native is not None and native.requestedLongitudinal and not native.longitudinalAllowed
      assert native.requestedLateral and native.lateralAllowed
      assert all(p.controlsAllowed for p in native.pandaInventory)
      assert state.enabled and axis.lateralActive and command.latActive
      assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    else:
      assert log.OnroadEvent.EventName.controlsMismatch in sd.events.names
      assert not state.enabled and not axis.lateralActive and not command.latActive
