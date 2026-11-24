"""Route1b: recovery after sustained, freshly witnessed inactive lag proofs."""
from contextlib import ExitStack
from unittest import mock
from openpilot.cereal import log

import pytest
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests.test_volt_lag_pedal_intent import callers, BASE
from openpilot.starpilot.aol.tests.test_gm_route18e_recovery import deliver_card


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('alpha', [False, True])
def test_long_isolated_lag_recovery_keeps_only_existing_intent(c9, alpha):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    card, sd, controls, parse, drive = callers(cleanup, sdgm_params(alpha=alpha, brake_c9=c9))
    feedback = card.aol_transport_feedback
    baseline = (feedback.lateral_baseline_ns, feedback.lateral_source_ns, feedback.lateral_native_ns,
                feedback.healthy_feedback_ns, feedback.healthy_native_ns)
    event_ns = None
    for tick in range(2, 43):
      cs = parse(tick)
      with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=.02):
        axis, _, command = drive(tick, cs, allowed=tick == 2, native_request_lat=tick == 2)
      assert not axis.lateralActive and not axis.longitudinalActive
      assert not command.latActive and not command.longActive
      assert str(axis.faultReason) == 'availabilityPause'
      assert axis.faultSessionId == sd.aol_session_id and not axis.nativeAcknowledged
      assert axis.sourceCarStateMonoTime == BASE + tick * 10_000_000
      assert sd.aol_published_fault_proof is None
      assert not axis.desiredLateral and not axis.desiredLongitudinal
      assert axis.validUntilMonoTime == axis.observedMonoTime + 30_000_000
      if event_ns is None:
        event_ns = axis.faultEventMonoTime
      assert axis.faultEventMonoTime == event_ns
      deliver_card(card, sd, controls, cs, tick)
      assert card.aol_card_intent.allowed_latch and not feedback.failed
      assert baseline == (feedback.lateral_baseline_ns, feedback.lateral_source_ns, feedback.lateral_native_ns,
                          feedback.healthy_feedback_ns, feedback.healthy_native_ns)
    assert BASE + 430_000_000 - event_ns > 200_000_000
    cs = parse(43)
    axis, _, command = drive(43, cs, allowed=False, native_request_lat=False)
    assert axis.desiredLateral and not axis.lateralActive and not axis.longitudinalActive
    assert not command.latActive and not command.longActive
    deliver_card(card, sd, controls, cs, 43)
    assert card.aol_card_intent.allowed_latch and not feedback.failed
    cs = parse(44)
    axis, state, command = drive(44, cs)
    assert axis.lateralActive and command.latActive and not command.longActive and not state.enabled
    deliver_card(card, sd, controls, cs, 44)
    assert card.aol_card_intent.allowed_latch
    cs = parse(45, main=False)
    drive(45, cs)
    deliver_card(card, sd, controls, cs, 45)
    assert not card.aol_card_intent.allowed_latch


@pytest.mark.parametrize('bad', ['main_off', 'cancel', 'door', 'native_invalid', 'epoch', 'other_fault'])
def test_stock_pcm_off_lag_never_exempts_other_rejection(bad):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    card, sd, controls, parse, drive = callers(cleanup, sdgm_params(alpha=False))
    cs = parse(2)
    with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=.02):
      axis, _, command = drive(2, cs)
    deliver_card(card, sd, controls, cs, 2)
    assert card.aol_card_intent.allowed_latch and not command.latActive
    cs = parse(3, 'cancel' if bad == 'cancel' else 'none', main=bad != 'main_off')
    cs.doorOpen = bad == 'door'
    # processNotRunning is a SOFT availability pause. This negative case must
    # inject the actual immediate fault whose disarm behavior it protects.
    options = {'critical_event': log.OnroadEvent.EventName.controlsMismatch} if bad == 'other_fault' else {}
    with mock.patch.object(sd.rk.avg_dt, 'get_average', return_value=.02):
      axis, _, command = drive(3, cs, allowed=False, native_request_lat=False, native_valid=bad != 'native_invalid', **options)
    if bad == 'epoch':
      wire = sd.pm.messages['aolAxisState'].as_builder()
      wire.aolAxisState.intentIntegrityEpoch += 1
      sd.pm.messages['aolAxisState'] = wire.as_reader()
    deliver_card(card, sd, controls, cs, 3)
    assert not command.latActive and not command.longActive
    if bad == 'cancel':
      # GM's ordinary Cancel withdraws LONG; MAIN still owns lateral intent.
      assert card.aol_card_intent.allowed_latch and not card.aol_card_intent._main_cycle_required
      assert any(str(event.type) == 'cancel' and event.pressed for event in cs.buttonEvents)
      assert not axis.longitudinalActive and not sd.events.contains('enable')
    else:
      assert not card.aol_card_intent.allowed_latch
