"""Current Card/SD location retention with explicit synthetic wire transport."""

from contextlib import ExitStack
from unittest import mock

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from openpilot.cereal import log
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.radar_interface import RadarInterface
from opendbc.car.chrysler.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_stock_location_family_intent as shared_cases
from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import car_state, hkg_family_case


def chrysler_callers(cleanup, candidate):
  cleanup.enter_context(OpenpilotPrefix())
  saved = Params()
  saved.put_bool('AlwaysOnLateral', True, block=True)
  for key in ('LKASButtonControl', 'MainCruiseButtonControl', 'CancelButtonControl',
              'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
    saved.put(key, 0, block=True)
  case = hkg_family_case('hkg_canfd', False)
  cleanup.callback(case.doCleanups)
  cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, False)
  case.family_cp = cp.as_reader()
  case.interface_type, case.radar_type = CarInterface, RadarInterface
  sd, _, drive, card, receive = case.family_callers(False)
  state = car_state()
  state.vEgo = state.vEgoRaw = 20.0
  return sd, drive, card, receive, state


@pytest.mark.parametrize('candidate', (CAR.CHRYSLER_PACIFICA_2018, CAR.RAM_1500_5TH_GEN))
@pytest.mark.parametrize('event_first', (False, True))
def test_location_loss_retains_existing_stock_lateral_intent(monkeypatch, candidate, event_first):
  monkeypatch.setattr(shared_cases, 'stock_family_callers', chrysler_callers)
  shared_cases.test_existing_stock_hkg_location_and_nominal_pcm_off_keep_established_intent(candidate, event_first)


@pytest.mark.parametrize('candidate', (CAR.CHRYSLER_PACIFICA_2018, CAR.RAM_1500_5TH_GEN))
@pytest.mark.parametrize('fault', ('native_denial', 'native_invalid', 'panda_rx', 'relay', 'permanent_eps',
  'invalid_can', 'can_timeout', 'door', 'seatbelt', 'brake', 'regen', 'gas', 'physical_cancel', 'reverse', 'unknown_gear',
  'extra_user', 'hard_event', 'epoch', 'session', 'expired', 'future_source', 'no_baseline'))
def test_location_retention_rejects_independent_faults(monkeypatch, candidate, fault):
  if fault == 'physical_cancel':
    physical_cancel_clears_location_intent(candidate)
  else:
    monkeypatch.setattr(shared_cases, 'stock_family_callers', chrysler_callers)
    shared_cases.test_stock_location_never_retains_intent_through_independent_fault(candidate, fault)


def physical_cancel_clears_location_intent(candidate):
  # Chrysler CANCEL is a physical disarm even with no optional button mapping.
  # The shared HKG case instead permits its unassigned CANCEL to pause output.
  with ExitStack() as cleanup:
    sd, drive, card, receive, state = chrysler_callers(cleanup, candidate)
    assert card.aol_card_intent.settings.cancel_actions == (0, 0, 0)
    for tick in range(2, 5):
      axis, _, command = drive(tick, observed_state=state, panda_controls_allowed=False, native_request_long=False)
      assert axis.lateralActive and command.latActive
      assert not axis.longitudinalActive and not command.longActive
      assert receive(10_000_000_000 + tick * 10_000_000, state=state,
                     axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    assert card.aol_transport_feedback.lateral_baseline_ns
    original = sd.sm.update_msgs

    def update(timestamp, messages):
      selected = []
      for message in messages:
        if message.which() == 'deviceMotion':
          message = message.as_builder()
          message.deviceMotion.inputsOK = False
          message = message.as_reader()
        selected.append(message)
      return original(timestamp, selected)

    with mock.patch.object(sd.sm, 'update_msgs', side_effect=update):
      axis, _, command = drive(5, observed_state=state, panda_controls_allowed=False, native_request_long=False)
    assert log.OnroadEvent.EventName.locationdTemporaryError in sd.events.names
    assert log.OnroadEvent.EventName.pcmDisable in sd.events.names
    assert not axis.lateralActive and not axis.longitudinalActive
    assert not command.latActive and not command.longActive
    state.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
    assert not receive(10_050_000_001, state=state,
                       axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'])
    assert not card.aol_card_intent.allowed_latch
