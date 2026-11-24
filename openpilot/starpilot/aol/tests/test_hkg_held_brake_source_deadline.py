"""Elantra route04's held-brake source cutoff must not renew an output lease."""
from contextlib import ExitStack
from unittest import mock

import pytest
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from opendbc.car.hyundai.values import CAR
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_classic_long_profiles import params as classic_params
from openpilot.starpilot.aol.tests.test_ev6_physical_authorization import ev6_params
from openpilot.starpilot.aol.tests.test_ordinary_source_availability import settings
from openpilot.starpilot.aol.tests.test_canfd_stock_profiles import params as canfd_params
from opendbc.car.hyundai.torque_ev_startup import candidate

SOURCE = 10_020_000_000
PUBLISHED_OFFSET = 5_393_122
SOURCE_AGE = 33_913_075


def profile(identity):
  if identity == CAR.KIA_EV6:
    return ev6_params()
  if identity == CAR.HYUNDAI_IONIQ_5:
    return candidate(canfd_params(identity), requested=True, is_release=False)
  return classic_params(identity, lda=True)


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_ELANTRA_2021, CAR.HYUNDAI_SONATA_HYBRID,
                                     CAR.KIA_EV6, CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_PALISADE, CAR.KIA_NIRO_EV])
@pytest.mark.parametrize('socket_timeout', [False, True])
@pytest.mark.parametrize('published_offset,source_age', [(PUBLISHED_OFFSET, SOURCE_AGE), (5_631_474, 30_894_772)])
def test_hkg_held_brake_keeps_only_the_recorded_unexpired_proof(identity, socket_timeout, published_offset, source_age):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, controls, drive = case._authority_callers(False, cp=profile(identity), genuine_events=True)
    for tick in (0, 1):
      stamp = 10_000_000_000 + tick * 10_000_000
      drive(tick, brake=True, long_allowed=False, **settings('lateral_only', stamp, tick + 1))
    old, state, command = drive(2, state_stamp=SOURCE, offset_ns=published_offset, brake=True,
      long_allowed=False, **settings('lateral_only', SOURCE, 3))
    assert old.lateralActive and command.latActive and not old.longitudinalActive and not command.longActive
    assert not state.enabled
    deadline = old.validUntilMonoTime
    assert deadline == SOURCE + published_offset + 30_000_000
    old_data = old.to_dict()
    sd.car_state_poller = mock.Mock()
    sd.car_state_poller.poll.return_value = []
    axis, state, command = drive(5, timeout=socket_timeout, state_stamp=SOURCE, offset_ns=source_age - 30_000_000,
      brake=True, long_allowed=False, **settings('lateral_only', SOURCE, 3))
    assert sd.aol_axis_publication_deferred, 'HKG_HELD_BRAKE_SOURCE_WAIT_EXCLUDED'
    assert axis.to_dict() == old_data and axis.validUntilMonoTime == deadline
    assert not state.enabled and not command.longActive
    assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    with mock.patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=deadline + 1):
      expired, _ = controls.state_control()
    assert not expired.latActive and not expired.longActive
    fresh = SOURCE + 40_000_000
    axis, state, command = drive(6, state_stamp=fresh, brake=True,
      long_allowed=False, **settings('lateral_only', fresh, 4))
    assert axis.lateralActive and command.latActive and not state.enabled and not command.longActive


@pytest.mark.parametrize('fault', ['new_brake', 'lateral_denied', 'rx', 'steer_fault', 'source_invalid'])
def test_hkg_held_brake_extension_does_not_preserve_a_new_fault(fault):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, _, drive = case._authority_callers(False, cp=classic_params(CAR.HYUNDAI_ELANTRA_2021, lda=True), genuine_events=True)
    held = fault != 'new_brake'
    for tick in (0, 1, 2):
      stamp = 10_000_000_000 + tick * 10_000_000
      drive(tick, brake=held, state_stamp=stamp, offset_ns=PUBLISHED_OFFSET if tick == 2 else 0,
        long_allowed=False, **settings('lateral_only', stamp, tick + 1))
    sd.car_state_poller = mock.Mock()
    sd.car_state_poller.poll.return_value = []
    options = settings('lateral_only', SOURCE, 3)
    changes = {}
    if fault == 'lateral_denied':
      options['allowed'] = False
    elif fault == 'rx':
      changes['panda_fault'] = True
    elif fault in ('steer_fault', 'source_invalid'):
      state = fixture.car_state(brake=True)
      if fault == 'steer_fault':
        state.steerFaultPermanent = True
      else:
        state.canValid = False
      changes['observed_state'] = state
    axis, _, command = drive(5, state_stamp=SOURCE, offset_ns=SOURCE_AGE - 30_000_000,
      brake=True, long_allowed=False, **options, **changes)
    assert not sd.aol_axis_publication_deferred
    assert not command.longActive


@pytest.mark.parametrize('corruption', ['retained_valid', 'retained_source', 'queue_invalid', 'expired', 'native_denied', 'native_rx'])
def test_timeout_cannot_borrow_an_untrusted_or_expired_receipt(corruption):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, _, drive = case._authority_callers(False, cp=profile(CAR.HYUNDAI_PALISADE), genuine_events=True)
    for tick in (0, 1, 2):
      stamp = 10_000_000_000 + tick * 10_000_000
      old, _, command = drive(tick, brake=True, long_allowed=False, state_stamp=stamp,
        offset_ns=5_631_474 if tick == 2 else 0, **settings('lateral_only', stamp, tick + 1))
    assert old.lateralActive and command.latActive
    sd.car_state_poller = mock.Mock()
    sd.car_state_poller.poll.return_value = []
    original = sd.defer_expired_axis_publication
    def corrupt_receipt(cs, initial_state, **kwargs):
      if corruption == 'retained_valid':
        sd.car_state_retained_valid = False
      elif corruption == 'retained_source':
        sd.car_state_retained_source_ns -= 1
      elif corruption == 'queue_invalid':
        sd.car_state_queue_invalid = True
      return original(cs, initial_state, **kwargs)
    options = settings('lateral_only', SOURCE, 3)
    if corruption == 'native_denied':
      options['allowed'] = False
    elif corruption == 'native_rx':
      options['panda_fault'] = True
    with mock.patch.object(sd, 'defer_expired_axis_publication', side_effect=corrupt_receipt) as intercept:
      _, _, command = drive(6 if corruption == 'expired' else 5, timeout=True, state_stamp=SOURCE,
        offset_ns=0 if corruption == 'expired' else 894_772, brake=True, long_allowed=False, **options)
    if corruption in ('retained_valid', 'retained_source', 'queue_invalid'):
      assert intercept.called
    assert not sd.aol_axis_publication_deferred
    assert not command.latActive and not command.longActive
