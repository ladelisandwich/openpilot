"""Recorded EV6 8f2e source cutoff keeps only its original unexpired proof."""
from contextlib import ExitStack
from unittest import mock

import pytest
from openpilot.cereal import log, messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_ev6_physical_authorization import ev6_params
from openpilot.starpilot.aol.tests.test_ordinary_source_availability import settings

SOURCE = 10_020_000_000


@pytest.mark.parametrize('socket_timeout', [False, True])
@pytest.mark.parametrize('age_ns,published_offset_ns', [
  (30_960_101, 4_945_854),
  (31_244_572, 5_262_889),
  (31_383_633, 5_039_179),
  (30_890_079, 4_798_596),
  (31_008_968, 5_325_976),
  (31_351_205, 5_392_434),
  (31_544_229, 5_502_541),
  (32_161_269, 6_033_539),
  (31_684_749, 5_741_086),
  (31_896_523, 5_808_274),
  (32_254_551, 5_671_607),
  (31_565_476, 5_393_216),
  (30_879_839, 5_223_629),
])
def test_ev6_recorded_source_cutoff_retains_only_original_unexpired_output(age_ns, published_offset_ns, socket_timeout):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    cp = ev6_params()
    sd, controls, drive = case._authority_callers(True, cp=cp, genuine_events=True)
    assert sd.CP.carFingerprint == 'KIA_EV6'
    assert sd.CP.safetyConfigs[0].safetyParam == 0x815 and sd.CP.alternativeExperience == 32
    for tick in (0, 1):
      drive(tick, **settings('combined', 10_000_000_000 + tick * 10_000_000, tick + 1))
    original, state, command = drive(2, state_stamp=SOURCE, offset_ns=published_offset_ns, **settings('combined', SOURCE, 3))
    assert original.lateralActive and original.longitudinalActive and command.latActive and command.longActive
    deadline = original.validUntilMonoTime
    assert deadline == SOURCE + published_offset_ns + 30_000_000
    old = original.to_dict()
    timer = sd.state_machine.soft_disable_timer
    sd.car_state_poller = mock.Mock()
    sd.car_state_poller.poll.return_value = []
    axis, state, command = drive(5, timeout=socket_timeout, state_stamp=SOURCE, offset_ns=age_ns - 30_000_000,
                                 **settings('combined', SOURCE, 3))
    assert sd.aol_axis_publication_deferred
    if socket_timeout:
      assert sd.aol_car_state_log_ns == 0 and not sd.conditional_car_state_valid
    else:
      assert sd.aol_car_state_log_ns == SOURCE and sd.conditional_car_state_valid
    assert axis.to_dict() == old and axis.validUntilMonoTime == deadline
    assert state.enabled and sd.state_machine.soft_disable_timer == timer
    assert log.OnroadEvent.EventName.commIssue not in sd.events.names
    # The consumer stops at the original cutoff; retaining a proof never renews it.
    with mock.patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=deadline + 1):
      expired, _ = controls.state_control()
    assert not expired.latActive and not expired.longActive
    # A second stale loop cannot repeat the deferral even within that deadline.
    next_axis, _, next_command = drive(5, timeout=True, state_stamp=SOURCE, offset_ns=age_ns - 30_000_000 + 100_000,
                                       **settings('combined', SOURCE, 3))
    assert not sd.aol_axis_publication_deferred
    assert not next_axis.lateralActive and not next_axis.longitudinalActive
    assert not next_command.latActive and not next_command.longActive


@pytest.mark.parametrize('fault', ['can', 'epsTemporary', 'epsPermanent', 'acc', 'brake', 'gas',
                                 'pandaRx', 'relay', 'nativeInvalid', 'uncalibrated'])
def test_ev6_unexpired_output_never_defers_a_new_fault(fault):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    cleanup.enter_context(OpenpilotPrefix())
    sd, _, drive = case._authority_callers(True, cp=ev6_params(), genuine_events=True)
    for tick in (0, 1, 2):
      stamp = 10_000_000_000 + tick * 10_000_000
      axis, _, command = drive(tick, state_stamp=stamp, offset_ns=4_945_854 if tick == 2 else 0,
                               **settings('combined', stamp, tick + 1))
    assert axis.lateralActive and axis.longitudinalActive and command.latActive and command.longActive
    state = fixture.car_state()
    changes = {}
    if fault == 'can':
      state.canValid = False
    elif fault == 'epsTemporary':
      state.steerFaultTemporary = True
    elif fault == 'epsPermanent':
      state.steerFaultPermanent = True
    elif fault == 'acc':
      state.accFaulted = True
    elif fault == 'brake':
      changes['brake'] = True
    elif fault == 'gas':
      changes['gas'] = True
    elif fault == 'pandaRx':
      changes['panda_fault'] = True
    elif fault == 'relay':
      changes['relay_fault'] = True
    elif fault == 'nativeInvalid':
      changes['native_valid'] = False
    elif fault == 'uncalibrated':
      # Calibration publishes at 1 Hz; the normal fixture cadence correctly
      # skips a replacement only 30 ms later. Deliver the new envelope itself.
      stamp = SOURCE + 30_960_101
      calibration = messaging.new_message('extrinsicsCalibration', valid=True, logMonoTime=stamp)
      calibration.extrinsicsCalibration.calStatus = 'uncalibrated'
      sd.sm.update_msgs(stamp / 1e9, [calibration.as_reader()])
    if fault in ('can', 'epsTemporary', 'epsPermanent', 'acc'):
      changes['observed_state'] = state
    axis, _, command = drive(5, state_stamp=SOURCE, offset_ns=960_101,
                             **settings('combined', SOURCE, 3), **changes)
    if fault == 'uncalibrated':
      assert sd.sm['extrinsicsCalibration'].calStatus == log.ExtrinsicsCalibration.Status.uncalibrated
    assert not sd.aol_axis_publication_deferred
    # The existing fault/pedal rules select the remaining axes; the previous
    # combined proof cannot survive the new restriction.
    assert not (axis.lateralActive and axis.longitudinalActive)
    assert not (command.latActive and command.longActive)


@pytest.mark.parametrize('timing', ['timely', 'late_poll', 'processing_crosses_deadline'])
def test_ev6_fresh_pair_reconciles_without_extending_the_old_deadline(timing):
  from openpilot.starpilot.aol.tests import test_input_deadline_continuity as continuity

  def ev6_callers(case, full_op=True, *, stock_angle=False):
    assert full_op and not stock_angle
    sd, controls, drive = case._authority_callers(True, cp=ev6_params(), genuine_events=True)
    assert sd.CP.carFingerprint == 'KIA_EV6'
    assert sd.CP.safetyConfigs[0].safetyParam == 0x815 and sd.CP.alternativeExperience == 32
    for tick in (0, 1):
      stamp = 10_000_000_000 + tick * 10_000_000
      drive(tick, **settings('combined', stamp, tick + 1))
    return sd, controls, drive, None, None

  # Reuse the existing timing stimulus with the captured EV6 configuration;
  # production event, source, native and consumer checks remain real.
  with mock.patch.object(fixture.IoniqAvailabilityTests, '_ioniq_availability_callers', ev6_callers):
    continuity.test_fresh_source_replaces_expired_output_without_restoring_disengagement('ordinary', timing)
