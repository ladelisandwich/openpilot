"""Real SubMaster/observer work and refusal contracts under declared fixture inputs.

The native input lease is a fixture, not CAN-earned permission or road evidence.
No receiver health flag or all_checks implementation is replaced.
"""
from contextlib import contextmanager
from collections import deque
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.locationd.helpers import PoseCalibrator
from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence, ControlEvidence, FullrateCollector
from openpilot.starpilot.flm.original_source_capture import OriginalSourceClockContract, OriginalSourcePose, validate_pose
from openpilot.starpilot.flm.physical_observation import PendingPhysical
from openpilot.starpilot.flm.runtime_adapter_registry import original_source_pose_observer

BASE = 400_000_000_000
SERVICES = ('carState', 'deviceMotion', 'extrinsicsCalibration', 'driverMonitoringState')


def packet(service, stamp):
  event = messaging.new_message(service, valid=True, logMonoTime=stamp)
  value = getattr(event, service)
  if service == 'carState':
    value.canValid = True
    value.vEgo, value.steeringAngleDeg = 20., 0.
  elif service == 'deviceMotion':
    value.timestamp = stamp
    value.inputsOK = value.sensorsOK = value.posenetOK = True
    for name, xyz in (('angularVelocityDevice', (.05, .1, .02)), ('orientationNED', (.01, .02, .03))):
      vector = getattr(value, name)
      vector.valid = True
      vector.x, vector.y, vector.z = xyz
  elif service == 'extrinsicsCalibration':
    value.calStatus, value.validBlocks = 'calibrated', 50
    value.rpyCalib, value.rpyCalibSpread = (0., 0., 0.), (0., .001, .001)
  return event


def pair(stamp, *, speed=20., angle=0.):
  control = ControlEvidence('a'*64, 1, stamp+1_000_000, stamp, 'normalized_torque', 'stage', 'response',
    .1, (.1,), .1, True, False, True, True, False, False, False, False, False, False,
    source_epoch='b'*64, baseline_revision='baseline', controller_revision='controller',
    replay_inputs=(('vm.sR', 14.), ('vm.cF', 100000.), ('vm.cR', 100000.), ('vEgo', speed), ('steeringAngleDeg', angle)),
    native_required=True, native_allowed=True, native_identity='f'*64,
    native_observed_ns=BASE-100_000_000, native_expires_ns=BASE+100_000_000)
  output = AppliedEvidence(control.control_ns, stamp, stamp+2_000_000, 'normalized_torque', .1,
    True, True, False, False, False, True)
  return control, output


class CountedReady(deque):
  def __init__(self):
    super().__init__(maxlen=128)
    self.iterations = 0

  def __iter__(self):
    self.iterations += 1
    return super().__iter__()


@contextmanager
def fixture():
  with OpenpilotPrefix():
    sm = messaging.SubMaster(list(SERVICES), poll='carState')
    assert not sm.simulation
    for tick in range(-200, 1):
      stamp = BASE + tick*10_000_000
      events = [packet('carState', stamp)]
      if tick % 5 == 0:
        events += [packet('deviceMotion', stamp), packet('driverMonitoringState', stamp)]
      if tick % 25 == 0:
        events += [packet('extrinsicsCalibration', stamp)]
      sm.update_msgs(stamp/1e9, [event.as_reader() for event in events])
    assert sm.all_checks(list(SERVICES))
    adapter = SimpleNamespace(command_domain='normalized_torque', enable_original_context_capture=lambda: None)
    collector = FullrateCollector(adapter)
    collector.enable_original_source_capture(clock_contract=OriginalSourceClockContract(
      'monotonic', 'c'*64, 'd'*64, 'synthetic-fixture'))
    collector._calibration_generation = 'e'*32
    collector._learning_current_native_allowed = True  # Declared caller input, never a native qualification.
    collector.ready = CountedReady()
    collector.ready.append(pair(BASE-10_000_000))
    cp = SimpleNamespace(steerRatio=14., wheelbase=2.9, centerToFront=1.2, mass=2000.)
    yield collector, cp, sm


def observe(collector, cp, sm, now):
  with mock.patch('openpilot.starpilot.flm.runtime_adapter_registry.time.monotonic_ns', return_value=now), \
       mock.patch('openpilot.starpilot.flm.runtime_adapter_registry.time.clock_gettime_ns', return_value=now):
    return original_source_pose_observer(sm, cp, collector, now, distinguish_pending=True)


def right_bracket(collector, sm):
  stamp = BASE+10_000_000
  sm.update_msgs(stamp/1e9, [packet('carState', stamp).as_reader()])
  collector.ready.append(pair(stamp, speed=22., angle=2.))
  assert sm.all_checks(list(SERVICES))


def test_original_pose_is_converted_once_across_bracket_retry():
  with fixture() as (collector, cp, sm):
    original = PoseCalibrator.build_calibrated_pose
    with mock.patch.object(PoseCalibrator, 'build_calibrated_pose', autospec=True, side_effect=original) as conversion:
      assert type(observe(collector, cp, sm, BASE)) is PendingPhysical
      queued = collector._original_source_pose_queue[0]
      right_bracket(collector, sm)
      row = observe(collector, cp, sm, BASE+12_000_000)
      assert type(row) is OriginalSourcePose
      validate_pose(row, BASE+12_000_000)
      assert (row.mono_ns, row.received_ns, row.observed_ns) == (BASE, BASE, BASE)
      assert (row.speed_mps, row.steering_angle_deg) == (21., 1.)
      assert (row.yaw_rate, row.roll_rad) == (queued.yaw_rate, queued.roll_rad)
      assert conversion.call_count == 1


def test_consumed_duplicate_does_not_rebuild_state_brackets():
  with fixture() as (collector, cp, sm):
    assert type(observe(collector, cp, sm, BASE)) is PendingPhysical
    right_bracket(collector, sm)
    assert type(observe(collector, cp, sm, BASE+12_000_000)) is OriginalSourcePose
    assert not collector._original_source_pose_queue
    before = collector.ready.iterations
    waiting = observe(collector, cp, sm, BASE+13_000_000)
    assert type(waiting) is PendingPhysical
    assert waiting.mono_ns == waiting.received_ns == BASE
    assert collector.ready.iterations == before


@pytest.mark.parametrize('refusal', ['native', 'override', 'fault', 'envelope', 'pose', 'expiry', 'source', 'vm'])
def test_duplicate_wait_does_not_bypass_current_refusals(refusal):
  with fixture() as (collector, cp, sm):
    assert type(observe(collector, cp, sm, BASE)) is PendingPhysical
    right_bracket(collector, sm)
    now = BASE+12_000_000
    if refusal == 'native':
      collector._learning_current_native_allowed = False
    elif refusal in ('override', 'fault'):
      event = packet('carState', BASE+10_000_000)
      if refusal == 'override':
        event.carState.brakePressed = True
      else:
        event.carState.canValid = False
      sm.update_msgs((BASE+10_000_000)/1e9, [event.as_reader()])
    elif refusal in ('envelope', 'pose'):
      event = packet('deviceMotion', BASE)
      if refusal == 'envelope':
        event.valid = False
      else:
        event.deviceMotion.inputsOK = False
      sm.update_msgs((BASE+10_000_000)/1e9, [event.as_reader()])
    elif refusal == 'expiry':
      now = BASE+100_000_001
    elif refusal == 'source':
      control, output = collector.ready[-1]
      collector.ready[-1] = (replace(control, source_epoch='9'*64), output)
    elif refusal == 'vm':
      control, output = collector.ready[-1]
      collector.ready[-1] = (replace(control, replay_inputs=(('vm.sR', 16.), *control.replay_inputs[1:])), output)
    result = observe(collector, cp, sm, now)
    assert not isinstance(result, OriginalSourcePose)
    assert not collector._original_source_pose_queue


def conflated_source_receiver(*, slow_motion=False):
  from openpilot.starpilot.flm.steerlearnerd import source_receiver

  sm = source_receiver(list(SERVICES))
  assert not sm.simulation
  # A healthy 100Hz publisher can be received once per 20Hz motion window.
  # update_msgs is the actual SubMaster accounting path, not a flag override.
  for tick in range(-40, 1):
    stamp = BASE + tick * 50_000_000
    events = [packet('carState', stamp), packet('driverMonitoringState', stamp)]
    if not slow_motion or tick % 4 == 0:
      events.append(packet('deviceMotion', stamp))
    if tick % 5 == 0:
      events.append(packet('extrinsicsCalibration', stamp))
    sm.update_msgs(stamp / 1e9, [event.as_reader() for event in events])
  assert not sm.freq_ok['carState']
  assert sm.alive['carState'] and sm.valid['carState']
  return sm


def test_conflated_car_state_receive_rate_does_not_starve_original_motion():
  with fixture() as (collector, cp, _):
    sm = conflated_source_receiver()
    assert sm.all_checks(list(SERVICES))
    assert type(observe(collector, cp, sm, BASE)) is PendingPhysical
    stamp = BASE + 10_000_000
    sm.update_msgs(stamp / 1e9, [packet('carState', stamp).as_reader()])
    collector.ready.append(pair(stamp, speed=22., angle=2.))
    row = observe(collector, cp, sm, BASE + 12_000_000)
    assert type(row) is OriginalSourcePose
    validate_pose(row, BASE + 12_000_000)
    assert (row.mono_ns, row.received_ns, row.observed_ns) == (BASE, BASE, BASE)
    assert not sm.freq_ok['carState']  # The measured receiver verdict stays truthful.


@pytest.mark.parametrize('fault', ['invalid_state', 'stale_state', 'native', 'override', 'can_fault', 'motion_rate'])
def test_conflated_receiver_retains_source_native_and_motion_refusals(fault):
  with fixture() as (collector, cp, _):
    sm = conflated_source_receiver(slow_motion=fault == 'motion_rate')
    if fault in ('invalid_state', 'stale_state', 'override', 'can_fault'):
      stamp = BASE - 100_000_001 if fault == 'stale_state' else BASE
      event = packet('carState', stamp)
      if fault == 'invalid_state':
        event.valid = False
      elif fault == 'override':
        event.carState.brakePressed = True
      elif fault == 'can_fault':
        event.carState.canValid = False
      sm.update_msgs(BASE / 1e9, [event.as_reader()])
    elif fault == 'native':
      collector._learning_current_native_allowed = False
    before = tuple(tuple(getattr(sm, name)[service] for service in SERVICES)
                   for name in ('alive', 'valid', 'freq_ok', 'logMonoTime', 'recv_time'))
    assert observe(collector, cp, sm, BASE) is None
    after = tuple(tuple(getattr(sm, name)[service] for service in SERVICES)
                  for name in ('alive', 'valid', 'freq_ok', 'logMonoTime', 'recv_time'))
    assert after == before
    assert not collector._original_source_pose_queue
