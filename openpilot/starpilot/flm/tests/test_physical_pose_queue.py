"""The real optional observer retains original pose receipts until state brackets arrive."""

from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm.runtime_adapter_registry import physical_observer
from openpilot.starpilot.flm.tests.test_live_calibration_paths import PhysicalSources

BASE = 20_000_000_000


def command(stamp, *, speed=20., angle=0., **changes):
  values = {'identity': 'a' * 64, 'generation': 1, 'source_epoch': 'b' * 64,
                'baseline_revision': 'baseline', 'controller_revision': 'controller', 'native_identity': 'native',
                'state_ns': stamp, 'replay_inputs': (('vm.sR', 14.), ('vm.cF', 100000.), ('vm.cR', 100000.),
                                               ('vEgo', speed), ('steeringAngleDeg', angle))}
  values.update(changes)
  return SimpleNamespace(**values)


def sources(stamp):
  dm = messaging.new_message('deviceMotion').deviceMotion
  dm.timestamp = stamp
  dm.inputsOK = dm.sensorsOK = dm.posenetOK = True
  for name, xyz in (('angularVelocityDevice', (.05, .1, .02)), ('orientationNED', (.01, .02, .03))):
    value = getattr(dm, name)
    value.valid = True
    value.x, value.y, value.z = xyz
  cal = messaging.new_message('extrinsicsCalibration').extrinsicsCalibration
  cal.calStatus, cal.validBlocks = 'calibrated', 50
  cal.rpyCalib, cal.rpyCalibSpread = (0., 0., 0.), (0., .001, .001)
  cs = SimpleNamespace(steeringPressed=False, brakePressed=False, gasPressed=False, regenBraking=False,
                       buttonEvents=(), canTimeout=False, canValid=True, steerFaultTemporary=False,
                       steerFaultPermanent=False, accFaulted=False)
  return PhysicalSources({'deviceMotion': dm, 'extrinsicsCalibration': cal, 'carState': cs,
                              'lateralDelay': SimpleNamespace(lateralDelay=.1),
                              'driverMonitoringState': SimpleNamespace(noResponseForceDecel=False)}, stamp)


def fixture():
  collector = SimpleNamespace(session='session', model_block_sequence=0, _calibration_generation='c' * 32,
    _learning_current_native_allowed=True, ready=[], adapter=SimpleNamespace(command_domain='normalized_torque'))
  collector.split = lambda *args, **kwargs: collector.ready.clear()
  cp = SimpleNamespace(steerRatio=14., wheelbase=2.9, centerToFront=1.2, mass=2000.)
  return collector, cp, sources(BASE)


def observe(collector, cp, sm, now):
  with mock.patch('openpilot.starpilot.flm.runtime_adapter_registry.time.monotonic_ns', return_value=now), \
       mock.patch('openpilot.starpilot.flm.runtime_adapter_registry.time.clock_gettime_ns', return_value=now):
    return physical_observer(sm, cp, collector, now)


def add_state(collector, sm, stamp, **changes):
  collector.ready.append((command(stamp, **changes), SimpleNamespace(native_allowed=True, apply_ns=stamp)))
  sm.logMonoTime['carState'] = stamp


def test_actual_observer_retries_original_pose_after_later_right_bracket():
  collector, cp, sm = fixture()
  add_state(collector, sm, BASE - 10_000_000, speed=20., angle=0.)
  assert observe(collector, cp, sm, BASE) is None
  add_state(collector, sm, BASE + 10_000_000, speed=22., angle=2.)
  sm.recv_time['deviceMotion'] = (BASE + 9_000_000) / 1e9  # A poll cannot renew the original receipt.
  result = observe(collector, cp, sm, BASE + 10_000_000)
  assert result is not None
  assert result.mono_ns == result.source_boot_ns == result.received_ns == BASE
  assert result.clock_offset_ns == result.clock_uncertainty_ns == 0
  assert result.speed_mps == 21. and result.steering_angle_deg == 1.
  assert result.yaw_rate == pytest.approx(.02)
  assert observe(collector, cp, sm, BASE + 10_000_001) is None


@pytest.mark.parametrize('transition', ['native', 'source', 'reset', 'baseline', 'calibration', 'vm', 'override', 'fault', 'invalid', 'expired'])
def test_pending_pose_cannot_cross_health_or_identity_transition(transition):
  collector, cp, sm = fixture()
  add_state(collector, sm, BASE - 10_000_000)
  assert observe(collector, cp, sm, BASE) is None
  changes = {}
  if transition == 'native':
    changes['native_identity'] = 'new-native'
  elif transition == 'source':
    changes['source_epoch'] = 'd' * 64
  elif transition == 'reset':
    collector.model_block_sequence += 1
  elif transition == 'baseline':
    changes['baseline_revision'] = 'changed'
  elif transition == 'calibration':
    sm['extrinsicsCalibration'].rpyCalib = (0., .001, .001)
  elif transition == 'vm':
    changes['replay_inputs'] = (('vm.sR', 16.), ('vm.cF', 100000.), ('vm.cR', 100000.), ('vEgo', 20.), ('steeringAngleDeg', 0.))
  elif transition == 'override':
    sm['carState'].brakePressed = True
  elif transition == 'fault':
    sm['carState'].canValid = False
  elif transition == 'invalid':
    sm['deviceMotion'].inputsOK = False
  add_state(collector, sm, BASE + 10_000_000, **changes)
  now = BASE + (100_000_001 if transition == 'expired' else 10_000_000)
  assert observe(collector, cp, sm, now) is None
  assert not collector._physical_pose_queue


def test_newer_pose_does_not_replace_queued_original_or_its_receipt():
  collector, cp, sm = fixture()
  add_state(collector, sm, BASE - 10_000_000)
  assert observe(collector, cp, sm, BASE) is None
  sm['deviceMotion'].timestamp = BASE + 50_000_000
  sm['deviceMotion'].angularVelocityDevice.z = .08
  sm.recv_time['deviceMotion'] = (BASE + 50_000_000) / 1e9
  add_state(collector, sm, BASE + 10_000_000)
  result = observe(collector, cp, sm, BASE + 50_000_000)
  assert result is not None and result.mono_ns == result.received_ns == BASE
  assert result.yaw_rate == pytest.approx(.02)
  add_state(collector, sm, BASE + 60_000_000)
  result = observe(collector, cp, sm, BASE + 60_000_000)
  assert result is not None and result.mono_ns == result.received_ns == BASE + 50_000_000
  assert result.yaw_rate == pytest.approx(.08)
