"""Original Controls, real SubMaster rate history and first identity binding."""
from contextlib import contextmanager
from unittest import mock

import pytest
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR
from openpilot.cereal import messaging
from openpilot.starpilot.flm.tests.test_live_calibration_paths import BASE, live_frame
from openpilot.starpilot.flm.tests.test_registered_controls_global import controls_for


@contextmanager
def calibration_receipts(controls, *, first=0, second=5, period=25, blocks_before_105=5):
  original = controls.sm.update_msgs
  sent = set()

  def deliver(now, events):
    rows = [event for event in events if event.which() != 'extrinsicsCalibration']
    tick = round((now * 1e9 - BASE) / 10_000_000)
    due = tick == first or tick >= second and (tick-second) % period == 0
    if now >= BASE / 1e9 and due and tick not in sent:
      event = messaging.new_message('extrinsicsCalibration', valid=True, logMonoTime=BASE+tick*10_000_000)
      event.extrinsicsCalibration.calStatus = 'calibrated'
      event.extrinsicsCalibration.validBlocks = blocks_before_105 if tick < 105 else 5
      event.extrinsicsCalibration.rpyCalib = (0., 0., 0.)
      event.extrinsicsCalibration.rpyCalibSpread = (0., 0., 0.)
      rows.append(event.as_reader())
      sent.add(tick)
    return original(now, rows)

  with mock.patch.object(controls.sm, 'update_msgs', side_effect=deliver):
    yield original


def no_source_before_85(event):
  if int(event.logMonoTime) < BASE+850_000_000:
    event.autoSteeringLearning.learning.sourceReceipt.version = 0


def frame(controls, tick, generation='a'*32, **kwargs):
  return live_frame(controls, tick, generation, blocks=5,
                    publish=tick % 25 == 10, wire_mutator=no_source_before_85, **kwargs)


def test_actual_nonzero_first_rate_failure_defers_binding_then_fresh_source_admits():
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls):
    original_tune = controls.LaC.torque_params.to_dict()
    for tick in range(121):
      frame(controls, tick)
      lifecycle = controls.global_learning._calibration_lifecycle
      if tick == 85:
        frequency = controls.sm.freq_tracker['extrinsicsCalibration']
        assert frequency.avg_dt.count == frequency.recent_avg_dt.count == 4
        assert frequency.avg_dt.get_average() == pytest.approx(.2)
        assert not controls.sm.freq_ok['extrinsicsCalibration']
        # OLD captures and retires here; NEW keeps no admitted identity.
        assert 'a'*32 not in lifecycle.rejected
        assert lifecycle.generation == '' and not lifecycle.current
        assert not controls.global_learning.eligible
      if 85 <= tick < 105:
        assert not controls.global_learning.eligible
        assert lifecycle.last_invalid_ns == BASE+tick*10_000_000
      if tick == 105:
        assert controls.sm.freq_ok['extrinsicsCalibration']
        assert not controls.global_learning.eligible  # Receipt85 predates last invalid104.
    assert lifecycle.generation == 'a'*32 and 'a'*32 not in lifecycle.rejected
    assert controls.global_learning.eligible
    assert controls.LaC.torque_params.to_dict() == original_tune


def test_actual_zero_interval_defers_without_admission_or_frequency_flag_writes():
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls, first=80, second=105):
    for tick in range(121):
      frame(controls, tick)
      lifecycle = controls.global_learning._calibration_lifecycle
      if tick == 85:
        frequency = controls.sm.freq_tracker['extrinsicsCalibration']
        assert frequency.avg_dt.count == frequency.recent_avg_dt.count == 0
        assert not controls.sm.freq_ok['extrinsicsCalibration']
        assert lifecycle.generation == '' and 'a'*32 not in lifecycle.rejected
        assert not lifecycle.current and not controls.global_learning.eligible
    assert controls.global_learning.eligible


@pytest.mark.parametrize('failure', ['invalid', 'future', 'invalid_wire', 'regression'])
def test_pending_true_calibration_fault_retires_and_same_generation_cannot_return(failure):
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls) as original_delivery:
    for tick in range(86):
      frame(controls, tick)
    runtime, sm = controls.global_learning, controls.sm
    assert runtime._calibration_pending == ('a'*32, 5)
    now = BASE+860_000_000
    header = now-500_000_001 if failure == 'stale' else now+1 if failure == 'future' else now
    event = messaging.new_message('extrinsicsCalibration', valid=failure != 'invalid_wire', logMonoTime=header)
    event.extrinsicsCalibration.calStatus = 'invalid' if failure == 'invalid' else 'calibrated'
    event.extrinsicsCalibration.validBlocks = 4 if failure == 'regression' else 5
    event.extrinsicsCalibration.rpyCalib = (0., 0., 0.)
    event.extrinsicsCalibration.rpyCalibSpread = (0., 0., 0.)
    original_delivery(now/1e9, [event.as_reader()])
    runtime.prepare(sm, now_ns=now, active=True, healthy=False)
    assert 'a'*32 in runtime._calibration_lifecycle.rejected
    assert not runtime._calibration_lifecycle.current and not runtime.eligible
    for tick in range(87, 146):
      frame(controls, tick)
      assert 'a'*32 in runtime._calibration_lifecycle.rejected
      assert not runtime.eligible


def test_pending_semantically_valid_block_regression_retires_generation():
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls) as original_delivery:
    for tick in range(86):
      frame(controls, tick)
    runtime, sm = controls.global_learning, controls.sm
    event = messaging.new_message('extrinsicsCalibration', valid=True, logMonoTime=BASE+860_000_000)
    event.extrinsicsCalibration = sm['extrinsicsCalibration']
    event.extrinsicsCalibration.validBlocks = 6
    original_delivery((BASE+860_000_000)/1e9, [event.as_reader()])
    runtime.prepare(sm, now_ns=BASE+860_000_000, active=True, healthy=False)
    assert runtime._calibration_pending == ('a'*32, 6)
    event.logMonoTime = BASE+870_000_000
    event.extrinsicsCalibration.validBlocks = 5
    original_delivery((BASE+870_000_000)/1e9, [event.as_reader()])
    runtime.prepare(sm, now_ns=BASE+870_000_000, active=True, healthy=False)
    assert 'a'*32 in runtime._calibration_lifecycle.rejected
    assert runtime._calibration_pending is None and not runtime.eligible
    for tick in range(88, 146):
      frame(controls, tick)
      assert 'a'*32 in runtime._calibration_lifecycle.rejected
      assert not runtime.eligible



def test_pending_generation_replacement_retires_previous_and_reappearance_cannot_admit():
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls, period=10):
    for tick in range(136):
      generation = 'a'*32 if tick < 110 or tick >= 135 else 'b'*32
      frame(controls, tick, generation)
      runtime = controls.global_learning
      if tick == 85:
        assert runtime._calibration_pending == ('a'*32, 5)
      if tick == 110:
        assert runtime._calibration_pending == ('b'*32, 5)
        assert 'a'*32 in runtime._calibration_lifecycle.rejected
      assert not runtime.eligible
    assert 'a'*32 in runtime._calibration_lifecycle.rejected
    assert 'b'*32 in runtime._calibration_lifecycle.rejected
    assert runtime._calibration_pending is None


def test_pending_generation_history_cannot_bypass_original_exhaustion_bound():
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls, period=10):
    for tick in range(936):
      generation = f'{max(1, (tick-85)//25+1):032x}'
      frame(controls, tick, generation)
      lifecycle = controls.global_learning._calibration_lifecycle
      assert len(lifecycle.rejected) <= 32
      assert not lifecycle.current and not controls.global_learning.eligible
    assert lifecycle.exhausted
    assert controls.global_learning._calibration_pending is None


@pytest.mark.parametrize('missing_tick', [100, 105])
def test_missing_source_preserves_regression_guard_before_or_at_rate_recovery(missing_tick):
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls, blocks_before_105=6):
    for tick in range(171):
      if tick == missing_tick:
        def missing_source(event):
          event.autoSteeringLearning.learning.sourceReceipt.version = 0
        live_frame(controls, tick, 'a'*32, blocks=5, publish=True, wire_mutator=missing_source)
      else:
        frame(controls, tick)
      runtime = controls.global_learning
      if tick == 85:
        assert runtime._calibration_pending == ('a'*32, 6)
      if tick == 100 and missing_tick == 100:
        # Missing source during the unqualified rate phase binds the pending
        # token without admitting it. The subsequent block drop retires it.
        assert runtime._calibration_lifecycle.generation == 'a'*32
        assert not runtime._calibration_lifecycle.current
        assert runtime._calibration_lifecycle.last_invalid_ns == BASE+1_000_000_000
        assert runtime._calibration_pending is None
      if tick == 105:
        assert controls.sm.freq_ok['extrinsicsCalibration']
        assert runtime._calibration_lifecycle.current
        if missing_tick == 100:
          assert runtime._calibration_lifecycle.generation == 'a'*32
          assert runtime._calibration_pending is None
        else:
          # A source gap first seen with genuinely current raw calibration
          # keeps the saved6 blocks; later same-token5 cannot erase history.
          assert runtime._calibration_lifecycle.generation == ''
          assert runtime._calibration_pending == ('a'*32, 6)
      if tick >= 105 and missing_tick == 100:
        assert 'a'*32 in runtime._calibration_lifecycle.rejected
      assert not runtime.eligible
    assert 'a'*32 in runtime._calibration_lifecycle.rejected
    assert runtime._calibration_pending is None


def test_actual_controls_stale_calibration_pauses_until_fresh_source():
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, calibration_receipts(controls) as delivery:
    for tick in range(121):
      frame(controls, tick)
    runtime, sm = controls.global_learning, controls.sm
    assert runtime.eligible
    now = BASE + 1_210_000_000
    event = messaging.new_message('extrinsicsCalibration', valid=True, logMonoTime=now - 500_000_001)
    event.extrinsicsCalibration = sm['extrinsicsCalibration']
    delivery(now / 1e9, [event.as_reader()])
    runtime.prepare(sm, now_ns=now, active=True, healthy=True)
    assert not runtime.eligible and not runtime._calibration_lifecycle.current
    assert not runtime._calibration_lifecycle.admit('a' * 32, now + 1)
    assert 'a' * 32 not in runtime._calibration_lifecycle.rejected
    for tick in range(122, 221):
      frame(controls, tick)
    assert runtime.eligible
    assert runtime._calibration_lifecycle.generation == 'a' * 32
