"""Calibration is retained configuration; physical samples keep their own gates."""
from types import SimpleNamespace
import pytest
from openpilot.starpilot.flm.calibration_lifecycle import ProducerCalibrationLifecycle
from openpilot.starpilot.flm.steerlearnerd import _configuration_calibration

BASE = 632501433320


class CalibrationSource:
  def __init__(self, *, seen=True, valid=True, calibrated=True):
    self.seen = {'extrinsicsCalibration': seen}
    self.valid = {'extrinsicsCalibration': valid}
    self.logMonoTime = {'extrinsicsCalibration': BASE}
    self.recv_time = {'extrinsicsCalibration': BASE/1e9}
    self.message = SimpleNamespace(rpyCalib=(-.0000129, -.0230993, -.0274958),
      rpyCalibSpread=(.000135, .00904, .00597), validBlocks=50,
      calStatus='calibrated' if calibrated else 'uncalibrated')

  def __getitem__(self, service):
    assert service == 'extrinsicsCalibration'
    return self.message

  def all_checks(self, services):
    return False  # Receiver cadence is not the calibration's meaning.


@pytest.mark.parametrize('age_ns', (20_000_000, 600_000_000, 5_000_000_000))
def test_known_calibration_survives_missing_cadence_without_restamping(age_ns):
  sm = CalibrationSource()
  lifecycle = ProducerCalibrationLifecycle(token_factory=lambda:'a'*32)
  frame = _configuration_calibration(sm, lifecycle, BASE+age_ns)
  assert frame == ('a'*32, sm.message.rpyCalib)
  assert sm.logMonoTime['extrinsicsCalibration'] == BASE
  assert sm.recv_time['extrinsicsCalibration'] == BASE/1e9


@pytest.mark.parametrize('seen,valid,calibrated,now', (
  (False, True, True, BASE+1), (True, False, True, BASE+1),
  (True, True, False, BASE+1), (True, True, True, BASE-1),
))
def test_unseen_invalid_uncalibrated_or_future_configuration_is_rejected(seen, valid, calibrated, now):
  sm = CalibrationSource(seen=seen, valid=valid, calibrated=calibrated)
  assert _configuration_calibration(sm, ProducerCalibrationLifecycle(), now) is None


def test_explicit_calibration_reset_retires_prior_frame():
  sm = CalibrationSource()
  tokens = iter(('a'*32, 'b'*32))
  lifecycle = ProducerCalibrationLifecycle(token_factory=lambda:next(tokens))
  assert _configuration_calibration(sm, lifecycle, BASE+1)[0] == 'a'*32
  sm.message.calStatus = 'uncalibrated'
  assert _configuration_calibration(sm, lifecycle, BASE+2) is None
  sm.message.calStatus = 'calibrated'
  assert _configuration_calibration(sm, lifecycle, BASE+3)[0] == 'b'*32
