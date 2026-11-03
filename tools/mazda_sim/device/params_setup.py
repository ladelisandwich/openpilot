"""Puts the simulated car's fitment and the comma's first-run state into the build's params, before manager starts."""
from __future__ import annotations

from ..common.config import SimConfig

# param name -> CarOptions attribute
CAR_TOGGLES = {
  "TorqueInterceptorEnabled": "torque_interceptor",
  "RadarEmulationEnabled": "radar_emulation",
  "MazdaHybridLong": "hybrid_long",
  "NoFSC": "no_fsc",
  "NoMRCC": "no_mrcc",
  "ManualTransmission": "manual_transmission",
  "ExperimentalMode": "experimental_mode",
  "LowerMinSetSpeed": "lower_min_set_speed",
}


def _put(params, key: str, value) -> bool:
  """Write a param the build may or may not know. Returns False for keys this build does not have."""
  try:
    if isinstance(value, bool):
      params.put_bool(key, value)
    elif isinstance(value, (bytes, int, float)):
      params.put(key, value)
    else:
      params.put(key, value if isinstance(value, str) else str(value))
    return True
  except Exception:
    try:
      params.put(key, str(value).encode() if not isinstance(value, bytes) else value)
      return True
    except Exception as e:
      print(f"params_setup: {key} not set ({e})")
      return False


def apply(cfg: SimConfig) -> None:
  import cereal.messaging as messaging
  from openpilot.common.params import Params
  params = Params()

  # past onboarding, as a car that has been driven: terms, training, openpilot on, calibrated
  try:
    from openpilot.system.version import terms_version, training_version
    _put(params, "HasAcceptedTerms", terms_version)
    _put(params, "CompletedTrainingVersion", training_version)
  except Exception as e:
    print(f"params_setup: onboarding versions unknown ({e})")
  _put(params, "OpenpilotEnabledToggle", True)
  msg = messaging.new_message('liveCalibration')
  msg.liveCalibration.validBlocks = 20
  msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
  _put(params, "CalibrationParams", msg.to_bytes())
  # a fresh fingerprint every start: the simulated car may have been re-configured
  params.remove("CarParamsCache")
  # StarPilot tars up the whole install (~1 GB, minutes of CPU) at every boot while there is room for three such
  # backups: pointless for a build that is rebuilt from source here, and it fills the comma's volume
  _put(params, "MinimumBackupSize", 1 << 50)

  car = cfg.car
  for key, attr in CAR_TOGGLES.items():
    _put(params, key, bool(getattr(car, attr)))
  _put(params, "RadarInterceptorEnabled", False)
  _put(params, "IsMetric", not car.imperial)
  for key, value in (car.extra_params or {}).items():
    _put(params, key, value)
