"""Versioned custom nested startup capsule; exact final CP and frozen settings."""
import hashlib
import struct
import time
import capnp

from opendbc.car import Bus, structs
from opendbc.car.tesla.preap.pedal_contract import PedalCalibration, PedalFeedback
from opendbc.car.tesla.preap.pedal_profile import qualified, accel_max
from opendbc.car.tesla.preap.pedal_parser import PedalCANParser
from opendbc.car.tesla.preap.pedal_control import PreAPPedalControl
from openpilot.cereal import custom
from openpilot.starpilot.car.tesla.pedal_preferences import PedalSettings

KEY = 'TeslaPreAPPedalStartup'
VERSION = 1
KIND = 5
MAX_BYTES = 512
SCHEMA = custom.StarPilotCarControl.PreapPedalStartup


def decode(cp, cp_bytes, raw):
  if type(raw) is not bytes or not 16 <= len(raw) <= MAX_BYTES or len(raw) % 8 or not qualified(cp):
    return None
  segments, words = struct.unpack_from('<II', raw)
  if segments != 0 or len(raw) != 8 + words * 8 or words > 63:
    return None
  try:
    with SCHEMA.from_bytes(raw, traversal_limit_in_words=63, nesting_limit=4) as value:
      calibration = PedalCalibration(float(value.factor), float(value.zero), int(value.pedalBus))
      if (int(value.version) != VERSION or int(value.kind) != KIND or int(value.safetyModel) != 39 or
          int(value.safetyParam) != int(cp.safetyConfigs[0].safetyParam) or not calibration.valid() or
          len(value.carParamsSha256) != 32 or bytes(value.carParamsSha256) != hashlib.sha256(cp_bytes).digest() or
          value.sourceMonoTime <= 0 or value.sourceMonoTime > time.monotonic_ns()):
        return None
    with structs.CarParams.from_bytes(cp_bytes) as published:
      return calibration if published.to_dict() == cp.to_dict() else None
  except (AttributeError, ValueError, TypeError, OverflowError, RuntimeError, capnp.KjException):
    return None


def install(ci, cp, cp_bytes, settings, params):
  if not qualified(cp):
    params.remove(KEY)
    return None
  if (type(settings) is not PedalSettings or not settings.valid or not settings.enabled or
      settings.calibration is None or not settings.calibration.valid()):
    raise ValueError('preAP pedal startup settings missing or invalid')
  calibration = settings.calibration
  msg = SCHEMA.new_message(version=VERSION, kind=KIND, safetyModel=39,
    safetyParam=int(cp.safetyConfigs[0].safetyParam), carParamsSha256=hashlib.sha256(cp_bytes).digest(),
    factor=calibration.factor, zero=calibration.zero, pedalBus=calibration.bus, sourceMonoTime=time.monotonic_ns())
  raw = msg.to_bytes()
  if decode(cp, cp_bytes, raw) != calibration:
    raise ValueError('preAP pedal startup capsule failed roundtrip')
  previous = getattr(ci, 'preap_pedal_cp_sha256', None)
  digest = hashlib.sha256(cp_bytes).digest()
  if previous is not None:
    raise ValueError('preAP pedal startup context may only be installed once')
  feedback = PedalFeedback(calibration)
  ci.can_parsers[Bus.ap_party] = PedalCANParser(feedback)
  ci.CS.preap.pedal_feedback = feedback
  ci.CS.preap.preap_accel_max = accel_max(0., settings.personality)
  ci.CS.preap.pedal_personality = settings.personality
  ci.CC.preap.pedal_control = PreAPPedalControl(calibration)
  ci.preap_pedal_cp_sha256 = digest
  params.put(KEY, raw, block=True)
  return calibration
