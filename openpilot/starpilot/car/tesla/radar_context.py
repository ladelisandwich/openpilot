"""Reserved custom startup context; no live Params or core schema extension."""
import hashlib
import struct

import capnp

from openpilot.cereal import custom
from opendbc.car.tesla.preap.radar_profile import RadarStartupContext, qualified, valid_context
from openpilot.starpilot.car.tesla.radar_settings import RadarSettings, offset_float32

VERSION = 1
KIND = 4
MAX_BYTES = 256
MAX_WORDS = (MAX_BYTES - 8) // 8
SCHEMA = custom.StarPilotCarControl.PreapRadarStartup


def encode_context(cp, cp_bytes, context):
  if not valid_context(cp, context, cp_bytes):
    raise ValueError("Invalid Tesla radar startup context")
  message = SCHEMA.new_message(version=VERSION, kind=KIND, carParamsSha256=context.car_params_sha256,
                               safetyModel=39, safetyParam=context.safety_param, lateralOffset=context.offset)
  raw = message.to_bytes()
  if len(raw) > MAX_BYTES:
    raise ValueError("Oversized Tesla radar startup context")
  return raw


def decode_context(cp, cp_bytes, raw):
  if type(raw) is not bytes or not 16 <= len(raw) <= MAX_BYTES or len(raw) % 8:
    return None
  segments, words = struct.unpack_from('<II', raw)
  if segments != 0 or words > MAX_WORDS or len(raw) != 8 + words * 8:
    return None
  try:
    with SCHEMA.from_bytes(raw, traversal_limit_in_words=MAX_WORDS, nesting_limit=4) as value:
      if int(value.version) != VERSION or int(value.kind) != KIND or int(value.safetyModel) != 39:
        return None
      context = RadarStartupContext(int(value.version), int(value.safetyParam), bytes(value.carParamsSha256), float(value.lateralOffset))
    return context if valid_context(cp, context, cp_bytes) else None
  except (ValueError, TypeError, OverflowError, RuntimeError, capnp.KjException):
    return None


def install_startup_context(ri, cp, cp_bytes, settings):
  if not qualified(cp):
    return None
  word = int(cp.safetyConfigs[0].safetyParam)
  if (type(settings) is not RadarSettings or type(settings.valid) is not bool or not settings.valid or
      type(settings.enabled) is not bool or not settings.enabled or type(settings.behind_nosecone) is not bool or
      word != ((6 if settings.behind_nosecone else 2) | int(cp.openpilotLongitudinalControl))):
    raise ValueError("Tesla radar capability does not match its frozen startup settings")
  context = RadarStartupContext(VERSION, word, hashlib.sha256(cp_bytes).digest(), offset_float32(settings.offset))
  decoded = decode_context(cp, cp_bytes, encode_context(cp, cp_bytes, context))
  if decoded is None:
    raise ValueError("Tesla radar startup context did not survive serialization")
  ri.configure_preap_radar(decoded, cp_bytes)
  return decoded
