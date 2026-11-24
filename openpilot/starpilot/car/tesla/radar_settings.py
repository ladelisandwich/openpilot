from dataclasses import dataclass
import json
import math
import struct

from openpilot.starpilot.saved_source import read_saved

DOCUMENT_KEY = "TeslaPreAPRadarConfig"
MAX_BYTES = 1024
MODES = ("off", "front", "nosecone")


def offset_float32(value):
  if isinstance(value, bool):
    raise ValueError("Invalid radar offset")
  value = struct.unpack("f", struct.pack("f", float(value)))[0]
  if not math.isfinite(value):
    raise ValueError("Invalid radar offset")
  return value


@dataclass(frozen=True)
class RadarSettings:
  enabled: bool
  behind_nosecone: bool
  offset: float
  valid: bool
  sources: tuple = ()


def encode(mode, offset):
  if mode not in MODES:
    raise ValueError("Invalid radar mode")
  return json.dumps({"version": 1, "mode": mode, "offset": offset_float32(offset)},
                    sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def read_radar_settings(params):
  sources = ()
  try:
    document, readable = read_saved(params, DOCUMENT_KEY, MAX_BYTES)
    sources = ((DOCUMENT_KEY, document),)
    if not readable:
      return RadarSettings(False, False, 0.0, False, sources)
    if document is not None:
      def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
          if key in result:
            raise ValueError("Duplicate radar configuration key")
          result[key] = value
        return result

      value = json.loads(document, object_pairs_hook=unique_pairs)
      if (not isinstance(value, dict) or set(value) != {"version", "mode", "offset"} or
          type(value["version"]) is not int or value["version"] != 1 or value["mode"] not in MODES or
          type(value["offset"]) not in (int, float)):
        raise ValueError("Invalid radar configuration")
      offset = offset_float32(value["offset"])
      return RadarSettings(value["mode"] != "off", value["mode"] == "nosecone", offset, True, sources)
    enabled, enabled_ok = read_saved(params, "NAPRadarEnabled", 8)
    mount, mount_ok = read_saved(params, "NAPRadarBehindNosecone", 8)
    sources += (("NAPRadarEnabled", enabled), ("NAPRadarBehindNosecone", mount))
    flags_valid = bool(enabled_ok and mount_ok and enabled in (None, b"0", b"1") and mount in (None, b"0", b"1"))
    if not flags_valid or (mount == b"1" and enabled != b"1"):
      return RadarSettings(False, False, 0.0, False, sources)
    if enabled != b"1":
      return RadarSettings(False, False, 0.0, True, sources)
    raw, offset_ok = read_saved(params, "NAPRadarOffset", 64)
    sources += (("NAPRadarOffset", raw),)
    offset = offset_float32(0.0 if raw is None else float(raw))
    return RadarSettings(True, mount == b"1", offset, bool(offset_ok), sources)
  except (OSError, TypeError, ValueError, OverflowError):
    return RadarSettings(False, False, 0.0, False, sources)
