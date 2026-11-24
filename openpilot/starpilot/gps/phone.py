import json
import math
import threading
import time

FIELDS = {"latitude", "longitude", "altitude", "speed", "bearing", "horizontalAccuracy", "verticalAccuracy",
          "speedAccuracy", "bearingAccuracy", "timestamp", "ageMs"}
MAX_AGE_MS = 2000


def _number(payload, key, required=False):
  value = payload.get(key)
  if value is None and not required:
    return math.nan
  if type(value) not in (int, float) or not math.isfinite(value):
    raise ValueError(f"Invalid {key}")
  return float(value)


def _optional(payload, key, low, high):
  # Phones report an unknown value as null or a negative number.
  value = _number(payload, key)
  return value if low <= value < high else math.nan


def phone_gps(payload, now_ns):
  if not isinstance(payload, dict) or set(payload) - FIELDS:
    raise ValueError("Unexpected location fields")
  latitude, longitude = _number(payload, "latitude", True), _number(payload, "longitude", True)
  accuracy, age_ms = _number(payload, "horizontalAccuracy", True), _number(payload, "ageMs", True)
  if not (-90 <= latitude <= 90 and -180 <= longitude <= 180) or (latitude == 0 and longitude == 0):
    raise ValueError("Invalid position")
  if not 0 < accuracy <= 10_000:
    raise ValueError("Invalid accuracy")
  if not 0 <= age_ms <= MAX_AGE_MS:
    raise ValueError("Location is too old")
  timestamp = payload.get("timestamp", 0)
  if type(timestamp) is not int or not 0 <= timestamp < 2 ** 53:
    raise ValueError("Invalid timestamp")
  bearing_accuracy = _optional(payload, "bearingAccuracy", 0, 180)
  return {
    "sourceMonoTime": now_ns - int(age_ms * 1e6),
    "latitude": latitude,
    "longitude": longitude,
    "altitude": _optional(payload, "altitude", -1000, 10_000),
    "speed": _optional(payload, "speed", 0, 100),
    "bearingDeg": _optional(payload, "bearing", 0, 360) if math.isfinite(bearing_accuracy) else math.nan,
    "horizontalAccuracy": accuracy,
    "verticalAccuracy": _optional(payload, "verticalAccuracy", 0, 10_000),
    "bearingAccuracyDeg": bearing_accuracy,
    "speedAccuracy": _optional(payload, "speedAccuracy", 0, 100),
    "unixTimestampMillis": timestamp,
    "hasFix": True,
  }


def handoff_path():
  import tempfile
  from pathlib import Path
  shm = Path("/dev/shm")
  return (shm if shm.is_dir() else Path(tempfile.gettempdir())) / "starpilot_phone_gps.json"


class PhoneGpsPublisher:
  """Hands the newest validated phone fix to navigationd, which publishes it inside starpilotNavigation."""

  def __init__(self, path=None, mono_clock=time.monotonic_ns):
    self._path, self._mono_clock = path, mono_clock
    self._lock = threading.Lock()

  def publish(self, payload):
    from openpilot.starpilot.gps.source import PHONE_MAX_AGE_NS, MAX_HORIZONTAL_ACCURACY
    path = self._path or handoff_path()
    with self._lock:
      now = self._mono_clock()
      try:
        gps = phone_gps(payload, now)
      except ValueError as error:
        self._status(path, now, False, str(error))
        raise
      # Keep the last usable measurement through a brief accuracy dropout, without
      # changing its timestamp. It will still expire at the original deadline.
      if gps["horizontalAccuracy"] > MAX_HORIZONTAL_ACCURACY:
        return self._status(path, now, False, "poorAccuracy")
      previous = read_phone_gps(now, path)
      if previous is not None:
        stamp, old = previous
        timestamp, old_timestamp = gps["unixTimestampMillis"], old.unixTimestampMillis
        if gps["sourceMonoTime"] <= stamp or (timestamp and old_timestamp and timestamp <= old_timestamp):
          return self._status(path, now, False, "outOfOrder")
        # Within a live stream, differences between phone timestamps reveal added
        # queue delay without assuming the phone's wall clock matches comma's.
        if timestamp and old_timestamp:
          elapsed = (timestamp - old_timestamp) * 1_000_000
          if elapsed <= PHONE_MAX_AGE_NS:
            gps["sourceMonoTime"] = min(gps["sourceMonoTime"], stamp + elapsed)
            if now - gps["sourceMonoTime"] > MAX_AGE_MS * 1_000_000:
              return self._status(path, now, False, "delayed")
      temporary = path.with_name(path.name + ".tmp")
      temporary.write_text(json.dumps(gps))
      temporary.replace(path)
      self._status(path, now, True, "accepted")
      return {"ok": True}

  @staticmethod
  def _status(path, now, accepted, reason):
    status = path.with_name(path.name + ".status")
    temporary = status.with_name(status.name + ".tmp")
    temporary.write_text(json.dumps({"receivedMonoTime": now, "accepted": accepted, "reason": reason}))
    temporary.replace(status)
    return {"ok": True, "accepted": accepted, "reason": reason}


def phone_gps_status(now_ns=None, path=None):
  """Coordinate-free diagnostics; a successful upload need not be a usable fix."""
  now_ns = time.monotonic_ns() if now_ns is None else now_ns
  path = path or handoff_path()
  try:
    value = json.loads(path.with_name(path.name + ".status").read_text())
    received = value["receivedMonoTime"]
    if type(received) is not int or not 0 < received <= now_ns:
      return None
    fix = read_phone_gps(now_ns, path)
    return {"reason": value["reason"], "accepted": value["accepted"], "usable": fix is not None,
            "receivedAgeMs": (now_ns - received) / 1e6,
            "fixAgeMs": (now_ns - fix[0]) / 1e6 if fix is not None else None}
  except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
    return None


def read_phone_gps(now_ns, path=None):
  """The handed-off fix as a Gps-like object, or None when absent, malformed or older than the phone hold."""
  from types import SimpleNamespace
  from openpilot.starpilot.gps.source import PHONE_MAX_AGE_NS, valid_fix
  try:
    with (path or handoff_path()).open("rb") as source:
      gps = json.loads(source.read(1 << 16))
    stamp = gps["sourceMonoTime"]
    if type(stamp) is not int:
      return None
    return valid_fix(SimpleNamespace(**gps), stamp, stamp, now_ns, PHONE_MAX_AGE_NS)
  except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
    return None
