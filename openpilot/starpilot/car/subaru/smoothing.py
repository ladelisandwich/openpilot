"""A single Subaru smoothing choice, frozen by Card for modeld and Controls."""
from dataclasses import dataclass
import hashlib
import json
import math

import capnp
from opendbc.car import structs
from opendbc.car.subaru.stock_aol import qualified
from opendbc.car.subaru.values import SubaruFlags
from openpilot.starpilot.saved_source import read_saved

KEY = 'LatSmoothSeconds'
CONTEXT_KEY = 'SubaruSmoothingContext'
MARKER = int(SubaruFlags.SMOOTHING_BOUND)
MAX_CONTEXT_BYTES = 512
MAX_CP_BYTES = 1_048_576
SOURCES = ('DeveloperUI', 'SafeMode', KEY)


def selected(raw):
  if raw is None:
    return None
  if type(raw) not in (str, bytes):
    raise ValueError('Invalid smoothing scalar type')
  try:
    value = float(raw)
    if (math.isfinite(value) and 0. <= value <= 2. and
        math.isclose(value / .005, round(value / .005), rel_tol=0., abs_tol=1e-7)):
      return 0.0 if value == 0. else round(value / .005) * .005
  except (TypeError, ValueError, OverflowError):
    pass
  raise ValueError('Invalid Subaru smoothing choice')


def _sources(params):
  return tuple((key, *read_saved(params, key, 64 if key == KEY else 8)) for key in SOURCES)


@dataclass(frozen=True)
class Snapshot:
  source: tuple
  manual: float | None


def _snapshot_from_sources(source):
  if (type(source) is not tuple or len(source) != len(SOURCES) or
      any(type(row) is not tuple or len(row) != 3 or row[0] != key or type(row[2]) is not bool or
          (row[1] is not None and (type(row[1]) is not bytes or len(row[1]) > (64 if key == KEY else 8)))
          for key, row in zip(SOURCES, source, strict=True))):
    raise ValueError('Invalid frozen smoothing sources')
  developer, safe, scalar = source
  allowed = developer[1:] == (b'1', True) and safe[2] and safe[1] in (None, b'0')
  manual = None
  if allowed and scalar[2]:
    try:
      manual = selected(scalar[1])
    except ValueError:
      pass
  return Snapshot(source, manual)


def read_snapshot(params, *, enabled):
  return _snapshot_from_sources(_sources(params)) if enabled is True else None


def prepare(cp, snapshot):
  if qualified(cp):
    cp.flags = int(cp.flags) & ~MARKER
    if type(snapshot) is Snapshot:
      cp.flags |= MARKER
  return cp


def maximum_for(cp, snapshot):
  if not qualified(cp) or type(snapshot) is not Snapshot:
    raise ValueError('Unbound Subaru smoothing configuration')
  return snapshot.manual if snapshot.manual is not None else (.4 if cp.carFingerprint == 'SUBARU_OUTBACK_2023' else 0.)


def _canonical(value):
  number = selected(str(value))
  if number is None:
    raise ValueError('Missing smoothing maximum')
  return format(number, '.3f').rstrip('0').rstrip('.') or '0'


def _unique(pairs):
  result = {}
  for key, value in pairs:
    if key in result:
      raise ValueError('Duplicate smoothing context key')
    result[key] = value
  return result


def _invalid_constant(value):
  raise ValueError('Non-finite smoothing context constant')


def decode_context(cp, cp_bytes, raw):
  if (not qualified(cp) or not cp.flags & MARKER or type(cp_bytes) is not bytes or
      not 16 <= len(cp_bytes) <= MAX_CP_BYTES or type(raw) is not bytes or not 0 < len(raw) <= MAX_CONTEXT_BYTES):
    return None
  try:
    doc = json.loads(raw.decode('ascii'), object_pairs_hook=_unique, parse_constant=_invalid_constant)
    if (type(doc) is not dict or set(doc) != {'version', 'carParamsSha256', 'maximum', 'sources'} or
        type(doc['version']) is not int or doc['version'] != 1 or type(doc['carParamsSha256']) is not str or
        doc['carParamsSha256'] != hashlib.sha256(cp_bytes).hexdigest() or type(doc['maximum']) is not str):
      return None
    if type(doc['sources']) is not list or len(doc['sources']) != len(SOURCES):
      return None
    source = []
    for key, row in zip(SOURCES, doc['sources'], strict=True):
      if type(row) is not list or len(row) != 3 or row[0] != key or (row[1] is not None and type(row[1]) is not str):
        return None
      value = bytes.fromhex(row[1]) if row[1] is not None else None
      if value is not None and value.hex() != row[1]:
        return None
      source.append((key, value, row[2]))
    snapshot = _snapshot_from_sources(tuple(source))
    maximum = selected(doc['maximum'])
    if (maximum is None or doc['maximum'] != _canonical(maximum) or
        doc['maximum'] != _canonical(maximum_for(cp, snapshot))):
      return None
    with structs.CarParams.from_bytes(cp_bytes, traversal_limit_in_words=MAX_CP_BYTES // 8, nesting_limit=16) as published:
      if published.to_dict() != cp.to_dict():
        return None
    return maximum
  except (ValueError, TypeError, OverflowError, RuntimeError, capnp.KjException):
    return None


def install(cp, cp_bytes, snapshot, params):
  if not qualified(cp) or not cp.flags & MARKER:
    return None
  if (type(snapshot) is not Snapshot or _sources(params) != snapshot.source or
      _snapshot_from_sources(snapshot.source) != snapshot):
    raise RuntimeError('Subaru smoothing settings changed during startup')
  maximum = maximum_for(cp, snapshot)
  raw = json.dumps({'version': 1, 'carParamsSha256': hashlib.sha256(cp_bytes).hexdigest(),
                    'maximum': _canonical(maximum),
                    'sources': [[key, value.hex() if value is not None else None, readable]
                                for key, value, readable in snapshot.source]}, sort_keys=True, separators=(',', ':')).encode('ascii')
  if decode_context(cp, cp_bytes, raw) != maximum:
    raise RuntimeError('Subaru smoothing context did not match its CarParams')
  params.put(CONTEXT_KEY, raw, block=True)
  if read_saved(params, CONTEXT_KEY, MAX_CONTEXT_BYTES) != (raw, True) or _sources(params) != snapshot.source:
    raise RuntimeError('Subaru smoothing context publication failed')
  return maximum


def read_frozen(cp, params):
  if not qualified(cp) or not cp.flags & MARKER:
    return 0.
  cp_bytes, cp_readable = read_saved(params, 'CarParams', MAX_CP_BYTES)
  raw, readable = read_saved(params, CONTEXT_KEY, MAX_CONTEXT_BYTES)
  result = decode_context(cp, cp_bytes, raw) if cp_readable and readable else None
  if result is None:
    raise RuntimeError('Missing or mismatched Subaru smoothing startup context')
  return result


def seconds(speed, maximum):
  if not math.isfinite(speed) or not math.isfinite(maximum) or not 0. <= maximum <= 2.:
    return 0.
  return maximum * max(0., min(1., (8. - speed) / 6.))


def filter_action(action, previous, speed, tau):
  """Apply once at the common model join; longitudinal fields remain untouched."""
  from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
  if tau > 0. and speed > .3:
    action.desiredCurvature = float(smooth_value(action.desiredCurvature, previous.desiredCurvature, tau))



def control_maximum(controls):
  """Legacy component hosts stay unchanged; a marked runtime must be initialized."""
  maximum = getattr(controls, 'subaru_smoothing', None)
  if maximum is None:
    cp = controls.CP
    if getattr(cp, 'brand', '') == 'subaru' and int(getattr(cp, 'flags', 0)) & MARKER:
      raise RuntimeError('Subaru smoothing Controls startup was not initialized')
    return 0.
  return maximum
