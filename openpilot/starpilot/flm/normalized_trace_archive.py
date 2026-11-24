"""Literal normalized learner snapshot semantics; references confer no authority."""
from dataclasses import asdict, dataclass, fields, replace
import hashlib
import math
import pickle

from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, MANIFEST_SHA256

VERSION = 'full30-normalized-literal-archive-v1'
SOURCE_VERSION = 'unpaired-original-source-normalized-v9'
REF_VERSION = 'full30-literal-row-reference-v1'
MAX_ROWS = 24001
RESERVED_BYTES = 512 * 1024
_ENVELOPE_ARRAYS = ('prior_coordinates', 'coordinates')


@dataclass(frozen=True)
class TraceRef:
  index: int
  prepare_ns: int


def _numbers(rows):
  if (type(rows) is not tuple or len(rows) != 30 or
      any(type(row) is not tuple or len(row) != 2 or type(row[0]) is not str for row in rows) or
      tuple(row[0] for row in rows) != NAMES):
    raise ValueError('normalized trace exact canonical names')
  values = tuple(row[1] for row in rows)
  _values(values)
  return values


def _values(values):
  if type(values) is not tuple or len(values) != 30 or any(
      type(value) not in (int, float) or
      type(value) is int and abs(value).bit_length() > 64 or not math.isfinite(value) for value in values):
    raise ValueError('normalized trace exact bounded numeric slots')


def ref_wire(ref):
  if type(ref) is not TraceRef or type(ref.index) is not int or type(ref.prepare_ns) is not int:
    raise ValueError('normalized exact row reference')
  return {'version': REF_VERSION, 'index': ref.index, 'prepare_ns': ref.prepare_ns}


def read_ref(value, archive):
  if (type(value) is not dict or any(type(key) is not str for key in value) or set(value) != {'version', 'index', 'prepare_ns'} or
      type(value['version']) is not str or value['version'] != REF_VERSION or type(archive) is not TraceArchive):
    raise ValueError('explicit normalized row reference and archive')
  ref = TraceRef(value['index'], value['prepare_ns'])
  archive.row(ref)
  return ref


class ArchiveBuilder:
  def __init__(self):
    self.envelopes, self.rows = [], []
    self._envelope_keys, self._row_keys = {}, {}

  def encode(self, trace):
    from openpilot.starpilot.flm.effect_trace import FullEffectTrace
    if type(trace) is not FullEffectTrace:
      raise ValueError('normalized original full trace required')
    trace.validate()
    key = pickle.dumps(tuple((field.name,getattr(trace.envelope,field.name))
      for field in fields(trace.envelope)),protocol=5)
    index = self._envelope_keys.get(key)
    if index is None:
      envelope = asdict(trace.envelope)
      for name in _ENVELOPE_ARRAYS:
        envelope[name] = _numbers(getattr(trace.envelope, name))
      if len(self.envelopes) >= MAX_ROWS:
        raise ValueError('normalized declaration bound')
      index = len(self.envelopes)
      self.envelopes.append(envelope)
      self._envelope_keys[key] = index
    row = (index, trace.version, trace.control_ns, trace.previous_ns, trace.qualified,
           _numbers(trace.before), _numbers(trace.target), _numbers(trace.applied), trace.applied_full_profile_sha256)
    key = pickle.dumps(row, protocol=5)
    row_index = self._row_keys.get(key)
    if row_index is None:
      if len(self.rows) >= MAX_ROWS:
        raise ValueError('normalized trace row bound')
      row_index = len(self.rows)
      self.rows.append(row)
      self._row_keys[key] = row_index
    return ref_wire(TraceRef(row_index, trace.control_ns))

  def record(self, value):
    trace = getattr(value, 'effect_trace', None)
    if trace is None:
      return asdict(value)
    body = asdict(replace(value,effect_trace=None))
    body['effect_trace'] = self.encode(trace)
    return body

  def literal(self):
    return {'version': VERSION, 'manifest': MANIFEST_SHA256,
            'envelopes': tuple(self.envelopes), 'rows': tuple(self.rows)}


class TraceArchive:
  def __init__(self, literal):
    if reservation_upper_bound() > RESERVED_BYTES:
      raise ValueError('normalized transient reservation insufficient')
    self.literal = literal
    self.validate_shape()

  def _header(self):
    literal = self.literal
    if (type(literal) is not dict or any(type(key) is not str for key in literal) or
        set(literal) != {'version','manifest','envelopes','rows'} or
        type(literal['version']) is not str or type(literal['manifest']) is not str or
        literal['version'] != VERSION or literal['manifest'] != MANIFEST_SHA256 or
        type(literal['envelopes']) is not tuple or type(literal['rows']) is not tuple or
        not 1 <= len(literal['envelopes']) <= len(literal['rows']) <= MAX_ROWS):
      raise ValueError('normalized literal archive source changed')

  @staticmethod
  def _declaration(declaration):
    from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope
    keys = tuple(field.name for field in fields(FullSurfaceEnvelope))
    if (type(declaration) is not dict or any(type(key) is not str for key in declaration) or
        set(declaration) != set(keys)):
      raise ValueError('normalized exact literal declaration fields')
    for name, value in declaration.items():
      if name in _ENVELOPE_ARRAYS:
        _values(value)
      elif name == 'trial_names':
        if type(value) is not tuple or not 1 <= len(value) <= 30 or any(type(item) is not str or item not in NAMES for item in value):
          raise ValueError('normalized declaration trial names')
      elif name == 'version':
        if type(value) is not int or value not in (1,2):
          raise ValueError('normalized declaration version')
      elif type(value) is not str or len(value.encode('utf8', 'surrogatepass')) > 256:
        raise ValueError('normalized declaration bounded metadata')

  def _row(self, ref):
    if (type(ref) is not TraceRef or type(ref.index) is not int or type(ref.prepare_ns) is not int or
        not 0 <= ref.index < len(self.literal['rows'])):
      raise ValueError('normalized trace row membership')
    row = self.literal['rows'][ref.index]
    if (type(row) is not tuple or len(row) != 9 or type(row[0]) is not int or
        not 0 <= row[0] < len(self.literal['envelopes']) or type(row[1]) is not int or row[1] != 1 or
        type(row[2]) is not int or type(row[3]) is not int or row[2] <= 0 or row[3] < 0 or
        row[2].bit_length() > 64 or row[3].bit_length() > 64 or
        row[2] != ref.prepare_ns or type(row[4]) is not bool or type(row[8]) is not str or
        len(row[8]) != 64 or any(char not in '0123456789abcdef' for char in row[8])):
      raise ValueError('normalized literal trace clocks/body')
    for values in row[5:8]:
      _values(values)
    return row

  def validate_shape(self):
    self._header()
    for declaration in self.literal['envelopes']:
      self._declaration(declaration)
    used = set()
    for index, row in enumerate(self.literal['rows']):
      if type(row) is not tuple or len(row) != 9:
        raise ValueError('normalized literal trace row')
      checked = self._row(TraceRef(index,row[2]))
      used.add(checked[0])
    if used != set(range(len(self.literal['envelopes']))):
      raise ValueError('normalized unused declaration')

  def row(self, ref):
    self._header()
    row = self._row(ref)
    declaration = self.literal['envelopes'][row[0]]
    self._declaration(declaration)
    return row, declaration

  def resolve(self, ref):
    """One unverified typed body; original consumer checks still run fresh."""
    from openpilot.starpilot.flm.effect_trace import FullEffectTrace
    from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope
    row, declaration = self.row(ref)
    envelope = dict(declaration)
    for name in _ENVELOPE_ARRAYS:
      envelope[name] = tuple(zip(NAMES, envelope[name], strict=True))
    trace = FullEffectTrace(row[1], FullSurfaceEnvelope(**envelope), row[2], row[3], row[4],
      tuple(zip(NAMES, row[5], strict=True)), tuple(zip(NAMES, row[6], strict=True)),
      tuple(zip(NAMES, row[7], strict=True)), row[8])
    return trace


def archive_of(owner):
  archive = getattr(owner, 'trace_archive', None)
  if archive is not None and type(archive) is not TraceArchive:
    raise ValueError('exact normalized source archive owner')
  return archive


def archive_key(owner):
  archive = archive_of(owner)
  if archive is None:
    return None
  archive.validate_shape()
  return VERSION, archive.literal


def trace_for(record, owner):
  trace = getattr(record, 'effect_trace', None)
  if type(trace) is TraceRef:
    archive = archive_of(owner)
    if archive is None:
      raise ValueError('normalized trace missing literal source archive')
    if trace.prepare_ns != record.effect_prepare_ns:
      raise ValueError('normalized trace actual prepare membership')
    return archive.resolve(trace)
  return trace


def record_body(record, owner):
  ref = getattr(record, 'effect_trace', None)
  if type(ref) is not TraceRef:
    return asdict(record)
  archive = archive_of(owner)
  if archive is None:
    raise ValueError('normalized record missing source archive')
  row, declaration = archive.row(ref)
  body = asdict(replace(record, effect_trace=None))
  body['effect_trace'] = {'version': VERSION, 'declaration': declaration, 'row': row}
  return body


def stored_record(record):
  ref = getattr(record, 'effect_trace', None)
  if type(ref) is not TraceRef:
    return asdict(record)
  body = asdict(replace(record, effect_trace=None))
  body['effect_trace'] = ref_wire(ref)
  return body


def current_record(value, literal):
  builder = ArchiveBuilder()
  body = builder.record(value)
  if not builder.rows:
    return body
  needle_row = builder.rows[0]
  needle_declaration = builder.envelopes[needle_row[0]]
  needle = pickle.dumps((needle_declaration, needle_row[1:]), protocol=5)
  archive = TraceArchive(literal)
  for index, row in enumerate(literal['rows']):
    if row[2] == needle_row[2] and pickle.dumps((literal['envelopes'][row[0]], row[1:]), protocol=5) == needle:
      body['effect_trace'] = ref_wire(TraceRef(index, row[2]))
      archive.row(TraceRef(index, row[2]))
      return body
  raise ValueError('current full trace missing from literal source archive')


def normalized_packet(packet):
  return type(packet) is dict and type(packet.get('version')) is str and packet['version'] == SOURCE_VERSION


def reserve_for_packet(packet):
  if not normalized_packet(packet):
    if type(packet) is dict and 'trace_archive' in packet:
      raise ValueError('legacy source cannot implicitly adopt normalized traces')
    return 0
  validate_packet(packet)
  return RESERVED_BYTES


def guarded_archive(*, position=0, keyword='engine'):
  """Rebind complete literal values across mutable work; never cache a verdict."""
  from functools import wraps
  def decorate(function):
    @wraps(function)
    def call(*args, **kwargs):
      owner = args[position] if len(args) > position else kwargs[keyword]
      before = archive_key(owner)
      if before is None:
        return function(*args, **kwargs)
      digest = hashlib.sha256(pickle.dumps(before, protocol=5)).digest()
      result = function(*args, **kwargs)
      if hashlib.sha256(pickle.dumps(archive_key(owner), protocol=5)).digest() != digest:
        raise ValueError('normalized literal source changed during mutable work')
      return result
    return call
  return decorate


def validate_packet(packet):
  """Complete table/reference membership; shape only, never source admission."""
  if not normalized_packet(packet):
    if type(packet) is dict and 'trace_archive' in packet:
      raise ValueError('legacy source cannot implicitly adopt normalized traces')
    return None
  archive = TraceArchive(packet['trace_archive'])
  controls = packet['context_receipts']['controls']
  frames = packet['actual_frames']
  if (type(controls) is not tuple or type(frames) is not tuple or
      not 1 <= len(controls) <= 12000 or len(frames) != len(controls)):
    raise ValueError('normalized complete original reference inventory')
  used = set()
  for bodies in (controls,frames):
    for body in bodies:
      if type(body) is not dict:
        raise ValueError('normalized exact original record body')
      value = body.get('effect_trace')
      if value is None:
        continue
      if (type(value) is not dict or any(type(key) is not str for key in value) or
          set(value) != {'version','index','prepare_ns'} or type(value['version']) is not str or
          value['version'] != REF_VERSION or type(value['index']) is not int or
          type(value['prepare_ns']) is not int or not 0 <= value['index'] < len(archive.literal['rows']) or
          type(body.get('effect_prepare_ns')) is not int or type(body.get('control_ns')) is not int or
          value['prepare_ns'] != body['effect_prepare_ns'] or
          not 0 < value['prepare_ns'] <= body['control_ns'] or
          archive.literal['rows'][value['index']][2] != value['prepare_ns']):
        raise ValueError('normalized actual row/prepare reference membership')
      used.add(value['index'])
  if used != set(range(len(archive.literal['rows']))):
    raise ValueError('normalized unused or omitted literal trace row')
  return archive


def reservation_upper_bound():
  """Worst logical contents of working trace and existing bounded content LRUs."""
  from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope
  from openpilot.starpilot.flm.effect_trace import FullEffectTrace
  from openpilot.starpilot.flm import effect_admission_cache as admission
  from openpilot.starpilot.flm import canonical_surface_adapter as canonical
  if (admission._verified.cache_parameters()['maxsize'] != 16 or
      canonical._full_profile_sha256_cached.cache_parameters()['maxsize'] != 128 or
      canonical._full_original_anchor_cached.cache_parameters()['maxsize'] != 32 or
      canonical._original_anchor_cached.cache_parameters()['maxsize'] != 32):
    raise ValueError('normalized retained-content reservation source changed')
  envelope_fields = tuple(field.name for field in fields(FullSurfaceEnvelope))
  trace_fields = tuple(field.name for field in fields(FullEffectTrace))
  name_cost = sum(9 + len(name.encode('utf8')) for name in NAMES)
  coordinates = 9 + 30 * (9 + 17) + name_cost
  names = 9 + name_cost
  # Exact integer<=64bits costs at most17; every metadata string<=256UTF8 bytes.
  envelope_values = 17 + 2 * coordinates + names + (len(envelope_fields) - 4) * (9 + 256)
  envelope = 9 + sum(9 + len(name.encode()) for name in envelope_fields) + envelope_values
  envelope_key = 9 + envelope_values
  trace = 9 + sum(9 + len(name.encode()) for name in trace_fields) + 17 + envelope + 2 * 17 + 2 + 3 * coordinates + 73
  # The16 envelope proofs retain both key and value. Include construction dicts,
  # 128 packed profile keys/results and both existing32-entry source-anchor LRUs.
  return (16 * (envelope + envelope_key) + 2 * envelope + trace +
          128 * (9 + 73 + 249 + 73) + 32 * (9 + 73 + 73 + 73) + 32 * (9 + 73 + names + 73))
