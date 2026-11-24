"""Lossless bounded column wire for untrusted original feedback observations.

No object decoder, numeric equality deduplication or admission token. Integer
columns use exact signed delta varints; floats retain all binary64 bits. This
reduces repeated field names and hex strings before the existing level1 IPC.
"""
import struct
from typing import cast

VERSION = 'original-feedback-columns-v1'
MAX_ROWS = 12000
MAX_BYTES = 16*1024*1024
_PUB = (('version','i'),('session','h16'),('sequence','i'),('publication_ns','i'),
  ('source_epoch','h32'),('cp_sha256','h32'),('output_known','b'),('output_control_ns','i'),
  ('output_state_ns','i'),('output_apply_ns','i'),('output_carrier_sequence','i'),
  ('reported_torque','f'),('valid','b'),('co_body_sha256','h32'),('applied_sha256','o32'),
  ('native_identity','o32'),('carrier_observed_ns','i'))
_CON = (('version','i'),('control_ns','i'),('publication_ns','i'),('recv_frame','i'),
  ('receipt_ns','i'),('seen','b'),('valid','b'),('co_body_sha256','h32'),
  ('reported_torque','f'),('requested_torque','f'),('lat_active','b'),('input_limited','b'),
  ('published_limited','b'),('prior_publish_ns','i'),('source_epoch','h32'),('cp_sha256','h32'))


def _column(values, kind):
  wire = bytearray()
  previous = 0
  for value in values:
    if kind == 'i':
      if type(value) is not int or not 0 <= value < 2**64:
        raise ValueError('feedback integer column')
      delta, previous = value-previous, value
      number = delta*2 if delta >= 0 else -delta*2-1
      while number >= 128:
        wire.append((number & 127)|128)
        number >>= 7
      wire.append(number)
    elif kind == 'b':
      if type(value) is not bool:
        raise ValueError('feedback Boolean column')
      wire.append(int(value))
    elif kind == 'f':
      if type(value) is not float:
        raise ValueError('feedback exact float column')
      wire.extend(struct.pack('!d',value))
    else:
      size = int(kind[1:])
      if type(value) is not str or len(value) not in ((0,size*2) if kind[0]=='o' else (size*2,)):
        raise ValueError('feedback digest column')
      raw = bytes.fromhex(value)
      if raw.hex() != value:
        raise ValueError('feedback canonical lowercase digest')
      if kind[0] == 'o':
        wire.append(bool(raw))
      wire.extend(raw)
  return bytes(wire)


def _values(wire, kind, count):
  if type(wire) is not bytes or len(wire) > MAX_BYTES:
    raise ValueError('bounded primitive feedback column')
  result, offset, previous = [], 0, 0
  for _ in range(count):
    if kind == 'i':
      number = shift = 0
      while True:
        if offset >= len(wire) or shift > 63:
          raise ValueError('bounded feedback integer encoding')
        byte = wire[offset]
        offset += 1
        number |= (byte&127)<<shift
        if byte < 128:
          if shift and byte == 0:
            raise ValueError('noncanonical feedback integer encoding')
          break
        shift += 7
      delta = number//2 if number%2 == 0 else -(number//2)-1
      value = previous+delta
      if not 0 <= value < 2**64:
        raise ValueError('feedback integer range')
      previous = value
    elif kind == 'b':
      if offset >= len(wire) or wire[offset] not in (0,1):
        raise ValueError('feedback Boolean encoding')
      value = bool(wire[offset])
      offset += 1
    elif kind == 'f':
      if offset+8 > len(wire):
        raise ValueError('feedback float encoding')
      value = struct.unpack_from('!d',wire,offset)[0]
      offset += 8
    else:
      size = int(kind[1:])
      if kind[0] == 'o':
        if offset >= len(wire) or wire[offset] not in (0,1):
          raise ValueError('feedback optional digest encoding')
        if not wire[offset]:
          size = 0
        offset += 1
      if offset+size > len(wire):
        raise ValueError('feedback digest encoding')
      value = wire[offset:offset+size].hex()
      offset += size
    result.append(value)
  if offset != len(wire):
    raise ValueError('feedback column trailing data')
  return result


def compact_observations(publications,consumptions,*,binding_digest):
  from openpilot.starpilot.flm.feedback_observation import export_observations
  body = export_observations(publications,consumptions,binding_digest=binding_digest)
  groups = []
  for name,schema in (('publications',_PUB),('consumptions',_CON)):
    rows = body[name]
    groups.append((len(rows),tuple(_column((row[field] for row in rows),kind) for field,kind in schema)))
  wire = {'version':VERSION,'protocol':body['protocol'],'binding':bytes.fromhex(binding_digest),
    'digest':bytes.fromhex(body['digest']),'groups':tuple(groups)}
  if sum(len(column) for _,columns in groups for column in columns) > MAX_BYTES:
    raise ValueError('feedback aggregate column bound')
  return wire


def expand_observations(wire,*,binding_digest):
  from openpilot.starpilot.flm.feedback_observation import PROTOCOL,decode_observations
  if (type(wire) is not dict or set(wire) != {'version','protocol','binding','digest','groups'} or
      wire['version'] != VERSION or wire['protocol'] != PROTOCOL or
      type(wire['binding']) is not bytes or len(wire['binding']) != 32 or wire['binding'].hex() != binding_digest or
      type(wire['digest']) is not bytes or len(wire['digest']) != 32 or
      type(wire['groups']) is not tuple or len(wire['groups']) != 2):
    raise ValueError('explicit bounded feedback column protocol')
  body = {'protocol':PROTOCOL,'binding_digest':binding_digest,'digest':wire['digest'].hex()}
  total = 0
  for name,schema,group in zip(('publications','consumptions'),(_PUB,_CON),wire['groups'], strict=False):
    if (type(group) is not tuple or len(group) != 2 or type(group[0]) is not int or not 1 <= group[0] <= MAX_ROWS or
        type(group[1]) is not tuple or len(group[1]) != len(schema)):
      raise ValueError('feedback column shape/count')
    count, columns = group[0], cast(tuple[bytes, ...], group[1])
    if any(type(column) is not bytes for column in columns):
      raise ValueError('primitive feedback columns only')
    total += sum(len(column) for column in columns)
    if total > MAX_BYTES:
      raise ValueError('feedback aggregate column bound')
    decoded = tuple(_values(column,kind,count) for column,(_,kind) in zip(columns,schema, strict=False))
    body[name] = [dict(zip((field for field,_ in schema),values, strict=False)) for values in zip(*decoded, strict=False)]
  # Returns untrusted records only. Authentic original replay is a separate gate.
  return decode_observations(body,binding_digest=binding_digest)
