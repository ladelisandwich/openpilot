"""Exact full30 applied state and bounded transition shared by owner and replay."""
from dataclasses import asdict, dataclass, fields
from collections import OrderedDict
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256
from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope, envelope_dict, envelope_from_dict

VERSION = 1
SLEW_PER_SECOND = .005


# At most four controller-owned current-frame traces. Never a permission cache,
# never a retained receiver history. Strong refs prevent identity reuse.
_SENDER_PROOFS = OrderedDict()
_ENVELOPE_FIELDS = tuple(field.name for field in fields(FullSurfaceEnvelope))
_TRACE_FIELDS = ('version','envelope','control_ns','previous_ns','qualified','before','target','applied','applied_full_profile_sha256')


def _field_references(trace):
  return (tuple(getattr(trace,name) for name in _TRACE_FIELDS),
          tuple(getattr(trace.envelope,name) for name in _ENVELOPE_FIELDS))


def _sender_proof_current(trace):
  if type(trace) is not FullEffectTrace:
    return False
  saved = _SENDER_PROOFS.get(id(trace))
  if saved is None or saved[0] is not trace:
    return False
  own, envelope = _field_references(trace)
  return (all(a is b for a,b in zip(own,saved[1],strict=True)) and
          all(a is b for a,b in zip(envelope,saved[2],strict=True)))


def retain_sender_verification(trace):
  if type(trace) is not FullEffectTrace:
    raise ValueError('exact sender trace class required')
  digest=trace.applied_full_profile_sha256
  if type(digest) is not str or len(digest)!=64 or any(c not in '0123456789abcdef' for c in digest):
    raise ValueError('exact immutable sender profile digest')
  # Full validation FIRST proves deep immutable primitive tuples and hashes.
  # An untrusted instance cannot mint this entry merely by setting attributes.
  trace.validate()
  own,envelope = _field_references(trace)
  _SENDER_PROOFS[id(trace)] = (trace,own,envelope)
  _SENDER_PROOFS.move_to_end(id(trace))
  while len(_SENDER_PROOFS)>4:
    _SENDER_PROOFS.popitem(last=False)
  return trace


def ordered(values):
  return tuple((n, float(values[n])) for n in NAMES)


def step_full(before, target, *, now_ns, previous_ns, qualified):
  if type(now_ns) is not int or now_ns <= 0 or type(previous_ns) is not int or previous_ns < 0:
    raise ValueError('effect transition clock')
  dt = .01 if previous_ns == 0 else (now_ns - previous_ns) / 1e9
  if not qualified or not 0 < dt <= .05:
    return dict.fromkeys(NAMES, 0.)
  step = SLEW_PER_SECOND * dt
  return {n:before[n] + max(-step, min(step, target[n] - before[n])) for n in NAMES}


@dataclass(frozen=True)
class FullEffectTrace:
  version: int
  envelope: FullSurfaceEnvelope
  control_ns: int
  previous_ns: int
  qualified: bool
  before: tuple[tuple[str,float], ...]
  target: tuple[tuple[str,float], ...]
  applied: tuple[tuple[str,float], ...]
  applied_full_profile_sha256: str

  def validate(self):
    if _sender_proof_current(self):
      return self
    if type(self.version) is not int or self.version != VERSION or type(self.qualified) is not bool:
      raise ValueError('effect trace version')
    if not isinstance(self.envelope, FullSurfaceEnvelope):
      raise ValueError('typed original envelope')
    from openpilot.starpilot.flm.effect_admission_cache import validated_envelope
    validated_envelope(self.envelope)
    for rows in (self.before, self.target, self.applied):
      if (type(rows) is not tuple or len(rows) != len(NAMES) or
          any(type(row) is not tuple or len(row)!=2 or type(row[0]) is not str or type(row[1]) not in (int,float) for row in rows) or
          tuple(n for n, _ in rows) != NAMES):
        raise ValueError('exact full30 effect trace')
      full_profile_sha256(self.envelope.selected_baseline_revision, rows)
    if self.target != self.envelope.coordinates:
      raise ValueError('effect target differs from admitted full profile')
    expected = ordered(step_full(dict(self.before), dict(self.target), now_ns=self.control_ns,
      previous_ns=self.previous_ns, qualified=self.qualified))
    if self.applied != expected or full_profile_sha256(self.envelope.selected_baseline_revision, self.applied) != self.applied_full_profile_sha256:
      raise ValueError('effect transition or applied hash changed')
    return self


def trace_dict(value):
  if not isinstance(value, FullEffectTrace):
    raise ValueError('typed full effect trace')
  value.validate()
  result=asdict(value)
  result['envelope']=envelope_dict(value.envelope)
  return result


def trace_from_dict(value):
  if type(value) is not dict or set(value) != set(FullEffectTrace.__dataclass_fields__):
    raise ValueError('effect trace fields')
  copied=dict(value)
  copied['envelope']=envelope_from_dict(copied['envelope'])
  for name in ('before','target','applied'):
    copied[name]=tuple(tuple(row) for row in copied[name])
  return FullEffectTrace(**copied).validate()
