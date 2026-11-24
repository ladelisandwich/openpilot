"""Bounded caches of exact immutable proof CONTENT, never live permission.

Current clocks, leases, source/reset/native/configuration and CAS are compared
by callers on EVERY cycle. Keys snapshot immutable values, not object identity.
"""
from dataclasses import fields
from functools import lru_cache
import struct
from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope
from openpilot.starpilot.flm.canonical_surface_adapter import parameters_for

FIELD_NAMES = tuple(field.name for field in fields(FullSurfaceEnvelope))


def envelope_key(envelope):
  if type(envelope) is not FullSurfaceEnvelope:
    raise ValueError('exact typed full effect envelope')
  result=[]
  for name in FIELD_NAMES:
    value=getattr(envelope,name)
    if type(value) not in (str,int,tuple):
      raise ValueError('immutable effect envelope content')
    if name in ('prior_coordinates','coordinates'):
      if type(value) is not tuple:
        raise ValueError('immutable named effect coordinates')
      rows=[]
      for row in value:
        if type(row) is not tuple or len(row)!=2 or type(row[0]) is not str or type(row[1]) not in (int,float):
          raise ValueError('immutable named effect coordinates')
        # Python float equality aliases signed zero. Preserve actual IEEE bits
        # so a malformed changed map cannot reuse a previously verified hash.
        rows.append((row[0],struct.pack('!d',float(row[1]))))
      value=tuple(rows)
    elif type(value) is tuple and any(type(item) is not str for item in value):
      raise ValueError('immutable canonical names')
    result.append(value)
  return tuple(result)


@lru_cache(maxsize=16)
def _verified(key):
  values=dict(zip(FIELD_NAMES,key,strict=True))
  for name in ('prior_coordinates','coordinates'):
    values[name]=tuple((n,struct.unpack('!d',value)[0]) for n,value in values[name])
  return FullSurfaceEnvelope(**values).validate()


def validated_envelope(envelope, profile=None):
  proof=_verified(envelope_key(envelope))
  if profile is not None:
    policy='event_effect_restore_v1' if getattr(proof,'operation','trial')=='restore' else 'event_effect_coordinate_v2'
    if (profile.policy!=policy or profile.parameters!=parameters_for(proof.trial_names) or
        tuple(profile.coefficients)!=tuple(dict(proof.coordinates)[n] for n in proof.trial_names)):
      raise ValueError('current profile changed immutable effect proof')
  return proof


def admit_context(envelope, *, cp_sha256, controller_revision, source_epoch, reset_generation,
                  selected_baseline_revision, context_key, current_full_profile_sha256, persistent_source_epoch='', profile=None):
  proof=validated_envelope(envelope,profile)
  expected=(cp_sha256,controller_revision,source_epoch,reset_generation,selected_baseline_revision,context_key,persistent_source_epoch)
  current=(proof.cp_sha256,proof.controller_revision,proof.source_epoch,proof.reset_generation,
           proof.selected_baseline_revision,proof.context_key,proof.persistent_source_epoch)
  if expected!=current or current_full_profile_sha256!=proof.prior_full_profile_sha256:
    raise ValueError('stale full profile/source CAS')
  return proof
