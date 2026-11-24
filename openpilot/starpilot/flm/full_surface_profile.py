"""Explicit full30 original-anchor envelope. No storage or driving authority."""

from dataclasses import dataclass, asdict, replace
import hashlib
from openpilot.starpilot.flm.canonical_surface_adapter import (
  NAMES,
  MANIFEST_SHA256,
  canonical_names,
  parameters_for,
  full_original_anchor,
  original_anchor,
  full_profile_sha256,
)

POLICY = 'event_effect_coordinate_v2'
RESTORE_POLICY = 'event_effect_restore_v1'
VERSION = 1


def hex64(value):
  return type(value) is str and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


@dataclass(frozen=True)
class FullSurfaceEnvelope:
  version: int
  manifest_sha256: str
  full_original_anchor: str
  trial_anchor: str
  context_key: str
  cp_sha256: str
  controller_revision: str
  source_epoch: str
  reset_generation: str
  selected_baseline_revision: str
  prior_full_profile_sha256: str
  full_profile_sha256: str
  prior_coordinates: tuple[tuple[str, float], ...]
  coordinates: tuple[tuple[str, float], ...]
  trial_names: tuple[str, ...]
  persistent_source_epoch: str = ''
  operation: str = 'trial'
  history_full_profile_sha256: str = ''
  restore_transaction_sha256: str = ''

  @classmethod
  def initial(
    cls, *, cp_sha256, controller_revision, source_epoch, reset_generation, selected_baseline_revision, trial_names, persistent_source_epoch='', prior=None
  ):
    names = canonical_names(trial_names)
    if prior is not None:
      raise ValueError('historical prior is not actual accepted current state; use explicit restore transaction')
    rows = neutral_coordinates()
    full_anchor = full_original_anchor(selected_baseline_revision)
    binding = (cp_sha256, controller_revision, source_epoch, reset_generation, selected_baseline_revision, full_anchor, source_epoch, persistent_source_epoch)
    if prior is not None and (not prior.historical_only or prior.envelope.source_binding() != binding):
      raise ValueError('foreign historical full surface prior')
    digest = full_profile_sha256(selected_baseline_revision, rows)
    return cls(
      VERSION,
      MANIFEST_SHA256,
      full_anchor,
      original_anchor(selected_baseline_revision, names),
      source_epoch,
      cp_sha256,
      controller_revision,
      source_epoch,
      reset_generation,
      selected_baseline_revision,
      digest,
      digest,
      rows,
      rows,
      names,
      persistent_source_epoch,
    ).validate()

  def validate(self, profile=None):
    if self.version not in (VERSION, 2) or type(self.version) is not int or self.manifest_sha256 != MANIFEST_SHA256:
      raise ValueError('full surface envelope version/manifest')
    if self.persistent_source_epoch and not hex64(self.persistent_source_epoch):
      raise ValueError('stable persistent source epoch')
    if self.context_key != self.source_epoch:
      raise ValueError('event context must equal exact source epoch')
    if not all(
      hex64(v)
      for v in (
        self.full_original_anchor,
        self.trial_anchor,
        self.context_key,
        self.cp_sha256,
        self.source_epoch,
        self.selected_baseline_revision,
        self.prior_full_profile_sha256,
        self.full_profile_sha256,
      )
    ):
      raise ValueError('full surface binding')
    if (
      type(self.controller_revision) is not str
      or not 1 <= len(self.controller_revision) <= 128
      or type(self.reset_generation) is not str
      or not 1 <= len(self.reset_generation) <= 64
      or any(c not in '0123456789abcdef' for c in self.reset_generation)
    ):
      raise ValueError('source/controller/reset binding')
    names = canonical_names(self.trial_names)
    if (
      self.trial_names != names
      or self.full_original_anchor != full_original_anchor(self.selected_baseline_revision)
      or self.trial_anchor != original_anchor(self.selected_baseline_revision, names)
    ):
      raise ValueError('changed original full/trial anchor')
    for rows, digest in ((self.prior_coordinates, self.prior_full_profile_sha256), (self.coordinates, self.full_profile_sha256)):
      if type(rows) is not tuple or len(rows) != 30 or tuple(n for n, _ in rows) != NAMES or len(dict(rows)) != 30:
        raise ValueError('exact canonical full30 map')
      if full_profile_sha256(self.selected_baseline_revision, rows) != digest:
        raise ValueError('full surface content hash')
    prior, current = dict(self.prior_coordinates), dict(self.coordinates)
    if self.operation == 'trial':
      if self.version != VERSION or self.history_full_profile_sha256 or self.restore_transaction_sha256:
        raise ValueError('trial cannot carry restore authority')
      if any(prior[n] != current[n] for n in NAMES if n not in names):
        raise ValueError('outside trial group changed')
      if any(abs(current[n] - prior[n]) > 0.005 + 1e-12 for n in names):
        raise ValueError('trial step exceeds .005 original effect')
      expected_policy = POLICY
    elif self.operation == 'restore':
      if (
        self.version != 2
        or not hex64(self.history_full_profile_sha256)
        or not hex64(self.restore_transaction_sha256)
        or self.prior_coordinates != neutral_coordinates()
        or self.history_full_profile_sha256 != self.full_profile_sha256
        or self.restore_transaction_sha256 != restore_transaction(self)
      ):
        raise ValueError('explicit neutral-to-history restore transaction')
      self.persistent_binding()  # Never restore another unknown vehicle/session.
      expected_policy = RESTORE_POLICY
    else:
      raise ValueError('unknown full surface operation')
    if profile is not None and (
      profile.policy != expected_policy or profile.parameters != parameters_for(names) or tuple(profile.coefficients) != tuple(current[n] for n in names)
    ):
      raise ValueError('profile/full-surface operation mismatch')
    return self

  def source_binding(self):
    return (
      self.cp_sha256,
      self.controller_revision,
      self.source_epoch,
      self.reset_generation,
      self.selected_baseline_revision,
      self.full_original_anchor,
      self.context_key,
      self.persistent_source_epoch,
    )

  def persistent_binding(self):
    if not hex64(self.persistent_source_epoch):
      raise ValueError('known VIN/static source required for persistence')
    return (
      self.cp_sha256,
      self.controller_revision,
      self.persistent_source_epoch,
      self.reset_generation,
      self.selected_baseline_revision,
      self.full_original_anchor,
    )

  def scope(self):
    return hashlib.sha256(repr(self.persistent_binding()).encode()).hexdigest()

  def replay_anchor(self):
    self.validate()
    payload = {
      'version': 1,
      'manifest_sha256': self.manifest_sha256,
      'full_original_anchor': self.full_original_anchor,
      'anchor_digest': self.trial_anchor,
      'accumulated': self.prior_coordinates,
      'full_profile_sha256': self.prior_full_profile_sha256,
      'context_key': self.context_key,
      'cp_sha256': self.cp_sha256,
      'controller_revision': self.controller_revision,
      'source_epoch': self.source_epoch,
      'reset_generation': self.reset_generation,
      'selected_baseline_revision': self.selected_baseline_revision,
      'persistent_source_epoch': self.persistent_source_epoch,
    }
    if self.operation == 'restore':
      payload.update(
        version=2,
        operation='restore',
        history_full_profile_sha256=self.history_full_profile_sha256,
        restore_transaction_sha256=self.restore_transaction_sha256,
        restore_target=self.coordinates,
      )
    return payload

  def restore_from_history(self, prior):
    self.validate()
    if (
      self.operation != 'trial'
      or self.prior_coordinates != neutral_coordinates()
      or not getattr(prior, 'historical_only', False)
      or prior.envelope.source_binding() != self.source_binding()
    ):
      raise ValueError('restore requires exact current neutral state and stable historical prior')
    history = prior.envelope.validate()
    proposed = replace(
      self,
      version=2,
      operation='restore',
      coordinates=history.coordinates,
      full_profile_sha256=history.full_profile_sha256,
      history_full_profile_sha256=history.full_profile_sha256,
    )
    proposed = replace(proposed, restore_transaction_sha256=restore_transaction(proposed))
    return proposed.validate()

  @property
  def restoration_groups(self):
    from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS

    values = dict(self.coordinates)
    return tuple(group for group, names in GROUPS.items() if any(values[name] != 0.0 for name in names))

  def for_trial(self, names):
    names = canonical_names(names)
    return replace(
      self,
      version=VERSION,
      operation='trial',
      history_full_profile_sha256='',
      restore_transaction_sha256='',
      trial_names=names,
      trial_anchor=original_anchor(self.selected_baseline_revision, names),
      coordinates=self.prior_coordinates,
      full_profile_sha256=self.prior_full_profile_sha256,
    ).validate()

  def with_proposal(self, coordinates):
    from openpilot.starpilot.flm.canonical_surface_adapter import validate_trial

    if self.operation != 'trial':
      raise ValueError('restoration is not a tiny fitted trial')
    proposal = dict(validate_trial(coordinates))
    if tuple(name for name in NAMES if name in proposal) != self.trial_names:
      raise ValueError('trial names changed')
    full = dict(self.prior_coordinates)
    full.update(proposal)
    merged = tuple((name, float(full[name])) for name in NAMES)
    return replace(self, coordinates=merged, full_profile_sha256=full_profile_sha256(self.selected_baseline_revision, merged)).validate()

  def admit_context(
    self,
    *,
    cp_sha256,
    controller_revision,
    source_epoch,
    reset_generation,
    selected_baseline_revision,
    context_key,
    current_full_profile_sha256,
    persistent_source_epoch='',
  ):
    self.validate()
    expected = (cp_sha256, controller_revision, source_epoch, reset_generation, selected_baseline_revision, context_key, persistent_source_epoch)
    observed = (
      self.cp_sha256,
      self.controller_revision,
      self.source_epoch,
      self.reset_generation,
      self.selected_baseline_revision,
      self.context_key,
      self.persistent_source_epoch,
    )
    if expected != observed or current_full_profile_sha256 != self.prior_full_profile_sha256:
      raise ValueError('stale full profile/source CAS')
    return self


def envelope_from_dict(value):
  if type(value) is not dict or set(value) != set(FullSurfaceEnvelope.__dataclass_fields__):
    raise ValueError('full surface envelope fields')
  copied = dict(value)
  for key in ('prior_coordinates', 'coordinates'):
    copied[key] = tuple(tuple(row) for row in copied[key])
  copied['trial_names'] = tuple(copied['trial_names'])
  return FullSurfaceEnvelope(**copied).validate()


def envelope_dict(value):
  if not isinstance(value, FullSurfaceEnvelope):
    raise ValueError('typed full surface envelope required')
  value.validate()
  return asdict(value)


def neutral_coordinates():
  return tuple((n, 0.0) for n in NAMES)


def restore_transaction(envelope):
  return hashlib.sha256(
    repr(
      (
        'full-surface-history-restore-v1',
        envelope.source_binding(),
        envelope.manifest_sha256,
        envelope.full_original_anchor,
        envelope.prior_full_profile_sha256,
        envelope.full_profile_sha256,
        envelope.history_full_profile_sha256,
        envelope.trial_anchor,
      )
    ).encode()
  ).hexdigest()
