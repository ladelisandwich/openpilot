"""Actual controller application acknowledgement, distinct from producer fit."""

from openpilot.starpilot.flm.adaptive_profile import ApplicationOutcome

VERSION = 1
MAX_AGE_NS = 500_000_000


def write_application(envelope, outcome):
  from openpilot.starpilot.flm.live_trial_owner import LiveTrialApplication
  if type(outcome) is LiveTrialApplication:
    from openpilot.starpilot.flm.live_trial_wire import write_application as write_live_application
    write_live_application(envelope, outcome)
    return
  if not isinstance(outcome, ApplicationOutcome):
    return
  envelope.learningApplication = {
    'version': 2 if outcome.effect_applied_profile_sha256 else VERSION,
    'identity': outcome.identity,
    'profileSequence': outcome.profile_sequence,
    'observedMonoTime': outcome.observed_ns,
    'status': outcome.status,
    'actualCoordinates': outcome.actual_coordinates,
    'reason': outcome.reason,
    'baselineRevision': outcome.baseline_revision,
    'physicalModelRevision': outcome.physical_model_revision,
    'effectManifestSha256': outcome.effect_manifest_sha256,
    'effectOriginalAnchor': outcome.effect_original_anchor,
    'effectAppliedProfileSha256': outcome.effect_applied_profile_sha256,
  }


def read_application(envelope, *, now_ns):
  row = envelope.learningApplication
  identity = str(row.identity)
  if (
    int(row.version) not in (VERSION, 2)
    or len(identity) != 64
    or any(c not in '0123456789abcdef' for c in identity)
    or type(now_ns) is not int
    or not 0 < int(row.observedMonoTime) <= now_ns <= int(row.observedMonoTime) + MAX_AGE_NS
    or str(row.status) not in ('collecting', 'applied', 'paused', 'rejected')
    or len(row.actualCoordinates) > 8
    or len(str(row.reason)) > 96
    or len(str(row.baselineRevision)) > 96
    or len(str(row.physicalModelRevision)) > 96
  ):
    return None
  values = tuple(float(v) for v in row.actualCoordinates)
  import math

  if not all(math.isfinite(v) for v in values):
    return None
  effect = tuple(str(getattr(row, name)) for name in
    ('effectManifestSha256','effectOriginalAnchor','effectAppliedProfileSha256'))
  if int(row.version) == 2:
    from openpilot.starpilot.flm.canonical_surface_adapter import MANIFEST_SHA256, full_original_anchor
    if (any(len(v) != 64 or any(c not in '0123456789abcdef' for c in v) for v in effect) or
        len(str(row.baselineRevision)) != 64 or any(c not in '0123456789abcdef' for c in str(row.baselineRevision)) or
        effect[0] != MANIFEST_SHA256 or effect[1] != full_original_anchor(str(row.baselineRevision))):
      return None
  elif any(effect):
    return None
  if str(row.status) == 'applied' and (
    not values or (int(row.version) == VERSION and not any(v != 0 for v in values)) or
    not row.profileSequence or not row.baselineRevision or not row.physicalModelRevision
  ):
    return None
  return ApplicationOutcome(
    identity,
    int(row.profileSequence),
    int(row.observedMonoTime),
    str(row.status),
    values,
    str(row.reason),
    str(row.baselineRevision),
    str(row.physicalModelRevision),
    *effect,
  )
