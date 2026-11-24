"""Learner-only confirmed live targets. Pending offers never enter this document."""

from dataclasses import asdict, dataclass
import hashlib
import json
from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding, LiveTrialOffer, full_coordinates, validate_offer
from openpilot.starpilot.flm.live_trial_owner import LiveTrialApplication

KEY = 'AutomaticSteeringLiveTrials'
MAX_BYTES = 32768
MAX_ROWS = 4
VERSION = 'live-confirmed-history-v1'


def _pairs(items):
  result = {}
  for key, value in items:
    if key in result:
      raise ValueError('duplicate live history field')
    result[key] = value
  return result


def stable_binding(binding):
  binding.validate()
  if not binding.persistent_source_epoch:
    raise ValueError('unknown vehicle has no crossdrive history authority')
  return (binding.cp_sha256, binding.controller_revision, binding.baseline_revision, binding.reset_generation, binding.persistent_source_epoch)


def scope(binding):
  return hashlib.sha256(repr(stable_binding(binding)).encode()).hexdigest()


def decode(raw):
  if raw is None:
    return {'version': VERSION, 'confirmed': {}}
  if type(raw) is not bytes or not 0 < len(raw) <= MAX_BYTES:
    raise ValueError('live history bytes bound')
  try:
    doc = json.loads(raw, object_pairs_hook=_pairs)
  except (UnicodeError, RecursionError) as error:
    raise ValueError('live history parse') from error
  if (
    type(doc) is not dict
    or set(doc) != {'version', 'confirmed'}
    or doc['version'] != VERSION
    or type(doc['confirmed']) is not dict
    or len(doc['confirmed']) > MAX_ROWS
  ):
    raise ValueError('live confirmed history document')
  for key, row in doc['confirmed'].items():
    expected = {
      'binding',
      'target',
      'target_sha256',
      'trial_id',
      'sequence',
      'application_ns',
      'previous_target_sha256',
      'certificate',
    }
    if type(row) is not dict or set(row) not in (expected, expected | {'retained_response'}):
      raise ValueError('confirmed row only; no pending trials')
    binding = LiveTrialBinding(**row['binding']).validate()
    from openpilot.starpilot.flm.live_tracking_response import certificate_from_dict

    cert = certificate_from_dict(row['certificate'])
    if cert.binding != binding or cert.model_revision != binding.physical_model_revision:
      raise ValueError('confirmed history model provenance')
    if row.get('retained_response') is not None:
      from openpilot.starpilot.flm.live_tracking_requalification import retained_from_dict

      retained, _ = retained_from_dict(row['retained_response'])
      if retained != cert:
        raise ValueError('retained basis belongs to another confirmed certificate')
    values = tuple((n, v) for n, v in row['target'])
    if (
      key != scope(binding)
      or full_coordinates(values, binding.baseline_revision) != row['target_sha256']
      or type(row['trial_id']) is not str
      or len(row['trial_id']) != 64
      or type(row['sequence']) is not int
      or row['sequence'] <= 0
      or type(row['application_ns']) is not int
      or row['application_ns'] <= 0
    ):
      raise ValueError('confirmed history identity')
  return doc


@dataclass(frozen=True)
class LiveConfirmedPrior:
  binding: LiveTrialBinding
  target: tuple[tuple[str, float], ...]
  target_sha256: str
  trial_id: str
  sequence: int
  certificate: object
  historical_only: bool = True
  response_basis: object | None = None


class LiveTrialPersistence:
  def __init__(self, params):
    self.params = params

  def load_prior(self, binding):
    from openpilot.starpilot.saved_source import read_saved

    raw, readable = read_saved(self.params, KEY, MAX_BYTES)
    if not readable:
      return None
    try:
      row = decode(raw)['confirmed'].get(scope(binding))
      if row is None:
        return None
      old = LiveTrialBinding(**row['binding']).validate()
      if stable_binding(old) != stable_binding(binding):
        return None
      return _prior_from_row(row)
    except (ValueError, TypeError, KeyError, RecursionError):
      return None

  def save_confirmed(self, offer, application, certificate, *, now_ns, current_binding, authorized, response_basis=None):
    from openpilot.starpilot.saved_source import read_saved
    from openpilot.starpilot.saved_document import commit_exact

    if type(offer) is not LiveTrialOffer or type(application) is not LiveTrialApplication:
      return False
    try:
      validate_offer(offer)
      if (
        offer.phase != 'confirmed'
        or current_binding() != offer.binding
        or not authorized()
        or application.binding != offer.binding
        or application.trial_id != offer.trial_id
        or not 0 < application.sequence <= offer.sequence
        or application.target_sequence != offer.target_sequence
        or application.status != 'applied'
        or application.applied != offer.target
        or application.applied_sha256 != offer.target_sha256
        or application.confirmed_target_sha256 != offer.target_sha256
        or not 0 < application.control_ns <= now_ns <= application.control_ns + 100_000_000
        or not offer.observed_ns <= now_ns <= offer.expires_ns
      ):
        return False
      raw, readable = read_saved(self.params, KEY, MAX_BYTES)
      if not readable:
        return False
      doc = decode(raw)
      key = scope(offer.binding)
      previous = doc['confirmed'].get(key)
      retained_response = None
      if response_basis is not None:
        from openpilot.starpilot.flm.live_tracking_requalification import retained_to_dict

        retained_response = retained_to_dict(certificate, response_basis)
      if (
        previous is not None
        and previous['target_sha256'] == offer.target_sha256
        and previous['trial_id'] == offer.trial_id
        and (retained_response is None or previous.get('retained_response') == retained_response)
      ):
        return True  # Already committed and re-read; publication renewals do not rewrite Params.
      if previous is not None and previous['target_sha256'] != offer.baseline_sha256 and previous['target_sha256'] != offer.target_sha256:
        return False
      if previous is None:
        from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256

        if offer.baseline_sha256 != full_profile_sha256(offer.binding.baseline_revision, tuple((n, 0.0) for n in NAMES)):
          return False
        if len(doc['confirmed']) >= MAX_ROWS:
          return False
      from openpilot.starpilot.flm.live_tracking_response import certificate_to_dict

      if certificate.binding != offer.binding or certificate.model_revision != offer.binding.physical_model_revision:
        return False
      certificate = certificate_to_dict(certificate)
      doc['confirmed'][key] = {
        'binding': asdict(offer.binding),
        'target': offer.target,
        'target_sha256': offer.target_sha256,
        'trial_id': offer.trial_id,
        'sequence': offer.sequence,
        'application_ns': application.control_ns,
        'previous_target_sha256': offer.baseline_sha256,
        'certificate': certificate,
        'retained_response': retained_response,
      }
      desired = json.dumps(doc, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
      decode(desired)
      if len(desired) > MAX_BYTES or current_binding() != offer.binding or not authorized():
        return False
      result = commit_exact(
        self.params,
        key=KEY,
        max_bytes=MAX_BYTES,
        raw=desired,
        expected=raw,
        authorized=lambda: authorized() and current_binding() == offer.binding,
        temp_prefix='.live-steering-',
      )
      return result.committed and result.verified
    except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
      return False


def load_confirmed_history(params):
  """Controls startup ONLY: immutable history, no current application permission."""
  from openpilot.starpilot.saved_source import read_saved

  raw, readable = read_saved(params, KEY, MAX_BYTES)
  if not readable:
    return ()
  try:
    doc = decode(raw)
    return tuple(_prior_from_row(row) for row in doc['confirmed'].values())
  except (ValueError, TypeError, KeyError, RecursionError):
    return ()


def _prior_from_row(row):
  from openpilot.starpilot.flm.live_tracking_response import certificate_from_dict

  certificate = certificate_from_dict(row['certificate'])
  basis = None
  if row.get('retained_response') is not None:
    from openpilot.starpilot.flm.live_tracking_requalification import retained_from_dict

    retained, basis = retained_from_dict(row['retained_response'])
    if retained != certificate:
      raise ValueError('retained historical certificate mismatch')
  return LiveConfirmedPrior(
    LiveTrialBinding(**row['binding']).validate(),
    tuple((n, v) for n, v in row['target']),
    row['target_sha256'],
    row['trial_id'],
    row['sequence'],
    certificate,
    True,
    basis,
  )
