"""Versioned causal candidate feedback; never original/current admission."""
from dataclasses import dataclass
import hashlib
import math
import pickle
from typing import cast

LEGACY_POLICY = 'original-consumed-feedback-regime-v1'
CLOSED_POLICY = 'candidate-native-consumed-co-closed-loop-v1'
PROOF_VERSION = 'candidate-native-feedback-chronology-v1'
_SEAL = object()


def policy(value):
  if type(value) is not str or value not in (LEGACY_POLICY,CLOSED_POLICY):
    raise ValueError('explicit candidate feedback policy')
  return value


def uses_closed_loop(engine):
  # Opting OUT of a shortcut grants no source/native/health authority. Unknown
  # shells/proxies retain the old path without attribute callbacks or recursion.
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  from openpilot.starpilot.flm.feedback_replay import FeedbackReplayEvidence,_SEAL as feedback_seal
  if type(engine) is not EventControllerReplay:
    return False
  owned = object.__getattribute__(engine,'__dict__')
  evidence = owned.get('verified_feedback_tape')
  if type(evidence) is not FeedbackReplayEvidence:
    return False
  values = object.__getattribute__(evidence,'__dict__')
  value = values.get('candidate_policy')
  return values.get('_seal') is feedback_seal and type(value) is str and value == CLOSED_POLICY


@dataclass(frozen=True)
class CandidateFeedbackProof:
  version: str
  source_digest: str
  original_digest: str
  coordinates_digest: str
  native_policy: str
  chronology_digest: str
  row_count: int
  _seal: object


def strict_reference_proof_key(proof, engine, coordinates, request_count):
  """Fresh exact strict chronology certificate; numerical FD reuse only.

  The returned immutable primitive bytes detect later mutation of a retained
  proof object. They never replace the fresh engine/source key at the caller.
  """
  from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
  if (type(proof) is not CandidateFeedbackProof or proof._seal is not _SEAL or
      type(coordinates) is not tuple or type(request_count) is not int or
      any(type(value) not in (int,float) or not math.isfinite(value) for value in coordinates)):
    return None
  evidence = require_feedback_engine(engine)
  fields = (proof.version,proof.source_digest,proof.original_digest,proof.coordinates_digest,
            proof.native_policy,proof.chronology_digest,proof.row_count)
  if (any(type(value) is not str for value in fields[:6]) or type(proof.row_count) is not int or
      proof.version != PROOF_VERSION or evidence.candidate_policy != CLOSED_POLICY or
      proof.source_digest != evidence.engine_input_digest or proof.original_digest != evidence.original.digest or
      proof.coordinates_digest != hashlib.sha256(pickle.dumps(coordinates,protocol=5)).hexdigest() or
      proof.native_policy != 'strict' or not 1 <= proof.row_count == request_count <= 12000 or
      len(proof.chronology_digest) != 64 or any(c not in '0123456789abcdef' for c in proof.chronology_digest)):
    return None
  return pickle.dumps(fields,protocol=5)

def _strict_reference_fields(proof, evidence, coordinates, request_count):
  """Numerical fields only; caller MUST own fresh full pre/post source checks.

  Public callers retain strict_reference_proof_key and its own fresh admission.
  This helper creates no seal, proof, cache entry or current/native permission.
  """
  from openpilot.starpilot.flm.feedback_replay import FeedbackReplayEvidence, _SEAL as feedback_seal
  if (type(proof) is not CandidateFeedbackProof or proof._seal is not _SEAL or
      type(evidence) is not FeedbackReplayEvidence or evidence._seal is not feedback_seal or
      type(coordinates) is not tuple or type(request_count) is not int or
      any(type(value) not in (int,float) or not math.isfinite(value) for value in coordinates)):
    return None
  fields = (proof.version,proof.source_digest,proof.original_digest,proof.coordinates_digest,
            proof.native_policy,proof.chronology_digest,proof.row_count)
  if (any(type(value) is not str for value in fields[:6]) or type(proof.row_count) is not int or
      proof.version != PROOF_VERSION or evidence.candidate_policy != CLOSED_POLICY or
      proof.source_digest != evidence.engine_input_digest or proof.original_digest != evidence.original.digest or
      proof.coordinates_digest != hashlib.sha256(pickle.dumps(coordinates,protocol=5)).hexdigest() or
      proof.native_policy != 'strict' or not 1 <= proof.row_count == request_count <= 12000 or
      len(proof.chronology_digest) != 64 or any(c not in '0123456789abcdef' for c in proof.chronology_digest)):
    return None
  return pickle.dumps(fields,protocol=5)


class CandidateFeedbackLoop:
  """One rollout, constant working state; native reports remain their real map.

  Immutable original observations are never rewritten. Only the next update's
  safety_limited input is reconstructed. The consumed CO coordinate is exact,
  not nearest-time lookup, physical delivery or an invented receiver contract.
  """
  def __init__(self, engine, reported, coordinates, *, native_policy, original_baseline=False):
    from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
    from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
    evidence = require_feedback_engine(engine)
    if (evidence.candidate_policy != CLOSED_POLICY or type(reported) is not dict or reported or
        type(engine.emissions) is not EmissionHistory or type(coordinates) is not tuple or
        any(type(value) not in (int,float) or not math.isfinite(value) for value in coordinates) or
        type(original_baseline) is not bool or type(native_policy) is not str or
        native_policy not in ('strict','native-shadow-identification')):
      raise ValueError('registered candidate native feedback scope')
    self.engine,self.evidence,self.reported = engine,evidence,reported
    self.coordinates_digest = hashlib.sha256(pickle.dumps(coordinates,protocol=5)).hexdigest()
    self.native_policy,self.original_baseline = native_policy,original_baseline
    self._iterator = iter(self._rows())
    self._pending = None
    self._previous_window = self._previous_ns = self._previous_flag = None
    self._stream,self._count = hashlib.sha256(),0

  def _rows(self):
    for bounds,subset,activation in self.evidence.windows:
      context = tuple(row for row in self.evidence.original.context_only if bounds[0] <= row.control_ns <= bounds[1])
      if len(context) > 1 or context and context[0].control_ns != bounds[0]:
        raise ValueError('candidate feedback only actual leading context')
      for row in context:
        if row.control_ns >= activation:
          raise ValueError('unknown leading CO cannot be a candidate treatment')
        yield bounds,activation,row,None
      for joined in subset.rows:
        yield bounds,activation,joined.consumption,joined.publication

  @staticmethod
  def _record(bounds,activation,row,pub,input_flag,request,reported,flag):
    return (PROOF_VERSION,bounds,activation,row.control_ns,row.prior_publish_ns,
      row.publication_ns,row.recv_frame,row.receipt_ns,row.co_body_sha256,
      None if pub is None else (pub.output_control_ns,pub.output_apply_ns,pub.output_carrier_sequence,
        pub.native_identity,pub.applied_sha256),input_flag,request,reported,flag)

  @staticmethod
  def _add(stream,record):
    body = pickle.dumps(record,protocol=5)
    stream.update(len(body).to_bytes(8,'big'))
    stream.update(body)

  def before(self, frame):
    if self._pending is not None:
      raise ValueError('candidate feedback previous control unfinished')
    try:
      bounds,activation,row,pub = next(self._iterator)
    except StopIteration as error:
      raise ValueError('candidate feedback extra controller frame') from error
    if type(frame.control_ns) is not int or frame.control_ns != row.control_ns:
      raise ValueError('candidate feedback exact controller chronology')
    same_window = bounds == self._previous_window
    if same_window and row.prior_publish_ns != self._previous_ns:
      raise ValueError('candidate feedback previous publication omitted')
    previous = cast(bool, self._previous_flag if same_window else row.input_limited)
    if row.control_ns < activation and previous is not row.input_limited:
      raise ValueError('candidate changed original preactivation input')
    values = tuple(pair for pair in frame.inputs if pair[0] == 'safety_limited')
    if (len(values) != 1 or type(values[0][1]) not in (int,float) or
        values[0][1] != float(row.input_limited)):
      raise ValueError('original actual safety_limited input changed')
    self._pending = bounds,activation,row,pub,previous
    if previous is row.input_limited:
      return frame.inputs
    return tuple((key,float(previous)) if key == 'safety_limited' else (key,value) for key,value in frame.inputs)

  def _equation(self,bounds,activation,row,pub,input_flag,request,reported_map):
    from openpilot.starpilot.flm.feedback_observation import f32
    request = f32(request)
    original = self.original_baseline or row.control_ns < activation or pub is None
    if pub is None or pub.output_control_ns < activation:
      reported = row.reported_torque if pub is None else pub.reported_torque
    else:
      if (not pub.output_control_ns <= pub.output_apply_ns <= pub.publication_ns <= row.control_ns or
          pub.output_apply_ns not in reported_map):
        raise ValueError('candidate consumed exact native CO not yet produced')
      reported = f32(reported_map[pub.output_apply_ns])
    flag = bool(row.lat_active and abs(request-reported) > .01)
    if original and (request != row.requested_torque or reported != row.reported_torque or
                     input_flag is not row.input_limited or flag is not row.published_limited):
      raise ValueError('candidate changed original baseline or preactivation feedback')
    return request,reported,flag

  def after(self,control_ns,request):
    if self._pending is None or control_ns != self._pending[2].control_ns:
      raise ValueError('candidate feedback controller pair missing')
    bounds,activation,row,pub,input_flag = self._pending
    request,reported,flag = self._equation(bounds,activation,row,pub,input_flag,request,self.reported)
    self._add(self._stream,self._record(bounds,activation,row,pub,input_flag,request,reported,flag))
    self._count += 1
    self._previous_window,self._previous_ns,self._previous_flag = bounds,control_ns,flag
    self._pending = None

  def finish(self,requests,reported):
    from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
    if self._pending is not None or reported is not self.reported or type(requests) is not dict:
      raise ValueError('candidate feedback complete native/controller maps required')
    try:
      next(self._iterator)
    except StopIteration:
      pass
    else:
      raise ValueError('candidate feedback original control omitted')
    check = hashlib.sha256()
    prior_window = prior_ns = prior_flag = None
    count = 0
    for bounds,activation,row,pub in self._rows():
      if row.control_ns not in requests:
        raise ValueError('candidate feedback complete request map required')
      same_window = prior_window == bounds
      if same_window and row.prior_publish_ns != prior_ns:
        raise ValueError('candidate feedback prior candidate omitted')
      input_flag = prior_flag if same_window else row.input_limited
      request,torque,flag = self._equation(bounds,activation,row,pub,input_flag,requests[row.control_ns],reported)
      self._add(check,self._record(bounds,activation,row,pub,input_flag,request,torque,flag))
      prior_window,prior_ns,prior_flag = bounds,row.control_ns,flag
      count += 1
    evidence = require_feedback_engine(self.engine)
    if (evidence.candidate_policy != CLOSED_POLICY or evidence.engine_input_digest != self.evidence.engine_input_digest or
        evidence.original.digest != self.evidence.original.digest or count != self._count or not 1 <= count <= 12000 or
        len(requests) != count or check.digest() != self._stream.digest()):
      raise ValueError('candidate feedback owned chronology/native results changed')
    return CandidateFeedbackProof(PROOF_VERSION,evidence.engine_input_digest,evidence.original.digest,
      self.coordinates_digest,self.native_policy,check.hexdigest(),count,_SEAL)


def report_policy_matches(submitted,offered):
  """Declaration matching only; physical/native/current guards remain callers'."""
  if type(submitted) is not tuple or len(submitted) not in (3,4) or type(offered) is not dict:
    return False
  expected = {'protocol','binding_digest','archive_digest','verified_digest','known_rows','context_rows'}
  if len(submitted) == 4:
    if type(submitted[3]) is not str or submitted[3] != CLOSED_POLICY:
      return False
    expected |= {'candidate_feedback_policy','candidate_feedback_proof_version'}
    if (type(offered.get('candidate_feedback_policy')) is not str or offered['candidate_feedback_policy'] != CLOSED_POLICY or
        type(offered.get('candidate_feedback_proof_version')) is not str or offered['candidate_feedback_proof_version'] != PROOF_VERSION):
      return False
  return set(offered) == expected
