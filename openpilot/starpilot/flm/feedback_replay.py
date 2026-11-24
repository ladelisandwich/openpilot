"""NEW event source feedback proof, separate from legacy event admission."""
from dataclasses import dataclass
from openpilot.starpilot.flm.candidate_feedback import LEGACY_POLICY,CLOSED_POLICY,PROOF_VERSION,policy as validate_candidate_policy

PROTOCOL='chronological-source-feedback-mean-fit-v1'
EVENT_VERSION=7
_SEAL=object()


@dataclass(frozen=True)
class FeedbackReplayEvidence:
  binding: str
  original: object
  windows: tuple
  engine_input_digest: str
  _seal: object
  candidate_policy: str = LEGACY_POLICY


def _inputs(engine, *, candidate_policy=LEGACY_POLICY):
  # Hash trusted in-process objects ONLY; no pickle data is ever decoded here.
  # Keep the same original CP/factory/controller/native/checkpoint/body coverage
  # as the existing causal owned-input key, without recursively hashing this seal.
  import hashlib
  import pickle
  from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
  from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
  factory=engine.factory
  if type(factory) not in (SelectedControllerFactory,PidControllerFactory):
    raise ValueError('registered original feedback factory required')
  policy=('torque',str(factory.policy)) if type(factory) is SelectedControllerFactory else ('stock-pid',None)
  selected=(type(factory).__module__,type(factory).__qualname__,factory.cp.as_builder().to_bytes(),
    str(factory.mode),policy,factory.turn_assist,factory.manual_baseline,factory.manual_source_epoch,
    factory.interface_class.__module__,factory.interface_class.__qualname__)
  native=(engine.emissions.cp.as_builder().to_bytes(),engine.emissions.identity,engine.emissions.baseline_revision,
    tuple(engine.emissions.events.items()),tuple(engine.emissions.carriers.items()))
  body=(engine.identity,engine.baseline_revision,engine.engine.model_revision,engine.engine.failed,
    engine.engine.raw_mode,selected,engine.engine.objective,engine.objective,engine.reference,
    engine.prior_full,engine.original_anchor,engine.activation_plan,engine.restore_envelope,
    tuple(engine.records),tuple(engine.checkpoints),tuple(engine.replay_windows),native)
  from openpilot.starpilot.flm.normalized_trace_archive import archive_key
  if archive_key(engine) is not None:
    body = (*body,archive_key(engine))
  validate_candidate_policy(candidate_policy)
  if candidate_policy != LEGACY_POLICY:
    body = (*body,candidate_policy,PROOF_VERSION)
  return hashlib.sha256(pickle.dumps(body,protocol=5)).hexdigest()


def verify_archive(packet, original):
  from openpilot.starpilot.flm.feedback_archive import archive_binding
  from openpilot.starpilot.flm.feedback_observation import verify_original
  from openpilot.starpilot.flm.feedback_archive_codec import expand_observations
  from openpilot.starpilot.flm.torque_raw_fit import output_from_dict
  from openpilot.starpilot.flm.normalized_trace_archive import normalized_packet,archive_of
  if normalized_packet(packet):
    import pickle
    owner_archive = archive_of(original)
    if owner_archive is None or pickle.dumps(owner_archive.literal,protocol=5) != pickle.dumps(packet['trace_archive'],protocol=5):
      raise ValueError('normalized feedback original literal source differs')
  binding=archive_binding(packet)
  publications,consumptions=expand_observations(packet['feedback_observations'],binding_digest=binding)
  frames=tuple(original.records)
  outputs={value.apply_ns:value for value in (output_from_dict(row) for row in packet['context_receipts']['outputs'])}
  return verify_original(publications,consumptions,original.original_context_controls,outputs,original,binding,
    expected_control_ids=tuple(frame.control_ns for frame in frames),context_packet=packet['context_receipts'],
    frames=frames,allow_initial_context_only=True)


def attach_feedback_engine(engine, tape, *, candidate_policy=LEGACY_POLICY):
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  from openpilot.starpilot.flm.feedback_observation import subset_verified
  if type(engine) is not EventControllerReplay or engine.activation_plan is None:
    raise ValueError('new feedback requires actual declared stage engine')
  windows=[]
  for bounds in engine.replay_windows:
    controls=tuple(frame.control_ns for frame in engine.records if bounds[0]<=frame.control_ns<=bounds[1])
    if not controls or controls[0]!=bounds[0] or controls[-1]!=bounds[1]:
      raise ValueError('complete feedback engine window required')
    context=tuple(row.control_ns for row in tape.context_only)
    if context:
      # The sealed global first q may precede a later stage-only replay.
      if len(context) != 1 or context[0] > engine.records[0].control_ns:
        raise ValueError('only original leading feedback context allowed')
      controls=tuple(stamp for stamp in controls if stamp not in context)
    subset=subset_verified(tape,controls,binding_digest=tape.binding_digest)
    activation=engine.activation_plan.activation_for(*bounds)
    if activation is None:
      raise ValueError('original feedback activation clock required')
    # An original context-only window has no treatment. Requiring every request
    # to equal original through its end proves the unchanged prefix instead.
    flag_activation=activation[0] or bounds[1]+1
    windows.append((bounds,subset,flag_activation))
  validate_candidate_policy(candidate_policy)
  evidence=FeedbackReplayEvidence(tape.binding_digest,tape,tuple(windows),
    _inputs(engine,candidate_policy=candidate_policy),_SEAL,candidate_policy)
  engine.verified_feedback_tape=evidence
  return evidence


def require_feedback_engine(engine):
  from openpilot.starpilot.flm.feedback_observation import validate_subset_verified
  evidence=getattr(engine,'verified_feedback_tape',None)
  if (type(evidence) is not FeedbackReplayEvidence or evidence._seal is not _SEAL or
      type(evidence.candidate_policy) is not str or evidence.candidate_policy not in (LEGACY_POLICY,CLOSED_POLICY) or
      evidence.engine_input_digest!=_inputs(engine,candidate_policy=evidence.candidate_policy) or
      tuple(bounds for bounds,_,_ in evidence.windows)!=tuple(engine.replay_windows)):
    raise ValueError('owned original feedback engine evidence required')
  for bounds,subset,activation in evidence.windows:
    controls=tuple(frame.control_ns for frame in engine.records if bounds[0]<=frame.control_ns<=bounds[1] and
      frame.control_ns not in tuple(row.control_ns for row in evidence.original.context_only))
    # Validate the owner seal/digest and exact full window again, never trust an
    # instance flag or wire-created VerifiedFeedbackTape-shaped object.
    validate_subset_verified(subset,controls,binding_digest=evidence.binding)
    if (engine.activation_plan.activation_for(*bounds)[0] or bounds[1]+1)!=activation:
      raise ValueError('feedback activation changed')
  return evidence


def verify_candidate_engine(engine,requests,reported):
  from openpilot.starpilot.flm.feedback_observation import verify_candidate_flags
  evidence=require_feedback_engine(engine)
  if evidence.candidate_policy != LEGACY_POLICY:
    raise ValueError('closed-loop candidate requires its own native chronology proof')
  for _bounds,subset,activation in evidence.windows:
    selected={row.consumption.control_ns:requests[row.consumption.control_ns] for row in subset.rows}
    verify_candidate_flags(subset,selected,reported,activation_ns=activation,binding_digest=evidence.binding)
  return True


def owned_feedback_key(engine):
  """New feedback evidence is part of every strict/FD/prefix owned-input key."""
  evidence=require_feedback_engine(engine)
  key = (PROTOCOL,evidence.binding,evidence.original.digest,
    tuple((bounds,subset.digest,activation) for bounds,subset,activation in evidence.windows))
  return key if evidence.candidate_policy == LEGACY_POLICY else (*key,evidence.candidate_policy,PROOF_VERSION)
