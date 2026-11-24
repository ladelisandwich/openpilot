"""Actual controller/native numeric reuse, explicitly mocked original seals.

Root fresh owner fitting with real whole keys is still required. No fixture
below grants original, shadow, profile or current/native admission.
"""
from dataclasses import replace
import pickle
from unittest import mock
import pytest


def closed_engine(monkeypatch,domain='torque',*,neutral_full=False):
  from openpilot.starpilot.flm.tests.test_candidate_feedback import actual_caller_fixture
  from openpilot.starpilot.flm import feedback_replay
  from openpilot.starpilot.flm.feedback_replay import FeedbackReplayEvidence,_SEAL
  engine,old,_,_=actual_caller_fixture(domain)
  if neutral_full:
    from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
    from openpilot.starpilot.flm.event_objective import EffectObjective
    from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
    envelope=engine.records[0].effect_trace.envelope.for_trial(GROUPS['ff.left'])
    records=[replace(frame,effect_trace=None) for frame in engine.records]
    engine.engine.records.clear()
    engine.engine.records.extend(records)
    objective=EffectObjective('ff.left',envelope.trial_names,envelope.trial_anchor,envelope.context_key)
    engine=EventControllerReplay(engine.engine,objective,envelope.replay_anchor(),activation_plan=engine.activation_plan)
  evidence=FeedbackReplayEvidence(old.binding,old.original,old.windows,old.engine_input_digest,_SEAL,old.candidate_policy)
  monkeypatch.setattr(engine, 'verified_feedback_tape', evidence, raising=False)
  monkeypatch.setattr(feedback_replay,'require_feedback_engine',lambda owner:evidence)
  # Unit isolation only. Existing full key mutation tests and fresh owner tests
  # independently qualify production key generation.
  monkeypatch.setattr(engine,'_causal_input_key',lambda epsilon:b'unit-owned-key')
  return engine,evidence


def certificate(engine):
  result=engine._rollout(engine.reference)
  saved=engine._EventControllerReplay__strict_rollouts
  assert saved is not None
  entry=saved[1]['baseline']
  assert len(entry)==3
  return (saved,entry,pickle.dumps(engine.reference,protocol=5)),result


@pytest.mark.parametrize('domain',['torque','starpilot','pid'])
def test_completed_actual_strict_closed_loop_is_exact_fd_reference_only(monkeypatch,domain):
  engine,_=closed_engine(monkeypatch,domain)
  proof,result=certificate(engine)
  with mock.patch.object(engine,'_rollout_uncached',side_effect=AssertionError('reuse must not execute')):
    copied=engine._strict_fd_reference(engine.reference,proof)
  assert copied==result[1] and copied is not result[1]
  first=next(iter(copied))
  copied[first]+=1
  assert engine._strict_fd_reference(engine.reference,proof)==result[1]
  for coordinates in (tuple(0 for _ in engine.reference),tuple(-0. for _ in engine.reference),tuple(.001 for _ in engine.reference)):
    assert engine._strict_fd_reference(coordinates,proof) is None


@pytest.mark.parametrize('change',['missing','shadow','coordinates','source','original','chronology','count','source_key','lost','feedback'])
def test_closed_reference_declines_changed_certificate_and_owned_inputs(monkeypatch,change):
  engine,evidence=closed_engine(monkeypatch)
  cert,_=certificate(engine)
  saved,entry,bits=cert
  if change=='source_key':
    monkeypatch.setattr(engine,'_causal_input_key',lambda epsilon:b'changed')
  elif change=='lost':
    saved[1].pop('baseline')
  elif change=='feedback':
    object.__setattr__(evidence,'engine_input_digest','f'*64)
  else:
    proof,fingerprint=entry[2]
    fields={'shadow':{'native_policy':'native-shadow-identification'},'coordinates':{'coordinates_digest':'0'*64},
      'source':{'source_digest':'0'*64},'original':{'original_digest':'0'*64},
      'chronology':{'chronology_digest':'0'*64},'count':{'row_count':proof.row_count-1}}
    replacement=(entry[0],entry[1]) if change=='missing' else (entry[0],entry[1],(replace(proof,**fields[change]),fingerprint))
    saved[1]['baseline']=replacement
    cert=(saved,replacement,bits)
  assert engine._strict_fd_reference(engine.reference,cert) is None


@pytest.mark.parametrize('index',[1,2])
def test_neutral_full_torque_turnin_unwind_induction_matches_actual_closed_native(monkeypatch,index):
  engine,_=closed_engine(monkeypatch,neutral_full=True)
  proof,result=certificate(engine)
  coordinate=list(engine.reference)
  coordinate[index]=.01
  coordinate=tuple(coordinate)
  expected=engine._rollout_uncached(coordinate,_identification_token=engine._EventControllerReplay__identification_token)
  assert expected==result
  assert engine._zero_gain_fd_reference(coordinate,proof)==expected[1]
  for other in (0,3):
    coordinate=list(engine.reference)
    coordinate[other]=.01
    assert engine._zero_gain_fd_reference(tuple(coordinate),proof) is None


def test_zero_gain_reuse_rejects_traces_and_nonzero_original_gain(monkeypatch):
  engine,_=closed_engine(monkeypatch)
  proof,_=certificate(engine)
  assert engine._zero_gain_fd_reference((.01,),proof) is None
  full,_=closed_engine(monkeypatch,neutral_full=True)
  full_proof,_=certificate(full)
  full.prior_full['ff_gain_left']=.001
  assert full._zero_gain_fd_reference((0.,.01,0.,0.),full_proof) is None
