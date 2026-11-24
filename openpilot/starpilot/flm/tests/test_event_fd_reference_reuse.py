"""Exact FD baseline reuse, actual registered controllers/native projection.

No fitted result/current permission is supplied. Native/reference parity and
owned-source mutation misses are verified independently from FD arithmetic.
"""
from dataclasses import replace
import pickle
from unittest import mock

import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.tests.test_event_controller_prefix import fixture


def engine(domain='torque'):
  original=fixture(domain)
  envelope=original.records[0].effect_trace.envelope.for_trial(GROUPS['deadband.high'])
  objective=EffectObjective('deadband.high',envelope.trial_names,envelope.trial_anchor,envelope.context_key)
  value=EventControllerReplay(original.engine,objective,envelope.replay_anchor(),activation_plan=original.activation_plan)
  assert all(parameter.low==0. for parameter in value.objective.parameters)
  return value


def run(value, *, reuse):
  traces=[]
  original=value._rollout_uncached
  def execute(coordinates,**kwargs):
    result=original(coordinates,**kwargs)
    traces.append((pickle.dumps(tuple(coordinates),protocol=5),
      kwargs.get('_identification_token') is not None,pickle.dumps(result,protocol=5)))
    return result
  with mock.patch.object(value,'_rollout_uncached',side_effect=execute), \
       mock.patch.object(value.emissions,'decode',wraps=value.emissions.decode) as decode:
    if reuse:
      count=value.rebuild_causal_features()
    else:
      with mock.patch.object(value,'_strict_fd_reference',return_value=None):
        count=value.rebuild_causal_features()
    decoded=decode.call_count
  return count,pickle.dumps(value.projected,protocol=5),traces,decoded


@pytest.mark.parametrize('domain',('torque','starpilot','pid'))
def test_actual_boundary_fd_matrix_raw_requests_saturation_and_native_outputs_exact(domain):
  fast=engine(domain)
  slow=EventControllerReplay(fast.engine,fast.objective,fast.original_anchor,activation_plan=fast.activation_plan)
  expected=run(slow,reuse=False)
  actual=run(fast,reuse=True)
  assert actual[:2]==expected[:2]
  columns=len(fast.objective.names)
  assert len(expected[2])==1+2*columns
  assert len(actual[2])==1+columns
  baseline_bits=pickle.dumps(fast.reference,protocol=5)
  baseline_trace=next(trace[2] for trace in actual[2] if trace[0]==baseline_bits)
  for coordinates,shadow,result in expected[2]:
    if coordinates==baseline_bits:
      # Strict successful baseline equals every omitted shadow reference side.
      assert result==baseline_trace
    else:
      assert (coordinates,shadow,result) in actual[2]
  assert actual[3]<expected[3] and actual[3]>0  # Every unique rollout still decodes native history.


def certificate(value):
  value._rollout(value.reference)
  saved=value._EventControllerReplay__strict_rollouts
  assert saved is not None and 'baseline' in saved[1]
  return saved,saved[1]['baseline'],pickle.dumps(value.reference,protocol=5)


def test_success_is_readonly_copy_and_not_shadow_or_live_permission():
  value=engine()
  proof=certificate(value)
  first=value._strict_fd_reference(value.reference,proof)
  assert first is not None
  expected=dict(first)
  first[next(iter(first))]+=1.
  assert value._strict_fd_reference(value.reference,proof)==expected
  assert value._strict_fd_reference((0,0.),proof) is None  # Type exact, not numeric equality.
  assert value._strict_fd_reference((-0.,0.),proof) is None  # Signed zero is not the recorded bits.
  assert value._strict_fd_reference((.01,0.),proof) is None
  assert value._strict_fd_reference(value.reference,None) is None


@pytest.mark.parametrize('change',('cp','native','record','checkpoint','prior','plan','unknown_key','lost_success'))
def test_every_owned_source_change_denies_reference_reuse(change):
  value=engine()
  proof=certificate(value)
  if change=='cp':
    cp=value.factory.cp.as_builder()
    cp.steerRatio+=.01
    value.factory.cp=cp.as_reader()
  elif change=='native':
    stamp,event=next(iter(value.emissions.events.items()))
    value.emissions.events[stamp]=replace(event,witness=event.witness._replace(final_raw=event.witness.final_raw+1))
  elif change=='record':
    row=value.records[0]
    value.records[0]=replace(row,inputs=row.inputs+(('extra_input',1.),))
  elif change=='checkpoint':
    stamp,state=value.checkpoints[0]
    value.checkpoints[0]=(stamp,replace(state,values=state.values+(('unknown',1.),)))
  elif change=='prior':
    value.prior_full['ff_gain_right']=.001
  elif change=='plan':
    value.activation_plan=replace(value.activation_plan,physical_context='c'*64)
  elif change=='unknown_key':
    with mock.patch.object(value,'_causal_input_key',return_value=None):
      assert value._strict_fd_reference(value.reference,proof) is None
    return
  else:
    value._EventControllerReplay__strict_rollouts=None
  assert value._strict_fd_reference(value.reference,proof) is None


def test_missing_success_falls_back_to_original_shadow_rollouts():
  value=engine()
  original=value._rollout
  calls=[]
  def rollout(coordinates,**kwargs):
    result=original(coordinates,**kwargs)
    calls.append((tuple(coordinates),kwargs.get('_identification_token') is not None))
    if len(calls)==1:
      # Remove only the successful reuse record; never mint a replacement proof.
      value._EventControllerReplay__strict_rollouts=None
    return result
  with mock.patch.object(value,'_rollout',side_effect=rollout):
    value.rebuild_causal_features()
  assert len(calls)==1+2*len(value.objective.names)
  assert any(coords==value.reference and shadow for coords,shadow in calls)


def test_failed_strict_baseline_never_enters_fd_or_creates_success():
  value=engine()
  with mock.patch.object(value,'_rollout',side_effect=ValueError('strict baseline refused')) as rollout:
    with pytest.raises(ValueError,match='strict baseline refused'):
      value.rebuild_causal_features()
  assert rollout.call_count==1
  assert value._EventControllerReplay__strict_rollouts is None
