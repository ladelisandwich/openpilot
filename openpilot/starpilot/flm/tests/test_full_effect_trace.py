"""Full30 wire/child state and EXACT owner/replay transition equivalence."""
from dataclasses import replace
import pytest

from openpilot.starpilot.flm.canonical_surface_adapter import full_profile_sha256
from openpilot.starpilot.flm.effect_trace import trace_dict, trace_from_dict, ordered
from openpilot.starpilot.flm.learning_wire import read_control
from openpilot.starpilot.flm.selected_effects import EffectTrial, EffectReplayTransition
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import snapshots, serialized_control
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture, NOW, BASE
from openpilot.starpilot.flm.torque_adapter import Admission


def current_trace():
  owner,p,v,context=fixture(accumulated=True)
  owner.prepare(Admission(p,NOW,True,False,True,v),active=True,current_context=context)
  return owner,p,v,context


def test_typed_wire_and_child_snapshot_keep_all30_and_original_hash():
  owner,p,_,_=current_trace()
  control,_,_=snapshots(NOW)
  control=replace(control,baseline_revision=BASE,source_epoch=p.effect_envelope.source_epoch,
    controller_revision=p.effect_envelope.controller_revision,applied_coordinates=(0.,0.),features=(1.,0.),
    effect_trace=owner.trace,effect_prepare_ns=NOW)
  assert read_control(serialized_control(control))==control
  assert trace_from_dict(trace_dict(owner.trace))==owner.trace
  assert len(owner.trace.applied)==30
  assert owner.trace.applied_full_profile_sha256==full_profile_sha256(BASE,owner.trace.applied)


@pytest.mark.parametrize('field', ['applied','target','before','applied_full_profile_sha256','control_ns'])
def test_changed_transition_state_is_rejected(field):
  owner,_,_,_=current_trace()
  trace=owner.trace
  value='0'*64 if field=='applied_full_profile_sha256' else -1 if field=='control_ns' else tuple(
    (n, v+.001 if n=='ff_gain_left' else v) for n,v in getattr(trace,field))
  with pytest.raises(ValueError):
    replace(trace,**{field:value}).validate()


def test_exact_prior_centered_transition_matches_real_owner_with_jitter_and_withdrawal():
  owner,p,v,context=fixture(accumulated=True)
  # The new trial baseline is the ACTUAL accepted full target, not its old prior.
  accepted=p.effect_envelope
  anchor=replace(accepted,prior_coordinates=accepted.coordinates,
    prior_full_profile_sha256=accepted.full_profile_sha256)
  objective=owner.objective
  trial=EffectTrial(objective,tuple(dict(anchor.prior_coordinates)[n] for n in objective.names),BASE,anchor.replay_anchor())
  replay=EffectReplayTransition(trial)
  stamps=(NOW,NOW+11_000_000,NOW+19_000_000,NOW+31_000_000)
  for stamp in stamps:
    owner.prepare(Admission(p,stamp,True,False,True,v),active=True,current_context=context)
    source=replay.source_for_frame(stamp,owner.trace)
    assert ordered(source.source.values)==owner.trace.applied
  owner.prepare(Admission(p,NOW+40_000_000,True,False,False,v),active=True,current_context=context)
  replay.source_for_frame(NOW+40_000_000,None)
  assert not any(replay.applied.values())
  owner.prepare(Admission(p,NOW+50_000_000,True,False,True,v),active=True,current_context=context)
  source=replay.source_for_frame(NOW+50_000_000,owner.trace)
  assert ordered(source.source.values)==owner.trace.applied


def test_nonzero_original_profile_without_applied_trace_is_rejected():
  owner,p,_,_=current_trace()
  trial=EffectTrial(owner.objective,(.004,),BASE,p.effect_envelope.replay_anchor())
  with pytest.raises(ValueError):
    EffectReplayTransition(trial).source_for_frame(NOW,None)
