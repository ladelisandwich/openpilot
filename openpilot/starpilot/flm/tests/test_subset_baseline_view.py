"""Actual controller/native numeric equality; original source seals mocked.

No manufactured fitting/profile/current permission. Whole source keys are real;
only the existing numeric fixture's original feedback seal is unit-isolated.
"""
from dataclasses import replace
from unittest import mock
import pickle

import pytest

from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
from openpilot.starpilot.flm.event_objective import EffectObjective
from openpilot.starpilot.flm.tests.test_closed_fd_reference_reuse import closed_engine


def full_subset(monkeypatch,domain='torque'):
  full,evidence=closed_engine(monkeypatch,domain,neutral_full=True)
  # Remove the older fixture's deliberately synthetic key: exercise actual full
  # source serialization/key freshness in this numerical equality test.
  del full.__dict__['_causal_input_key']
  assert type(full._causal_input_key(.01)) is bytes
  full.rebuild_causal_features()
  from openpilot.starpilot.flm.tests.test_full_surface_profile import base
  from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
  envelope=base(names=GROUPS['ff.left']).for_trial(('ff_gain_left',))
  objective=EffectObjective('ff.left',envelope.trial_names,envelope.trial_anchor,envelope.context_key)
  subset=EventControllerReplay(full.engine,objective,envelope.replay_anchor(),activation_plan=full.activation_plan)
  subset.verified_feedback_tape=evidence
  assert subset.reuse_subset_causal_features(full)
  assert subset._EventControllerReplay__subset_baseline_view is not None
  return full,subset,evidence


@pytest.mark.parametrize('domain',['torque','starpilot','pid'])
def test_numeric_view_matches_independent_actual_selected_controller_native_baseline(monkeypatch,domain):
  full,subset,_=full_subset(monkeypatch,domain)
  expected=subset._rollout_uncached(subset.reference)
  original_proof=full._EventControllerReplay__strict_rollouts[1]['baseline'][2][0]
  subset._EventControllerReplay__candidate_feedback_proof=None
  with mock.patch.object(subset,'_rollout_uncached',side_effect=AssertionError('duplicate baseline traversal')):
    actual=subset._rollout(subset.reference)
  assert pickle.dumps(actual,protocol=5)==pickle.dumps(expected,protocol=5)
  assert subset._EventControllerReplay__strict_rollouts is None
  assert subset._EventControllerReplay__candidate_feedback_proof is None
  assert full._EventControllerReplay__strict_rollouts[1]['baseline'][2][0] is original_proof
  first=next(iter(actual[0]))
  actual[0][first]+=.1
  assert subset._rollout(subset.reference)==expected


@pytest.mark.parametrize('change',['frame','native','coordinates','anchor','proof','missing_cache','replaced_cache',
  'replaced_entry','restore','changed_feedback','signed_zero','factory_callback'])
def test_changed_source_native_coordinates_anchor_proof_and_cache_decline_view(monkeypatch,change):
  full,subset,evidence=full_subset(monkeypatch)
  coordinates=subset.reference
  if change=='frame':
    full.records[0]=replace(full.records[0],selected_command=-0.)
  elif change=='native':
    stamp,event=next(iter(full.emissions.events.items()))
    full.emissions.events[stamp]=replace(event,witness=event.witness._replace(final_raw=event.witness.final_raw+1))
  elif change=='coordinates':
    coordinates=(.001,)
  elif change=='signed_zero':
    coordinates=(-0.,)
  elif change=='anchor':
    subset.original_anchor['accumulated']=list(subset.original_anchor['accumulated'])
  elif change=='proof':
    proof=full._EventControllerReplay__strict_rollouts[1]['baseline'][2][0]
    object.__setattr__(proof,'chronology_digest','0'*64)
  elif change=='missing_cache':
    full._EventControllerReplay__strict_rollouts=None
  elif change=='replaced_cache':
    cache=full._EventControllerReplay__strict_rollouts
    full._EventControllerReplay__strict_rollouts=(cache[0],dict(cache[1]))
  elif change=='replaced_entry':
    slots=full._EventControllerReplay__strict_rollouts[1]
    entry=slots['baseline']
    slots['baseline']=tuple(value for value in entry)
  elif change=='restore':
    subset.restore_envelope=object()
  elif change=='changed_feedback':
    object.__setattr__(evidence,'original',object())
  else:
    class Callback:
      def __str__(self):
        raise AssertionError('unknown mode must not become reusable numeric proof')
    subset.factory.mode=Callback()
  # This direct helper is a structural negative, not a native-fallback claim.
  source_key=subset._EventControllerReplay__subset_baseline_view.selected_key
  assert subset._equivalent_subset_baseline(coordinates,source_key) is None


def test_full_source_keys_are_rechecked_after_effect_basis_callbacks(monkeypatch):
  full,subset,_=full_subset(monkeypatch)
  from openpilot.starpilot.flm.selected_effects import EffectTrial
  original=EffectTrial.source
  calls=[]
  def mutate(trial):
    source=original(trial)
    calls.append(trial)
    full.factory.manual_source_epoch='changed-during-proof'
    return source
  monkeypatch.setattr(EffectTrial,'source',mutate)
  view=subset._EventControllerReplay__subset_baseline_view
  assert subset._equivalent_subset_baseline(subset.reference,view.selected_key) is None
  assert calls


def test_candidate_shadow_restore_and_fd_do_not_borrow_baseline(monkeypatch):
  full,subset,_=full_subset(monkeypatch)
  calls=[]
  original=subset._rollout_uncached
  def run(*args,**kwargs):
    calls.append((args,kwargs))
    return original(*args,**kwargs)
  monkeypatch.setattr(subset,'_rollout_uncached',run)
  subset._rollout((.00001,))
  assert len(calls)==1
  assert subset._EventControllerReplay__strict_rollouts[1].get('baseline') is None
  subset._rollout(subset.reference,_identification_token=subset._EventControllerReplay__identification_token)
  assert len(calls)==2
  assert subset._strict_fd_reference(subset.reference,None) is None
  with pytest.raises(ValueError,match='restore'):
    subset._rollout(subset.reference,restore_target=True)
  assert len(calls)==3


def test_unknown_private_view_and_argument_shapes_never_run_callbacks(monkeypatch):
  _,subset,_=full_subset(monkeypatch)
  view=subset._EventControllerReplay__subset_baseline_view
  class Unknown:
    def __reduce__(self):
      raise AssertionError('unknown coordinates must not be pickled')
    def __eq__(self,other):
      raise AssertionError('unknown source key must not compare')
  assert subset._equivalent_subset_baseline((Unknown(),),view.selected_key) is None
  assert subset._equivalent_subset_baseline(subset.reference,Unknown()) is None
  class Slots(dict):
    def get(self,*args):
      raise AssertionError('unknown cache dictionary must not be read')
  subset._EventControllerReplay__subset_baseline_view=view._replace(cache=(view.original_key,Slots()))
  assert subset._equivalent_subset_baseline(subset.reference,view.selected_key) is None


@pytest.mark.parametrize('custom_metaclass',[False,True])
def test_spoofed_interface_name_is_not_registered_constructor(monkeypatch,custom_metaclass):
  _,subset,_=full_subset(monkeypatch)
  actual=subset.factory.interface_class
  if custom_metaclass:
    class SpoofMeta(type(actual)):
      def __getattribute__(cls,name):
        if name in ('__module__','__qualname__'):
          raise AssertionError('unregistered metaclass metadata callback')
        return super().__getattribute__(name)
    class Interface(metaclass=SpoofMeta):
      pass
  else:
    Interface=type('CarInterface',(),{})
    Interface.__module__=actual.__module__
    Interface.__qualname__=actual.__qualname__
  subset.factory.interface_class=Interface
  view=subset._EventControllerReplay__subset_baseline_view
  assert subset._equivalent_subset_baseline(subset.reference,view.selected_key) is None


def test_unknown_factory_property_is_not_read_before_fallback(monkeypatch):
  full,subset,_=full_subset(monkeypatch)
  view=subset._EventControllerReplay__subset_baseline_view
  class UnknownFactory:
    @property
    def interface_class(self):
      raise AssertionError('unregistered factory property callback')
  full.engine.factory=UnknownFactory()
  assert subset._equivalent_subset_baseline(subset.reference,view.selected_key) is None
