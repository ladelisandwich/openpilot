"""Restore-attempt quarantine; numerical lifecycle only, no native permission."""
from dataclasses import replace
from types import SimpleNamespace

from openpilot.starpilot.flm.steerlearnerd import _LiveHistoryRestore
from openpilot.starpilot.flm.tests.test_live_trial_owner import offer


def actor_for(binding):
  holds = []
  actor = SimpleNamespace(seed=binding, response_recovery_generation=0,
    binding_for=lambda collector, source: binding, hold_restore=holds.append)
  return actor, holds


def test_new_response_recovery_clears_only_attempt_and_survives_collector_reset(monkeypatch):
  binding = offer().binding
  historical_rows = (object(),)
  history = _LiveHistoryRestore(historical_rows)
  actor, holds = actor_for(binding)
  history.seed = binding
  placeholder = object()
  monkeypatch.setattr(history, 'prior', placeholder)
  monkeypatch.setattr(history, 'requalifier', placeholder)
  monkeypatch.setattr(history, 'offer', placeholder)
  actor.response_recovery_generation = 1
  assert history.observe_response_recovery(actor)
  assert history.priors is historical_rows
  assert history.prior is history.requalifier is history.offer is None
  assert history.blocked_seed == binding
  assert holds == [False]
  history.reset()  # An unrelated collector reset cannot reselect this prior.
  collector = SimpleNamespace(ready=[None])
  assert history.advance(actor, collector, object(), now_ns=1, current_native=True,
    native_key=(), hardware_inventory=()) is None
  assert history.seed == history.blocked_seed == binding
  assert history.requalifier is None
  assert not history.observe_response_recovery(actor)
  assert history.response_recovery_generation == 1


def test_only_actual_binding_change_clears_current_history_quarantine():
  binding = offer().binding
  history = _LiveHistoryRestore(())
  actor, _ = actor_for(binding)
  actor.response_recovery_generation = 1
  assert history.observe_response_recovery(actor)
  collector = SimpleNamespace(ready=[None])
  history.reset()
  assert history.blocked_seed == binding
  assert history.advance(actor, collector, object(), now_ns=100_000_000_001,
    current_native=False, native_key=(), hardware_inventory=()) is None
  assert history.blocked_seed == binding
  changed = replace(binding, source_epoch='9'*64)
  actor.binding_for = lambda collector, source: changed
  assert history.advance(actor, collector, object(), now_ns=100_000_000_002,
    current_native=True, native_key=(), hardware_inventory=()) is None
  assert history.blocked_seed is None and history.seed == changed
  assert history.priors == ()


def test_fresh_incoming_control_rechecks_native_instead_of_inheriting_denied_previous(monkeypatch):
  from openpilot.starpilot.flm.steerlearnerd import _collect_source_control
  from openpilot.starpilot.flm import runtime_adapter_registry as registry
  from openpilot.starpilot.flm.native_health import current_native_health
  identity = '8'*64
  monkeypatch.setattr(registry, 'current_authority', lambda sm, cp, **kwargs: (True, object(), identity))
  now = 300_000_000
  old = SimpleNamespace(native_required=True, native_allowed=False, native_identity=identity,
    native_observed_ns=1, control_ns=100_000_000, native_expires_ns=200_000_001)
  fresh = SimpleNamespace(native_required=True, native_allowed=True, native_identity=identity,
    native_observed_ns=200_000_000, control_ns=290_000_000, native_expires_ns=400_000_000)
  observed=[]
  collector=SimpleNamespace(_learning_current_native_allowed=current_native_health(None,None,old,now))
  assert collector._learning_current_native_allowed is False
  collector.control=lambda control, stamp: observed.append((control,stamp,collector._learning_current_native_allowed))
  _collect_source_control(collector,fresh,None,None,now)
  assert observed==[(fresh,now,True)]
  # A current grant cannot authorize a record denied at its original issuance.
  _collect_source_control(collector,old,None,None,now)
  assert observed[-1]==(old,now,False)
  fresh.native_identity='9'*64
  _collect_source_control(collector,fresh,None,None,now)
  assert observed[-1]==(fresh,now,False)
  fresh.native_identity=identity
  monkeypatch.setattr(registry,'current_authority',lambda sm,cp,**kwargs:(False,object(),identity))
  _collect_source_control(collector,fresh,None,None,now)
  assert observed[-1]==(fresh,now,False)
