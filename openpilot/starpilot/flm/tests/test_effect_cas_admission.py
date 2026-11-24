"""Exact runtime bookkeeping regression; no simulated native proof claim."""
from types import SimpleNamespace
import pytest

from openpilot.starpilot.flm.global_runtime import GlobalRuntime
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture, NOW, BASE


def test_expired_profile_never_becomes_accepted_full30_cas_target():
  _,p,v,_=fixture()
  envelope=p.effect_envelope
  runtime=GlobalRuntime.__new__(GlobalRuntime)
  runtime.descriptor=SimpleNamespace(cp_sha256=envelope.cp_sha256,controller_revision=envelope.controller_revision,domain="torque")
  runtime.baseline_revision=BASE
  runtime.identity=p.identity
  runtime.effect_full_hash=''
  runtime.effect_owner=runtime.effect_envelope=None
  runtime.controller=SimpleNamespace(automatic_flm_effects=None)
  source=SimpleNamespace(enabled=True,manual=False,source_epoch=envelope.source_epoch,reset=envelope.reset_generation,
    persistent_source_epoch=envelope.persistent_source_epoch)
  with pytest.raises(ValueError,match='effect proof not currently admitted'):
    runtime._prepare_effect(p,v,source,now_ns=NOW+500_000_001,active=True,healthy=True)
  assert runtime.effect_full_hash==''
  assert runtime.effect_owner is None and runtime.effect_envelope is None
  assert runtime.controller.automatic_flm_effects is None
