from dataclasses import replace
from types import SimpleNamespace

from openpilot.cereal import messaging
from openpilot.starpilot.flm.adaptive_profile import ApplicationOutcome
from openpilot.starpilot.flm.application_wire import write_application, read_application
from openpilot.starpilot.flm.selected_effects import EffectOwner
from openpilot.starpilot.flm.tests.test_selected_effect_owner import fixture as owner_fixture, BASE
from openpilot.starpilot.flm.tests.test_training_report import fixture, publish, NOW
from openpilot.starpilot.flm.training_report import read_report
from openpilot.starpilot.flm.torque_adapter import Admission


def trace():
  old, p, v, _ = owner_fixture(accumulated=True)
  envelope = replace(p.effect_envelope, coordinates=p.effect_envelope.prior_coordinates, full_profile_sha256=p.effect_envelope.prior_full_profile_sha256)
  p = replace(p, observed_ns=NOW, expires_ns=NOW + 500_000_000, coefficients=(0.0,), effect_envelope=envelope)
  v = replace(v, profile=p)
  owner = EffectOwner(
    p.identity, old.objective, selected_baseline_revision=BASE, physical_model_revision=v.physical_model_revision, original_anchor=envelope.replay_anchor()
  )
  context = {
    'cp_sha256': envelope.cp_sha256,
    'controller_revision': envelope.controller_revision,
    'source_epoch': envelope.source_epoch,
    'reset_generation': envelope.reset_generation,
    'selected_baseline_revision': BASE,
    'context_key': envelope.context_key,
    'current_full_profile_sha256': envelope.prior_full_profile_sha256,
    'persistent_source_epoch': envelope.persistent_source_epoch,
  }
  owner.prepare(Admission(p, NOW, True, False, True, v), active=True, current_context=context)
  assert owner.applied_coordinates == (0.0,) and any(owner.applied_full.values())
  return owner, p, v


def test_zero_current_group_nonzero_actual_other_group_reports_truthful_full30(tmp_path):
  owner, p, v = trace()
  collector, control, reporter, source, path = fixture(tmp_path)
  actual = owner.trace
  control = replace(
    control,
    identity=p.identity,
    source_epoch=p.effect_envelope.source_epoch,
    baseline_revision=BASE,
    applied_coordinates=(0.0, 0.0),
    effect_trace=actual,
    effect_prepare_ns=NOW,
    applied_profile_sequence=p.sequence,
    applied_profile_observed_ns=p.observed_ns,
    applied_profile_expires_ns=p.expires_ns,
  )
  collector.ready[-1] = (control, collector.ready[-1][1])
  source.source_epoch = control.source_epoch
  outcome = ApplicationOutcome(
    p.identity,
    p.sequence,
    NOW,
    'applied',
    (0.0,),
    'actual_stage_used',
    BASE,
    v.physical_model_revision,
    actual.envelope.manifest_sha256,
    actual.envelope.full_original_anchor,
    actual.applied_full_profile_sha256,
  )
  event = messaging.new_message('starpilotLateralState', valid=True, logMonoTime=NOW)
  write_application(event.starpilotLateralState, outcome)
  assert read_application(event.starpilotLateralState, now_ns=NOW) == outcome
  reporter.control = (event.as_reader(), control)
  publish(reporter, collector, source)
  current = read_report(path, now_ns=NOW)['current']
  assert current['application']['applied']
  assert current['details']['Actual full surface coordinates'] == [[n, v] for n, v in actual.applied]
  assert current['details']['Actual full profile hash'] == actual.applied_full_profile_sha256
  assert len(current['details']['Applied correction coordinates']) == 1


def test_hash_mismatch_never_claims_applied(tmp_path):
  owner, p, v = trace()
  collector, control, reporter, source, path = fixture(tmp_path)
  actual = owner.trace
  control = replace(
    control,
    identity=p.identity,
    source_epoch=p.effect_envelope.source_epoch,
    baseline_revision=BASE,
    effect_trace=actual,
    effect_prepare_ns=NOW,
    applied_profile_sequence=p.sequence,
  )
  collector.ready[-1] = (control, collector.ready[-1][1])
  source.source_epoch = control.source_epoch
  outcome = ApplicationOutcome(
    p.identity,
    p.sequence,
    NOW,
    'applied',
    (0.0,),
    'actual_stage_used',
    BASE,
    v.physical_model_revision,
    actual.envelope.manifest_sha256,
    actual.envelope.full_original_anchor,
    '0' * 64,
  )
  event = messaging.new_message('starpilotLateralState', valid=True, logMonoTime=NOW)
  write_application(event.starpilotLateralState, outcome)
  reporter.control = (event.as_reader(), control)
  publish(reporter, collector, source)
  current = read_report(path, now_ns=NOW)['current']
  assert not current['application']['applied']
  assert 'Actual full profile hash' not in current['details']


def test_actual_controller_changed_outside_group_sets_global_applied_status():
  from opendbc.car import structs
  from opendbc.car.car_helpers import interfaces
  from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
  from openpilot.starpilot.flm.global_runtime import GlobalRuntime
  from openpilot.starpilot.flm.tests.test_selected_effect_callers import actual_paths

  owner, p, v = trace()
  key, raw = actual_paths()[0]
  with structs.CarParams.from_bytes(raw) as cp:
    factory = SelectedControllerFactory(cp, key[1], turn_assist=False)
    controller, vm = factory.create()
    runtime = GlobalRuntime(cp, interfaces[cp.carFingerprint](cp), controller)
    runtime.effect_owner = owner
    runtime.identity = p.identity
    runtime.profile, runtime.validation = p, v
    runtime.baseline_revision = BASE
    runtime.now_ns = NOW
    runtime.eligible = runtime.active = runtime.healthy = True
    runtime.reason = 'qualified'
    controller.automatic_flm_effects = owner
    cs = structs.CarState.new_message(vEgo=30.0, canValid=True, steeringAngleDeg=0.0, steeringRateDeg=0.0, gearShifter='drive')
    params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
    output, _, log = controller.update(True, cs, vm, params, False, 0.00001, False, 0.15)
    assert owner.used and owner.changed
    runtime._finish_frame(cs, vm, params, log, desired_curvature=0.00001, curvature_limited=False, lat_delay=0.15, requested=output, selected_command=output)
    assert runtime.outcome.status == 'applied'
    assert runtime.outcome.actual_coordinates == (0.0,)
    assert runtime.outcome.effect_applied_profile_sha256 == owner.trace.applied_full_profile_sha256
