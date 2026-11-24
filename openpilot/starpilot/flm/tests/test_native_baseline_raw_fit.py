"""Actual registered controller/CAN/child fit across exact changing native tunes.

Independent synthetic plant only; no physical vehicle acceptance is claimed.
"""
from dataclasses import replace
from openpilot.starpilot.flm.adaptive_profile import readmit_profile
from openpilot.starpilot.flm.learning_wire import result_message, read_runtime_admission
from openpilot.starpilot.flm.learner_factory import fit_snapshot, decode_fit_result
from openpilot.starpilot.flm.runtime_adapter_registry import baseline_values
from openpilot.starpilot.flm.tests.test_actual_raw_worker import raw_job, BASE, CONTEXT, CAR, interfaces
from openpilot.starpilot.flm.tests.test_registered_controls_global import controls_for, frame
from openpilot.starpilot.flm.tests.test_registered_accepted_profile import accepted_frame
from openpilot.cereal import messaging


def test_raw_native_parameter_changes_preserve_fit_and_require_current_exact_probe():
  cp = interfaces[CAR.TOYOTA_RAV4_TSS2].get_non_essential_params(CAR.TOYOTA_RAV4_TSS2)
  with controls_for(cp, 'standard', True) as controls, controls_for(cp, 'standard', True) as reference:
    for tick in range(30):
      frame(controls, BASE - 300_000_000 + tick * 10_000_000, tick)
      frame(reference, BASE - 300_000_000 + tick * 10_000_000, tick)
    runtime = controls.global_learning
    selected_revision = runtime.baseline_revision
    snapshot, adapter, control, output, costs = raw_job(controls.CP, runtime, controls.params, native_refinements=True)
    tuples = {point.torque_values for point in adapter.replay.records}
    assert len(tuples) >= 32
    assert runtime.baseline_revision == selected_revision
    # Real reconstruction/physical fitting/heldout replay; no supplied profile,
    # coefficients, fake inverse, or fabricated validation result.
    report = fit_snapshot(snapshot)
    decoded = decode_fit_result(report)
    assert decoded is not None and decoded[0].profile is not None, report
    result, validation = decoded
    assert any(result.profile.coefficients), report
    assert adapter.install_fit(report, result, validation, physical_context=CONTEXT)
    adapter.cohort(control, costs['last_physical'])
    current = adapter.current_admission(result.profile, control, output, CONTEXT)
    assert current.healthy
    assert current.native_baseline_values == control.baseline_values
    renewed = readmit_profile(result.profile, validation, current,
      now_ns=output.apply_ns + 2_000_000, sequence=result.profile.sequence + 1)
    assert renewed is not None
    profile, fresh_validation = renewed
    assert fresh_validation.native_baseline_values == control.baseline_values
    packet = result_message(replace(result, profile=profile), 'native-tuple-raw-test', profile.sequence,
      profile.observed_ns, replay_validation=fresh_validation)
    decoded_profile, decoded_validation = read_runtime_admission(messaging.log_from_bytes(packet.to_bytes()))
    assert decoded_validation == fresh_validation and decoded_profile == profile
    assert baseline_values(controls.LaC) == runtime.initial_values
    # Install the exact same healthy native stream in both actual Controls
    # instances. Only one gets the real child/current-probe-qualified FF fit.
    for target in (controls, reference):
      for stamp in (BASE - 500_000_000, BASE - 250_000_000, BASE):
        event = messaging.new_message('lateralTorqueParameters', valid=True, logMonoTime=stamp)
        row = event.lateralTorqueParameters
        row.version, row.valid, row.useParams = 1, True, True
        row.latAccelFactorFiltered, row.latAccelOffsetFiltered, row.frictionCoefficientFiltered = control.baseline_values[1:4]
        target.sm.update_msgs(stamp / 1e9, [event.as_reader()])
      target._global_fixture_source_ns = BASE - 250_000_000
    effect = False
    applied_result = replace(result, profile=profile)
    for tick in range(20):
      now = BASE + tick * 10_000_000
      ordinary, _ = frame(reference, now, tick + 30)
      actual, _ = accepted_frame(controls, now, tick + 30, applied_result, fresh_validation)
      assert runtime.baseline_revision == selected_revision
      assert runtime.reason == 'qualified' and runtime.used
      assert runtime.outcome is not None and runtime.outcome.status == 'applied'
      assert actual.latActive == ordinary.latActive and actual.longActive == ordinary.longActive
      assert baseline_values(controls.LaC) == control.baseline_values
      effect |= actual.actuators.torque != ordinary.actuators.torque
    assert effect
    identity = runtime.identity
    event = messaging.new_message('lateralTorqueParameters', valid=True, logMonoTime=BASE + 250_000_000)
    row = event.lateralTorqueParameters
    row.version, row.valid, row.useParams = 1, True, True
    row.latAccelFactorFiltered = control.baseline_values[1] * 1.001
    row.latAccelOffsetFiltered = control.baseline_values[2]
    row.frictionCoefficientFiltered = control.baseline_values[3]
    controls.sm.update_msgs((BASE + 250_000_000) / 1e9, [event.as_reader()])
    command, _ = accepted_frame(controls, BASE + 250_000_000, 55, applied_result, fresh_validation)
    assert command.latActive
    assert runtime.identity == identity and runtime.baseline_revision == selected_revision
    assert runtime.applied_coordinates == (0., 0.)
    assert runtime.outcome is None or runtime.outcome.status != 'applied'
    packet.autoSteeringLearning.learning.runtimeProfile.replay.nativeBaselineValues = [float('nan')]
    assert read_runtime_admission(packet.as_reader()) is None
