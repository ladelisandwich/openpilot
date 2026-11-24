"""Real selected LaC/native calls; unit-issued live witness is not physical proof."""

from dataclasses import replace
from types import SimpleNamespace
import math
import pytest
from openpilot.starpilot.flm.live_trial_owner import TrackingTrialOwner
from openpilot.starpilot.flm.tests.test_live_trial_owner import offer


@pytest.mark.parametrize('domain', ['torque', 'starpilot', 'pid'])
@pytest.mark.parametrize('curvature', [-0.0003, 0.0003])
def test_actual_controller_hooks_and_native_packet_are_preserved(domain, curvature):
  from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
  from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
  from openpilot.starpilot.flm.tests.test_event_lazy_native import TOYOTA, HYUNDAI, HONDA, make_interface, install_observer, settle, control, apply, BASE_NS

  platform = {'torque': TOYOTA.TOYOTA_COROLLA_TSS2, 'starpilot': HYUNDAI.HYUNDAI_IONIQ_6, 'pid': HONDA.HONDA_CIVIC}[domain]
  ci = make_interface(platform)
  assert install_observer(ci)
  settle(ci, control(0.0))
  factory = PidControllerFactory(ci.CP, 'native') if domain == 'pid' else SelectedControllerFactory(ci.CP, 'starpilot' if domain == 'starpilot' else 'standard')
  controller, vm = factory.create()
  from openpilot.starpilot.flm.global_runtime import GlobalRuntime
  runtime = GlobalRuntime(ci.CP, ci, controller)
  assert runtime.descriptor is not None
  assert runtime.descriptor.domain == ('torque_pid' if domain == 'pid' else 'torque')
  assert controller.automatic_flm_observation is runtime.observation and runtime.observation is not None
  if domain != 'pid':
    assert controller.automatic_flm_stages is runtime and controller.automatic_flm_centre is runtime
  start = BASE_NS + ci.CC.frame * 10_000_000
  o = offer(start)
  owner = TrackingTrialOwner(o.binding, domain='torque_pid' if domain == 'pid' else 'torque')
  seen = 0
  from openpilot.starpilot.flm.live_tracking_response import ResponseWitness, prepare_response_witness, validate_response_witness

  raw = validate_response_witness(o.response_witness, o.binding, start)
  for tick in range(120):
    now = start + tick * 10_000_000
    if tick % 5 == 0:
      w = prepare_response_witness(
        ResponseWitness(
          raw.certificate, o.binding, now - raw.certificate.lag_ns, now - raw.certificate.lag_ns, now, start - 500_000_000, 8, 0.001, now + 100_000_000
        ),
        o.binding,
        now,
      )
      current = replace(o, sequence=tick // 5 + 1, observed_ns=now, expires_ns=now + 100_000_000, response_witness=w)
    assert owner.prepare(current, now_ns=now, permission=True, binding=o.binding)
    runtime.observation.begin_frame()
    controller.automatic_flm_effects = owner
    result, _, _ = controller.update(True, ci.CS.out, vm, SimpleNamespace(angleOffsetDeg=0.0, roll=0.0), False, curvature, False, 0.2)
    assert math.isfinite(float(result)) and abs(float(result)) <= 1.0
    assert owner.used and owner.source.stage_calls == 1
    assert owner.frame_complete(owner.domain)
    ack = owner.acknowledge_frame(True)
    assert ack is not None and ack.status == 'applied'
    if domain == 'pid':
      assert owner.source.pid_error_calls == 1
    else:
      assert owner.source.friction_calls == 1
    _, packets, witness = apply(ci, control(float(result)))
    if witness is not None:
      seen += 1
      assert (witness.can_address, witness.can_data, witness.can_bus) in packets
  assert seen > 0
  assert owner.applied_full['ff_gain_left'] == pytest.approx(0.005)
  # A confirmed acknowledgement requires the actual domain hooks again.
  now = start + 1_200_000_000
  w = prepare_response_witness(
    ResponseWitness(
      raw.certificate, o.binding, now - raw.certificate.lag_ns, now - raw.certificate.lag_ns, now, start - 500_000_000, 8, 0.001, now + 100_000_000
    ),
    o.binding,
    now,
  )
  confirmed = replace(o, phase='confirmed', sequence=25, observed_ns=now, expires_ns=now + 100_000_000, response_witness=w)
  assert owner.prepare(confirmed, now_ns=now, permission=True, binding=o.binding)
  runtime.observation.begin_frame()
  controller.automatic_flm_effects = owner
  controller.update(True, ci.CS.out, vm, SimpleNamespace(angleOffsetDeg=0.0, roll=0.0), False, curvature, False, 0.2)
  ack = owner.acknowledge_frame(True)
  assert ack is not None and ack.status == 'applied' and ack.confirmed_target_sha256 == o.target_sha256
  assert owner.confirmed_full == o.target
  owner.neutral('driver_override')
  controller.automatic_flm_effects = owner
  assert owner.source is None and not any(owner.applied_full.values())
  assert owner.acknowledge_frame(True) is None


def test_pid_registry_keeps26_and_exact4_unsupported():
  from openpilot.starpilot.flm.pid_stages import SUPPORTED, UNSUPPORTED
  from openpilot.starpilot.flm.canonical_surface_adapter import NAMES

  assert len(NAMES) == 30 and len(SUPPORTED) == 26 and len(UNSUPPORTED) == 4
  assert set(SUPPORTED) | set(UNSUPPORTED) == set(NAMES)
  assert UNSUPPORTED == frozenset(
    ('turn_in_threshold_reduction_left', 'turn_in_threshold_reduction_right', 'unwind_threshold_increase_left', 'unwind_threshold_increase_right')
  )
