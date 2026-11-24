"""Wire/lifecycle/unit tests; no fixture claims actual CAN/plant qualification."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.live_tracking_probe_process import ProbeProcess, decode_probe_job, VERSION


@pytest.mark.parametrize('prefix', [(), (None,)*101])
def test_bounded_probe_export_rejects_invented_or_unbounded_recorder(prefix):
  from openpilot.starpilot.flm.live_tracking_probe_process import probe_replay_snapshot
  with pytest.raises(ValueError, match='genuine bounded'):
    probe_replay_snapshot(SimpleNamespace(), prefix)


@pytest.mark.parametrize('foreign', ['tuning_leaf', 'frame_object', 'frame_clock', 'checkpoint_clock', 'container'])
def test_exporter_rejects_callback_objects_before_selection_or_deepcopy(foreign):
  from collections import OrderedDict, deque
  from openpilot.starpilot.flm.controller_replay import ControllerReplay, SelectedControllerFactory, ReplayFrame, Snapshot
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence
  from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
  from openpilot.starpilot.flm.live_tracking_probe_process import probe_replay_snapshot
  calls = []

  class Trap:
    def __deepcopy__(self, memo):
      calls.append('deepcopy')
      return self
    def __le__(self, other):
      calls.append('compare')
      return True
    def __ge__(self, other):
      calls.append('compare')
      return True
    @property
    def control_ns(self):
      calls.append('clock')
      return 100

  class ForeignDeque(deque):
    def __reversed__(self):
      calls.append('reverse')
      return super().__reversed__()

  # Invalid-only exact-class shells. No controller, source/native lease or
  # successful probe is manufactured, and no constructor may be reached.
  snapshot = Snapshot('invalid-only', (('x', 1.),))
  control = ControlEvidence(identity='invalid-only', generation=1, control_ns=100, state_ns=100,
    domain='normalized_torque', stage_revision='invalid-only', response_revision='invalid-only', requested=0.,
    features=(), baseline_prediction=0., enabled=True, manual=False, active=True, valid=True,
    override=False, fault=False, saturated=False, limited=False, model_wobble=False, controller_chatter=False,
    replay_snapshot=snapshot.values)
  replay = object.__new__(ControllerReplay)
  replay.factory = object.__new__(SelectedControllerFactory)
  replay.failed, replay.raw_mode = False, True
  replay.emissions = object.__new__(EmissionHistory)
  replay.emissions.events, replay.emissions.carriers = OrderedDict(), OrderedDict()
  replay.checkpoints = deque(((100, snapshot),))
  replay.records = deque((ReplayFrame(100, (), 0., ()),))
  object.__setattr__(replay.records[0], 'torque_values', (('x', Trap()),))
  if foreign == 'frame_object':
    replay.records = deque((Trap(),))
  elif foreign == 'frame_clock':
    object.__setattr__(replay.records[0], 'control_ns', Trap())
  elif foreign == 'checkpoint_clock':
    replay.checkpoints = deque(((Trap(), snapshot),))
  elif foreign == 'container':
    replay.records = ForeignDeque(replay.records)
  with pytest.raises(ValueError):
    probe_replay_snapshot(replay, (control,))
  assert calls == []


@pytest.mark.parametrize('field', ['mode', 'policy'])
def test_factory_metadata_rejects_foreign_string_conversion_before_cp_access(field):
  from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
  from openpilot.starpilot.flm.live_tracking_probe import _factory_configuration
  from openpilot.starpilot.lateral.controller_selection import ControllerMode
  calls = []

  class Trap:
    def __str__(self):
      calls.append('str')
      return 'standard'
  factory = object.__new__(SelectedControllerFactory)
  factory.mode, factory.policy = ControllerMode.STANDARD, 'invalid-only'
  setattr(factory, field, Trap())
  with pytest.raises(ValueError):
    _factory_configuration(factory)
  assert calls == []


def test_genuine_ioniq6_native_namedtuple_projection_roundtrip_and_unknown_leaf_rejection():
  from opendbc.car.hyundai.values import CAR as HYUNDAI
  from opendbc.car.steering_projection import TorqueProjection, projection_from_dict, projection_to_dict
  from openpilot.starpilot.flm.tests.test_state_aligned_encoder import captured_event
  from openpilot.starpilot.flm.live_tracking_probe import _projection_literal
  # Real registered CI/CAN packer unit fixture; no permission/model verdict is
  # mocked or obtained by copying this observation into restricted transport.
  witness = captured_event(HYUNDAI.HYUNDAI_IONIQ_6).witness
  assert type(witness) is TorqueProjection and witness.can_data
  body = _projection_literal(witness)
  assert body == projection_to_dict(witness)
  assert projection_from_dict(body) == witness
  calls = []

  class Trap:
    def __deepcopy__(self, memo):
      calls.append('deepcopy')
      return self
    def __bytes__(self):
      calls.append('bytes')
      return b''
  with pytest.raises(ValueError):
    _projection_literal(witness._replace(can_data=Trap()))
  assert calls == []


def test_historical_native_expiry_does_not_become_current_permission():
  from openpilot.starpilot.flm.live_tracking_probe import _native_lease, _evidence_clock
  # Historical numeric clock/lease subchecks only, not counterfeit actual CAN.
  control = SimpleNamespace(native_allowed=True, native_observed_ns=1_000_000_000,
    control_ns=1_050_000_000, native_expires_ns=1_200_000_000)
  output = SimpleNamespace(native_allowed=True, apply_ns=1_060_000_000)
  binding = SimpleNamespace(native_required=True)
  real_queue = 90_000_000_000
  historical, evidence = _evidence_clock(evidence_ns=1_650_000_000, queued_ns=real_queue,
    now_ns=real_queue+500_000_000, current_native=False)
  assert historical and evidence == 1_650_000_000
  _native_lease(control, output, binding, now_ns=real_queue+500_000_000, historical=True)
  with pytest.raises(ValueError, match='native lease'):
    _native_lease(control, output, binding, now_ns=real_queue+500_000_000, historical=False)
  output.apply_ns = control.native_expires_ns+1
  with pytest.raises(ValueError, match='native lease'):
    _native_lease(control, output, binding, now_ns=real_queue, historical=True)


@pytest.mark.parametrize('change', [{'current_native': True}, {'now_ns': 31_000_000_002},
  {'queued_ns': 2_000_000_000}, {'evidence_ns': 0}])
def test_historical_numerical_clock_rejects_current_grant_and_invalid_queue(change):
  from openpilot.starpilot.flm.live_tracking_probe import _evidence_clock
  args = {'evidence_ns': 123, 'queued_ns': 1_000_000_000, 'now_ns': 1_500_000_000, 'current_native': False}
  args.update(change)
  with pytest.raises(ValueError):
    _evidence_clock(**args)


def test_future_live_evidence_requires_explicit_archival_diagnostic_clock_domain():
  from openpilot.starpilot.flm.live_tracking_probe import _evidence_clock
  args = {'evidence_ns': 9_000_000_000, 'queued_ns': 1_000_000_000,
    'now_ns': 1_500_000_000, 'current_native': False}
  with pytest.raises(ValueError):
    _evidence_clock(**args)
  assert _evidence_clock(**args, archived_clock=True) == (True, 9_000_000_000)
  with pytest.raises(ValueError):
    _evidence_clock(**dict(args, current_native=True), archived_clock=True)


def test_normal_process_rejects_archival_diagnostic_jobs_before_spawn(monkeypatch):
  from openpilot.starpilot.flm import live_tracking_probe_process as module
  payload = {'queued_ns': 100, 'evidence_ns': 9_000_000_000, 'archived_clock': True}
  monkeypatch.setattr(module, 'decode_probe_job', lambda job: payload)
  process = ProbeProcess()
  assert process.submit_probe({}, token=(), now_ns=100) is False
  assert process.child is None


def test_stale_actual_queue_is_rejected_before_child_start(monkeypatch):
  from openpilot.starpilot.flm import live_tracking_probe_process as module
  monkeypatch.setattr(module, 'decode_probe_job', lambda job: {'queued_ns': 100})
  process = ProbeProcess()
  assert process.submit_probe({}, token=(), now_ns=100+100_000_001) is False
  assert process.child is None


def _transport_envelope(index):
  """Deliberately INVALID replay/model bytes: wire-only, no authority fixture."""
  import hashlib
  from openpilot.starpilot.flm.controller_replay import pack_primitive_snapshot
  payload = {'version': VERSION, 'replay': f'invalid-transport-only-{index}'.encode(),
    'queued_ns': 100+index*50, 'evidence_ns': 10+index*50, 'archived_clock': False,
    'binding': {'invalid_transport_only': True}, 'accepted_full30': (), 'anchor': (), 'group': 'ff.left',
    'names': ('ff_gain_left',), 'prefix_controls': ({'control_ns': 1, 'replay_snapshot': (('invalid', 0.),)},
      {'control_ns': 10+index*10, 'replay_snapshot': ()}),
    'control': {'control_ns': 10+index*10, 'replay_snapshot': ()}, 'output': {'apply_ns': 11+index*10}}
  packed = pack_primitive_snapshot(payload)
  return {'version': VERSION, 'payload': packed.hex(), 'sha256': hashlib.sha256(packed).hexdigest()}


def test_batch_preserves_separately_observed_clocks_and_literal_snapshots():
  from openpilot.starpilot.flm.live_tracking_probe_process import make_probe_batch
  jobs = tuple(_transport_envelope(index) for index in range(4))
  batch = decode_probe_job(make_probe_batch(jobs, queued_ns=300))
  assert batch['queued_ns'] == 300
  assert len(batch['batch']) == 4
  for index, row in enumerate(batch['batch']):
    assert row == decode_probe_job(jobs[index])
    assert row['evidence_ns'] == 10+index*50
    assert row['queued_ns'] == 100+index*50
  # These invalid bytes MUST NOT be mistaken for a real child/controller proof.
  from openpilot.starpilot.flm.live_tracking_probe_worker import evaluate_job
  with pytest.raises((TypeError, ValueError)):
    evaluate_job(make_probe_batch(jobs, queued_ns=300), now_ns=400)


def test_batch_rejects_duplicate_endpoints_and_unbounded_inputs():
  from openpilot.starpilot.flm.live_tracking_probe_process import make_probe_batch
  first = _transport_envelope(0)
  with pytest.raises(ValueError):
    make_probe_batch((first, first), queued_ns=300)
  with pytest.raises(ValueError):
    make_probe_batch((first,)*5, queued_ns=300)
  with pytest.raises(ValueError):
    make_probe_batch((), queued_ns=300)


@pytest.mark.parametrize('reason', ['context', 'wall'])
def test_actual_optional_child_cancellation_reaps_without_blocking_publication(reason):
  import os
  import time
  process = ProbeProcess()
  process.worker_module = 'openpilot.starpilot.flm.tests._probe_lifecycle_fixture'
  process.wall_ns = 200_000_000
  now = time.monotonic_ns()
  try:
    assert process.submit({'stall': True}, token=('original',), now_ns=now)
    pid = process.child.pid
    deadline = time.monotonic()+5.
    while process.child is not None and time.monotonic() < deadline:
      before = time.monotonic()
      assert process.poll(token=('changed',) if reason == 'context' else ('original',), now_ns=time.monotonic_ns()) is None
      assert time.monotonic()-before < .5
      time.sleep(.01)
    assert process.child is None
    with pytest.raises(ProcessLookupError):
      os.kill(pid, 0)
  finally:
    process.close()


def test_actual_optional_child_applies_resource_and_scheduler_bounds():
  import os
  import time
  process = ProbeProcess()
  process.worker_module = 'openpilot.starpilot.flm.tests._probe_lifecycle_fixture'
  result = None
  try:
    assert process.submit({}, token=('resources',), now_ns=time.monotonic_ns())
    deadline = time.monotonic()+10.
    while process.child is not None and time.monotonic() < deadline:
      value = process.poll(token=('resources',), now_ns=time.monotonic_ns())
      if value is not None:
        result = value
      time.sleep(.01)
    assert process.child is None and result is not None
    assert 0 < result['cpu'] <= 20 and 0 < result['as'] <= 2*1024*1024*1024
    assert result['nice'] >= 10 and result['parent'] == os.getpid()
    if hasattr(os, 'sched_getscheduler'):
      assert result['scheduler'] == os.SCHED_OTHER
    assert result['real_now'] <= time.monotonic_ns()
  finally:
    process.close()


@pytest.mark.parametrize('job', [None, {}, {'version': VERSION, 'payload': 'zz', 'sha256': 'x'},
  {'version': VERSION, 'payload': '00', 'sha256': '0'*64}])
def test_malformed_or_changed_numerical_envelopes_rejected(job):
  with pytest.raises(ValueError):
    decode_probe_job(job)


def test_second_pending_job_is_rejected_without_inspecting_foreign_payload():
  process = ProbeProcess()
  from subprocess import Popen
  process.child = object.__new__(Popen)
  assert process.submit_probe(object(), token=(), now_ns=1) is False
  process.child = None


def test_child_context_change_cannot_return_owned_numerical_result(monkeypatch):
  from openpilot.starpilot.flm.fit_process import FitProcess
  process = ProbeProcess()
  process.expected = ('a'*64, {})
  monkeypatch.setattr(FitProcess, 'poll', lambda *args, **kwargs: {'version': VERSION, 'sha256': 'b'*64, 'rows': []})
  with pytest.raises(ValueError, match='owned child'):
    process.poll_probe(token=(), binding=None, now_ns=1)


@pytest.mark.parametrize('mode', ['standard', 'starpilot'])
@pytest.mark.parametrize('speed,curvature', [(30., 0.), (5., .005)])
def test_genuine_ioniq6_configured_assist_snapshot_replays_exact_actual_controller(mode, speed, curvature):
  from opendbc.car import structs
  from openpilot.starpilot.flm.tests.test_controller_replay import _params
  from openpilot.starpilot.flm.controller_replay import (
    SelectedControllerFactory, capture_snapshot, restore_snapshot, frame_input, _run, same_final_command,
  )
  from openpilot.starpilot.flm.torque_adapter import Objective
  from openpilot.starpilot.flm.runtime_adapter_registry import selected_descriptor
  factory = SelectedControllerFactory(_params().as_reader(), mode, turn_assist=True)
  actual, vm = factory.create()
  clone, clone_vm = factory.create()
  assert factory.turn_assist is True
  descriptor = selected_descriptor(factory.cp, factory.interface_class(factory.cp), actual)
  assert descriptor.turn_assist is True
  cs = structs.CarState.new_message(vEgo=speed, gearShifter='drive', steeringAngleDeg=0.)
  params = SimpleNamespace(angleOffsetDeg=0., roll=0.)
  before = capture_snapshot(actual, vm)
  assert any(name.startswith('turn_assist.') for name, _ in before.values)
  if mode == 'starpilot':
    assert dict(before.values)['policy.turn_assist'] == 1.
  inputs = frame_input(cs, vm, params, active=True, safety_limited=False,
    desired_curvature=curvature, curvature_limited=False, lat_delay=.2)
  expected, _ = _run(actual, vm, inputs, Objective('feedforward', 'low', 'steady'), (0., 0.),
    actual_stage_residual=(0., 0.))
  restore_snapshot(clone, clone_vm, before)
  reproduced, _ = _run(clone, clone_vm, inputs, Objective('feedforward', 'low', 'steady'), (0., 0.),
    actual_stage_residual=(0., 0.))
  assert same_final_command(expected, reproduced)
  assert capture_snapshot(clone, clone_vm) == capture_snapshot(actual, vm)
  assert factory.turn_assist is True
  # Configuration differs in exact source receipts; it is never rewritten False.
  assert replace(descriptor, turn_assist=False) != descriptor


def test_genuine_ioniq6_assist_can_be_noop_or_active_without_changing_configuration():
  from opendbc.car import structs
  from openpilot.starpilot.flm.tests.test_controller_replay import _params
  from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
  factory = SelectedControllerFactory(_params().as_reader(), 'standard', turn_assist=True)
  controller, vm = factory.create()
  cs = structs.CarState.new_message(vEgo=30., gearShifter='drive', steeringAngleDeg=0.)
  params = SimpleNamespace(angleOffsetDeg=0., roll=0.)
  assert controller.turn_assist.apply(cs, vm, params, 0., .1, 1.) == .1
  cs.vEgo = 5.
  assert controller.turn_assist.apply(cs, vm, params, .005, 0., 1.) != 0.
  assert factory.turn_assist is True
