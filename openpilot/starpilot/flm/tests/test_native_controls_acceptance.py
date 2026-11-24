"""Actual selected Controls consumes trusted native updates without identity churn."""
from contextlib import contextmanager
import json
import os
from types import SimpleNamespace
from unittest import mock

import pytest
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR as HYUNDAI
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory, capture_snapshot
from openpilot.starpilot.flm.learner_factory import TorqueCollectionAdapter, RAW_TORQUE
from openpilot.starpilot.flm.runtime_adapter_registry import baseline_values, baseline_revision, selected_descriptor, selected_identity
from openpilot.starpilot.flm.tests.native_controls_fixture import native_frame
from openpilot.starpilot.flm.tests.test_registered_controls_global import controls_for, published_cp
from openpilot.starpilot.lateral.controller_selection import DOCUMENT_KEY as SELECTION_KEY, ControllerMode, replace_mode
from openpilot.starpilot.lateral.learning_preferences import KEY, replace_preference
from openpilot.starpilot.lateral.torque_settings import DOCUMENT_KEY, FieldChoice, PlatformProfile, serialize_document

NOW = 400_000_000_000
CASES = [(HYUNDAI.HYUNDAI_IONIQ_6, 'starpilot', 1.22), (TOYOTA.TOYOTA_RAV4_TSS2, 'standard', 1.)]


def cp_for(vehicle):
  return interfaces[vehicle].get_non_essential_params(vehicle)


@pytest.mark.parametrize('vehicle,mode,multiplier', CASES)
def test_actual_controls_65_seconds_variable_four_hz_preserves_nominal_identity_and_single_transform(vehicle, mode, multiplier):
  cp = cp_for(vehicle)
  with controls_for(cp, mode, True) as controls:
    cp_bytes = controls.CP.as_builder().to_bytes()
    anchor = tuple(controls.LaC.native_learning_baseline)
    revision = controls.global_learning.baseline_revision
    identity = None
    for tick in range(6501):
      # A bounded staircase of real4Hz native packets; all other inputs retain
      # their declared-rate production SubMaster fixtures.
      delta = .01 + .001 * ((tick // 25) % 8)
      native_frame(controls, NOW + tick * 10_000_000, tick, native_overrides={
        'factor': float(cp.lateralTuning.torque.latAccelFactor) * (1. + delta),
        'friction': float(cp.lateralTuning.torque.friction) * .98,
      })
      runtime = controls.global_learning
      assert runtime is not None
      assert runtime.baseline_revision == revision
      assert tuple(controls.LaC.native_learning_baseline) == anchor
      row = controls.sm['lateralTorqueParameters']
      assert controls.LaC.torque_params.latAccelFactor == pytest.approx(float(row.latAccelFactorFiltered) * multiplier, rel=2e-7)
      assert controls.LaC.torque_params.friction == pytest.approx(float(row.frictionCoefficientFiltered), rel=2e-7)
      if tick >= 50:
        assert runtime.identity
        identity = identity or runtime.identity
        assert runtime.identity == identity
        assert runtime.native_refined
    assert controls.CP.as_builder().to_bytes() == cp_bytes
    assert controls.params.get(KEY) == replace_preference(None, cp, True)


@pytest.mark.parametrize('vehicle,mode,multiplier', CASES)
@pytest.mark.parametrize('bad', ['inner_invalid', 'version', 'stale', 'future', 'factor', 'friction_nan'])
def test_actual_controls_reject_invalid_native_receipts(vehicle, mode, multiplier, bad):
  cp = cp_for(vehicle)
  with controls_for(cp, mode, True) as controls:
    original = baseline_values(controls.LaC)
    values = {'factor': float(cp.lateralTuning.torque.latAccelFactor) * 1.05}
    if bad == 'inner_invalid':
      values['valid'] = False
    elif bad == 'version':
      values['version'] = 2
    elif bad == 'stale':
      values['source_offset_ns'] = -500_000_001
    elif bad == 'future':
      values['source_offset_ns'] = 1
    elif bad == 'factor':
      values['factor'] = float(cp.lateralTuning.torque.latAccelFactor) * 2.
    else:
      values['friction'] = float('nan')
    for tick in range(30):
      native_frame(controls, NOW + tick * 10_000_000, tick, native_overrides=values)
      assert baseline_values(controls.LaC) == original
      assert controls.LaC.native_learning_refinement is None


@pytest.mark.parametrize('vehicle,mode,multiplier', CASES)
def test_off_keeps_historical_controller_output_despite_qualified_native_packet(vehicle, mode, multiplier):
  cp = cp_for(vehicle)
  with controls_for(cp, mode, False) as actual, controls_for(cp, mode, False) as reference:
    original = baseline_values(actual.LaC)
    for tick in range(80):
      now = NOW + tick * 10_000_000
      command, _ = native_frame(actual, now, tick, native_overrides={
        'factor': float(cp.lateralTuning.torque.latAccelFactor) * 1.1})
      baseline, _ = native_frame(reference, now, tick, native=False)
      assert command.to_dict() == baseline.to_dict()
      assert baseline_values(actual.LaC) == original
      assert actual.LaC.native_learning_refinement is None


@contextmanager
def manual_ioniq():
  cp = cp_for(HYUNDAI.HYUNDAI_IONIQ_6)
  torque = cp.lateralTuning.torque
  basis = (float(torque.latAccelFactor), float(torque.latAccelOffset), float(torque.friction))
  document = serialize_document({str(cp.carFingerprint): PlatformProfile(basis, FieldChoice('custom', basis[0] * .95), FieldChoice())})
  with OpenpilotPrefix(), mock.patch.dict(os.environ, {'REPLAY': '1', 'SIMULATION': '0', 'TORQUE_REPLAY_RUNTIME': '0',
       'AOL_REPLAY_RUNTIME': '0', 'LANE_CENTERING_REPLAY_RUNTIME': '0'}), mock.patch('openpilot.selfdrive.controls.controlsd.messaging.PubMaster'):
    params = Params()
    params.put('CarParams', published_cp(cp, params).to_bytes(), block=True)
    params.put(KEY, replace_preference(None, cp, True), block=True)
    params.put(SELECTION_KEY, json.loads(replace_mode(None, cp, ControllerMode.STARPILOT)), block=True)
    params.put(DOCUMENT_KEY, json.loads(document), block=True)
    yield Controls(), document, basis


def test_actual_ioniq_manual_factor_remains_authoritative_over_native():
  with manual_ioniq() as (controls, document, basis):
    assert controls.torque_host is not None
    for tick in range(150):
      native_frame(controls, NOW + tick * 10_000_000, tick,
                   native_overrides={'factor': basis[0] * 1.2, 'friction': basis[2] * 1.1})
    assert controls.LaC.torque_params.latAccelFactor == pytest.approx(basis[0] * .95 * 1.22, rel=2e-7)
    assert controls.params.get(DOCUMENT_KEY) == json.loads(document)
    assert controls.LaC.native_learning_refinement is None


@pytest.mark.parametrize('vehicle,mode,multiplier', CASES)
def test_factory_rejects_in_envelope_tune_forgery_without_native_provenance(vehicle, mode, multiplier):
  cp = cp_for(vehicle)
  factory = SelectedControllerFactory(cp, mode)
  controller, vm = factory.create()
  descriptor = selected_descriptor(cp, factory.interface_class(cp), controller)
  anchor = baseline_values(controller)
  revision = baseline_revision(descriptor, anchor)
  epoch = 'a' * 64
  snapshot = capture_snapshot(controller, vm).values
  fields = dict(snapshot)
  fields['tune.latAccelFactor'] *= 1.03
  fields['pid.pos_limit'] *= 1.03
  fields['pid.neg_limit'] *= 1.03
  evidence = SimpleNamespace(stage_schema_revision=RAW_TORQUE, controller_mode=mode, turn_assist=False,
    policy=descriptor.policy, controller_revision=descriptor.controller_revision, source_epoch=epoch,
    baseline_revision=revision, identity=selected_identity(descriptor, epoch, revision),
    replay_snapshot=tuple(fields.items()), control_ns=NOW, generation=0, effect_trace=None, effect_prepare_ns=0)
  with pytest.raises(ValueError, match='checkpoint provenance'):
    TorqueCollectionAdapter(cp, evidence)
  # Changing nominal identity to suit forged values must fail before restore.
  forged = list(anchor)
  forged[1] *= 1.03
  wrong_revision = baseline_revision(descriptor, tuple(forged))
  evidence.baseline_revision = wrong_revision
  evidence.identity = selected_identity(descriptor, epoch, wrong_revision)
  with mock.patch('openpilot.starpilot.flm.controller_replay.restore_snapshot', side_effect=AssertionError('must not restore')):
    with pytest.raises(ValueError, match='constructor identity'):
      TorqueCollectionAdapter(cp, evidence)
