"""Stdlib-only production loader failure and command/provenance boundaries."""
import sys
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
from openpilot.starpilot.models.runtime_bridge import runtime
from openpilot.starpilot.models.tests.runtime_bridge_fixtures import ProcessCustody
from tools.laptop_device_build import qcom_artifact_contract as contract
PREFIX = 'openpilot.starpilot.models.runtime_bridge'

class RuntimeBoundaryTests(ProcessCustody):

  def test_compiler_selection_does_not_require_inference_libraries(self):
    selected = {'role': 'driving'}
    with (
      patch.object(runtime, '_selection', None),
      patch.object(runtime, 'activate', return_value=selected) as activate,
      patch.object(runtime, '_verify_policy', side_effect=AssertionError('compiler cannot require inference libraries')),
    ):
      self.assertIs(runtime.prepare_compiler(), selected)
      self.assertEqual(selected['purpose'], 'compiler')
      activate.assert_called_once()
      with self.assertRaisesRegex(RuntimeError, 'before model inference'):
        runtime.prepare_compiler()

  def test_compiler_cannot_infer_or_change_role_before_any_device_import(self):
    with (
      patch.object(runtime, '_selection', {'role': 'driving', 'purpose': 'compiler'}),
      patch.object(runtime, '_verify_policy', side_effect=AssertionError('must reject before guards')),
    ):
      with self.assertRaisesRegex(RuntimeError, 'cannot become'):
        runtime.prepare('driving')
      with self.assertRaisesRegex(RuntimeError, 'cannot load'):
        runtime._guard(True)
      with self.assertRaisesRegex(RuntimeError, 'cannot load'):
        runtime.load_oob('/missing')
      with self.assertRaises(RuntimeError):
        runtime.load_warp('/missing')

  def test_inference_cannot_reselect_compiler(self):
    with patch.object(runtime, '_selection', {'role': 'driving'}), patch.object(runtime, 'activate', side_effect=AssertionError('cannot activate')):
      with self.assertRaisesRegex(RuntimeError, 'before model inference'):
        runtime.prepare_compiler()

  def test_compilation_requires_compiler_purpose(self):
    for selection in (None, {'role': 'driving'}, {'role': 'monitoring'}):
      with patch.object(runtime, '_selection', selection):
        with self.assertRaisesRegex(RuntimeError, 'selected compiler entry point'):
          runtime.require_compiler()
    with patch.object(runtime, '_selection', {'role': 'driving', 'purpose': 'compiler'}):
      runtime.require_compiler()

  def test_failed_guard_install_is_not_published_and_retry_works(self):
    count = []

    class Candidate:

      def __init__(self, *args):
        count.append(self)

      def install_fresh_guard(self):
        if len(count) == 1:
          raise ValueError('owned hook setup failure')
    fake = types.ModuleType(PREFIX + '.usb')
    fake.USBProtection = Candidate
    with (
      patch.dict(sys.modules, {PREFIX + '.usb': fake}),
      patch.object(runtime, '_selection', {'role': 'driving'}),
      patch.object(runtime, '_protection', None),
      patch.object(runtime, '_verify_policy'),
      patch.object(runtime, '_library', side_effect=lambda name: name),
    ):
      with self.assertRaises(ValueError):
        runtime._guard(False)
      self.assertIsNone(runtime._protection)
      accepted = runtime._guard(False)
      self.assertIs(accepted, count[1])
      self.assertEqual(len(count), 2)
      self.assertIs(runtime._guard(True), accepted)

  def test_actual_global_compiler_command_rejected_before_provenance_access(self):
    fake = types.ModuleType('tools.laptop_device_build.validate_artifacts')
    fake.require_qcom_pickle = lambda _: None
    root = Path('/checkout')
    legacy = 'python3 /checkout/tinygrad_model_runtime/legacy9d/tinygrad_repo/examples/openpilot/compile_warp.py --output x'
    wrong = legacy.replace('/tinygrad_model_runtime/legacy9d/tinygrad_repo/', '/tinygrad_repo/')
    with (
      patch.dict(sys.modules, {'tools.laptop_device_build.validate_artifacts': fake}),
      patch.object(contract, 'commands', return_value={'x': legacy}),
      patch.object(contract, 'source_inventory', side_effect=AssertionError('must not reach inventory')),
    ):
      with self.assertRaisesRegex(ValueError, 'command recipe changed'):
        contract.verify(Path('/missing'), root, 'x', wrong)

  def test_stock_loader_requires_original_or_materialized_trusted_name(self):
    with tempfile.TemporaryDirectory() as tmp:
      p = Path(tmp) / 'arbitrary.pkl'
      p.write_bytes(b'untrusted')
      with self.assertRaisesRegex(ValueError, 'Unqualified stock'):
        runtime._stock_bytes(p)

  def test_policy_rejects_changed_source_before_import(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      (root / 'guard-policy.json').write_text('{}')
      with patch.object(runtime, '__file__', str(root / 'runtime.py')):
        with self.assertRaisesRegex(RuntimeError, 'policy changed'):
          runtime._verify_policy()
if __name__ == '__main__':
  unittest.main()
