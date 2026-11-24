"""Keep lifecycle build identity mandatory without imposing its outputs on other gates."""
from contextlib import ExitStack, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.ci import run_vehicle_tests as vehicle
from tools import test_runner


class TestLifecycleProvenanceScope(unittest.TestCase):
  def test_general_source_provenance_does_not_require_lifecycle_outputs(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      (root / 'upstream-sync.json').write_text('{"dependencies": {}}')
      for name in ('tools/test_runner.py', 'tools/ci/vehicle_pytest.py', 'tools/ci/lifecycle_shards.py'):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('# fixture\n')
      with patch.object(vehicle, 'ROOT', root), patch.object(vehicle, 'source_input_snapshot', return_value={'original': True}), \
           patch.object(vehicle.subprocess, 'check_output', return_value=b'fixture'), \
           patch.object(vehicle.lifecycle_shards, 'build_identity', side_effect=AssertionError('unrelated artifact inventory')) as identity:
        source = vehicle.source_provenance()
      self.assertEqual(source['inputs'], {'original': True})
      self.assertNotIn('build_identity', source)
      identity.assert_not_called()

  def test_missing_lifecycle_build_identity_still_denies_collection(self):
    with tempfile.TemporaryDirectory() as directory, \
         patch.object(vehicle, 'targets_for', return_value=['fixture.py']), \
         patch.object(vehicle, 'source_provenance', return_value={}), \
         patch.object(vehicle.lifecycle_shards, 'build_identity', side_effect=ValueError('missing declared artifact')), \
         patch.object(vehicle.test_runner, 'collect') as collect, redirect_stdout(io.StringIO()):
      self.assertEqual(vehicle.run('aol-lifecycle', directory, collect_only=True), 1)
      result = json.loads((Path(directory) / 'results.json').read_text())
      self.assertEqual(result['error_phase'], 'initialization')
      self.assertIn('missing declared artifact', result['collection_errors'][0])
      collect.assert_not_called()

  def test_lifecycle_records_native_input_digest_and_rejects_full_identity_change(self):
    identity = {'inputs': {'source': 'original'}, 'declared_outputs': {'exact.so': {'sha256': 'a' * 64, 'mode': 420}}}
    changed = {'inputs': {'source': 'original'}, 'declared_outputs': {'exact.so': {'sha256': 'b' * 64, 'mode': 420}}}
    for after, expected in ((identity, 0), (changed, 1)):
      with self.subTest(changed=after != identity), tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
        test = unittest.FunctionTestCase(lambda: None)
        stack.enter_context(patch.object(test, 'id', return_value='fixture.Case.test_pass'))
        for name, value in (('targets_for', ['fixture.py']), ('source_provenance', {'inputs': 'stable'}),
                            ('vehicle_inventory', {}), ('native_provenance', {}), ('build_safety_library', {}),
                            ('build_aol_protocol', {})):
          stack.enter_context(patch.object(vehicle, name, side_effect=(lambda *args, value=value: value.copy() if isinstance(value, dict) else value)))
        stack.enter_context(patch.object(vehicle.lifecycle_shards, 'build_identity', side_effect=[identity, after]))
        stack.enter_context(patch.object(vehicle.vehicle_pytest, 'split_targets', return_value=(['fixture.py'], [])))
        stack.enter_context(patch.object(vehicle.vehicle_pytest, 'collect', return_value=([], [])))
        stack.enter_context(patch.object(vehicle.vehicle_pytest, 'run', return_value=([], [])))
        stack.enter_context(patch.object(test_runner, 'collect', return_value=([test], [])))
        stack.enter_context(patch.object(test_runner, 'run_batch', return_value=[test_runner.make_record(test.id())]))
        stack.enter_context(redirect_stdout(io.StringIO()))
        self.assertEqual(vehicle.run('aol-lifecycle', directory), expected)
        coverage = json.loads((Path(directory) / 'coverage.json').read_text())
        self.assertEqual(coverage['source']['build_identity'], identity)
        self.assertEqual(coverage['source_after']['build_identity'], after)
        self.assertEqual(coverage['native']['safety']['build_inputs_sha256'], coverage['native']['aol_protocol']['build_inputs_sha256'])
        self.assertEqual(len(coverage['native']['safety']['build_inputs_sha256']), 64)
        self.assertEqual(bool(coverage['gate_errors']), expected == 1)
