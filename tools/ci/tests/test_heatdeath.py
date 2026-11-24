"""Portable qualification of the exported benchmark control plane and reporting."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import types
import unittest
import time
import uuid
from unittest.mock import patch

HERE = Path(__file__).resolve().parents[2] / 'heatdeath'
# Load as a distinct package so this suite does not leak generic helper module
# names or change sys.path for the rest of the full-repository test process.
package_spec = importlib.util.spec_from_file_location('heatdeath_bench_tests_package', HERE / '__init__.py', submodule_search_locations=[str(HERE)])
package = importlib.util.module_from_spec(package_spec)
sys.modules[package_spec.name] = package
package_spec.loader.exec_module(package)
from heatdeath_bench_tests_package import actor, heatdeath as cli, placement, report

install = actor.install
apply, load_plan, parked = placement.apply, placement.load_plan, placement.parked
phase, summarize = report.phase, report.summarize


class HeatdeathTests(unittest.TestCase):
  def test_cli_help_needs_no_device_or_native_import(self):
    for command in ([], ['prepare'], ['build'], ['run'], ['report']):
      result = subprocess.run([sys.executable, '-B', str(HERE / 'heatdeath.py'), *command, '--help'], capture_output=True, text=True, timeout=5)
      self.assertEqual(result.returncode, 0, result.stderr)

  def test_placement_rejects_malformed_or_foreign_actors(self):
    for value in ('[]', '{"manager":[1]}', '{"card":[]}', '{"card":[true]}', '{"card":[8]}', '{"card":[-1]}', '{"card":[1,1]}', '{"card":"6"}'):
      with self.subTest(value=value), self.assertRaises(ValueError):
        load_plan(value)
    self.assertEqual(load_plan('{"controlsd":[6],"peer":[0,1]}'), {'controlsd': [6], 'peer': [0, 1]})

  def test_affinity_override_after_both_production_placement_calls(self):
    seen = []
    module = types.SimpleNamespace(config_realtime_process=lambda *args: seen.append(args), set_core_affinity=lambda value: seen.append(value))
    with patch.object(actor, 'apply') as requested:
      install(module, 'controlsd')
      module.config_realtime_process(4, 53)
      module.set_core_affinity([6])
      self.assertEqual(seen, [(4, 53), [6]])
      self.assertEqual(requested.call_count, 2)
      requested.assert_called_with('controlsd')

  def test_apply_only_changes_own_process_and_verifies(self):
    with (
      patch.dict(os.environ, HEATDEATH_PLACEMENT='{"card":[4]}'),
      patch.object(placement.os, 'sched_setaffinity', create=True) as setter,
      patch.object(placement.os, 'sched_getaffinity', return_value={4}, create=True),
    ):
      self.assertEqual(apply('card'), [4])
      setter.assert_called_once_with(0, {4})
      setter.reset_mock()
      apply('peer')
      setter.assert_not_called()

  def test_offroad_and_engaged_are_both_required(self):
    for offroad, engaged, accepted in ((b'1', b'0', True), (b'0', b'0', False), (b'1', b'1', False)):
      with self.subTest(offroad=offroad, engaged=engaged), patch.object(Path, 'read_bytes', side_effect=[offroad, engaged]):
        if accepted:
          parked()
        else:
          with self.assertRaises(RuntimeError):
            parked()

  def test_phase_reports_actual_duration_and_gap(self):
    rows = [(i, 10, i, stamp, stamp + 2_000_000, 0, 1_000_000, 0) for i, stamp in enumerate((0, 10_000_000, 30_000_000))]
    result = phase(rows, 0, 1_000_000_000)
    self.assertEqual(result['observedCallsPerSecond'], 3)
    self.assertEqual(result['startGapMsMax'], 20)
    self.assertEqual(result['wallMsP99'], 2)
    self.assertEqual(result['threadCpuMsP95'], 1)

  def test_report_checks_hash_and_ring_overflow(self):
    with tempfile.TemporaryDirectory() as temp:
      out = Path(temp)
      (out / 'release.bin').write_bytes(struct.pack('<Q', 1))
      result = {
        'sourcePins': {'sourceHead': 'abc'},
        'sourceDurationNs': 1_000_000_000,
        'failure': None,
        'everAuthorityReady': False,
        'activeObserverSamples': 0,
        'placement': {},
        'syntheticReserve': None,
        'limits': [],
      }
      (out / 'result.json').write_text(json.dumps(result))
      raw = struct.pack('<' + 'q' * 20, 0, 1, 0, 1, 2, 0, 1, 0, *([0] * 12))
      (out / 'timing.bin').write_bytes(raw)
      entry = {'path': '/old/path/timing.bin', 'sha256': hashlib.sha256(raw).hexdigest(), 'rowsDropped': 0, 'rowsRetained': 1}
      for actor in ('card', 'controlsd', 'selfdrived'):
        (out / f'{actor}-timing.json').write_text(json.dumps({'timing': {'step': entry}, 'sampleColumns': {}}))
      report = summarize(out, 0)
      self.assertEqual(report['actors']['controlsd']['step']['calls'], 1)
      self.assertFalse(report['qualifiedActiveLatencyBench'])
      self.assertFalse(report['everAuthorityReady'])
      with self.assertRaises(ValueError):
        summarize(out, 1)
      (out / 'timing.bin').write_bytes(b'wrong')
      with self.assertRaisesRegex(ValueError, 'hash'):
        summarize(out, 0)
      (out / 'timing.bin').write_bytes(raw)
      entry['rowsDropped'] = 1
      (out / 'card-timing.json').write_text(json.dumps({'timing': {'step': entry}, 'sampleColumns': {}}))
      with self.assertRaisesRegex(ValueError, 'lost rows'):
        summarize(out, 0)

  def test_bundle_refuses_modified_helper_or_changed_source(self):
    with tempfile.TemporaryDirectory() as temp:
      bundle = Path(temp)
      (bundle / 'packet').mkdir()
      (bundle / 'owned.py').write_text('original')
      (bundle / 'packet/joined-source-pins.json').write_text('{"sources":{}}')
      info = {'repo': temp, 'sourceHead': 'abc', 'files': {'owned.py': cli.sha(bundle / 'owned.py')}}
      (bundle / 'bundle.json').write_text(json.dumps(info))
      with patch.object(cli, 'clean', return_value='abc'):
        cli.verify(bundle)
        (bundle / 'owned.py').write_text('changed')
        with self.assertRaisesRegex(RuntimeError, 'Bundle changed'):
          cli.verify(bundle)
      with patch.object(cli, 'clean', return_value='other'):
        with self.assertRaisesRegex(RuntimeError, 'revision'):
          cli.verify(bundle)

  def test_all_local_imports_resolve_without_private_audit_packet(self):
    import ast

    for source in HERE.glob('*.py'):
      tree = ast.parse(source.read_text())
      for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith(('driver_', 'peer_', 'owned_', 'bounded_', 'phase_')):
          self.assertTrue((HERE / (node.module + '.py')).is_file(), node.module)

  @unittest.skipUnless(sys.platform == 'linux' and os.geteuid() == 0, 'Root Linux process-identity check')
  def test_actual_owned_child_reaped_without_touching_foreign_child(self):
    token = uuid.uuid4().hex
    owned = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], env=dict(os.environ, SP_PRIVATE_IRQ_LEASE_TOKEN=token))
    foreign = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    try:
      self.assertIn(owned.pid, cli.token_processes(token))
      cli.reap_token(token)
      owned.wait(timeout=3)
      self.assertIsNone(foreign.poll())
      self.assertEqual(cli.token_processes(token), [])
    finally:
      for child in (owned, foreign):
        if child.poll() is None:
          child.kill()
        child.wait(timeout=3)

  @unittest.skipUnless(sys.platform == 'linux', 'Linux synthetic workload')
  def test_actual_bounded_synthetic_reserve(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      release, output = root / 'release.bin', root / 'reserve.json'
      release.write_bytes(struct.pack('<Q', time.monotonic_ns() + 100_000_000))
      subprocess.run(
        [
          sys.executable,
          '-B',
          str(HERE / 'reserve.py'),
          '--core',
          str(min(os.sched_getaffinity(0))),
          '--cpu-ms',
          '1',
          '--period-ms',
          '10',
          '--duration-ns',
          '40000000',
          '--release',
          str(release),
          '--output',
          str(output),
        ],
        check=True,
        timeout=5,
      )
      result = json.loads(output.read_text())
      self.assertTrue(result['completed'])
      self.assertEqual(len(result['rows']), 4)
      self.assertEqual(result['kind'], 'synthetic-cfs-cpu-reserve')
      self.assertTrue(all(row['cpuNs'] >= 0 for row in result['rows']))

  def test_output_refuses_source_or_existing_paths(self):
    with tempfile.TemporaryDirectory() as temp:
      repo = Path(temp)
      for output in (repo / 'new', repo):
        with self.assertRaises(ValueError):
          cli.external(output, repo)


if __name__ == '__main__':
  unittest.main()
