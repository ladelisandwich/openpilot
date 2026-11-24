"""Actual child death must preserve the last test and failed selected outcomes."""
from concurrent.futures import Future
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from tools import test_runner
from tools.ci import run_host_tests, worker_diagnostics
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[3]


def child_environment(directory, diagnostics):
  return dict(os.environ, PYTHONPATH=os.pathsep.join((str(ROOT), str(directory))),
              PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', PYTEST_ADDOPTS='', STARPILOT_TEST_DIAGNOSTICS=str(diagnostics))


class TestHostFailureDiagnostics(unittest.TestCase):
  @unittest.skipUnless(os.name == 'posix', 'actual POSIX child signal evidence')
  def test_aborted_pool_retains_fault_identity_exit_and_errors(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      diagnostics = root / 'diagnostics'
      (root / 'signal_fixture.py').write_text(
        'import os, resource, unittest\nresource.setrlimit(resource.RLIMIT_CORE, (0, 0))\n' +
        'class TestSignal(unittest.TestCase):\n' +
        ' def test_abort(self): os.abort()\n' +
        ' def test_other(self): self.assertTrue(True)\n')
      selected = ['signal_fixture.TestSignal.test_abort', 'signal_fixture.TestSignal.test_other']
      result = subprocess.run([sys.executable, str(ROOT / 'tools/test_runner.py'), '-j', '2',
                               '--json-output', str(root / 'outcomes.json'), *selected],
                              cwd=root, env=child_environment(root, diagnostics), capture_output=True, text=True, timeout=15)
      self.assertEqual(result.returncode, 1, result.stderr)
      outcomes = json.loads((root / 'outcomes.json').read_text())
      self.assertEqual(set(outcomes['selected_test_ids']), set(selected))
      records = {row['id']: row for row in outcomes['results']}
      self.assertEqual(records[selected[0]]['status'], 'error')
      self.assertEqual(len(records), 2)
      witness = json.loads((diagnostics / 'pool-failure.json').read_text())
      self.assertTrue(any(row['exitcode'] == -signal.SIGABRT for row in witness['processes']))
      self.assertTrue(any(row['active_test'] == selected[0] and row['batch'] == [selected[0]] for row in witness['workers']))
      self.assertIn('BrokenProcessPool', witness['detail'])
      self.assertLess(len(records[selected[0]]['detail']), 1024)
      fault = '\n'.join(path.read_text() for path in diagnostics.glob('*.fault.log'))
      self.assertIn('Aborted', fault)
      self.assertIn('signal_fixture.py', fault)

  @unittest.skipUnless(os.name == 'posix', 'actual POSIX child signal evidence')
  def test_initial_unknown_exit_refreshes_only_from_actual_child_exit(self):
    with tempfile.TemporaryDirectory() as directory:
      child = subprocess.Popen([sys.executable, '-c',
        'import resource,signal; resource.setrlimit(resource.RLIMIT_CORE,(0,0)); print("ready",flush=True); signal.pause()'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
      class Observation:
        pid = child.pid
        @property
        def exitcode(self):
          return child.poll()
      try:
        self.assertEqual(child.stdout.readline().strip(), 'ready')
        with patch.dict(os.environ, {'STARPILOT_TEST_DIAGNOSTICS': directory}):
          path = worker_diagnostics.pool_failure([Observation()], 'first exception')
          initial = json.loads(path.read_text())
          self.assertIsNone(initial['processes'][0]['exitcode'])
          child.send_signal(signal.SIGABRT)
          self.assertEqual(child.wait(timeout=5), -signal.SIGABRT)
          worker_diagnostics.pool_failure([Observation()], 'later exception')
          final = json.loads(path.read_text())
          self.assertEqual(final['processes'], [{'pid': child.pid, 'exitcode': -signal.SIGABRT}])
          self.assertEqual(final['detail'], initial['detail'])
          self.assertEqual(final['detail_sha256'], initial['detail_sha256'])
          self.assertEqual(final['workers'], initial['workers'])
      finally:
        if child.poll() is None:
          child.kill()
        child.wait(timeout=5)
        child.stdout.close()

  def test_final_exit_observation_follows_shutdown_and_preserves_failure(self):
    future = Future()
    future.set_exception(RuntimeError('original batch failure'))
    shutdown = []
    observations = []
    def observe(processes, detail):
      observations.append(bool(shutdown))
      if shutdown:
        raise ValueError('malformed diagnostics')
      return None
    with patch.object(test_runner, 'ProcessPoolExecutor') as executor, \
         patch.object(worker_diagnostics, 'pool_failure', side_effect=observe), \
         patch.object(test_runner.sys, 'stderr', io.StringIO()) as errors:
      executor.return_value.__enter__.return_value.submit.return_value = future
      executor.return_value.__exit__.side_effect = lambda *args: shutdown.append(True)
      records = list(test_runner.run_parallel([['first', 'second']], 2, 'error', False))[0]
    self.assertEqual(observations, [False, True])
    self.assertEqual([record['id'] for record in records], ['first', 'second'])
    self.assertTrue(all(record['status'] == 'error' and 'original batch failure' in record['detail'] for record in records))
    self.assertIn('ValueError', errors.getvalue())

  def test_still_live_worker_stays_unknown_without_waiting(self):
    with tempfile.TemporaryDirectory() as directory:
      child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
      class Observation:
        pid = child.pid
        @property
        def exitcode(self):
          return child.poll()
      try:
        with patch.dict(os.environ, {'STARPILOT_TEST_DIAGNOSTICS': directory}):
          path = worker_diagnostics.pool_failure([Observation()], 'original')
          started = time.monotonic()
          worker_diagnostics.pool_failure([Observation()], 'again')
          self.assertLess(time.monotonic() - started, 1.0)
          self.assertIsNone(child.poll())
          self.assertEqual(json.loads(path.read_text())['processes'], [{'pid': child.pid, 'exitcode': None}])
          path.write_text('{malformed')
          self.assertEqual(worker_diagnostics.pool_failure([Observation()], 'third'), path)
          self.assertEqual(path.read_text(), '{malformed')
        with patch.dict(os.environ, {}, clear=True):
          self.assertIsNone(worker_diagnostics.pool_failure([object()], 'no diagnostics'))
      finally:
        child.kill()
        child.wait(timeout=5)

  @unittest.skipUnless(os.name == 'posix', 'actual POSIX interruption evidence')
  def test_pytest_interruption_retains_collection_reports_and_active_node(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      diagnostics = root / 'diagnostics'
      fixture = root / 'test_interrupted_fixture.py'
      fixture.write_text('import time\ndef test_a_pass(): assert True\ndef test_b_fail(): assert False\n' +
                         'def test_c_pending(): time.sleep(60)\n')
      process = subprocess.Popen([sys.executable, '-m', 'pytest', '-q', '-p', 'tools.ci.pytest_progress', str(fixture)],
                                 cwd=root, env=child_environment(root, diagnostics), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      try:
        deadline = time.monotonic() + 10
        states = []
        while time.monotonic() < deadline:
          states = [json.loads(path.read_text()) for path in diagnostics.glob('worker-*.json')]
          if any(str(row.get('active_test', '')).endswith('test_c_pending') for row in states):
            break
          if process.poll() is not None:
            self.fail('pytest exited before the interruption fixture')
          time.sleep(0.02)
        self.assertTrue(any(str(row.get('active_test', '')).endswith('test_c_pending') for row in states))
        process.terminate()
        self.assertEqual(process.wait(timeout=5), -signal.SIGTERM)
        selected = json.loads((diagnostics / 'pytest-collection.json').read_text())
        self.assertEqual(len(selected), 3)
        reports = [json.loads(line) for path in diagnostics.glob('worker-*.jsonl') for line in path.read_text().splitlines()]
        call = {row['id']: row for row in reports if row['event'] == 'pytest_report' and row['phase'] == 'call'}
        self.assertEqual(call[selected[0]]['status'], 'passed')
        self.assertEqual(call[selected[1]]['status'], 'failed')
        self.assertIn('assert False', call[selected[1]]['failure'])
        self.assertNotIn(selected[2], call)
      finally:
        if process.poll() is None:
          process.kill()
        process.wait(timeout=5)

  def test_completed_pytest_keeps_junit_and_failed_exit_status(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      diagnostics = root / 'diagnostics'
      diagnostics.mkdir()
      fixture = root / 'test_outcome_fixture.py'
      fixture.write_text('def test_pass(): assert True\ndef test_fail(): assert False\n')
      result = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'tools.ci.pytest_progress',
                               '--junitxml=' + str(diagnostics / 'pytest.xml'), str(fixture)],
                              cwd=root, env=child_environment(root, diagnostics), capture_output=True, text=True, timeout=15)
      self.assertEqual(result.returncode, 1, result.stderr)
      report = ElementTree.parse(diagnostics / 'pytest.xml')
      self.assertEqual(len(report.findall('.//testcase')), 2)
      self.assertEqual(len(report.findall('.//failure')), 1)
      states = [json.loads(path.read_text()) for path in diagnostics.glob('worker-*.json')]
      self.assertEqual(states[0]['phase'], 'session_complete')
      self.assertEqual(states[0]['exitstatus'], 1)

  def test_host_wrapper_preserves_catalog_and_failure_with_diagnostics(self):
    with tempfile.TemporaryDirectory() as directory:
      diagnostics = Path(directory) / 'diagnostics'
      selected = ['first/test_owner.py', 'second/test_owner.py']
      seen = []
      def invoke(command, *, cwd, env, check):
        seen.append((command, env))
        self.assertTrue(Path(env['PARAMS_ROOT']).is_dir())
        self.assertEqual(env['PYTEST_DISABLE_PLUGIN_AUTOLOAD'], '1')
        return SimpleNamespace(returncode=7)
      with patch.dict(os.environ, {'STARPILOT_TEST_DIAGNOSTICS': str(diagnostics)}), \
           patch.object(run_host_tests, 'pytest_files', return_value=selected), \
           patch.object(run_host_tests.subprocess, 'run', side_effect=invoke):
        self.assertEqual(run_host_tests.main(['--pytest']), 7)
      command, env = seen[0]
      self.assertEqual(command[-2:], selected)
      self.assertIn('tools.ci.pytest_progress', command)
      self.assertIn('--junitxml=' + str(diagnostics.resolve() / 'pytest.xml'), command)
      self.assertFalse(Path(env['PARAMS_ROOT']).exists())
