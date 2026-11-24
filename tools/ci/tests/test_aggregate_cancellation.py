from pathlib import Path
import re
import unittest


class AggregateCancellationTests(unittest.TestCase):
  def test_complete_catalogs_remain_mandatory_unless_workflow_cancelled(self):
    workflow = Path(__file__).parents[3].joinpath('.github/workflows/tests.yaml').read_text()
    for name, needs, command in (
        ('custom_pytest_complete', 'custom_pytest', 'custom_pytest_shards aggregate'),
        ('aol_lifecycle_complete', 'aol_lifecycles', 'lifecycle_shards.py ci-results/aol-lifecycles')):
      with self.subTest(job=name):
        job = re.search(r'^  ' + name + r':\n(.*?)(?=^  [a-z_]+:|\Z)', workflow, re.M | re.S)
        self.assertIsNotNone(job)
        body = job.group(1)
        self.assertIn('    needs: ' + needs + '\n', body)
        guard = re.search(r'^    if: (.+)$', body, re.M).group(1)
        self.assertEqual(guard, '${{ always() && !cancelled() }}')
        self.assertIn(command, body)
        self.assertNotIn('continue-on-error', body)
        # Failure, timeout or an individually missing shard must still reach the
        # unmodified fail-closed aggregate whenever the workflow is not cancelled.
        for upstream in ('success', 'failure', 'skipped', 'cancelled'):
          for workflow_cancelled in (False, True):
            with self.subTest(upstream=upstream, workflow_cancelled=workflow_cancelled):
              expression = guard.removeprefix('${{ ').removesuffix(' }}')
              expression = expression.replace('always()', 'True').replace('cancelled()', str(workflow_cancelled))
              expression = expression.replace('&&', 'and').replace('!', 'not ')
              self.assertEqual(eval(expression, {'__builtins__': {}}), not workflow_cancelled)
