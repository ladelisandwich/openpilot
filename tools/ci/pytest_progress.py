"""Explicit pytest plugin retaining identities/outcomes when a host job is interrupted."""
import json
import os
from pathlib import Path

from tools.ci import worker_diagnostics


def pytest_sessionstart(session):
  worker_diagnostics.initialize()


def pytest_collection_finish(session):
  directory = os.environ.get('STARPILOT_TEST_DIAGNOSTICS')
  if directory:
    root = Path(directory).resolve()
    root.mkdir(parents=True, exist_ok=True)
    (root / 'pytest-collection.json').write_text(json.dumps([item.nodeid for item in session.items]) + '\n')


def pytest_runtest_logstart(nodeid, location):
  worker_diagnostics.node_started(nodeid)


def pytest_runtest_logreport(report):
  row = {'id': report.nodeid, 'phase': report.when, 'status': report.outcome, 'duration': report.duration}
  if report.failed:
    row['failure'] = str(report.longrepr)[:4096]
  worker_diagnostics.event('pytest_report', **row)
  if report.when == 'teardown':
    worker_diagnostics.update(active_test=None, phase='between_tests')


def pytest_sessionfinish(session, exitstatus):
  worker_diagnostics.update(active_test=None, phase='session_complete', exitstatus=int(exitstatus))
