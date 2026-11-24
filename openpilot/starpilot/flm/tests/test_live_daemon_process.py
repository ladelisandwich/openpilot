"""Actual stalled child exercises nonblocking IPC/cleanup, not calibration proof."""

import os
import subprocess
import sys
from openpilot.starpilot.flm import fit_process
from openpilot.starpilot.flm.live_tracking_bootstrap_process import LiveBootstrapProcess
from openpilot.starpilot.flm.live_tracking_bootstrap_worker import PURPOSE


def test_actual_backpressured_child_never_waits_in_poll_and_shutdown_reaps(monkeypatch):
  real_popen = subprocess.Popen
  commands = []

  def stalled_child(command, **kwargs):
    commands.append(command)
    return real_popen([sys.executable, '-B', '-c', 'import time; time.sleep(60)'], **kwargs)

  monkeypatch.setenv('STARPILOT_LIVE_BOOTSTRAP_PURPOSE', PURPOSE)
  monkeypatch.setattr(fit_process.subprocess, 'Popen', stalled_child)
  process = LiveBootstrapProcess()
  child = None
  wait_calls = []
  try:
    assert process.submit({'bounded_unit_payload': 'x' * 200_000}, token=('unit-context',), now_ns=1_000_000_000)
    child = process.child
    assert child is not None and child.stdin is not None and child.stdout is not None
    assert commands[0][-1] == 'openpilot.starpilot.flm.live_tracking_bootstrap_worker'
    assert not os.get_blocking(child.stdin.fileno()) and not os.get_blocking(child.stdout.fileno())
    real_wait = child.wait

    def wait_at_shutdown(*args, **kwargs):
      wait_calls.append((args, kwargs))
      return real_wait(*args, **kwargs)

    monkeypatch.setattr(child, 'wait', wait_at_shutdown)
    backpressured = False
    for tick in range(64):
      previous = process.position
      assert process.poll(token=('unit-context',), now_ns=1_010_000_000 + tick) is None
      assert not wait_calls and child.poll() is None
      if process.position == previous:
        backpressured = True
        break
    assert backpressured and process.position < len(process.pending)
  finally:
    assert process.close('unit_parent_shutdown')
  assert child is not None and child.poll() is not None and process.child is None
  assert len(wait_calls) == 1
