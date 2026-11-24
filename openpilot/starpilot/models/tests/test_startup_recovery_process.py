"""An explicit failed GPU startup gets one new, isolated Small process."""
from unittest.mock import Mock, patch

import pytest

from openpilot.starpilot.models.recovery import STARTUP_FALLBACK_EXIT
from openpilot.starpilot.models.recovery_process import ModeldProcess
from openpilot.starpilot.models.tests.test_recovery import BASE
from openpilot.starpilot.models.tests.test_recovery_process import Feed


@pytest.mark.parametrize('alive,exitcode,fresh,started,same_drive,shutting,small_only', [
  (False, STARTUP_FALLBACK_EXIT, True, True, True, False, False),
  (True, STARTUP_FALLBACK_EXIT, True, True, True, False, False),
  (False, 1, True, True, True, False, False),
  (False, -9, True, True, True, False, False),
  (False, None, True, True, True, False, False),
  (False, STARTUP_FALLBACK_EXIT, False, True, True, False, False),
  (False, STARTUP_FALLBACK_EXIT, True, False, True, False, False),
  (False, STARTUP_FALLBACK_EXIT, True, True, False, False, False),
  (False, STARTUP_FALLBACK_EXIT, True, True, True, True, False),
  (False, STARTUP_FALLBACK_EXIT, True, True, True, False, True),
])
def test_startup_restart_requires_owned_exit_and_current_drive(alive, exitcode, fresh, started, same_drive, shutting, small_only):
  owner = ModeldProcess('modeld', 'module', lambda *args: True)
  child = Mock(pid=123, exitcode=exitcode)
  child.is_alive.return_value = alive
  owner.proc = child
  owner.launch_drive_id = BASE if same_drive else BASE - 1
  owner.recovery.drive_id = BASE
  owner.recovery.small_only = small_only
  owner.shutting_down = shutting
  feed = Feed(2.)
  if not fresh:
    feed.logMonoTime['deviceState'] = BASE
  with patch('openpilot.starpilot.models.recovery_process.time.monotonic_ns', return_value=BASE + 2_000_000_000), \
       patch('openpilot.starpilot.models.recovery_process.process_start_ticks', return_value=456), \
       patch('openpilot.starpilot.models.recovery_process.read_receipt', return_value=None):
    owner.observe(feed, started)
  qualifies = not alive and exitcode == STARTUP_FALLBACK_EXIT and fresh and started and same_drive and not shutting and not small_only
  if qualifies:
    child.join.assert_called_once_with(timeout=0)
    assert owner.proc is None
    assert owner.recovery.small_only
    with patch('openpilot.starpilot.models.recovery_process.Process') as spawn:
      owner.start()
      assert spawn.call_args.kwargs['args'] == ('module', 'modeld', True)
      spawn.return_value.start.assert_called_once_with()
      owner.start()
      spawn.assert_called_once()
  else:
    child.join.assert_not_called()
    assert owner.proc is child


def test_launch_records_current_drive_and_never_retries_failed_small():
  owner = ModeldProcess('modeld', 'module', lambda *args: True)
  owner.context_ready = True
  owner.recovery.drive_id = BASE
  owner.recovery.small_only = True
  owner.recovery.phase = 'small'
  with patch('openpilot.starpilot.models.recovery_process.Process') as spawn:
    owner.start()
    assert owner.launch_drive_id == BASE
    child = spawn.return_value
    child.pid = 123
    child.is_alive.return_value = False
    child.exitcode = STARTUP_FALLBACK_EXIT
    with patch('openpilot.starpilot.models.recovery_process.time.monotonic_ns', return_value=BASE + 2_000_000_000):
      owner.observe(Feed(2.), True)
    owner.start()
    spawn.assert_called_once()
    child.join.assert_not_called()


def _failed_gpu_startup():
  import threading
  threading.Thread(target=threading.Event().wait, daemon=True).start()
  raise SystemExit(STARTUP_FALLBACK_EXIT)


def test_real_failed_startup_exits_with_live_daemon_before_small_can_start():
  from multiprocessing import Process
  child = Process(target=_failed_gpu_startup)
  child.start()
  try:
    child.join(timeout=3)
    assert not child.is_alive()
    assert child.exitcode == STARTUP_FALLBACK_EXIT
    owner = ModeldProcess('modeld', 'module', lambda *args: True)
    owner.proc = child
    owner.launch_drive_id = BASE
    owner.recovery.drive_id = BASE
    with patch('openpilot.starpilot.models.recovery_process.time.monotonic_ns', return_value=BASE + 2_000_000_000), \
         patch('openpilot.starpilot.models.recovery_process.Process') as spawn:
      owner.observe(Feed(2.), True)
      owner.start()
      assert spawn.call_args.kwargs['args'] == ('module', 'modeld', True)
      spawn.assert_called_once()
  finally:
    if child.is_alive():
      child.kill()
      child.join(timeout=3)
    child.close()


@pytest.mark.parametrize("cleanup", ["daemon", "non_daemon", "atexit"])
def test_actual_failed_startup_exits_without_waiting_for_cleanup(cleanup):
  import ast
  from pathlib import Path
  import subprocess
  import sys

  source = Path(__file__).resolve().parents[3] / "selfdrive/modeld/modeld.py"
  tree = ast.parse(source.read_text())
  failure = next(node for node in ast.walk(tree) if isinstance(node, ast.If) and
                 ast.unparse(node.test) == "model is None" and
                 any(isinstance(child, ast.Constant) and
                     child.value == "Chestnut startup failed; requesting isolated Small restart"
                     for child in ast.walk(node)))
  program = """
import atexit, os, threading
from types import SimpleNamespace
blocker = threading.Event()
cleanup = CLEANUP
if cleanup == 'atexit':
  atexit.register(blocker.wait)
else:
  threading.Thread(target=blocker.wait, daemon=cleanup == 'daemon').start()
cloudlog = SimpleNamespace(error=lambda *args: None)
STARTUP_FALLBACK_EXIT = EXIT_CODE
model = None
""".replace("CLEANUP", repr(cleanup)).replace("EXIT_CODE", str(STARTUP_FALLBACK_EXIT))
  program += ast.unparse(failure)
  child = subprocess.Popen([sys.executable, "-I", "-B", "-c", program])
  try:
    assert child.wait(timeout=3) == STARTUP_FALLBACK_EXIT
  finally:
    if child.poll() is None:
      child.kill()
    child.wait(timeout=3)
