"""Guard-unit fixtures only; actual source/native binding and CPU remain root gates."""
import ctypes
from importlib.machinery import ModuleSpec, SourceFileLoader
import os
import resource
import signal
from types import ModuleType
from typing import cast

import pytest

from openpilot.starpilot.flm import native_primitives as native

PURPOSE = 'live-response-bootstrap-v1'
MODULE = 'openpilot.starpilot.flm.live_tracking_response_worker'


@pytest.fixture
def context(monkeypatch, tmp_path):
  # These controlled guard values are NOT an actual executed worker receipt.
  worker = tmp_path/'live_tracking_response_worker.py'
  worker.write_text('# guard fixture, never executable production authority\n')
  main = ModuleType('__main__')
  main.__dict__.update(__package__='openpilot.starpilot.flm', __file__=str(worker),
    __spec__=ModuleSpec(MODULE, SourceFileLoader(MODULE, str(worker)), origin=str(worker)),
    PURPOSE=PURPOSE, MAX_CPU=180, MAX_AS=2*1024**3, MAX_WALL_NS=240_000_000_000)
  monkeypatch.setattr(native, 'HERE', tmp_path)
  monkeypatch.setattr(native, '_enabled', False)
  monkeypatch.setattr(native.sys, 'platform', 'linux')
  monkeypatch.setattr(native.sys, 'orig_argv', [native.sys.executable, '-B', '-m', MODULE])
  monkeypatch.setitem(native.sys.modules, '__main__', main)
  monkeypatch.setattr(resource, 'getrlimit', lambda kind: (180, 180) if kind == resource.RLIMIT_CPU else (2*1024**3, 2*1024**3))
  monkeypatch.setattr(os, 'getpriority', lambda *args: 10)
  monkeypatch.setattr(os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(os, 'sched_getscheduler', lambda pid: 0, raising=False)
  monkeypatch.setattr(os, 'getppid', lambda: 1234)
  monkeypatch.setenv('STARPILOT_FLM_PARENT_PID', '1234')
  for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    monkeypatch.setenv(name, '1')

  class Library:
    def prctl(self, operation, pointer, *args):
      assert operation == 2
      pointer._obj.value = signal.SIGKILL
      return 0
  monkeypatch.setattr(ctypes, 'CDLL', lambda name: Library())
  return main


def test_context_enable_does_not_import_or_claim_a_native_binding(context, monkeypatch):
  def forbidden():
    raise AssertionError('enable must not load extension or manufacture receipt')
  monkeypatch.setattr(native, '_extension', forbidden)
  monkeypatch.setattr(native, 'native_identity', forbidden)
  assert native.enable_response_worker(PURPOSE) is True
  assert native._enabled is True


@pytest.mark.parametrize('purpose', ['live-response-bootstrap-diagnostic-v1', '', 180, True])
def test_wrong_explicit_purpose_never_enables(context, purpose):
  with pytest.raises(ValueError):
    native.enable_response_worker(purpose)
  assert native._enabled is False


@pytest.mark.parametrize('change', ['argv', 'package', 'spec', 'file', 'policy', 'cpu', 'memory',
  'priority', 'scheduler', 'threads', 'parent', 'death_signal', 'platform'])
def test_each_actual_execution_guard_is_required(context, monkeypatch, tmp_path, change):
  if change == 'argv':
    monkeypatch.setattr(native.sys, 'orig_argv', [native.sys.executable, '-B', '-m', 'openpilot.starpilot.flm.fit_worker'])
  elif change == 'package':
    context.__package__ = 'other'
  elif change == 'spec':
    context.__spec__ = ModuleSpec('other', SourceFileLoader(MODULE, context.__file__))
  elif change == 'file':
    context.__file__ = str(tmp_path/'other.py')
  elif change == 'policy':
    context.__dict__['MAX_CPU'] = 181
  elif change == 'cpu':
    monkeypatch.setattr(resource, 'getrlimit', lambda kind: (181, 181) if kind == resource.RLIMIT_CPU else (2*1024**3, 2*1024**3))
  elif change == 'memory':
    monkeypatch.setattr(resource, 'getrlimit', lambda kind: (180, 180) if kind == resource.RLIMIT_CPU else (-1, -1))
  elif change == 'priority':
    monkeypatch.setattr(os, 'getpriority', lambda *args: 9)
  elif change == 'scheduler':
    monkeypatch.setattr(os, 'sched_getscheduler', lambda pid: 1)
  elif change == 'threads':
    monkeypatch.setenv('OMP_NUM_THREADS', '2')
  elif change == 'parent':
    monkeypatch.setenv('STARPILOT_FLM_PARENT_PID', '2345')
  elif change == 'death_signal':
    class Library:
      def prctl(self, *args):
        return 0
    monkeypatch.setattr(ctypes, 'CDLL', lambda name: Library())
  else:
    monkeypatch.setattr(native.sys, 'platform', 'darwin')
  with pytest.raises(ValueError):
    native.enable_response_worker(PURPOSE)
  assert native._enabled is False


def test_response_allowance_does_not_widen_existing_legacy20_guard(context, monkeypatch):
  monkeypatch.setattr(resource, 'getrlimit', lambda kind: (21, 180) if kind == resource.RLIMIT_CPU else (2*1024**3, 2*1024**3))
  with pytest.raises(ValueError, match='worker context'):
    native.enable_fit_worker()
  assert native._enabled is False
  assert native.enable_response_worker(PURPOSE) is True


def test_unknown_purpose_and_main_metadata_cannot_invoke_custom_equality(context):
  calls = []
  class Trap:
    def __eq__(self, other):
      calls.append('eq')
      return True
  with pytest.raises(ValueError):
    native.enable_response_worker(cast(str, Trap()))
  context.__dict__['__package__'] = Trap()
  with pytest.raises(ValueError):
    native.enable_response_worker(PURPOSE)
  assert calls == []
