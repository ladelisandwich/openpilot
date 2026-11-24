"""One-shot worker lifecycle tests, not fabricated replay qualification."""
import gc
import io
import json
import sys
from collections.abc import Callable
from types import SimpleNamespace
from typing import NoReturn, cast

import pytest

from openpilot.starpilot.flm import fit_worker, native_primitives


@pytest.fixture
def worker_io(monkeypatch):
  prior = gc.isenabled()
  incoming, outgoing = io.BytesIO(b'{"test":true}'), io.BytesIO()
  monkeypatch.setattr(fit_worker, 'sys', SimpleNamespace(platform='darwin',
    stdin=SimpleNamespace(buffer=incoming), stdout=SimpleNamespace(buffer=outgoing)))
  # Both worker modules use the same explicitly simulated macOS context.
  # Linux native-worker resource guards remain real in their own tests.
  monkeypatch.setattr(native_primitives, 'sys', fit_worker.sys)
  monkeypatch.setattr(fit_worker.os, 'nice', lambda _: None)
  monkeypatch.setattr(fit_worker.os, 'sched_setscheduler', lambda *args: None, raising=False)
  monkeypatch.setattr(fit_worker.os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(fit_worker.os, 'sched_param', lambda priority: SimpleNamespace(sched_priority=priority), raising=False)
  limits = []
  monkeypatch.setattr(fit_worker.resource, 'getrlimit', lambda kind: (-1, -1))
  monkeypatch.setattr(fit_worker.resource, 'setrlimit', lambda *args: limits.append(args))
  yield incoming, outgoing, limits
  if prior:
    gc.enable()
  else:
    gc.disable()


def install_fit(monkeypatch, call):
  monkeypatch.setitem(sys.modules, 'openpilot.starpilot.flm.learner_factory', SimpleNamespace(fit_snapshot=call))


@pytest.mark.parametrize('enabled', (True, False))
def test_success_restores_exact_gc_state_and_original_bounds(monkeypatch, worker_io, enabled):
  incoming, outgoing, limits = worker_io
  (gc.enable if enabled else gc.disable)()
  calls = []

  def fit(snapshot):
    calls.append(snapshot)
    assert not gc.isenabled()
    # Even a helper toggling GC cannot alter the caller's final state.
    gc.enable()
    return {'status': 'collecting'}

  original_dumps = json.dumps

  def encode(*args, **kwargs):
    assert gc.isenabled() is enabled
    return original_dumps(*args, **kwargs)

  install_fit(monkeypatch, fit)
  monkeypatch.setattr(fit_worker.json, 'dumps', encode)
  assert fit_worker.main() == 0
  assert gc.isenabled() is enabled
  assert calls == [{'test': True}]
  assert outgoing.getvalue() == b'{"status":"collecting"}'
  assert limits == [(fit_worker.resource.RLIMIT_AS, (2 * 1024**3, -1)),
                    (fit_worker.resource.RLIMIT_CPU, (20, -1))]
  assert fit_worker.MAX_INPUT == 8 * 1024**2 and fit_worker.MAX_OUTPUT == 65536
  assert incoming.tell() == len(b'{"test":true}')


@pytest.mark.parametrize('enabled', (True, False))
@pytest.mark.parametrize('failure', (ValueError, MemoryError))
def test_fit_failure_restores_prior_gc_and_emits_nothing(monkeypatch, worker_io, enabled, failure):
  _, outgoing, _ = worker_io
  (gc.enable if enabled else gc.disable)()

  def fit(_):
    assert not gc.isenabled()
    raise failure('optional fit failure')

  install_fit(monkeypatch, fit)
  assert fit_worker.main() == 1
  assert gc.isenabled() is enabled
  assert outgoing.getvalue() == b''


def test_oversized_input_never_enters_fit_or_changes_gc(monkeypatch, worker_io):
  incoming, outgoing, _ = worker_io
  gc.enable()
  incoming.seek(0)
  incoming.write(b'x' * (fit_worker.MAX_INPUT + 1))
  incoming.seek(0)
  install_fit(monkeypatch, lambda _: cast(Callable[[str], NoReturn], pytest.fail)('oversized input reached fitter'))
  assert fit_worker.main() == 1
  assert gc.isenabled() and outgoing.getvalue() == b''


def test_output_bound_remains_after_gc_restoration(monkeypatch, worker_io):
  _, outgoing, _ = worker_io
  gc.enable()

  def fit(_):
    assert not gc.isenabled()
    return {'data': 'x' * fit_worker.MAX_OUTPUT}

  install_fit(monkeypatch, fit)
  assert fit_worker.main() == 1
  assert gc.isenabled() and outgoing.getvalue() == b''
