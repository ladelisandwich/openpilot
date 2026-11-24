"""Native modal input consumes a complete batch without invoking vehicle actions."""

from unittest.mock import Mock

from openpilot.starpilot.controllers.input import Press
from openpilot.starpilot.controllers.tests.test_owner import TestControllerOwner as OwnerFixture
from openpilot.starpilot.controllers.tests.test_runtime import TestControllerRuntime as RuntimeFixture


def test_modal_resolves_binding_without_action_or_learning():
  fixture = OwnerFixture()
  fixture.setUp()
  try:
    fixture.configure()
    fixture.learn(3)
    fixture.now += 1_000_000
    capture = Mock()
    press = Press(fixture.device['id'], 42, fixture.now)
    fixture.owner.feed(press, capture=capture)
    capture.assert_called_once_with('safe.action', fixture.now)
    assert fixture.calls == 0 and not fixture.favorite_calls
    fixture.owner.feed(press, capture=capture)
    assert capture.call_count == 1
    fixture.owner.feed(Press('unknown', 42, fixture.now), capture=capture)
    fixture.owner.feed(Press(fixture.device['id'], 42, fixture.now - 260_000_000), capture=capture)
    assert capture.call_count == 1
    fixture.now += 1_000_000
    fixture.owner.feed(Press(fixture.device['id'], 42, fixture.now))
    assert fixture.calls == 1
    fixture.owner.action({'operation': 'learn', 'revision': fixture.owner.snapshot()['revision'], 'slot': 4})
    fixture.now += 1_000_000
    before = fixture.path.read_bytes()
    fixture.owner.feed(Press(fixture.device['id'], 55, fixture.now), capture=capture)
    assert fixture.path.read_bytes() == before
    capture.assert_called_with(None, fixture.now)
  finally:
    fixture.doCleanups()


def test_batch_capture_is_stable_when_first_press_closes_modal():
  fixture = RuntimeFixture()
  runtime, _, reader, owner, _ = fixture.runtime()
  try:
    capture = Mock()
    current: list[Mock | None] = [capture]
    runtime.input_capture = lambda: current[0]
    reader.poll.return_value = [Press('device', 1, 10), Press('device', 2, 20)]
    def consume(_press, *, capture):
      current[0] = None
    owner.feed.side_effect = consume
    runtime.poll()
    assert owner.feed.call_count == 2
    assert all(call.kwargs['capture'] is capture for call in owner.feed.call_args_list)
  finally:
    fixture.doCleanups()
