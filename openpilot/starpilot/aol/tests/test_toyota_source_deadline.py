"""Toyota source cutoffs retain only the original, still-live command proof."""
from contextlib import ExitStack
from unittest import mock
import pytest
from openpilot.cereal import log
from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import callers
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE
from openpilot.starpilot.aol.tests.test_ordinary_source_availability import settings


@pytest.mark.parametrize('mode', ('combined', 'combined_unarmed'))
@pytest.mark.parametrize('entry_expired', (False, True))
@pytest.mark.parametrize('condition', ('healthy', 'deadline', 'native_denied', 'cancel'))
def test_toyota_source_cutoff_keeps_original_lease_only(mode, entry_expired, condition):
  with ExitStack() as cleanup:
    sd, controls, drive, _, _ = callers(cleanup, 'toyota', False)
    # Recorded shape: source -> publication 4.310ms, next selection at
    # source+31.443ms, then fresh successor at source+30.051ms.
    old, _, command = drive(2, state_stamp=BASE, offset_ns=4_309_575, **settings(mode, BASE, 3))
    assert old.lateralActive and command.latActive
    previous = sd.pm.messages['aolAxisState'].as_builder().to_bytes()
    now = [BASE + (31_443_025 if entry_expired else 29_999_000)]
    original_drain = sd.drain_axis_intent

    def drain(companion, **kwargs):
      result = original_drain(companion, **kwargs)
      now[0] = old.validUntilMonoTime + 1 if condition == 'deadline' else BASE + 31_443_025
      return result

    options = settings(mode, BASE, 3)
    if condition == 'native_denied':
      options.update(allowed=False, long_allowed=False)
    if condition == 'cancel':
      options['user_disable'] = True
    with mock.patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', side_effect=lambda: now[0]), \
         mock.patch.object(sd, 'drain_axis_intent', side_effect=drain):
      axis, _, command = drive(5, timeout=True, offset_ns=now[0] - (BASE + 30_000_000), **options)
    if condition != 'healthy':
      assert not sd.aol_axis_publication_deferred
      assert not command.latActive and not command.longActive
      return
    assert sd.aol_axis_publication_deferred
    assert sd.pm.messages['aolAxisState'].as_builder().to_bytes() == previous
    assert axis.sourceCarStateMonoTime == BASE and axis.validUntilMonoTime == old.validUntilMonoTime
    assert not sd.events.contains(log.OnroadEvent.EventName.commIssue)
    assert command.latActive and command.longActive
    with mock.patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=old.validUntilMonoTime + 1):
      expired, _ = controls.state_control()
    assert not expired.latActive and not expired.longActive
    fresh = BASE + 30_050_889
    axis, _, command = drive(6, state_stamp=fresh, offset_ns=-2_000_000, **settings(mode, fresh, 4))
    assert axis.sourceCarStateMonoTime == fresh and axis.lateralActive and command.latActive
    assert axis.longitudinalActive and command.longActive
    assert not sd.events.contains(log.OnroadEvent.EventName.commIssue)
