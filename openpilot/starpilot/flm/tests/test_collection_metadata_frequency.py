"""Actual Controls consumers; synthetictiming fixtures, not road-effect proof."""
import pytest
from openpilot.cereal import messaging
from openpilot.starpilot.flm.evidence_hooks import control_message
from openpilot.starpilot.flm.tests.test_live_trial_global_runtime import live_controls


def paced(fixture, proposal=None, **kwargs):
  # Real20Hz publication opportunities, skip everyfourth: actual15Hz delivery.
  next_tick = fixture.tick + 1
  publish = (next_tick // 5) % 4 != 0
  return fixture.frame(proposal, publish=publish, **kwargs)


def capture(fixture, now):
  message = messaging.new_message('starpilotLateralState')
  written = control_message(message, fixture.controls.LaC, fixture.last_command,
    control_ns=now, state_ns=int(fixture.controls.sm.logMonoTime['carState']),
    limited=fixture.controls.steer_limited_by_safety)
  return written is True and message.starpilotLateralState.learningEvidence.version == 1


def warm_slow_metadata(fixture):
  for _ in range(350):
    paced(fixture)
  assert not fixture.controls.sm.freq_ok['starpilotSteeringTracking']
  assert fixture.controls.sm.all_checks(list(fixture.runtime.health_services))
  assert fixture.controls.flm_native_witness is not None


def test_fresh_slow_metadata_keeps_actual_controller_collection_without_authorizing_output():
  with live_controls() as fixture:
    warm_slow_metadata(fixture)
    captured = 0
    for _ in range(50):
      now, _ = paced(fixture)
      assert not fixture.controls.sm.freq_ok['starpilotSteeringTracking']
      captured += capture(fixture, now)
      fixture.assert_neutral()
    assert captured == 50


def test_slow_metadata_frequency_still_refuses_live_correction():
  with live_controls() as fixture:
    warm_slow_metadata(fixture)
    proposal = fixture.initial_offer()
    for _ in range(50):
      now, _ = paced(fixture, proposal)
      assert not fixture.controls.sm.freq_ok['starpilotSteeringTracking']
      fixture.assert_neutral()
      assert capture(fixture, now)


@pytest.mark.parametrize('failure', ['native', 'driver', 'model', 'invalid_source', 'stale_source'])
def test_collection_still_rejects_current_authority_and_source_failure(failure):
  with live_controls() as fixture:
    warm_slow_metadata(fixture)
    kwargs = {'native': {'native_allowed': False}, 'driver': {'driver': True},
      'model': {'model_age': 1_000_000_000}, 'invalid_source': {'source_valid': False},
      'stale_source': {}}[failure]
    for _ in range(70 if failure == 'stale_source' else 20):
      if failure == 'stale_source':
        now, _ = fixture.frame(publish=False)
      else:
        now, _ = paced(fixture, **kwargs)
    fixture.assert_neutral()
    assert not capture(fixture, now)
