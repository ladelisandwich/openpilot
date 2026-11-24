"""Actual native Params document behavior; unit ack is NOT actuation proof."""

from dataclasses import replace
from pathlib import Path
import pytest
from openpilot.common.params import Params
from openpilot.starpilot.flm.live_trial_owner import LiveTrialApplication
from openpilot.starpilot.flm.live_trial_store import KEY, LiveTrialPersistence, decode, load_confirmed_history
from openpilot.starpilot.flm.tests.test_live_trial_owner import offer


def confirmed():
  from openpilot.starpilot.flm.live_tracking_response import validate_response_witness

  pending = offer()
  o = replace(pending, phase='confirmed', sequence=2, observed_ns=pending.observed_ns + 10_000_000)
  app = LiveTrialApplication(
    1,
    o.binding,
    o.trial_id,
    o.sequence,
    o.observed_ns,
    'applied',
    o.target,
    o.target_sha256,
    o.target_sha256,
    'unit_actual_application_fixture',
    o.target_sequence,
  )
  return o, app, validate_response_witness(o.response_witness, o.binding, o.observed_ns).certificate


def test_actual_native_params_confirmed_only_and_new_drive_is_historical(tmp_path):
  params = Params(str(tmp_path))
  params.put('AutomaticSteeringProfiles', b'legacy-replay-document-untouched', block=True)
  store = LiveTrialPersistence(params)
  o, app, c = confirmed()
  assert store.save_confirmed(o, app, c, now_ns=o.observed_ns, current_binding=lambda: o.binding, authorized=lambda: True)
  raw = params.get(KEY)
  assert raw is not None
  assert params.get('AutomaticSteeringProfiles') == b'legacy-replay-document-untouched'
  assert len(decode(raw)['confirmed']) == 1
  new = replace(o.binding, source_epoch='c' * 64, calibration_generation='new-drive-generation', physical_model_revision='d' * 64)
  prior = LiveTrialPersistence(Params(str(tmp_path))).load_prior(new)
  assert prior is not None and prior.historical_only is True and prior.target == o.target
  assert prior.binding == o.binding and prior.binding != new
  assert not hasattr(prior, 'expires_ns') and not hasattr(prior, 'permission')
  assert load_confirmed_history(params) == (prior,)
  before = params.get(KEY)
  assert store.save_confirmed(o, app, c, now_ns=o.observed_ns, current_binding=lambda: o.binding, authorized=lambda: True)
  assert params.get(KEY) == before


@pytest.mark.parametrize('kind', ['pending', 'neutral_ack', 'wrong_ack_target', 'stale_ack', 'denied_permission', 'changed_reset', 'unknown_vehicle'])
def test_no_pending_or_invalid_application_can_persist(tmp_path, kind):
  params = Params(str(tmp_path))
  store = LiveTrialPersistence(params)
  o, app, c = confirmed()
  current = o.binding
  allowed = True
  now = o.observed_ns
  if kind == 'pending':
    o = replace(o, phase='trial')
  elif kind == 'neutral_ack':
    app = replace(app, status='neutral', confirmed_target_sha256='')
  elif kind == 'wrong_ack_target':
    app = replace(app, applied=o.baseline)
  elif kind == 'stale_ack':
    now += 110_000_000
  elif kind == 'denied_permission':
    allowed = False
  elif kind == 'changed_reset':
    current = replace(current, reset_generation='newreset')
  elif kind == 'unknown_vehicle':
    o = replace(o, binding=replace(o.binding, persistent_source_epoch=''))
  assert not store.save_confirmed(o, app, c, now_ns=now, current_binding=lambda: current, authorized=lambda: allowed)
  assert params.get(KEY) is None


def test_pending_document_is_never_loaded_or_repaired(tmp_path):
  params = Params(str(tmp_path))
  raw = b'{"version":"live-confirmed-history-v1","confirmed":{},"pending":{"target":1}}'
  params.put(KEY, raw, block=True)
  assert params.get(KEY) == raw
  assert Path(params.get_param_path(KEY)).read_bytes() == raw
  assert LiveTrialPersistence(params).load_prior(offer().binding) is None
  assert params.get(KEY) == raw
  assert Path(params.get_param_path(KEY)).read_bytes() == raw
  with pytest.raises(ValueError):
    decode(raw)
