"""Native Params persistence; explicit unit basis is not physical identification."""

from dataclasses import replace
from pathlib import Path
import json
import pytest
from openpilot.common.params import Params
from openpilot.starpilot.flm.live_trial_store import LiveTrialPersistence, KEY, decode, load_confirmed_history
from openpilot.starpilot.flm.live_tracking_requalification import StableResponseBasis
from openpilot.starpilot.flm.tests.test_live_trial_store import confirmed


def unit_basis(binding):
  return StableResponseBasis(
    binding.identity,
    binding.cp_sha256,
    binding.controller_revision,
    binding.baseline_revision,
    (0.0, 0.0, 0.0),
    (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
    ('unit-hardware-not-device-proof',),
    binding.native_required,
  )


def test_confirmed_response_basis_roundtrips_without_current_permission(tmp_path):
  params = Params(str(tmp_path))
  store = LiveTrialPersistence(params)
  offer, app, cert = confirmed()
  basis = unit_basis(offer.binding)
  assert store.save_confirmed(offer, app, cert, now_ns=offer.observed_ns, current_binding=lambda: offer.binding, authorized=lambda: True, response_basis=basis)
  raw = params.get(KEY)
  row = next(iter(decode(raw)['confirmed'].values()))
  assert row['retained_response']['current_permission'] is False
  assert row['retained_response']['pending_trial'] is None
  prior = load_confirmed_history(params)[0]
  assert prior.historical_only is True and prior.response_basis == basis
  assert prior.certificate == cert and prior.binding == offer.binding
  assert store.save_confirmed(offer, app, cert, now_ns=offer.observed_ns, current_binding=lambda: offer.binding, authorized=lambda: True, response_basis=basis)
  assert params.get(KEY) == raw


def test_legacy_confirmed_document_without_basis_keeps_target_for_fresh_bootstrap(tmp_path):
  params = Params(str(tmp_path))
  store = LiveTrialPersistence(params)
  offer, app, cert = confirmed()
  assert store.save_confirmed(offer, app, cert, now_ns=offer.observed_ns, current_binding=lambda: offer.binding, authorized=lambda: True)
  doc = decode(params.get(KEY))
  for row in doc['confirmed'].values():
    del row['retained_response']
  raw = json.dumps(doc).encode()
  params.put(KEY, raw, block=True)
  assert params.get(KEY) == raw
  assert Path(params.get_param_path(KEY)).read_bytes() == raw
  prior = load_confirmed_history(params)[0]
  assert prior.target == offer.target and prior.response_basis is None and prior.historical_only is True
  assert params.get(KEY) == raw
  assert Path(params.get_param_path(KEY)).read_bytes() == raw


@pytest.mark.parametrize('kind', ['pending', 'denied', 'wrong_basis'])
def test_unconfirmed_or_wrong_context_basis_cannot_be_saved(tmp_path, kind):
  params = Params(str(tmp_path))
  offer, app, cert = confirmed()
  basis = unit_basis(offer.binding)
  if kind == 'pending':
    offer = replace(offer, phase='trial')
  if kind == 'wrong_basis':
    basis = replace(basis, cp_sha256='f' * 64)
  assert not LiveTrialPersistence(params).save_confirmed(
    offer, app, cert, now_ns=offer.observed_ns, current_binding=lambda: offer.binding, authorized=lambda: kind != 'denied', response_basis=basis
  )
  assert params.get(KEY) is None
