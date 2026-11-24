"""Actual parent history loading never rewrites its observed live target."""

from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.flm.event_collection import EventCollectionAdapter
from openpilot.starpilot.flm.full_surface_store import FullSurfacePrior
from openpilot.starpilot.flm.tests.test_full_surface_profile import base


def parent(envelope=None):
  owner = EventCollectionAdapter.__new__(EventCollectionAdapter)
  owner._source = SimpleNamespace(reset='0')
  owner.envelope = base() if envelope is None else envelope
  owner.restore_envelope = None
  owner.optimizer_prior = None
  owner.accepted_restore_envelope = None
  return owner


def saved_prior(current):
  accepted = current.with_proposal({'ff_gain_left': 0.005})
  return FullSurfacePrior(replace(accepted, prior_coordinates=accepted.coordinates, prior_full_profile_sha256=accepted.full_profile_sha256).validate())


def test_actual_parent_keeps_live_neutral_and_routes_history_to_explicit_transaction():
  owner = parent()
  neutral = owner.envelope
  prior = saved_prior(neutral)
  owner.install_full_prior(prior)
  assert owner.envelope is neutral
  assert owner.envelope.coordinates != prior.envelope.coordinates
  assert owner.optimizer_prior is prior and owner.restore_envelope is not None
  assert owner.restore_envelope.operation == 'restore'
  assert owner.restore_envelope.prior_coordinates == neutral.coordinates
  assert owner.restore_envelope.coordinates == prior.envelope.coordinates


def test_actual_parent_no_history_keeps_actual_target_and_no_restore():
  owner = parent()
  neutral = owner.envelope
  owner.install_full_prior(None)
  assert owner.envelope is neutral
  assert owner.optimizer_prior is None and owner.restore_envelope is None


def test_actual_parent_does_not_restore_over_already_accepted_live_map():
  accepted = base().with_proposal({'ff_gain_left': 0.005})
  current = replace(accepted, prior_coordinates=accepted.coordinates, prior_full_profile_sha256=accepted.full_profile_sha256).validate()
  owner = parent(current)
  prior = FullSurfacePrior(current)
  owner.install_full_prior(prior)
  assert owner.envelope is current
  assert owner.restore_envelope is None


@pytest.mark.parametrize('change', [{'source_epoch': '7' * 64, 'context_key': '7' * 64}, {'persistent_source_epoch': '7' * 64}, {'reset_generation': '1'}])
def test_actual_parent_rejects_foreign_history_even_before_child_submission(change):
  owner = parent()
  prior = saved_prior(owner.envelope)
  foreign = FullSurfacePrior(replace(prior.envelope, **change))
  with pytest.raises(ValueError):
    owner.install_full_prior(foreign)
  assert owner.envelope.coordinates == base().coordinates
  assert owner.restore_envelope is None
