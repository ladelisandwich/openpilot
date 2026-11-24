"""Qualified stock Mazda rows use the original guarded Reverse proof."""
import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.stock_aol import qualified
from opendbc.car.mazda.values import CAR
from openpilot.cereal import log
from openpilot.starpilot.aol.reverse_intent import retain_reverse_intent
from openpilot.starpilot.tests import test_ford_reverse_intent as vectors


@pytest.fixture(autouse=True, params=(CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021))
def stock_mazda_vectors(request, monkeypatch):
  def cp_state():
    cp = CarInterface.get_params(request.param, gen_empty_fingerprint(), [], False, False, False)
    cp.alternativeExperience = 32
    assert qualified(cp, marked_only=True)
    cs = structs.CarState(canValid=True, canTimeout=False, vEgo=20., gearShifter=structs.CarState.GearShifter.drive)
    cs.cruiseState.available = True
    return cp, cs

  def retain(feedback, sm, cp, cs, axis, now_ns, *, latched, producer):
    return retain_reverse_intent(feedback, sm, cp, cs, axis, now_ns, latched=latched, producer=producer,
                                 qualified_cp=qualified(cp, marked_only=True),
                                 informational_events=(log.OnroadEvent.EventName.userBookmarkNotPaired,), allow_withdrawal_join=True)

  original_sample = vectors.sample

  def initial_epoch_sample(*args, **kwargs):
    sm, native = original_sample(*args, **kwargs)
    vectors.change(sm, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch', 0))
    return sm, native

  monkeypatch.setattr(vectors, 'sample', initial_epoch_sample)
  monkeypatch.setattr(vectors, 'cp_state', cp_state)
  monkeypatch.setattr(vectors, 'retain_reverse_intent', retain)


def test_reverse_retains_only_existing_intent_without_renewing_leases():
  vectors.test_first_reverse_join_retains_only_intent_without_renewing_prior_leases()


def test_off_proof_and_drive_need_fresh_native_admission():
  vectors.test_actual_off_proof_and_drive_recovery_use_unchanged_native_admission()


@pytest.mark.parametrize('fault', vectors.NEGATIVES)
def test_reverse_preserves_fault_source_identity_and_long_vetoes(fault):
  vectors.test_reverse_join_never_bypasses_current_fault_source_identity_or_long_veto(fault)
