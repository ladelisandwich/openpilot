"""Exact stock Subaru profiles share the guarded physical Reverse join."""

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.stock_aol import qualified
from opendbc.car.subaru.values import CAR
from openpilot.cereal import log
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.reverse_intent import retain_reverse_intent
from openpilot.starpilot.tests import test_ford_reverse_intent as vectors


@pytest.fixture(autouse=True, params=(CAR.SUBARU_ASCENT_2023, CAR.SUBARU_FORESTER))
def stock_subaru_vectors(request, monkeypatch):
  def cp_state():
    candidate = request.param
    fp = gen_empty_fingerprint()
    with OpenpilotPrefix():
      cp = CarInterface.get_params(candidate, fp, [], False, False, False)
    cp.alternativeExperience = 32
    assert qualified(cp, marked_only=True)
    cs = structs.CarState(canValid=True, canTimeout=False, vEgo=15., gearShifter=structs.CarState.GearShifter.drive)
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


@pytest.mark.parametrize('fault', [fault for fault in vectors.NEGATIVES if fault != 'long-cp'])
def test_reverse_preserves_fault_source_identity_and_long_vetoes(fault):
  vectors.test_reverse_join_never_bypasses_current_fault_source_identity_or_long_veto(fault)


def test_first_unacknowledged_off_joins_fresh_native_off_without_renewing_intent():
  cp, cs, feedback = vectors.baseline()
  cs.gearShifter = structs.CarState.GearShifter.reverse
  now = vectors.NOW + 20_000_000
  sm, _ = vectors.sample(cp, vectors.NOW + 10_000_000, now, 2, active=False, requested=False, allowed=False)
  vectors.change(sm, 'aolAxisState', lambda axis: setattr(axis, 'nativeAcknowledged', False))
  before = dict(vars(feedback))
  assert vectors.retain_reverse_intent(feedback, sm, cp, cs, sm['aolAxisState'], now,
                                       latched=True, producer=vectors.PRODUCER)
  assert vars(feedback) == before
  assert not feedback.observe(sm, cp, cs, now, latched=True, intent_producer_session=vectors.PRODUCER)
  assert vars(feedback) == before


@pytest.mark.parametrize('fault', [fault for fault in vectors.NEGATIVES if fault not in ('long-cp', 'axis-unacknowledged')])
def test_off_withdrawal_join_keeps_original_fault_source_identity_and_long_vetoes(fault, monkeypatch):
  original_sample = vectors.sample

  def withdrawing_sample(cp, source, observed, sequence, **kwargs):
    if source > vectors.NOW:
      kwargs.update(active=False, requested=False, allowed=False)
    sm, native = original_sample(cp, source, observed, sequence, **kwargs)
    if source > vectors.NOW:
      vectors.change(sm, 'aolAxisState', lambda axis: setattr(axis, 'nativeAcknowledged', False))
    return sm, native

  monkeypatch.setattr(vectors, 'sample', withdrawing_sample)
  vectors.test_reverse_join_never_bypasses_current_fault_source_identity_or_long_veto(fault)
