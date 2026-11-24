"""Live owner unit contracts; unit model fixtures are NOT physical qualification."""

from dataclasses import replace
import pytest
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256
from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding, LiveTrialOffer, VERSION, MAX_STEP
from openpilot.starpilot.flm.live_trial_owner import TrackingTrialOwner


def binding():
  return LiveTrialBinding(
    '1' * 64, '2' * 64, 'actual-controller-registry-v2:' + '3' * 64, '4' * 64, '5' * 64, 'reset', 'generation', '6' * 64, '', '7' * 64, '8' * 64, False
  )


def response(b, now):
  from openpilot.starpilot.flm.live_tracking_response import ResponseCertificate, ResponseWitness, _revision, validate_certificate, prepare_response_witness

  c = ResponseCertificate(
    'live-tracking-response-v1', b, '0' * 64, 0, 1.0, 0.0, -1.0, 1.0, 0.001, ('a', 'b', 'c', 'd'), ('e', 'f', 'g', 'h'), '9' * 64, 'a' * 64, now - 1_000_000_000
  )
  revision = _revision(c)
  b = replace(b, physical_model_revision=revision)
  c = validate_certificate(replace(c, binding=b, model_revision=revision))
  witness = prepare_response_witness(ResponseWitness(c, b, now, now, now, now - 500_000_000, 8, 0.001, now + 100_000_000), b, now)
  return b, witness


def offer(now=2_000_000_000):
  b, w = response(binding(), now)
  baseline = tuple((n, 0.0) for n in NAMES)
  target = tuple((n, 0.005 if n == 'ff_gain_left' else 0.0) for n in NAMES)
  return LiveTrialOffer(
    VERSION,
    b,
    'b' * 64,
    1,
    'trial',
    'ff.left',
    ('ff_gain_left',),
    baseline,
    target,
    full_profile_sha256(b.baseline_revision, baseline),
    full_profile_sha256(b.baseline_revision, target),
    now,
    now,
    now + 100_000_000,
    w,
    1,
    now + 5_000_000_000,
  )


def test_pending_trial_is_distinct_from_replay_and_small_slewed():
  o = offer()
  owner = TrackingTrialOwner(o.binding)
  assert owner.prepare(o, now_ns=o.observed_ns, permission=True, binding=o.binding)
  assert 0 < owner.applied_full['ff_gain_left'] <= 0.00005
  assert owner.confirmed_full == o.baseline
  assert owner.application.confirmed_target_sha256 == ''
  assert owner.trace is None


@pytest.mark.parametrize('field,value', [('phase', 'restore'), ('sequence', 0), ('trial_id', 'bad'), ('version', 'replay-v1')])
def test_unknown_or_reordered_offer_denies(field, value):
  o = replace(offer(), **{field: value})
  owner = TrackingTrialOwner(o.binding)
  with pytest.raises(ValueError):
    owner.prepare(o, now_ns=o.observed_ns, permission=True, binding=o.binding)


def test_no_permission_immediately_neutral_no_rollback_command():
  o = offer()
  owner = TrackingTrialOwner(o.binding)
  owner.prepare(o, now_ns=o.observed_ns, permission=True, binding=o.binding)
  assert not owner.prepare(o, now_ns=o.observed_ns + 10_000_000, permission=False, binding=o.binding)
  assert owner.source is None and not any(owner.applied_full.values())
  assert owner.confirmed_full == o.baseline


def test_interrupted_pending_cannot_resume_and_new_trial_keeps_confirmed_cas():
  o = offer()
  owner = TrackingTrialOwner(o.binding)
  owner.prepare(o, now_ns=o.observed_ns, permission=True, binding=o.binding)
  owner.neutral('driver')
  with pytest.raises(ValueError):
    owner.prepare(replace(o, sequence=2), now_ns=o.observed_ns + 10_000_000, permission=True, binding=o.binding)
  new = replace(o, trial_id='c' * 64, sequence=2, target_sequence=2, observed_ns=o.observed_ns + 10_000_000)
  assert owner.prepare(new, now_ns=new.observed_ns, permission=True, binding=new.binding)
  assert owner.confirmed_full == o.baseline


def test_pending_never_becomes_confirmed_without_actual_stage_ack():
  o = offer()
  owner = TrackingTrialOwner(o.binding)
  owner.prepare(o, now_ns=o.observed_ns, permission=True, binding=o.binding)
  c = replace(o, phase='confirmed', sequence=2, observed_ns=o.observed_ns + 10_000_000)
  owner.prepare(c, now_ns=o.observed_ns + 10_000_000, permission=True, binding=o.binding)
  assert owner.confirmed_full == o.baseline
  assert owner.acknowledge_frame(False).confirmed_target_sha256 == ''
  assert owner.confirmed_full == o.baseline


def test_original_baseline25percent_bound_and_trial_delta_checked():
  o = offer()
  bad = tuple((n, 0.251 if n == 'ff_gain_left' else v) for n, v in o.target)
  with pytest.raises(ValueError):
    replace(o, target=bad).validate()
  bad = tuple((n, MAX_STEP + .001 if n == 'ff_gain_left' else v) for n, v in o.target)
  with pytest.raises(ValueError):
    replace(o, target=bad, target_sha256=full_profile_sha256(o.binding.baseline_revision, bad)).validate()


def test_missed_pose_publication_does_not_restamp_witness():
  o = offer()
  owner = TrackingTrialOwner(o.binding)
  with pytest.raises(ValueError):
    owner.prepare(o, now_ns=o.observed_ns + 110_000_000, permission=True, binding=o.binding)


def test_100hz_current_checks_with_genuine20hz_publications():
  base = offer()
  owner = TrackingTrialOwner(base.binding)
  for tick in range(50):
    now = base.observed_ns + tick * 10_000_000
    if tick % 5 == 0:
      # This helper constructs a NEW immutable raw source observation per20Hz tick.
      # Certificate revision must remain the same model; only witness times renew.
      from openpilot.starpilot.flm.live_tracking_response import ResponseWitness, prepare_response_witness, validate_response_witness

      raw = validate_response_witness(base.response_witness, base.binding, base.observed_ns)
      w = prepare_response_witness(
        ResponseWitness(raw.certificate, base.binding, now, now, now, base.observed_ns - 500_000_000, 8, 0.001, now + 100_000_000), base.binding, now
      )
      current = replace(base, sequence=tick // 5 + 1, observed_ns=now, expires_ns=now + 100_000_000, response_witness=w)
    assert owner.prepare(current, now_ns=now, permission=True, binding=base.binding)
    assert owner.source is not None


def test_interrupted_pending_cannot_be_confirmed_even_with_new_publication():
  o = offer()
  owner = TrackingTrialOwner(o.binding)
  owner.prepare(o, now_ns=o.observed_ns, permission=True, binding=o.binding)
  owner.neutral('native_or_driver_interruption')
  rejected = replace(o, phase='confirmed', sequence=2, observed_ns=o.observed_ns + 10_000_000)
  with pytest.raises(ValueError, match='interrupted pending'):
    owner.prepare(rejected, now_ns=rejected.observed_ns, permission=True, binding=o.binding)
  assert owner.confirmed_full == o.baseline
  assert owner.source is None


@pytest.mark.parametrize('name', ['baseline_sha256', 'target_sha256'])
def test_offer_hash_requires_exact_digest_string(name):
  from openpilot.starpilot.flm.live_trial_contract import validate_offer

  class EqualToAnything:
    def __eq__(self, other):
      return True

  with pytest.raises(ValueError, match='full30'):
    validate_offer(replace(offer(), **{name: EqualToAnything()}))


def test_owned_offer_content_rejects_class_accessor_mutation(monkeypatch):
  from openpilot.starpilot.flm.live_trial_contract import validate_offer, LiveTrialOffer

  o = offer()
  validate_offer(o)
  monkeypatch.setattr(LiveTrialOffer, 'phase', property(lambda self: 'confirmed'), raising=False)
  with pytest.raises(ValueError, match='plain live offer shape'):
    validate_offer(o)


def _full_cap_offer():
  original = offer()
  target = tuple((name, MAX_STEP if name == 'ff_gain_left' else value) for name, value in original.target)
  return replace(original, target=target, target_sha256=full_profile_sha256(original.binding.baseline_revision, target))


def test_larger_trial_cap_preserves_original_first_tick_slew_and_permission():
  from openpilot.starpilot.flm.live_trial_contract import SLEW_PER_SECOND
  from openpilot.starpilot.flm.effect_trace import SLEW_PER_SECOND as EFFECT_SLEW

  assert MAX_STEP == .05 and SLEW_PER_SECOND == EFFECT_SLEW == .005
  current = _full_cap_offer().validate()
  owner = TrackingTrialOwner(current.binding)
  assert owner.prepare(current, now_ns=current.observed_ns, permission=True, binding=current.binding)
  assert 0 < owner.applied_full['ff_gain_left'] <= .00005
  assert owner.applied_full['ff_gain_left'] < dict(current.target)['ff_gain_left']
  assert owner.confirmed_full == current.baseline
  assert not owner.prepare(current, now_ns=current.observed_ns + 10_000_000, permission=False, binding=current.binding)
  assert owner.source is None and not any(owner.applied_full.values())


def test_full_cap_all_thirty_targets_still_take_ten_seconds_at_original_slew():
  from openpilot.starpilot.flm.canonical_surface_adapter import coordinate_bounds
  from openpilot.starpilot.flm.effect_trace import step_full

  assert len(NAMES) == 30
  before = dict.fromkeys(NAMES, 0.)
  target = {name: min(coordinate_bounds(name)[1], MAX_STEP) for name in NAMES}
  assert all(value > 0 for value in target.values())
  previous_ns = 0
  for tick in range(1000):
    now_ns = 1_000_000_000 + tick * 10_000_000
    applied = step_full(before, target, now_ns=now_ns, previous_ns=previous_ns, qualified=True)
    for name in NAMES:
      assert 0 <= applied[name] - before[name] <= .005 * .01 + 1e-12
      assert applied[name] <= target[name] + 1e-12
    if tick == 998:
      assert applied['ff_gain_left'] < .05
    before, previous_ns = applied, now_ns
  assert all(before[name] == pytest.approx(target[name]) for name in NAMES)


def test_old_trial_cap_receiver_guard_rejects_larger_offer(monkeypatch):
  # Emulates the previous receiver's numeric cap, not a mixed-version road test.
  import openpilot.starpilot.flm.live_trial_contract as contract

  larger = _full_cap_offer()
  monkeypatch.setattr(contract, 'MAX_STEP', .005)
  with pytest.raises(ValueError, match='small selected live trial step'):
    larger.validate()
