"""Controls-owned bounded live trial, with no replay validation or persistence IO."""

from dataclasses import dataclass, replace
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, CanonicalSurfaceEffects, full_profile_sha256, original_anchor
from openpilot.starpilot.flm.effect_trace import step_full
from openpilot.starpilot.flm.live_trial_contract import LiveTrialBinding, validate_offer


@dataclass(frozen=True)
class LiveTrialApplication:
  version: int
  binding: LiveTrialBinding
  trial_id: str
  sequence: int
  control_ns: int
  status: str
  applied: tuple[tuple[str, float], ...]
  applied_sha256: str
  confirmed_target_sha256: str
  reason: str
  target_sequence: int = 1


class TrackingTrialOwner:
  def __init__(self, binding, *, domain="torque"):
    if type(binding) is not LiveTrialBinding:
      raise ValueError('typed live binding')
    self.binding = binding.validate()
    if domain not in ('torque', 'torque_pid'):
      raise ValueError('live selected torque domain')
    self.domain = domain
    self.confirmed_full = tuple((n, 0.0) for n in NAMES)
    self.applied_full = dict(self.confirmed_full)
    self.last_ns = 0
    self.offer = None
    self.source = None
    self.trace = None
    self.used = self.changed = False
    self.original_effect_anchor = ''
    self.application = None
    self.restore_offer = None
    self.interrupted = False

  def neutral(self, reason):
    self.source = None
    self.trace = None
    self.applied_full = dict.fromkeys(NAMES, 0.0)
    self.used = self.changed = False
    self.application = None
    self.interrupted = True
    self.last_ns = 0

  def prepare(self, offer, *, now_ns, permission, binding):
    self.source = None
    self.application = None
    self.used = self.changed = False
    if not permission or type(now_ns) is not int or now_ns <= 0 or binding != self.binding:
      self.neutral('current_live_permission_lost')
      return False
    validate_offer(offer)
    from openpilot.starpilot.flm.live_tracking_response import validate_response_witness

    validate_response_witness(offer.response_witness, binding, now_ns)
    if offer.binding != self.binding or not offer.observed_ns <= now_ns <= offer.expires_ns:
      raise ValueError('live binding/lease changed')
    old = self.offer
    if self.restore_offer is not None and offer.sequence <= self.restore_offer.sequence:
      raise ValueError('trial sequence must follow restored acknowledgement')
    if old is not None:
      if offer.sequence < old.sequence or offer.sequence == old.sequence and offer != old:
        raise ValueError('reordered/changed live offer')
      if offer.trial_id == old.trial_id:
        if self.interrupted and old.phase == 'trial' and offer.phase != 'rollback':
          raise ValueError('interrupted pending trial cannot resume')
        if (offer.baseline, offer.baseline_sha256, offer.started_ns, offer.group, offer.names, offer.target_sequence, offer.deadline_ns) != (
          old.baseline,
          old.baseline_sha256,
          old.started_ns,
          old.group,
          old.names,
          old.target_sequence,
          old.deadline_ns,
        ):
          raise ValueError('live trial immutable baseline changed')
        if offer.phase != 'rollback' and (offer.target != old.target or offer.target_sha256 != old.target_sha256):
          raise ValueError('live pending target changed')
        if old.phase == 'rollback' and offer.phase != 'rollback' or old.phase == 'confirmed' and offer.phase != 'confirmed':
          raise ValueError('terminal trial state changed')
      elif (old.phase == 'trial' and not self.interrupted) or offer.phase != 'trial' or offer.target_sequence <= old.target_sequence:
        raise ValueError('one live trial at a time')
    elif offer.phase != 'trial':
      raise ValueError('live phase without pending trial')
    if old is None or offer.trial_id != old.trial_id:
      if offer.baseline != self.confirmed_full or offer.baseline_sha256 != full_profile_sha256(binding.baseline_revision, self.confirmed_full):
        raise ValueError('live full30 confirmed baseline CAS')
    if old is not None and offer.sequence > old.sequence and offer.observed_ns <= old.observed_ns:
      raise ValueError('live publication clock reversal')
    if offer.phase == 'trial' and now_ns > offer.deadline_ns:
      raise ValueError('live trial original deadline expired')
    if old is not None and old.phase == 'trial' and offer.phase == 'confirmed' and offer.observed_ns > offer.deadline_ns:
      raise ValueError('confirmation after pending deadline')
    if self.last_ns and not 0 < now_ns - self.last_ns <= 50_000_000:
      self.neutral('live_control_clock')
      self.last_ns = now_ns
      return False
    if self.binding != binding:
      raise ValueError('live current binding changed')
    self.applied_full = step_full(self.applied_full, dict(offer.target), now_ns=now_ns, previous_ns=self.last_ns, qualified=True)
    self.last_ns = now_ns
    self.offer = offer
    self.restore_offer = None
    self.interrupted = False
    self.original_effect_anchor = original_anchor(binding.baseline_revision, offer.names)
    self.source = CanonicalSurfaceEffects(
      {n: self.applied_full[n] for n in offer.names}, binding.baseline_revision, accumulated=self.applied_full, allow_limited_profile=True
    )
    self.source.begin_frame(now_ns)
    actual = tuple((n, self.applied_full[n]) for n in NAMES)
    actual_hash = offer.target_sha256 if actual == offer.target else full_profile_sha256(binding.baseline_revision, actual)
    confirmed = offer.phase == 'confirmed' and actual == offer.target
    self.application = LiveTrialApplication(
      1,
      binding,
      offer.trial_id,
      offer.sequence,
      now_ns,
      'applied',
      actual,
      actual_hash,
      offer.target_sha256 if confirmed else '',
      'live_current_permission',
      offer.target_sequence,
    )
    return True

  def rejected_pending(self, *, control_ns):
    """Current consumer refusal only; never an applied or confirmed target."""
    offer = self.offer
    if (offer is None or offer.phase != 'trial' or not self.interrupted or self.source is not None
        or type(control_ns) is not int or control_ns <= 0):
      return None
    actual = tuple((name, self.applied_full[name]) for name in NAMES)
    return LiveTrialApplication(1, self.binding, offer.trial_id, offer.sequence, control_ns,
      'rejected', actual, full_profile_sha256(self.binding.baseline_revision, actual), '',
      'pending_trial_interrupted', offer.target_sequence)

  def prepare_restore(self, offer, prior, *, now_ns, permission, binding):
    from openpilot.starpilot.flm.live_trial_contract import LiveRestoreOffer
    from openpilot.starpilot.flm.live_trial_store import LiveConfirmedPrior, scope, stable_binding

    self.source = None
    self.application = None
    self.used = self.changed = False
    if not permission or binding != self.binding:
      self.neutral('restore_current_permission_lost')
      return False
    if type(offer) is not LiveRestoreOffer or type(prior) is not LiveConfirmedPrior or prior.historical_only is not True:
      raise ValueError('known confirmed startup history required')
    offer.validate()
    from openpilot.starpilot.flm.live_tracking_response import validate_response_witness

    validate_response_witness(offer.response_witness, binding, now_ns)
    if (
      self.offer is not None
      or offer.binding != binding
      or not offer.observed_ns <= now_ns <= offer.expires_ns
      or offer.history_scope != scope(prior.binding)
      or stable_binding(prior.binding) != stable_binding(binding)
      or offer.history_trial_id != prior.trial_id
      or offer.target != prior.target
      or offer.target_sha256 != prior.target_sha256
    ):
      raise ValueError('restore differs from startup confirmed history')
    old = self.restore_offer
    if old is not None and (
      offer.target != old.target or offer.history_scope != old.history_scope or offer.sequence < old.sequence or offer.sequence == old.sequence and offer != old
    ):
      raise ValueError('restore target/sequence changed')
    if self.last_ns and not 0 < now_ns - self.last_ns <= 50_000_000:
      self.neutral('restore_control_clock')
      self.last_ns = now_ns
      return False
    self.applied_full = step_full(self.applied_full, dict(offer.target), now_ns=now_ns, previous_ns=self.last_ns, qualified=True)
    self.last_ns = now_ns
    self.restore_offer = offer
    names = (NAMES[0],)
    self.original_effect_anchor = original_anchor(binding.baseline_revision, names)
    self.source = CanonicalSurfaceEffects(
      {n: self.applied_full[n] for n in names}, binding.baseline_revision, accumulated=self.applied_full, allow_limited_profile=True
    )
    self.source.begin_frame(now_ns)
    actual = tuple((n, self.applied_full[n]) for n in NAMES)
    self.application = LiveTrialApplication(
      1,
      binding,
      offer.history_trial_id,
      offer.sequence,
      now_ns,
      'applied',
      actual,
      offer.target_sha256 if actual == offer.target else full_profile_sha256(binding.baseline_revision, actual),
      '',
      'current_restore_permission',
    )
    return True

  def frame_complete(self, domain):
    if self.source is None or self.source.stage_calls != 1:
      return False
    return self.source.pid_error_calls == 1 if domain == 'torque_pid' else self.source.friction_calls == 1

  def acknowledge_frame(self, actual):
    actual = bool(actual and self.frame_complete(self.domain))
    application = self.application
    if application is None:
      return None
    if (
      actual
      and self.restore_offer is not None
      and application.applied == self.restore_offer.target
      and application.applied_sha256 == self.restore_offer.target_sha256
    ):
      self.confirmed_full = self.restore_offer.target
      return replace(application, status='restored', confirmed_target_sha256=self.restore_offer.target_sha256, reason='actual_confirmed_history_restored')
    if (
      actual
      and self.offer is not None
      and self.offer.phase == 'confirmed'
      and application.applied == self.offer.target
      and application.applied_sha256 == self.offer.target_sha256
    ):
      self.confirmed_full = self.offer.target
    return replace(
      application,
      status='applied' if actual else 'neutral',
      confirmed_target_sha256=application.confirmed_target_sha256 if actual else '',
      reason='actual_live_stage_used' if actual else 'live_stage_not_used',
    )

  @property
  def applied_coordinates(self):
    return () if self.offer is None else tuple(self.applied_full[n] for n in self.offer.names)

  def _record(self, before, after):
    self.used = True
    self.changed = self.changed or before != after
    return after

  def selected_stage(self, frame, *, original_anchor):
    before = frame.baseline_ff, frame.baseline_threshold
    return before if self.source is None else self._record(before, self.source.selected_stage(frame, original_anchor=original_anchor))

  @property
  def pid_error_calls(self):
    return 0 if self.source is None else self.source.pid_error_calls

  def selected_pid_error(self, frame, *, original_anchor):
    return frame.error_deg if self.source is None else self._record(frame.error_deg, self.source.selected_pid_error(frame, original_anchor=original_anchor))

  def selected_pid_stage(self, frame, *, original_anchor):
    return frame.baseline_ff if self.source is None else self._record(frame.baseline_ff, self.source.selected_pid_stage(frame, original_anchor=original_anchor))

  def selected_friction(self, baseline, *, original_anchor):
    return baseline if self.source is None else self._record(baseline, self.source.selected_friction(baseline, original_anchor=original_anchor))

  def selected_deadzone(self, *args, **kwargs):
    return args[1] if self.source is None else self._record(args[1], self.source.selected_deadzone(*args, **kwargs))

  def selected_output(self, *args, **kwargs):
    return args[4] if self.source is None else self._record(args[4], self.source.selected_output(*args, **kwargs))
