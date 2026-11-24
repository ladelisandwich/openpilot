from openpilot.starpilot.flm.normalized_trace_archive import guarded_archive
from openpilot.starpilot.flm.normalized_trace_archive import trace_for,archive_key
"""Child-only canonical effect trials through complete selected controller/CAN history."""

from typing import cast, NamedTuple
from collections.abc import Mapping
import math

from openpilot.starpilot.flm.clipping_quality import NATIVE_SHADOW_IDENTIFICATION

from openpilot.starpilot.flm.adaptive_profile import ReplayValidation
from openpilot.starpilot.flm.controller_replay import (
  MAX_FRAME_GAP_NS, _run, capture_snapshot, final_command, restore_snapshot, same_final_command, same_float,
)
from openpilot.starpilot.flm.runtime_adapter_registry import canonical_physical_context, physical_model_revision
from openpilot.starpilot.flm.torque_eligibility import exact_unclipped_pair

POLICY = 'event_effect_coordinate_v2'
RESTORE_POLICY = 'event_effect_restore_v1'


class _LazyControllerRequests(Mapping[int, float]):
  """Only original native replay may pull this child-local controller iterator.

  Values are cached by their exact control timestamp. Native apply-order stays
  owned by EmissionHistory.rollout, including duplicate/older requested stamps.
  No result is successful until finish() exhausts all original controller guards.
  """
  _values: dict[int, float]

  def __init__(self, values: dict[int, float], frames):
    self._values, self.frames = values, frames
    self.finished = False

  def __contains__(self, stamp):
    while stamp not in self._values and not self.finished:
      try:
        next(self.frames)
      except StopIteration:
        self.finished = True
    return stamp in self._values

  def __getitem__(self, stamp):
    if stamp not in self:
      raise KeyError(stamp)
    return self._values[stamp]

  def __iter__(self):
    self.finish()
    return iter(self._values)

  def __len__(self):
    self.finish()
    return len(self._values)

  def close(self):
    self.frames.close()

  def finish(self):
    if not self.finished:
      for _ in self.frames:
        pass
      self.finished = True


class _EquivalentBaselineView(NamedTuple):
  owner: object
  cache: tuple
  entry: tuple
  original_key: object
  selected_key: object
  proof_fingerprint: bytes


class EventControllerReplay:
  """Original source proof plus explicitly versioned candidate controller replay.

  Legacy trials retain frozen feedback. Declared closed-loop candidates rebuild
  only causal safety_limited inputs from their actual encoded native reports.
  """
  def __init__(self, engine, objective, original_anchor, *, restore_envelope=None, activation_plan=None):
    if (engine.failed or not engine.raw_mode or not 1 <= len(objective.names) <= 4 or
        len(set(objective.names)) != len(objective.names) or len(objective.anchor_digest) != 64):
      raise ValueError('event selected replay contract')
    self.engine, self.objective, self.original_anchor = engine, objective, original_anchor
    self.restore_envelope = restore_envelope
    self.activation_plan = activation_plan
    self.__identification_token = object()
    self.__strict_rollouts = None
    self.__subset_baseline_view = None
    self.__causal_features = None
    self.__prepared_car_states = None
    self.__controller_prefix = None
    self.__pending_controller_prefix = None
    if activation_plan is not None:
      from openpilot.starpilot.flm.restore_activation import WholeMapRestorePlan, restore_plan_matches
      activation_plan.validate()
      if (restore_envelope is not None and not restore_plan_matches(activation_plan, restore_envelope) or
          restore_envelope is None and type(activation_plan) is WholeMapRestorePlan):
        raise ValueError('restore activation exact operation/transaction binding')
      if (activation_plan.identity != engine.identity or
          activation_plan.baseline_revision != engine.baseline_revision or activation_plan.source_epoch != objective.context_key or
          tuple(row[:2] for row in activation_plan.windows) != tuple(engine.replay_windows)):
        raise ValueError('event activation source/window binding')
    if restore_envelope is not None:
      restore_envelope.validate()
      if (restore_envelope.version != 2 or restore_envelope.operation != 'restore' or
          restore_envelope.replay_anchor() != original_anchor):
        raise ValueError('explicit whole-map restore replay context')
    self.projected = {}
    accumulated = dict(original_anchor['accumulated']) if isinstance(original_anchor, dict) else {}
    self.prior_full = accumulated
    self.reference = tuple(float(accumulated.get(name, 0.0)) for name in objective.names)

  def __getattr__(self, name):
    return getattr(self.engine, name)

  def _prior_matches(self, frame):
    trace = trace_for(frame,self)
    if trace is None:
      return not any(self.prior_full.values()) and not any(frame.applied_coordinates)
    try:
      trace.validate()
      envelope = trace.envelope
      return (dict(trace.target) == self.prior_full and envelope.selected_baseline_revision == self.baseline_revision and
              envelope.source_epoch == self.objective.context_key and
              getattr(frame, 'effect_prepare_ns', 0) == trace.control_ns)
    except (AttributeError, TypeError, ValueError):
      return False

  def _rollout(self, coordinates, *, restore_target=False, restore_omit_group=None, _identification_token=None):
    """At most baseline+last successful strict trial result, never authority.

    Every hit rebinds exact owned input bytes. No errors, shadow probes, restore
    paths or current admissions are memoized. Final validate still checks its
    current source/model/sample/support/heldout predicates independently.
    """
    from openpilot.starpilot.flm.event_activation import EventActivationPlan, ZERO_REFERENCE_ACTIVATION_VERSION
    import pickle
    from types import MappingProxyType

    plan = self.__dict__.get('activation_plan')
    eligible = (type(plan) is EventActivationPlan and plan.version == ZERO_REFERENCE_ACTIVATION_VERSION and
      self.restore_envelope is None and restore_target is False and restore_omit_group is None and _identification_token is None and
      type(coordinates) is tuple and all(type(value) in (int, float) and math.isfinite(value) for value in coordinates))
    if not eligible:
      # The controller's ORIGINAL prefix is independent of native strict/shadow
      # policy. Never reuse native output or a native-policy verdict here.
      prefix_key = None
      if (_identification_token is not None and
          _identification_token is self.__dict__.get('_EventControllerReplay__identification_token') and
          self.__dict__.get('_EventControllerReplay__controller_prefix') is not None and
          type(plan) is EventActivationPlan and plan.version == ZERO_REFERENCE_ACTIVATION_VERSION and
          self.restore_envelope is None and restore_target is False and restore_omit_group is None):
        prefix_key = self._causal_input_key(.01)
      if prefix_key is None:
        return self._rollout_uncached(coordinates, restore_target=restore_target,
          restore_omit_group=restore_omit_group, _identification_token=_identification_token)
      result = self._rollout_uncached(coordinates, restore_target=restore_target,
        restore_omit_group=restore_omit_group, _identification_token=_identification_token,
        _prefix_context=(prefix_key, False))
      if self._causal_input_key(.01) != prefix_key:
        self.__controller_prefix = None
        raise ValueError('controller prefix changed owned input during execution')
      return result
    source_key = self._causal_input_key(.01)
    if source_key is None:
      self.__strict_rollouts = None
      self.__controller_prefix = None
      return self._rollout_uncached(coordinates)
    equivalent = self._equivalent_subset_baseline(coordinates, source_key)
    if equivalent is not None:
      return equivalent
    if self.__strict_rollouts is None or self.__strict_rollouts[0] != source_key:
      self.__strict_rollouts = source_key, {}
    slots = self.__strict_rollouts[1]
    coordinate_key = pickle.dumps(coordinates, protocol=5)  # Type/bit exact, including signed zero.
    slot = 'baseline' if coordinate_key == pickle.dumps(self.reference, protocol=5) else 'candidate'
    key = plan.version, 'strict-trial', coordinate_key
    cached = slots.get(slot)
    if cached is not None and cached[0] == key:
      return tuple(dict(part) for part in cached[1])
    from openpilot.starpilot.flm.controller_replay import ControllerReplay
    from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
    prefix_scoped = (type(self) is EventControllerReplay and type(self.engine) is ControllerReplay and
      type(self.emissions) is EmissionHistory)
    build_prefix = slot == 'baseline' and prefix_scoped
    self.__pending_controller_prefix = None
    # A cache entry may carry only the proof completed by THIS strict execution.
    self.__candidate_feedback_proof = None
    if prefix_scoped:
      result = self._rollout_uncached(coordinates, _prefix_context=(source_key, build_prefix))
    else:
      result = self._rollout_uncached(coordinates)
    if self._causal_input_key(.01) != source_key:
      self.__strict_rollouts = None
      self.__controller_prefix = self.__pending_controller_prefix = None
      raise ValueError('strict rollout changed owned input during execution')
    if build_prefix:
      self.__controller_prefix = self.__pending_controller_prefix
      self.__pending_controller_prefix = None
    frozen = tuple(MappingProxyType(dict(part)) for part in result)
    from openpilot.starpilot.flm.candidate_feedback import uses_closed_loop, strict_reference_proof_key
    if uses_closed_loop(self):
      proof = self.__candidate_feedback_proof
      proof_key = strict_reference_proof_key(proof,self,coordinates,len(frozen[0]))
      if proof_key is not None:
        slots[slot] = key, frozen, (proof,proof_key)
      # No complete strict chronology: do not create a reusable result.
    else:
      slots[slot] = key, frozen
    return tuple(dict(part) for part in frozen)

  @guarded_archive()
  def _rollout_uncached(self, coordinates, *, restore_target=False, restore_omit_group=None, _identification_token=None,
                        _prefix_context=None):
    from openpilot.starpilot.flm.selected_effects import (EffectTrial, EffectReplayTransition, EffectReplayRecordedSource,
      EffectRestoreTrial, EffectRestoreAblationTrial)

    shadow = _identification_token is not None
    if shadow and (_identification_token is not self.__identification_token or self.activation_plan is None or
                   self.activation_plan.identification_policy != NATIVE_SHADOW_IDENTIFICATION or restore_target or self.restore_envelope is not None):
      raise ValueError('event identification scope/policy mismatch')
    if (len(coordinates) != len(self.objective.names) or
        any(not math.isfinite(value) or abs(value) > .25 for value in coordinates)):
      raise ValueError('event original anchor coordinate envelope')
    if restore_target:
      if self.restore_envelope is None:
        raise ValueError('restore endpoint cannot be an optimizer trial')
      restore_coordinates = tuple(dict(self.restore_envelope.coordinates)[n] for n in self.objective.names)
      if tuple(coordinates) != restore_coordinates:
        raise ValueError('restore markers changed historical map')
      if restore_omit_group is None:
        trial = EffectRestoreTrial(self.objective, tuple(coordinates), self.baseline_revision,
          self.original_anchor, self.restore_envelope)
      else:
        if self.activation_plan is not None:
          from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
          if tuple(restore_omit_group) not in tuple(GROUPS[group] for group in self.restore_envelope.restoration_groups):
            raise ValueError('restore ablation must be one active exact canonical group')
        trial = EffectRestoreAblationTrial(self.objective, tuple(coordinates), self.baseline_revision,
          self.original_anchor, self.restore_envelope, tuple(restore_omit_group))
      trial.basis()
    elif restore_omit_group is not None:
      raise ValueError('group-off analysis requires explicit full restore')
    elif self.restore_envelope is not None:
      trial = None  # Restore baseline is the recorded neutral trajectory.
    else:
      trial = EffectTrial(self.objective, tuple(coordinates), self.baseline_revision, self.original_anchor)
    frames, windows = tuple(self.records), tuple(self.replay_windows)
    if not frames or len(frames) > 12000 or not 1 <= len(windows) <= 24:
      raise ValueError('event exact window bound')
    if any(a > b or i and a <= windows[i - 1][1] for i, (a, b) in enumerate(windows)):
      raise ValueError('event windows overlap')
    controller, vm = self.factory.create()
    # Parent/default/current replay never allocates this child-only reader cache.
    from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory, _PreparedCarStates
    from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
    from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
    from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
    supported = (len(frames) <= 12000 and type(self.factory) in (SelectedControllerFactory, PidControllerFactory) and
                 type(controller) in (LatControlTorque, LatControlPID))
    prepared_states = self.__dict__.get('_EventControllerReplay__prepared_car_states') if supported else None
    if supported and type(prepared_states) is not _PreparedCarStates:
      prepared_states = self.__prepared_car_states = _PreparedCarStates()
    transition = None if trial is None or restore_target else EffectReplayTransition(trial)
    checkpoints, requests, saturated = dict(self.checkpoints), {}, {}
    # Only the wrapper supplies a freshly hashed owned-input context. A new
    # certificate is published there AFTER full strict success and rehashing.
    from openpilot.starpilot.flm.event_activation import EventActivationPlan, ZERO_REFERENCE_ACTIVATION_VERSION
    from types import MappingProxyType
    from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
    from openpilot.starpilot.lateral.torque_extension import TorqueExtension
    from openpilot.starpilot.lateral.ioniq6_policy import Ioniq6TorquePolicy
    extension = getattr(controller, 'starpilot_extension', None)
    prefix_supported = (type(controller) is LatControlPID or type(controller) is LatControlTorque and
      (extension is None or type(extension) is TorqueExtension and type(extension.policy) is Ioniq6TorquePolicy))
    prefix_eligible = (supported and prefix_supported and type(self.emissions) is EmissionHistory and
      type(self.activation_plan) is EventActivationPlan and
      self.activation_plan.version == ZERO_REFERENCE_ACTIVATION_VERSION and self.restore_envelope is None and
      not restore_target and restore_omit_group is None and _prefix_context is not None)
    build_prefix = bool(prefix_eligible and _prefix_context[1] and not shadow and tuple(coordinates) == self.reference)
    certificate = self.__dict__.get('_EventControllerReplay__controller_prefix') if prefix_eligible and not build_prefix else None
    if certificate is not None and certificate[0] != cast(tuple[object, bool], _prefix_context)[0]:
      certificate = None
    prefix_requests, prefix_saturated, prefix_states = {}, {}, {}

    candidate_feedback = None

    def controller_frames():
      nonlocal build_prefix
      prior_ns = prior_sequence = index = 0
      neutral = not restore_target and (self.restore_envelope is not None or tuple(coordinates) == self.reference)
      for frame in frames:
        while index < len(windows) and frame.control_ns > windows[index][1]:
          index += 1
        if index >= len(windows) or not windows[index][0] <= frame.control_ns <= windows[index][1]:
          raise ValueError('event frame outside original window')
        if frame.control_ns == windows[index][0]:
          if frame.control_ns not in checkpoints:
            raise ValueError('event actual pre-maneuver state missing')
          restore_snapshot(controller, vm, checkpoints[frame.control_ns])
          if transition is not None:
            transition.reset()
          prior_ns = prior_sequence = 0
        elif not prior_ns:
          raise ValueError('event window beginning omitted')
        if (not self.engine._frame_context(frame) or not self._prior_matches(frame) or any(frame.centre_values) or
            frame.actual_stage_residual is not None and any(frame.actual_stage_residual) or
            prior_ns and not 0 < frame.control_ns - prior_ns <= MAX_FRAME_GAP_NS or
            prior_sequence and frame.frame_sequence != prior_sequence + 1):
          raise ValueError('event source/history mismatch')
        trace = trace_for(frame,self)
        prepare_ns = getattr(frame, 'effect_prepare_ns', 0)
        activation = None if self.activation_plan is None else self.activation_plan.activation_for(*windows[index])
        if self.activation_plan is not None and activation is None:
          raise ValueError('event activation window omitted')
        original_prefix = bool(activation is not None and (activation[0] == 0 or frame.control_ns < activation[0]))
        activation_tick = bool(activation is not None and frame.control_ns == activation[0])
        if activation_tick and prepare_ns != activation[1]:
          raise ValueError('event activation original prepare clock changed')
        reusable_prefix = bool(original_prefix and activation[0] > windows[index][0])
        if activation_tick:
          if build_prefix and activation[0] > windows[index][0]:
            try:
              prefix_states[windows[index]] = capture_snapshot(controller, vm)
            except (AttributeError, TypeError, ValueError, KeyError, OverflowError):
              # An unavailable optimization checkpoint must NOT reject an
              # otherwise ordinary replay. Never publish a partial certificate.
              build_prefix = False
              prefix_requests.clear()
              prefix_saturated.clear()
              prefix_states.clear()
          elif certificate is not None and windows[index] in certificate[3]:
            restore_snapshot(controller, vm, certificate[3][windows[index]])
        if self.activation_plan is not None and restore_target and not original_prefix and not 0 < prepare_ns <= frame.control_ns:
          raise ValueError('restore activation original owner clock unavailable')
        if neutral or original_prefix:
          source = None if trace is None else EffectReplayRecordedSource(trace)
        elif restore_target:
          # Whole-map endpoint qualification is independent from the separately
          # replayed/probed gradual owner transition. Never reduce this map to the
          # protocol's <=4 marker coordinates or infer neutral from zero markers.
          source = trial
        elif prepare_ns > 0:
          source = transition.source_for_frame(prepare_ns, trace, new_owner=activation_tick)
        elif self.activation_plan is not None:
          raise ValueError('event activation post-treatment owner clock missing')
        elif trace is None and not any(self.prior_full.values()):
          # Historical zero-effect data may qualify a stationary shape proposal;
          # it contains no new owner prepare stamp and confers no slew admission.
          # Parent current_effect_probe separately requires fresh exact prepare_ns.
          source = trial
        else:
          raise ValueError('event exact owner transition clock unavailable')
        trace = None
        run_inputs = frame.inputs if candidate_feedback is None else candidate_feedback.before(frame)
        if certificate is not None and reusable_prefix and windows[index] in certificate[3]:
          # Retain every source/history/clock/prepare/recorded-source check above.
          # Later state is restored ONLY at the exact declared activation tick.
          output = certificate[1][frame.control_ns]
          is_saturated = certificate[2][frame.control_ns]
        else:
          prepared = None if prepared_states is None else prepared_states.get(frame.inputs)
          kwargs = {} if prepared is None else {'_prepared_car_state': prepared}
          output, log = _run(controller, vm, run_inputs, self.engine.objective, (0.0, 0.0), frame.torque_values,
                             actual_stage_residual=frame.actual_stage_residual, effect_trial=source, **kwargs)
          is_saturated = bool(log.saturated)
        if (neutral or original_prefix) and (not same_final_command(output, frame.selected_command) or
                        frame.actual_lac_command is not None and not same_float(output, frame.actual_lac_command)):
          raise ValueError('event neutral selected trajectory changed')
        raw_output = output  # Preserve raw Float64 for the original actual_lac guard.
        output = final_command(output)
        if not math.isfinite(output) or abs(output) > 1.000001:
          raise ValueError('event selected command envelope')
        requests[frame.control_ns], saturated[frame.control_ns] = output, is_saturated
        if candidate_feedback is not None:
          candidate_feedback.after(frame.control_ns,output)
        if build_prefix and reusable_prefix:
          prefix_requests[frame.control_ns], prefix_saturated[frame.control_ns] = raw_output, is_saturated
        prior_ns, prior_sequence = frame.control_ns, frame.frame_sequence
        yield frame.control_ns
      if index != len(windows) - 1 or prior_ns != windows[-1][1]:
        raise ValueError('event last actual tick missing')

    lazy_requests = _LazyControllerRequests(requests, controller_frames())
    try:
      from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
      native_requests = lazy_requests
      if not supported or type(self.emissions) is not EmissionHistory:
        lazy_requests.finish()  # Unknown/legacy shells retain original eager order.
        native_requests = requests  # Preserve their original plain-dict API too.
      # Original native replay owns every emission, including unscored context.
      quality = {} if self.activation_plan is None else {'quality_contract': self.activation_plan.clipping_quality}
      if shadow:
        quality['identification_policy'] = NATIVE_SHADOW_IDENTIFICATION
      feedback = getattr(self,'verified_feedback_tape',None)
      reported = None
      if feedback is not None:
        from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
        evidence = require_feedback_engine(self)
        reported = {}
        quality['feedback_reported'] = reported
        from openpilot.starpilot.flm.candidate_feedback import CLOSED_POLICY,CandidateFeedbackLoop
        if evidence.candidate_policy == CLOSED_POLICY:
          if not supported or type(self.emissions) is not EmissionHistory:
            raise ValueError('closed-loop candidate requires actual registered controller/native replay')
          candidate_feedback = CandidateFeedbackLoop(self,reported,tuple(coordinates),
            native_policy='native-shadow-identification' if shadow else 'strict',
            original_baseline=not restore_target and tuple(coordinates) == self.reference)
      encoded = self.emissions.rollout(native_requests, windows[0][0], windows[-1][1], windows, **quality)
      # Native success is insufficient: late controller-only frames and the exact
      # original final-window/tick guard must ALSO finish before any result exists.
      lazy_requests.finish()
      if reported is not None:
        if candidate_feedback is not None:
          self.__candidate_feedback_proof = candidate_feedback.finish(requests,reported)
        else:
          from openpilot.starpilot.flm.feedback_replay import verify_candidate_engine
          verify_candidate_engine(self,requests,reported)
      if build_prefix and prefix_requests and prefix_states:
        self.__pending_controller_prefix = (cast(tuple[object, bool], _prefix_context)[0], MappingProxyType(prefix_requests),
          MappingProxyType(prefix_saturated), MappingProxyType(prefix_states))
      return requests, encoded, saturated
    finally:
      # Release suspended controller/native traceback resources on rejection.
      # close() NEVER substitutes for the required successful finish() above.
      lazy_requests.close()

  def _causal_input_key(self, epsilon):
    """Exact owned input bytes, not equality of floats or a caller-ready flag.

    Includes every original frame/native witness/checkpoint, full-map anchor,
    selected factory configuration, objective, epsilon and declared plan.
    Pickle is only serialized+hashed here; no cache bytes are ever unpickled.
    """
    import hashlib
    import pickle

    if 'engine' not in self.__dict__:
      return None  # A legacy/test shell has no owned input history to cache.
    try:
      from openpilot.starpilot.flm.controller_replay import SelectedControllerFactory
      from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
      factory = self.factory
      if type(factory) not in (SelectedControllerFactory, PidControllerFactory):
        return None
      policy = ('torque', str(factory.policy)) if type(factory) is SelectedControllerFactory else ('stock-pid', None)
      selected = (type(factory).__module__, type(factory).__qualname__,
        factory.cp.as_builder().to_bytes(), str(factory.mode), policy, factory.turn_assist,
        factory.manual_baseline, factory.manual_source_epoch,
        factory.interface_class.__module__, factory.interface_class.__qualname__)
      native = (self.emissions.cp.as_builder().to_bytes(), self.emissions.identity, self.emissions.baseline_revision,
        tuple(self.emissions.events.items()), tuple(self.emissions.carriers.items()))
      data = (epsilon, self.identity, self.baseline_revision, self.engine.model_revision,
        self.engine.failed, self.engine.raw_mode, selected, self.engine.objective,
        self.objective, self.reference, self.prior_full, self.original_anchor,
        self.activation_plan, self.restore_envelope, tuple(self.records), tuple(self.checkpoints),
        tuple(self.replay_windows), native)
      if archive_key(self) is not None:
        data = (*data,archive_key(self))
      if getattr(self,'verified_feedback_tape',None) is not None:
        from openpilot.starpilot.flm.feedback_replay import owned_feedback_key
        data += (owned_feedback_key(self),)
      return hashlib.sha256(pickle.dumps(data, protocol=5)).digest()
    except (AttributeError, TypeError, ValueError, OverflowError, pickle.PicklingError):
      return None  # Unsupported cache key means full replay, never readiness.

  def reuse_subset_causal_features(self, original, *, epsilon=.01):
    """Project a verified full FD matrix only for the identical owned subtrial.

    This transfers numeric columns, never a native verdict, shadow token,
    profile, validation, lease, or optimizer choice. Any unknown case rebuilds.
    """
    from openpilot.starpilot.flm.candidate_feedback import uses_closed_loop
    if uses_closed_loop(self) != uses_closed_loop(original):
      return False

    import pickle
    import struct
    from types import MappingProxyType
    from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, parameters_for
    from openpilot.starpilot.flm.event_activation import EventActivationPlan, ZERO_REFERENCE_ACTIVATION_VERSION
    from openpilot.starpilot.flm.event_objective import EffectObjective
    from openpilot.starpilot.flm.selected_effects import EffectTrial, EffectReplayTransition

    try:
      if (type(self) is not EventControllerReplay or type(original) is not EventControllerReplay or self is original or
          'engine' not in self.__dict__ or 'engine' not in original.__dict__):
        return False
      if (type(epsilon) is not float or not math.isfinite(epsilon) or not 0 < epsilon <= .03 or
          self.restore_envelope is not None or original.restore_envelope is not None or
          self.engine is not original.engine or
          self.failed or original.failed or not self.raw_mode or not original.raw_mode or
          any(getattr(self, name) is not getattr(original, name) for name in
              ('factory', 'records', 'checkpoints', 'replay_windows', 'emissions', 'activation_plan')) or
          type(self.objective) is not EffectObjective or type(original.objective) is not EffectObjective or
          self.objective.group != original.objective.group or self.objective.context_key != original.objective.context_key):
        return False
      if self.activation_plan is not None and (type(self.activation_plan) is not EventActivationPlan or
          self.activation_plan.version != ZERO_REFERENCE_ACTIVATION_VERSION):
        return False
      names, full_names = self.objective.names, original.objective.names
      if (not set(names) < set(full_names) or self.objective.parameters != parameters_for(names) or
          original.objective.parameters != parameters_for(full_names)):
        return False
      if (self.__causal_features is not None or self.projected or
          pickle.dumps((self.identity, self.baseline_revision), protocol=5) !=
          pickle.dumps((original.identity, original.baseline_revision), protocol=5)):
        return False
      saved = original.__causal_features
      old_key = original._causal_input_key(epsilon)
      if (old_key is None or type(saved) is not tuple or len(saved) != 2 or saved[0] != old_key or
          type(saved[1]) is not MappingProxyType):
        return False

      def bits(values):
        if type(values) is not dict or tuple(values) != NAMES or any(type(values[n]) is not float for n in NAMES):
          raise ValueError('subset requires exact builtin full30 float prior')
        return struct.pack('!30d', *(values[n] for n in NAMES))

      before = EffectTrial(original.objective, original.reference, original.baseline_revision, original.original_anchor)
      after = EffectTrial(self.objective, self.reference, self.baseline_revision, self.original_anchor)
      _, _, old_prior = before.basis()
      _, _, new_prior = after.basis()
      prior_bits = bits(old_prior)
      if any(bits(value) != prior_bits for value in (new_prior, original.prior_full, self.prior_full)):
        return False
      for engine in (original, self):
        reference = tuple(engine.prior_full[name] for name in engine.objective.names)
        if (type(engine.reference) is not tuple or any(type(value) is not float for value in engine.reference) or
            struct.pack('!' + 'd' * len(reference), *engine.reference) != struct.pack('!' + 'd' * len(reference), *reference)):
          return False
      # The only allowed anchor-payload difference declares the smaller names.
      old_anchor, new_anchor = dict(original.original_anchor), dict(self.original_anchor)
      old_anchor.pop('anchor_digest')
      new_anchor.pop('anchor_digest')
      if pickle.dumps(old_anchor, protocol=5) != pickle.dumps(new_anchor, protocol=5):
        return False
      indices = tuple(full_names.index(name) for name in names)
      # Independently validate BOTH bases/targets/static full30 source geometry
      # for each exact clamped probe. Same original trace, plan, checkpoint and
      # prepare clocks then give identical step_full paths by induction.
      for index, parameter in enumerate(self.objective.parameters):
        for direction in (-1, 1):
          full = list(original.reference)
          subset = list(self.reference)
          probe = max(parameter.low, min(parameter.high, self.reference[index] + direction * epsilon))
          full[indices[index]], subset[index] = probe, probe
          left = EffectTrial(original.objective, tuple(full), self.baseline_revision, original.original_anchor)
          right = EffectTrial(self.objective, tuple(subset), self.baseline_revision, self.original_anchor)
          left_transition, right_transition = EffectReplayTransition(left), EffectReplayTransition(right)
          left_source, right_source = left.source(), right.source()
          if (bits(left_transition.original_target) != bits(right_transition.original_target) or
              bits(left_transition.target) != bits(right_transition.target) or
              bits(left_source.values) != bits(right_source.values) or
              left_source.full_profile_sha256 != right_source.full_profile_sha256 or
              left_source.original_effect_anchor != original.objective.anchor_digest or
              right_source.original_effect_anchor != self.objective.anchor_digest):
            return False
      copied = {}
      for stamp, (baseline, features) in saved[1].items():
        if (type(stamp) is not int or type(features) is not tuple or len(features) != len(full_names) or
            any(type(value) is not float or not math.isfinite(value) for value in (baseline, *features))):
          return False
        copied[stamp] = baseline, tuple(features[index] for index in indices)
      if not copied or original._causal_input_key(epsilon) != old_key:
        return False
      new_key = self._causal_input_key(epsilon)
      if new_key is None:
        return False
      self.__causal_features = new_key, MappingProxyType(copied)
      self.projected = dict(copied)
      # ONLY after every original subset/full30/source proof succeeded, rebind
      # the controller-only ORIGINAL prefix. _run consumes the common base
      # engine objective, recorded source and inputs before activation; trial
      # names/coordinates/transition are unused there. Native replay stays fresh.
      # Prefix keys ALWAYS use .01, independently from an FD caller's epsilon.
      from openpilot.starpilot.flm.controller_replay import ControllerReplay, Snapshot
      from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
      prefix = original.__dict__.get('_EventControllerReplay__controller_prefix')
      if (epsilon == .01 and type(self.engine) is ControllerReplay and type(self.emissions) is EmissionHistory and
          type(self.activation_plan) is EventActivationPlan and self.activation_plan.version == ZERO_REFERENCE_ACTIVATION_VERSION and
          self.__dict__.get('_EventControllerReplay__controller_prefix') is None and
          type(prefix) is tuple and len(prefix) == 4 and prefix[0] == old_key and
          all(type(part) is MappingProxyType for part in prefix[1:]) and
          prefix[1].keys() == prefix[2].keys() and
          all(type(stamp) is int and type(value) is float and math.isfinite(value) for stamp, value in prefix[1].items()) and
          all(type(value) is bool for value in prefix[2].values()) and
          all(window in self.replay_windows and type(value) is Snapshot for window, value in prefix[3].items())):
        self.__controller_prefix = (new_key, *(MappingProxyType(dict(part)) for part in prefix[1:]))
      # Separate numerical view only. No borrowed strict proof or native verdict
      # is installed in this selected engine's strict cache or proof fields.
      cache = original.__dict__.get('_EventControllerReplay__strict_rollouts')
      entry = cache[1].get('baseline') if type(cache) is tuple and len(cache) == 2 and type(cache[1]) is dict else None
      if (epsilon == .01 and uses_closed_loop(self) and type(cache) is tuple and len(cache) == 2 and cache[0] == old_key and
          type(entry) is tuple and len(entry) == 3 and type(entry[2]) is tuple and len(entry[2]) == 2 and
          type(entry[2][1]) is bytes):
        self.__subset_baseline_view = _EquivalentBaselineView(original,cache,entry,old_key,new_key,entry[2][1])
      return True
    except (AttributeError, TypeError, ValueError, KeyError, OverflowError, pickle.PicklingError):
      return False

  def _equivalent_subset_baseline(self, coordinates, source_key):
    """Only an identical baseline's numeric maps; never a new execution proof.

    The full owner's strict proof stays bound to its original objective. Both
    baselines take the neutral recorded-source branch on EVERY frame; shared
    records/checkpoints/inputs/native state therefore give identical updates.
    Full30 target/source checks conservatively preserve the existing trial guards.
    Every use freshly binds both owners before and after callback-capable work.
    """
    import pickle
    import struct
    from types import MappingProxyType
    from openpilot.starpilot.flm.candidate_feedback import uses_closed_loop,strict_reference_proof_key,CandidateFeedbackProof,_SEAL
    from openpilot.starpilot.flm.canonical_surface_adapter import NAMES,parameters_for
    from openpilot.starpilot.flm.controller_replay import ControllerReplay,SelectedControllerFactory
    from openpilot.starpilot.flm.event_objective import EffectObjective
    from openpilot.starpilot.flm.event_activation import EventActivationPlan,ZERO_REFERENCE_ACTIVATION_VERSION
    from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
    from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
    from openpilot.starpilot.flm.selected_effects import EffectTrial,EffectReplayTransition
    from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
    from openpilot.starpilot.lateral.controller_selection import ControllerMode
    from capnp.lib.capnp import _DynamicStructReader
    from opendbc.car.car_helpers import interfaces
    from abc import ABCMeta
    from collections import OrderedDict

    try:
      view = self.__dict__.get('_EventControllerReplay__subset_baseline_view')
      if (type(view) is not _EquivalentBaselineView or type(self) is not EventControllerReplay or
          type(source_key) is not bytes or type(view.original_key) is not bytes or type(view.selected_key) is not bytes or
          type(coordinates) is not tuple or any(type(value) not in (int,float) or not math.isfinite(value) for value in coordinates) or
          type(view.cache) is not tuple or len(view.cache) != 2 or type(view.cache[0]) is not bytes or
          type(view.cache[1]) is not dict or any(type(key) is not str for key in view.cache[1]) or
          type(view.entry) is not tuple or len(view.entry) != 3 or type(view.proof_fingerprint) is not bytes):
        return None
      original = view.owner
      if (type(original) is not EventControllerReplay or original is self or
          type(self.engine) is not ControllerReplay or self.engine is not original.engine or
          type(self.emissions) is not EmissionHistory or
          self.restore_envelope is not None or original.restore_envelope is not None or
          type(self.activation_plan) is not EventActivationPlan or self.activation_plan.version != ZERO_REFERENCE_ACTIVATION_VERSION or
          self.failed or original.failed or not self.raw_mode or not original.raw_mode or
          any(getattr(self,name) is not getattr(original,name) for name in
            ('factory','records','checkpoints','replay_windows','emissions','activation_plan')) or
          not uses_closed_loop(self) or not uses_closed_loop(original) or source_key != view.selected_key or
          pickle.dumps(coordinates,protocol=5) != pickle.dumps(self.reference,protocol=5)):
        return None
      # Unknown callback-bearing factory/collection shapes retain real replay.
      factory = self.factory
      if type(factory) not in (SelectedControllerFactory,PidControllerFactory):
        return None
      interface_metaclass = type(factory.interface_class)
      if (type(factory.cp) is not _DynamicStructReader or type(self.emissions.cp) is not _DynamicStructReader or
          type(factory.mode) not in (str,ControllerMode) or
          (interface_metaclass is not type and interface_metaclass is not ABCMeta) or
          factory.interface_class is not interfaces.get(str(factory.cp.carFingerprint)) or
          type(self.emissions.events) is not OrderedDict or type(self.emissions.carriers) is not OrderedDict or
          type(factory) is SelectedControllerFactory and type(factory.policy) not in (str,type(None))):
        return None
      if (original._causal_input_key(.01) != view.original_key or
          original.__dict__.get('_EventControllerReplay__strict_rollouts') is not view.cache or
          view.cache[0] != view.original_key or view.cache[1].get('baseline') is not view.entry or
          type(self.objective) is not EffectObjective or type(original.objective) is not EffectObjective or
          not set(self.objective.names) < set(original.objective.names) or
          self.objective.group != original.objective.group or self.objective.context_key != original.objective.context_key or
          self.objective.parameters != parameters_for(self.objective.names) or
          original.objective.parameters != parameters_for(original.objective.names)):
        return None
      old_evidence,new_evidence = require_feedback_engine(original),require_feedback_engine(self)
      if (old_evidence.original is not new_evidence.original or old_evidence.binding != new_evidence.binding or
          tuple((bounds,subset.digest,activation) for bounds,subset,activation in old_evidence.windows) !=
          tuple((bounds,subset.digest,activation) for bounds,subset,activation in new_evidence.windows)):
        return None
      expected = self.activation_plan.version,'strict-trial',pickle.dumps(original.reference,protocol=5)
      if (view.entry[0] != expected or type(view.entry[1]) is not tuple or len(view.entry[1]) != 3 or
          any(type(part) is not MappingProxyType for part in view.entry[1]) or
          type(view.entry[2]) is not tuple or len(view.entry[2]) != 2 or view.entry[2][1] != view.proof_fingerprint or
          strict_reference_proof_key(view.entry[2][0],original,original.reference,len(view.entry[1][0])) != view.proof_fingerprint):
        return None
      def bits(values):
        if type(values) is not dict or tuple(values) != NAMES or any(type(values[name]) is not float for name in NAMES):
          raise ValueError('exact full30 baseline values')
        return struct.pack('!30d',*(values[name] for name in NAMES))
      left = EffectTrial(original.objective,original.reference,original.baseline_revision,original.original_anchor)
      right = EffectTrial(self.objective,self.reference,self.baseline_revision,self.original_anchor)
      left_transition,right_transition = EffectReplayTransition(left),EffectReplayTransition(right)
      left_source,right_source = left.source(),right.source()
      prior_bits = bits(original.prior_full)
      if (bits(self.prior_full) != prior_bits or bits(left_transition.original_target) != prior_bits or
          bits(right_transition.original_target) != prior_bits or bits(left_transition.target) != prior_bits or
          bits(right_transition.target) != prior_bits or bits(left_source.values) != bits(right_source.values) or
          left_source.full_profile_sha256 != right_source.full_profile_sha256 or
          left_source.original_effect_anchor != original.objective.anchor_digest or
          right_source.original_effect_anchor != self.objective.anchor_digest):
        return None
      for owner in (original,self):
        reference = tuple(owner.prior_full[name] for name in owner.objective.names)
        if (any(type(value) is not float for value in owner.reference) or
            pickle.dumps(reference,protocol=5) != pickle.dumps(owner.reference,protocol=5)):
          return None
      old_anchor,new_anchor = dict(original.original_anchor),dict(self.original_anchor)
      old_anchor.pop('anchor_digest')
      new_anchor.pop('anchor_digest')
      if pickle.dumps(old_anchor,protocol=5) != pickle.dumps(new_anchor,protocol=5):
        return None
      result = tuple(dict(part) for part in view.entry[1])
      # This repeat may execute original source callbacks; both source keys are
      # recomputed AFTER it. The final proof snapshot below is primitive-only.
      if strict_reference_proof_key(view.entry[2][0],original,original.reference,len(result[0])) != view.proof_fingerprint:
        return None
      def proof_snapshot(proof):
        if type(proof) is not CandidateFeedbackProof:
          return None
        owned = object.__getattribute__(proof,'__dict__')
        if type(owned) is not dict or any(type(key) is not str for key in owned) or owned.get('_seal') is not _SEAL:
          return None
        values = tuple(owned.get(name) for name in ('version','source_digest','original_digest',
          'coordinates_digest','native_policy','chronology_digest','row_count'))
        if any(type(value) is not str for value in values[:6]) or type(values[6]) is not int:
          return None
        return pickle.dumps(values,protocol=5)
      if (original._causal_input_key(.01) != view.original_key or self._causal_input_key(.01) != view.selected_key or
          self.__dict__.get('_EventControllerReplay__subset_baseline_view') is not view or
          original.__dict__.get('verified_feedback_tape') is not old_evidence or
          self.__dict__.get('verified_feedback_tape') is not new_evidence or
          original.__dict__.get('_EventControllerReplay__strict_rollouts') is not view.cache or
          view.cache[1].get('baseline') is not view.entry or view.entry[2][1] != view.proof_fingerprint or
          proof_snapshot(view.entry[2][0]) != view.proof_fingerprint):
        return None
      return result
    except (AttributeError,TypeError,ValueError,KeyError,OverflowError,pickle.PicklingError):
      return None

  def ensure_causal_features(self, *, epsilon=.01):
    """Reuse only this engine's successful original full rebuild; never IPC flags."""
    if self.restore_envelope is not None:
      return self.rebuild_causal_features(epsilon=epsilon)
    key = self._causal_input_key(epsilon)
    if key is None or self.__causal_features is None or self.__causal_features[0] != key:
      return self.rebuild_causal_features(epsilon=epsilon)
    # Restore only the readonly derived result, never a caller-edited projection.
    self.projected = dict(self.__causal_features[1])
    return len(self.projected)

  def _strict_fd_reference(self, coordinates, certificate):
    """Reuse one proven strict baseline only for an identical shadow FD side.

    Exact full input bytes are rebound on every hit. This is a native-limited
    numerical result, never a profile/current admission or a saved shadow proof.
    """
    if certificate is None:
      return None
    import pickle
    from types import MappingProxyType
    from openpilot.starpilot.flm.controller_replay import ControllerReplay
    from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory
    from openpilot.starpilot.flm.event_activation import EventActivationPlan, ZERO_REFERENCE_ACTIVATION_VERSION
    try:
      plan = self.__dict__.get('activation_plan')
      if (type(self) is not EventControllerReplay or type(self.__dict__.get('engine')) is not ControllerReplay or
          type(self.emissions) is not EmissionHistory or self.restore_envelope is not None or
          type(plan) is not EventActivationPlan or plan.version != ZERO_REFERENCE_ACTIVATION_VERSION or
          plan.identification_policy != NATIVE_SHADOW_IDENTIFICATION):
        return None
      from openpilot.starpilot.flm.candidate_feedback import uses_closed_loop, _strict_reference_fields
      closed = uses_closed_loop(self)
      saved, entry, reference_bits = certificate
      current = self.__dict__.get('_EventControllerReplay__strict_rollouts')
      if (current is not saved or type(saved) is not tuple or len(saved) != 2 or type(saved[1]) is not dict or
          saved[1].get('baseline') is not entry or type(entry) is not tuple or len(entry) != (3 if closed else 2) or
          entry[0] != (plan.version, 'strict-trial', reference_bits) or
          type(entry[1]) is not tuple or len(entry[1]) != 3 or
          any(type(part) is not MappingProxyType for part in entry[1]) or
          pickle.dumps(tuple(coordinates), protocol=5) != reference_bits):
        return None
      key = self._causal_input_key(.01)  # Exact same owned-key contract as strict rollout cache.
      if key is None or key != saved[0]:
        return None
      if closed:
        retained = entry[2]
        if type(retained) is not tuple or len(retained) != 2 or type(retained[1]) is not bytes:
          return None
        # The complete pre-key above has ALREADY validated this owned
        # evidence's source digest and every subset/activation seal. Only the
        # primitive proof projection is needed here; the complete post-key
        # below remains mandatory after ALL callback-capable field reads.
        evidence = object.__getattribute__(self,'__dict__').get('verified_feedback_tape')
        fresh = _strict_reference_fields(retained[0],evidence,self.reference,len(entry[1][0]))
        if fresh is None or fresh != retained[1]:
          return None
        # Proof validation itself must not open a mutable-input gap.
        if self._causal_input_key(.01) != key:
          return None
      return dict(entry[1][1])  # Copy native outputs; no mutable cache alias.
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError, pickle.PicklingError):
      return None  # Unknown/stale proof keeps the original shadow execution.

  def _zero_gain_fd_reference(self, coordinates, certificate):
    """Exact strict baseline for a provably input-neutral torque FD probe only.

    Turn-in/unwind FF modifiers multiply ff_gain in the canonical torque
    formula. With full neutral original/reference state and no effect traces,
    changing only one such modifier leaves ALL selected controller hook outputs
    identical at every frame, including the gradual owner transition. This
    never covers crawl, PID, restore, candidate feasibility or failed probes.
    """
    from openpilot.starpilot.flm.controller_replay import ControllerReplay, SelectedControllerFactory
    from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
    from openpilot.starpilot.flm.selected_effects import EffectTrial, EffectReplayTransition
    from openpilot.starpilot.flm.canonical_surface_adapter import CanonicalSurfaceEffects, NAMES
    from openpilot.starpilot.flm import torque_surface
    try:
      if (type(self) is not EventControllerReplay or type(self.engine) is not ControllerReplay or
          type(self.factory) is not SelectedControllerFactory or type(self.actual) is not LatControlTorque or
          self.restore_envelope is not None or type(coordinates) is not tuple or
          len(coordinates) != len(self.reference) or not 1 <= len(coordinates) <= 4 or
          any(type(value) not in (int,float) or not math.isfinite(value) or abs(value) > .25 for value in coordinates) or
          any(value != 0. for value in self.reference) or set(self.prior_full) != set(NAMES) or
          any(value != 0. for value in self.prior_full.values()) or
          any(getattr(frame,'effect_trace',None) is not None for frame in self.records)):
        return None
      changed = tuple(name for name,value,base in zip(self.objective.names,coordinates,self.reference,strict=True) if value != base)
      if len(changed) != 1 or changed[0] not in (
          'turn_in_boost_left','turn_in_boost_right','unwind_taper_left','unwind_taper_right'):
        return None
      trial = EffectTrial(self.objective,coordinates,self.baseline_revision,self.original_anchor)
      transition = EffectReplayTransition(trial)  # Validate the exact original full30 target/anchor.
      source = trial.source()
      if (type(source) is not CanonicalSurfaceEffects or source.math is not torque_surface or
          tuple(source.values) != NAMES or source.values['ff_gain_left'] != 0. or source.values['ff_gain_right'] != 0. or
          any(value != 0. for value in transition.original_target.values()) or
          any(value != 0. for name,value in transition.target.items() if name != changed[0])):
        return None
      # Neutral strict replay does not execute a candidate owner transition.
      # Reproduce that candidate-only lifecycle over EVERY original frame;
      # source equivalence cannot waive missing/regressing prepare clocks.
      windows = tuple(self.replay_windows)
      if not windows:
        return None
      index, previous_prepare = 0, 0
      for frame in self.records:
        while index < len(windows) and frame.control_ns > windows[index][1]:
          index += 1
        if index >= len(windows) or not windows[index][0] <= frame.control_ns <= windows[index][1]:
          return None
        if frame.control_ns == windows[index][0]:
          transition.reset()
          previous_prepare = 0
        activation = self.activation_plan.activation_for(*windows[index])
        if activation is None:
          return None
        original_prefix = activation[0] == 0 or frame.control_ns < activation[0]
        activation_tick = frame.control_ns == activation[0]
        prepare_ns = getattr(frame,'effect_prepare_ns',0)
        if activation_tick and prepare_ns != activation[1]:
          return None
        if original_prefix:
          continue
        if not 0 < prepare_ns <= frame.control_ns or previous_prepare and prepare_ns <= previous_prepare:
          return None  # Unknown clock patterns keep the original full probe.
        actual_source = transition.source_for_frame(prepare_ns,None,new_owner=activation_tick).source
        if (type(actual_source) is not CanonicalSurfaceEffects or actual_source.math is not torque_surface or
            tuple(actual_source.values) != NAMES or
            any(value != 0. for name,value in actual_source.values.items() if name != changed[0])):
          return None
        previous_prepare = prepare_ns
      # Existing proof checks exact plan, strict successful baseline, owned
      # factory/CP/checkpoints/frames/native packets and signed-bit input key.
      # It returns a copy only after this source equivalence proof succeeds.
      return self._strict_fd_reference(self.reference,certificate)
    except (AttributeError,TypeError,ValueError,KeyError,OverflowError):
      return None  # Unknown source/transition retains the original actual FD.

  def rebuild_causal_features(self, *, epsilon=.01):
    self.__causal_features = None
    if self.restore_envelope is not None:
      raise ValueError('restoration requires joint full-map/group-off proof, not marker finite differences')
    if not math.isfinite(epsilon) or not 0 < epsilon <= .03:
      raise ValueError('event finite difference envelope')
    _, baseline, _ = self._rollout(self.reference)
    plan = self.__dict__.get('activation_plan')
    identify = {} if plan is None or plan.identification_policy != NATIVE_SHADOW_IDENTIFICATION else {
      '_identification_token': self.__identification_token}
    reference_certificate = None
    if identify:
      import pickle
      saved = self.__dict__.get('_EventControllerReplay__strict_rollouts')
      if type(saved) is tuple and len(saved) == 2 and type(saved[1]) is dict:
        entry = saved[1].get('baseline')
        if entry is not None:
          reference_certificate = (saved, entry, pickle.dumps(self.reference, protocol=5))
    sensitivities = []
    for index, parameter in enumerate(self.objective.parameters):
      plus, minus = list(self.reference), list(self.reference)
      plus[index], minus[index] = min(self.reference[index] + epsilon, parameter.high), max(self.reference[index] - epsilon, parameter.low)
      span = plus[index] - minus[index]
      if span <= 0:
        raise ValueError('event parameter has no admitted excitation domain')
      # Only the explicitly bound combined experiment uses native-limited
      # SHADOW secants. These are conditional algorithmic sensitivities, never
      # admitted profiles. Final validate() uses a separate strict rollout.
      # Legacy experiments retain their original final-quality probe gates.
      high = low = None
      try:
        high = self._strict_fd_reference(tuple(plus), reference_certificate)
        if high is None and identify:
          high = self._zero_gain_fd_reference(tuple(plus), reference_certificate)
        if high is None:
          _, high, _ = self._rollout(tuple(plus), **identify)
      except ValueError:
        pass
      try:
        low = self._strict_fd_reference(tuple(minus), reference_certificate)
        if low is None and identify:
          low = self._zero_gain_fd_reference(tuple(minus), reference_certificate)
        if low is None:
          _, low, _ = self._rollout(tuple(minus), **identify)
      except ValueError:
        pass
      if high is None and low is None:
        sensitivities.append(dict.fromkeys(baseline, 0.0))
        continue
      if high is None:
        high, plus[index] = baseline, self.reference[index]
      if low is None:
        low, minus[index] = baseline, self.reference[index]
      span = plus[index] - minus[index]
      if set(high) != set(baseline) or set(low) != set(baseline):
        raise ValueError('event emission support changed')
      if span <= 0:
        if plus[index] != self.reference[index] or minus[index] != self.reference[index]:
          raise ValueError('event finite difference lost admitted domain')
        # At a parameter boundary the only successful direction can be the
        # exact neutral trajectory. A rejected nonzero probe remains rejected;
        # this coordinate has ZERO identifiable support, not a fit derivative.
        sensitivities.append(dict.fromkeys(baseline, 0.0))
        continue
      sensitivities.append({stamp: (high[stamp] - low[stamp]) / span for stamp in baseline})
    self.projected = {stamp: (value, tuple(feature[stamp] for feature in sensitivities)) for stamp, value in baseline.items()}
    from types import MappingProxyType
    key = self._causal_input_key(epsilon)
    if key is not None:
      self.__causal_features = key, MappingProxyType(dict(self.projected))
    return len(self.projected)

  def native_feasible(self, coordinates):
    """Child trial-only feasibility, without losses, physical targets or gains.

    This is not a ReplayValidation and never grants application authority.
    Full validate() repeats all guards and measures heldout benefit separately.
    """
    from openpilot.starpilot.flm.event_activation import ZERO_REFERENCE_ACTIVATION_VERSION
    if (self.activation_plan is None or self.activation_plan.version != ZERO_REFERENCE_ACTIVATION_VERSION or
        self.restore_envelope is not None):
      raise ValueError('native feasibility requires declared trial plan')
    try:
      # The strict rollout cache performs full owned-input binding. Do not keep
      # a second, weaker plan/reference-only baseline memo here.
      # A rejected candidate cannot grant feasibility; avoid an unnecessary
      # baseline lookup/hash before that strict candidate rejection.
      trial_requests, proposed, trial_saturated = self._rollout(coordinates)
      base_requests, baseline, base_saturated = self._rollout(self.reference)
      return bool(set(proposed) == set(baseline) and
        not any(trial_saturated[stamp] and not base_saturated[stamp] for stamp in base_requests) and
        self._slew_ok(base_requests, trial_requests, encoded=False) and self._slew_ok(baseline, proposed, encoded=True))
    except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
      return False

  def validate(self, profile, control, output, physical_context, blocks, model):
    from openpilot.starpilot.flm.clipping_quality import RESTORE_FEEDBACK_VERSION
    if self.activation_plan is not None and self.activation_plan.version == RESTORE_FEEDBACK_VERSION:
      from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
      try:
        if getattr(self,'event_mean_model',None) is not None:
          return None
        require_feedback_engine(self)
      except (ValueError,TypeError,KeyError,AttributeError):
        return None
    if (self.failed or profile.policy != (RESTORE_POLICY if self.restore_envelope is not None else POLICY) or
        self.restore_envelope is not None and profile.effect_envelope != self.restore_envelope or profile.identity != self.identity or
        profile.parameters != self.objective.parameters or len(profile.coefficients) != len(self.objective.names) or
        model.identity != self.identity or model.domain != 'normalized_torque' or
        model.context != canonical_physical_context(physical_context) or
        control.identity != self.identity or control.baseline_revision != self.baseline_revision or
        output.limited or output.override or output.fault or not output.active or not output.valid or not output.native_allowed or
        control.limited and not exact_unclipped_pair(control, output) or not self.applied_matches(control, output) or
        set(profile.train_ids) & set(profile.heldout_ids) or len(profile.train_ids) < 4 or len(profile.heldout_ids) < 4):
      return None
    selected = [block for block in blocks if block.block_id in profile.heldout_ids]
    if {block.block_id for block in selected} != set(profile.heldout_ids):
      return None
    try:
      base_requests, baseline, base_saturated = self._rollout(self.reference)
      trial_requests, proposed, trial_saturated = self._rollout(profile.coefficients, restore_target=self.restore_envelope is not None)
      if set(proposed) != set(baseline) or any(trial_saturated[stamp] and not base_saturated[stamp] for stamp in base_requests):
        return None
      if not self._slew_ok(base_requests, trial_requests, encoded=False) or not self._slew_ok(baseline, proposed, encoded=True):
        return None
      improvements = {cohort: [] for cohort in self.objective.required_cohorts}
      used = set()
      for block in selected:
        if block.identity != self.identity or block.cohort not in improvements or len(block.samples) < 40:
          return None
        base_loss = trial_loss = weight = 0.0
        for row in block.samples:
          key = row.control_ns, row.applied_ns
          if (key in used or not row.active or row.override or row.fault or row.saturated or not row.native_allowed or
              not row.physical_valid or not row.stage_valid or row.model_wobble or row.controller_chatter or
              row.applied_ns not in self.projected or row.applied_ns not in baseline):
            return None
          used.add(key)
          target = row.residual_target + self.projected[row.applied_ns][0] - sum(
            a * b for a, b in zip(row.features, self.reference, strict=True))
          base, trial = baseline[row.applied_ns], proposed[row.applied_ns]
          if not all(model.command_low <= -value <= model.command_high for value in (target, base, trial)):
            return None
          base_loss += row.weight * (model.slope * (target - base)) ** 2
          trial_loss += row.weight * (model.slope * (target - trial)) ** 2
          weight += row.weight
        if weight <= 0:
          return None
        improvements[block.cohort].append((base_loss - trial_loss) / weight)
      lower_bounds = []
      for values in improvements.values():
        if len(values) < 2:
          return None
        mean = sum(values) / len(values)
        variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
        lower = mean - 12.707 * math.sqrt(variance / len(values))
        event_mean_model = getattr(self,'event_mean_model',None)
        if event_mean_model is not None:
          from openpilot.starpilot.flm.event_mean_evidence import lower as event_lower
          cohort = next(name for name,items in improvements.items() if items is values)
          lower = event_lower(event_mean_model,tuple(block for block in selected if block.cohort==cohort),tuple(values),cohort=cohort)
        if not math.isfinite(lower) or lower <= 0:
          return None
        lower_bounds.append(lower)
      return ReplayValidation(profile, self.baseline_revision, physical_model_revision(model), True, True,
                              min(lower_bounds), tuple(control.baseline_values))
    except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
      return None

  def _slew_ok(self, baseline, proposed, *, encoded):
    previous = None
    maximum_base = maximum_trial = 0.0
    for stamp in sorted(baseline):
      event = self.emissions.events[stamp] if encoded else None
      control_ns = event.control_ns if encoded else stamp
      window = next((index for index, (a, b) in enumerate(self.replay_windows) if a <= control_ns <= b), None)
      if window is None:
        return False
      if previous is not None and window == previous[3]:
        elapsed = (stamp - previous[0]) / 1e9
        if elapsed <= 0:
          return False
        maximum_base = max(maximum_base, abs(baseline[stamp] - previous[1]) / elapsed)
        maximum_trial = max(maximum_trial, abs(proposed[stamp] - previous[2]) / elapsed)
      previous = stamp, baseline[stamp], proposed[stamp], window
    return maximum_trial <= maximum_base + 1e-6
