from openpilot.starpilot.flm.normalized_trace_archive import guarded_archive
from openpilot.starpilot.flm.normalized_trace_archive import trace_for
"""Fresh original/CAN probe of the exact FIRST full30 application transition.

Heldout whole-trajectory fitting is separate. This never manufactures future
vehicle states or extends a source/native/profile lease.
"""
from itertools import islice
import math

from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256
from openpilot.starpilot.flm.controller_replay import _run, final_command, restore_snapshot, same_final_command
from openpilot.starpilot.flm.effect_trace import step_full
from openpilot.starpilot.flm.selected_effects import EffectReplaySource, EffectReplayRecordedSource
from openpilot.starpilot.flm.canonical_surface_adapter import CanonicalSurfaceEffects


@guarded_archive()
def current_effect_probe(engine, profile, control, output, model):
  try:
    envelope = profile.effect_envelope.validate(profile)
    if (engine.failed or not engine.raw_mode or engine.baseline_revision != envelope.selected_baseline_revision or
        envelope.source_epoch != control.source_epoch or control.identity != profile.identity or
        not engine._frame_context(next(frame for frame in reversed(engine.records) if frame.control_ns == control.control_ns)) or
        control.effect_prepare_ns <= 0 or control.override or control.fault or control.manual or not control.valid or
        not control.active or not control.enabled or output.override or output.fault or output.limited or
        not output.active or not output.valid or not output.native_allowed or not engine.applied_matches(control, output)):
      return False
    frame=next((row for row in islice(reversed(engine.records),60) if row.control_ns==control.control_ns),None)
    if frame is None or frame.effect_prepare_ns != control.effect_prepare_ns or not 0 <= engine.last_ns-frame.control_ns <= 500_000_000:
      return False
    trace=trace_for(frame,engine)
    before=dict.fromkeys(NAMES,0.) if trace is None else dict(trace.applied)
    prior=dict(envelope.prior_coordinates)
    # A proposal is based on the actual CURRENT accepted target, not a saved
    # history map and not the previous proposal's pre-change prior.
    accepted=dict.fromkeys(NAMES,0.) if trace is None else dict(trace.target)
    renewing = bool(trace is not None and trace.envelope == envelope and
      accepted == dict(envelope.coordinates))
    if (not renewing and accepted != prior or
        trace is not None and trace.envelope.source_binding()!=envelope.source_binding()):
      return False
    if (not renewing and full_profile_sha256(envelope.selected_baseline_revision,accepted)!=envelope.prior_full_profile_sha256):
      return False
    state=engine._current_raw_state(control.control_ns)
    if state is None:
      return False
    baseline,bvm=engine.factory.create()
    proposed,pvm=engine.factory.create()
    restore_snapshot(baseline,bvm,state)
    restore_snapshot(proposed,pvm,state)
    recorded=None if trace is None else EffectReplayRecordedSource(trace)
    base,base_log=_run(baseline,bvm,frame.inputs,engine.objective,(0.,0.),frame.torque_values,
      actual_stage_residual=frame.actual_stage_residual,effect_trial=recorded)
    if not same_final_command(base,frame.selected_command):
      return False
    # Exact already-current envelope renewal reconstructs the actual current
    # transition, preserving its original dt. Only a NEW proposal creates a new
    # owner with the deliberately fixed .01s first tick.
    values = dict(trace.applied) if renewing else step_full(before,dict(envelope.coordinates),
      now_ns=control.effect_prepare_ns,previous_ns=0,qualified=True)
    source=EffectReplaySource.__new__(EffectReplaySource)
    source.source=CanonicalSurfaceEffects({n:values[n] for n in envelope.trial_names},
      envelope.selected_baseline_revision,accumulated=values)
    source.original_effect_anchor=source.source.original_effect_anchor
    source.applied_coordinates=source.source.applied_coordinates
    candidate,candidate_log=_run(proposed,pvm,frame.inputs,engine.objective,(0.,0.),frame.torque_values,
      actual_stage_residual=frame.actual_stage_residual,effect_trial=source)
    base,candidate=final_command(base),final_command(candidate)
    if (not all(math.isfinite(v) and abs(v)<=1.000001 for v in (base,candidate)) or
        candidate_log.saturated and not base_log.saturated):
      return False
    event=engine.emissions.events.get(output.apply_ns)
    if event is None or event.control_ns!=control.control_ns:
      return False
    encoded_base=engine.emissions.one_apply(event.apply_ns,base)
    encoded_trial=engine.emissions.one_apply(event.apply_ns,candidate)
    return all(model.command_low<=-v<=model.command_high for v in (encoded_base,encoded_trial))
  except (StopIteration,AttributeError,KeyError,TypeError,ValueError,OverflowError):
    return False
