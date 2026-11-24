"""Original consumed vehicle states; no actuation or physical permission."""

from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, JOIN_NS, MAX_AGE_NS


def state_brackets(collector, identity, now_ns):
  states = {}

  def add(control):
    if (control.identity, control.generation, control.source_epoch, control.baseline_revision,
        control.controller_revision, control.native_identity) != identity:
      return
    values = dict(control.replay_inputs)
    if 'vEgo' in values and 'steeringAngleDeg' in values:
      row = (float(values['vEgo']), float(values['steeringAngleDeg']))
      if control.state_ns in states and states[control.state_ns] != row:
        raise ValueError('conflicting original vehicle state brackets')
      states[control.state_ns] = row

  for control, _ in collector.ready:
    add(control)
  # These controls already passed collector admission. Their independently
  # recorded CarState can bracket a pose before Card emits their command.
  # The observer still requires its original joined native pair; these rows
  # supply only speed/angle, and never enter ready or renew any witness.
  for control in collector.controls.values():
    if (type(control) is not ControlEvidence or control.domain != collector.ready[-1][0].domain
        or control.native_required is not collector.ready[-1][0].native_required
        or not control.enabled or not control.active or not control.valid
        or control.manual or control.override or control.fault or control.saturated
        or not 0 < control.state_ns <= control.control_ns <= now_ns
        or control.control_ns - control.state_ns > JOIN_NS or now_ns - control.control_ns > MAX_AGE_NS):
      continue
    if control.native_required and (not control.native_allowed or
        not 0 < control.native_observed_ns <= control.control_ns <= control.native_expires_ns <= control.native_observed_ns + 200_000_000):
      continue
    add(control)
  return states
