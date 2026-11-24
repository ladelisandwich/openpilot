"""Discard an unchanged repeat without admitting a late actuation sample."""


def redundant_late_output(collector, control, output):
  from openpilot.starpilot.flm.fullrate_collector import JOIN_NS
  from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current
  from opendbc.car.steering_projection import TorqueProjection, projection_to_dict

  live = collector.live_tracking
  if (live is None or getattr(getattr(live, 'episodes', None), 'source_tracking', False) is not True
      or getattr(collector, '_learning_current_native_allowed', False) is not True
      or control.domain != 'normalized_torque' or output.domain != control.domain or not collector.ready):
    return False
  previous_control, previous = collector.ready[-1]
  if (previous_control is not control or not consumed_sources_current(control, previous)
      or not control.state_ns <= control.control_ns <= output.state_ns <= output.apply_ns
      or control.control_ns - control.state_ns > JOIN_NS or output.apply_ns - output.state_ns > JOIN_NS
      or output.apply_ns - control.control_ns <= JOIN_NS
      or not previous.apply_ns < output.apply_ns <= previous.apply_ns + JOIN_NS
      or not control.enabled or not control.active or not control.valid or control.manual or control.override or control.fault
      or control.saturated or control.model_wobble
      or any(not row.valid or not row.active or row.override or row.fault or row.limited or not row.native_allowed
             for row in (previous, output))):
    return False
  if control.native_required and (not control.native_allowed or
      not 0 < control.native_observed_ns <= control.control_ns <= output.apply_ns <= control.native_expires_ns <= control.native_observed_ns + 200_000_000):
    return False
  old, new = previous.projection, output.projection
  if (type(old) is not TorqueProjection or type(new) is not TorqueProjection
      or projection_to_dict(old) is None or projection_to_dict(new) is None
      or output.control_ns != control.control_ns or output.applied != previous.applied
      or new.requested_torque != control.requested or new.reported_torque != output.applied or not new.steer_requested):
    return False
  # Current CAN bytes/counter and native prestate are independently verified
  # below. Every other encoded command/codec field must remain exactly equal.
  changing = {'frame', 'can_data', 'previous_torque', 'measured_torque'}
  if any(getattr(old, name) != getattr(new, name) for name in TorqueProjection._fields if name not in changing):
    return False
  replay = getattr(collector.adapter, 'replay', None)
  if replay is None:
    return False
  return replay.applied_matches(control, output) is True
