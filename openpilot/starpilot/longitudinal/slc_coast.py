"""Lead evidence that keeps cruise comfort tuning out of braking situations."""

import math


def braking_lead_relevant(lead: object, v_ego: float) -> bool:
  if not getattr(lead, 'present', False):
    return False
  try:
    lead_speed = float(lead.vLead)
    lead_accel = float(lead.aLeadK)
    distance = float(lead.dRel)
  except (AttributeError, TypeError, ValueError, OverflowError):
    return True
  if not all(math.isfinite(value) for value in (v_ego, lead_speed, lead_accel, distance)):
    return True
  return v_ego - lead_speed > 0.5 or lead_accel < -0.4 or distance < max(18.0, 2.0 * v_ego)
