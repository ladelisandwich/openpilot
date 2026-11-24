"""Version5 fixed-map support: original information budget over whole events."""
import math

PHYSICAL_EVENT_POINTS = 40
EVENT_CREDIT_CAP = 40
REQUIRED_EFFECTFUL_EVENTS = 2
HALF_CREDITS = REQUIRED_EFFECTFUL_EVENTS * EVENT_CREDIT_CAP


def capped_group_credits(events):
  """No outcome/gain input. Preserve all original event/endpoint identities.

  Each row is (whole_event_id, ((control_ns, apply_ns, encoded_delta), ...)).
  This counts the original native signal floor; it never creates a model or
  grants replay qualification. Cap prevents one event filling another's budget.
  """
  ids, endpoints, support_credits = set(), set(), []
  previous_control = 0
  for event_id, points in events:
    if type(event_id) is not str or not event_id or event_id in ids or len(points) < PHYSICAL_EVENT_POINTS:
      raise ValueError('distinct supported whole physical events required')
    ids.add(event_id)
    changed = 0
    for control_ns, apply_ns, delta in points:
      key = control_ns, apply_ns
      if (type(control_ns) is not int or type(apply_ns) is not int or not 0 < control_ns <= apply_ns or
          control_ns <= previous_control or key in endpoints or
          type(delta) not in (int, float) or not math.isfinite(delta) or delta < 0):
        raise ValueError('unique chronological supported native endpoints required')
      endpoints.add(key)
      previous_control = control_ns
      changed += delta > 1e-8
    support_credits.append(min(EVENT_CREDIT_CAP, changed))
  return tuple(support_credits)


def independent_half_supported(support_credits):
  if any(type(value) is not int or not 0 <= value <= EVENT_CREDIT_CAP for value in support_credits):
    raise ValueError('bounded whole-event support support_credits required')
  return sum(support_credits) >= HALF_CREDITS and sum(value > 0 for value in support_credits) >= REQUIRED_EFFECTFUL_EVENTS


def all_event_lower(gains, lower):
  """Original conservative estimator on EVERY original event, without selection."""
  if len(gains) < 2 or any(type(value) not in (int, float) or not math.isfinite(value) for value in gains):
    return None
  bound = lower(tuple(gains))
  return bound if math.isfinite(bound) and bound > 0 else None
