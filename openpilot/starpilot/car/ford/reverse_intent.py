"""Ford qualification for the shared physical Reverse/native join."""

from opendbc.car.ford.aol import qualified
from openpilot.cereal import log
from openpilot.starpilot.aol.reverse_intent import retain_reverse_intent as retain_qualified_reverse_intent


def retain_reverse_intent(feedback, sm, cp, cs, axis, now_ns, *, latched, producer):
  try:
    qualified_cp = qualified(cp, marked_only=True)
  except (AttributeError, KeyError, IndexError, TypeError, ValueError, OverflowError):
    return False
  return retain_qualified_reverse_intent(feedback, sm, cp, cs, axis, now_ns, latched=latched, producer=producer,
                                         qualified_cp=qualified_cp,
                                         # The preceding brake USER_DISABLE can arrive with the first Reverse frame.
                                         # Retain only the existing fresh LAT-only proof; Reverse still denies axes.
                                         extra_user_events=(log.OnroadEvent.EventName.pedalPressed,))
