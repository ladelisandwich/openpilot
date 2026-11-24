"""Only normal gateway MQB auto/manual/direct OP-long; native DEBUG grant is independent of lateral intent."""
from opendbc.car.volkswagen.stock_aol import _qualified

WORDS = frozenset((9, 25, 41))


def qualified(cp, *, marked_only=False):
  try:
    return bool(_qualified(cp, marked_only=marked_only, longitudinal=True))
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
    return False
