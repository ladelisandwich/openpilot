"""A saved OFF remains OFF; an absent legacy Alpha key keeps upstream MEB defaults."""
from openpilot.starpilot.saved_source import read_saved


def stock_long_requested(params):
  try:
    raw, readable = read_saved(params, 'AlphaLongitudinalEnabled', 8)
    return not readable or raw not in (None, b'1')
  except (OSError, TypeError, ValueError):
    return True
