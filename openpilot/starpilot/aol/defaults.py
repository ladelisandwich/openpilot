"""Global AOL preference default, separate from runtime authority."""
from pathlib import Path

from openpilot.starpilot.state_migration import _atomic_write, _params_lock, _read_value


def default_enabled(_cp=None) -> bool:
  # A preference default grants no vehicle, native or steering authority.
  return True


def enroll_missing_preference(params, cp) -> bool:
  if not default_enabled(cp):
    return False
  try:
    namespace = Path(params.get_param_path()).absolute()
    with _params_lock(namespace):
      try:
        _read_value(namespace / 'AlwaysOnLateral')
      except FileNotFoundError:
        _atomic_write(namespace / 'AlwaysOnLateral', b'1')
        return True
  except (OSError, ValueError):
    return False
  return False
