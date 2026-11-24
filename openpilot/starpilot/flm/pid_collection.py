"""Explicit event-only adapter for existing torque-actuated stock PID laws."""
from openpilot.starpilot.flm.pid_controller_replay import PID_STAGE_REVISION, PidControllerFactory
from openpilot.starpilot.flm.event_collection import EventCollectionAdapter


class PidEventCollectionAdapter(EventCollectionAdapter):
  command_family = 'torque_pid'
  raw_stage_revision = PID_STAGE_REVISION
  factory_type = PidControllerFactory

  def select_objective(self, key):
    if getattr(self, '_event_initialized', False):
      from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
      from openpilot.starpilot.flm.pid_stages import SUPPORTED
      if key[1] not in GROUPS or not set(GROUPS[key[1]]).issubset(SUPPORTED):
        raise ValueError('PID canonical component absent from actual controller')
    return super().select_objective(key)


def event_keys():
  from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS
  from openpilot.starpilot.flm.pid_stages import SUPPORTED
  return tuple(('event', group) for group, names in GROUPS.items() if set(names).issubset(SUPPORTED))


def event_key(evidence):
  keys = event_keys()
  frame = int(dict(evidence.replay_inputs)['control_frame'])
  return keys[(frame // 6000) % len(keys)]
