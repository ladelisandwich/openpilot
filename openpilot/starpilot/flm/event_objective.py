"""Exact canonical Dom control groups for bounded automatic effect trials."""
from dataclasses import dataclass
import re

from openpilot.starpilot.flm.canonical_surface_adapter import GROUPS, canonical_names, parameters_for


@dataclass(frozen=True)
class EffectObjective:
  group: str
  names: tuple[str, ...]
  anchor_digest: str
  context_key: str

  def __post_init__(self):
    if (self.group not in GROUPS or type(self.names) is not tuple or canonical_names(self.names) != self.names or
        not set(self.names).issubset(GROUPS[self.group]) or
        any(type(value) is not str or re.fullmatch('[a-f0-9]{64}', value) is None
            for value in (self.anchor_digest, self.context_key))):
      raise ValueError('canonical event group/anchor/context')

  @property
  def parameters(self):
    return parameters_for(self.names)

  @property
  def retains_controller_chatter(self):
    # Quiet-goal command reversals are the symptom for these exact canonical
    # feedback/friction groups. This permits estimation only; unchanged raw
    # provenance, independent physical model and full native heldout replay
    # still decide whether a beneficial, identifiable trial actually exists.
    return self.group in ('center', 'deadband.low', 'deadband.high', 'friction.left', 'friction.right')

  @property
  def required_cohorts(self):
    # Whole physical episodes, never the former phase-sliced FF cohorts.
    side = 'left' if self.group.endswith('.left') else 'right' if self.group.endswith('.right') else 'mixed'
    return (f'event.{self.context_key}.{side}',)
