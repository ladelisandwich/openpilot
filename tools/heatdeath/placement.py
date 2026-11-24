"""Affinity changes confined to the benchmark's owned processes."""

import json
import os
from pathlib import Path

NAMES = {'card', 'controlsd', 'selfdrived', 'radard', 'plannerd', 'peer', 'observer'}


def load_plan(value):
  plan = json.loads(value)
  if not isinstance(plan, dict) or not set(plan) <= NAMES:
    raise ValueError('Placement must map known actor names to CPU lists')
  for cpus in plan.values():
    if not isinstance(cpus, list) or not cpus or any(type(cpu) is not int or not 0 <= cpu < 8 for cpu in cpus):
      raise ValueError('Each placement needs CPU integers from 0 through 7')
    if len(set(cpus)) != len(cpus):
      raise ValueError('Duplicate CPUs in placement')
  return plan


def apply(name):
  plan = load_plan(os.environ.get('HEATDEATH_PLACEMENT', '{}'))
  if name in plan:
    os.sched_setaffinity(0, set(plan[name]))
    if os.sched_getaffinity(0) != set(plan[name]):
      raise RuntimeError('Requested affinity was not applied')
  return sorted(os.sched_getaffinity(0))


def parked():
  for name, expected in (('IsOffroad', b'1'), ('IsEngaged', b'0')):
    if (Path('/data/params/d') / name).read_bytes().strip() != expected:
      raise RuntimeError('Heatdeath requires an offroad, disengaged bench device')
