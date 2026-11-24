from dataclasses import replace

import pytest

from openpilot.starpilot.aol.tests.test_full_engagement import CP, SETTINGS, observe, state
from openpilot.starpilot.car.hyundai.classic_scc_intent import ClassicSccCardIntent
from openpilot.starpilot.car.hyundai.forte_intent import ForteCardIntent


@pytest.mark.parametrize('main', [0, 9])
def test_new_successful_stock_engagement_arms_after_resolved_fault(main):
  owner = ClassicSccCardIntent(CP, replace(SETTINGS, main_action=main))
  cs = state()
  observe(owner, cs, 1, enabled=False, active=False)
  owner.fault_rearm = True
  observe(owner, cs, 2, active=False)
  assert owner.fault_rearm and not owner.allowed_latch
  observe(owner, cs, 3)
  assert owner.physical_latch and owner.allowed_latch and not owner.fault_rearm
  assert not owner.pause_lateral and owner._last_latch_edge_ns == 3
  cs.cruiseState.enabled = False
  owner.update(cs, now_ns=4, fault_active=False)
  observe(owner, cs, 4, enabled=False, active=False)
  assert owner.output(cs)[0] and not owner.pause_longitudinal


@pytest.mark.parametrize('veto', ['fault', 'rejection', 'invalid', 'preEnabled', 'duplicate', 'calibration',
  'permanent_eps', 'temporary_eps', 'gas', 'brake', 'gear', 'main_off', 'manual_lkas', 'disabled'])
def test_current_veto_never_clears_vehicle_fault_rearm(veto):
  owner = ClassicSccCardIntent(CP, SETTINGS)
  cs = state()
  observe(owner, cs, 10, enabled=False, active=False)
  owner.fault_rearm = True
  if veto == 'calibration':
    owner._calibration_rearm_required = True
  elif veto == 'permanent_eps':
    cs.steerFaultPermanent = True
  elif veto == 'temporary_eps':
    cs.steerFaultTemporary = True
  elif veto == 'gas':
    cs.gasPressed = True
  elif veto == 'brake':
    cs.brakePressed = True
  elif veto == 'gear':
    cs.gearShifter = 'park'
  elif veto == 'main_off':
    cs.cruiseState.available = False
  elif veto == 'manual_lkas':
    from opendbc.car.structs import car
    cs.buttonEvents = [car.CarState.ButtonEvent(type='lkas', pressed=True)]
  elif veto == 'disabled':
    owner.settings = replace(SETTINGS, enabled=False)
  observe(owner, cs, 10 if veto == 'duplicate' else 20, active=veto != 'preEnabled',
          valid=veto != 'invalid', fault=veto == 'fault', rejection=15 if veto == 'rejection' else 0)
  assert owner.fault_rearm and not owner.physical_latch and not owner.allowed_latch


def test_generic_forte_physical_owner_keeps_its_fault_rearm_guard():
  owner = ForteCardIntent(CP, SETTINGS)
  cs = state()
  observe(owner, cs, 1, enabled=False, active=False)
  owner.fault_rearm = True
  observe(owner, cs, 2)
  assert owner.fault_rearm and not owner.allowed_latch
