"""Stock SCC buttons follow the configured mapping, independently of PCM MAIN."""
from dataclasses import replace

import pytest

from opendbc.car.structs import car
from openpilot.starpilot.aol.tests.test_forte_intent import params, settings, state
from openpilot.starpilot.car.hyundai.classic_scc_intent import ClassicSccCardIntent
from opendbc.car.hyundai.values import CAR

Button = car.CarState.ButtonEvent.Type


@pytest.mark.parametrize('button', [Button.mainCruise, Button.lkas])
@pytest.mark.parametrize('available', [False, True])
def test_mapped_buttons_toggle_once_without_requiring_stock_main(button, available):
  owner = ClassicSccCardIntent(params(CAR.HYUNDAI_SONATA_HYBRID), settings(main=9))
  def update(events=(), **kwargs):
    owner.update(state(main=available, events=events), fault_active=False, **kwargs)
  update(now_ns=1)
  assert not owner.allowed_latch
  update(((button, True),), now_ns=2)
  assert owner.allowed_latch and not owner.pause_lateral
  update(((button, True),), now_ns=3)
  assert owner.allowed_latch and not owner.pause_lateral
  update(((button, False),), now_ns=4)
  update(((button, True),), now_ns=5)
  assert not owner.allowed_latch and owner.pause_lateral


@pytest.mark.parametrize('button', [Button.mainCruise, Button.lkas])
def test_fault_and_disabled_settings_do_not_arm_mapped_buttons(button):
  configured = settings(main=9)
  for disabled in (False, True):
    owner = ClassicSccCardIntent(params(CAR.HYUNDAI_SONATA_HYBRID), replace(configured, enabled=not disabled))
    owner.update(state(), fault_active=False, now_ns=1)
    owner.update(state(events=((button, True),)), fault_active=not disabled, now_ns=2)
    assert not owner.allowed_latch
