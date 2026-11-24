"""Recorded SDGM route18: a steering request is not proof of EPS assistance."""
import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.tests import test_ascm_intercept as ascm_fixture
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.gm.values import DBC


@pytest.mark.parametrize("c9", [False, True])
def test_sdgm_long_refused_steering_is_not_silently_active(c9):
  cp = sdgm_params(alpha=True, brake_c9=c9)
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  case = ascm_fixture.TestAscmIntercept(methodName="runTest")
  for tick in range(40):
    cs = case.authority_frame(ci, packer, tick, status=1)
  for tick in range(40, 81):
    cs = case.authority_frame(ci, packer, tick, status=0)
  assert ci.CS.steering_authority.latched
  assert cs.steerFaultTemporary


@pytest.mark.parametrize("alpha", [False, True])
@pytest.mark.parametrize("status,active,speed,driver", [(1, True, 8., 0.), (0, False, 8., 0.),
  (0, True, 0., 0.), (0, True, 8., 3.), (2, True, 8., 0.), (3, True, 8., 0.)])
def test_sdgm_monitor_preserves_deliberate_pauses_and_factory_faults(alpha, status, active, speed, driver):
  cp = sdgm_params(alpha=alpha)
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  case = ascm_fixture.TestAscmIntercept(methodName="runTest")
  for tick in range(60):
    cs = case.authority_frame(ci, packer, tick, status=status, active=active and alpha, speed=speed, driver=driver)
  assert not ci.CS.steering_authority.latched
  assert cs.steerFaultTemporary == (status == 2)
  assert cs.steerFaultPermanent == (status == 3)


@pytest.mark.parametrize("c9", [False, True])
def test_sdgm_refusal_requires_physical_disable_neutral_and_fresh_rearm(monkeypatch, c9):
  # Reuse the full original parser/controller physical sequence for this exact factory CP.
  monkeypatch.setattr(ascm_fixture, "params", lambda *args, **kwargs: sdgm_params(brake_c9=c9, radar=True))
  ascm_fixture.TestAscmIntercept(methodName="runTest").test_volt_steering_loss_requires_driver_rearm()
