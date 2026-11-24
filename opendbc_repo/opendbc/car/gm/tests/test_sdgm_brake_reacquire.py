"""SDGM route26: reacquire EPS assistance after a measured brake pause."""
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.tests.test_volt_camera_control import feed_camera
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.gm.values import DBC


def frame(ci, packer, tick, *, brake=False, status=1):
  now = 1_000_000_000 + tick * 10_000_000
  _, rows = feed_camera(SimpleNamespace(update=lambda _: None), packer, now,
                       counter=tick % 4, brake=brake, active=False, eps_status=status)
  rows = [row for row in rows if row[0] != 0xBE and (tick % 10 == 0 or row[0] != 0x184)]
  rows.append(packer.make_can_msg('ECMAcceleratorPos', 0, {'BrakePedalPos': 30 if brake else 0}))
  cs = ci.update([(now, rows)])
  control = structs.CarControl(enabled=False, latActive=True, longActive=False)
  control.actuators.torque = .2
  control.hudControl.setSpeed = 30.
  _, sends = ci.apply(control.as_reader(), now)
  steering = [data for address, data, bus in sends if address == 0x180 and bus == 0]
  return cs, steering


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('recovered', [False, True])
def test_sdgm_brake_pause_emits_neutral_then_reacquires_or_keeps_refusal(c9, recovered):
  cp = sdgm_params(brake_c9=c9)
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  for tick in range(40):
    frame(ci, packer, tick)
  assert ci.CS.steering_authority.seen_active
  emitted = []
  for tick in range(40, 140):
    cs, steering = frame(ci, packer, tick, brake=True, status=0)
    emitted += steering
    assert cs.brakePressed
    assert not ci.CS.steering_authority.latched
  assert emitted and all(not (data[0] & 8) and data[:2] in (b'\x00\x00', b'\x10\x00', b'\x20\x00', b'\x30\x00') for data in emitted)
  emitted = []
  for tick in range(140, 185):
    cs, steering = frame(ci, packer, tick, status=1 if recovered and tick >= 150 else 0)
    emitted += steering
  assert any(data[0] & 8 for data in emitted)
  assert ci.CS.steering_authority.latched == (not recovered)
  assert cs.steerFaultTemporary == (not recovered)
  if recovered:
    assert ci.CS.steering_authority.seen_active


@pytest.mark.parametrize('status', [1, 2, 3])
def test_sdgm_active_feedback_and_factory_faults_are_not_brake_recovery(status):
  cp = sdgm_params()
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  emitted = []
  for tick in range(40):
    cs, steering = frame(ci, packer, tick, brake=True, status=status)
    emitted += steering
  assert any(data[0] & 8 for data in emitted)
  assert cs.steerFaultTemporary == (status == 2)
  assert cs.steerFaultPermanent == (status == 3)


@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('brake', [False, True])
def test_sdgm_brake_recovery_does_not_enable_unengaged_stock_control(alpha, brake):
  cp = sdgm_params(alpha=alpha)
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  emitted = []
  for tick in range(20):
    _, steering = frame(ci, packer, tick, brake=brake, status=0)
    if tick >= 10:
      emitted += steering
  assert emitted
  # The stock ACC fixture is disengaged; this recovery never enables it.
  assert all(bool(data[0] & 8) == (alpha and not brake) for data in emitted)
