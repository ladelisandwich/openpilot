"""Route 11177: a removed camera must never receive forwarded or host traffic."""
import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.tests.test_volt_camera_removed import removed_params, feed_removed
from opendbc.car.gm.tests.test_volt_camera_control import camera_params, feed_camera
from opendbc.car.gm.values import CAR, DBC
from opendbc.safety.tests.libsafety import libsafety_py


@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('alternate', [False, True])
def test_removed_camera_controller_uses_only_vehicle_bus(alpha, alternate):
  cp = removed_params(alpha=alpha, alternate=alternate)
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  sent = []
  for tick in range(24):
    now = 1_000_000_000 + tick * 40_000_000
    cs, _ = feed_removed(ci, packer, now, counter=tick % 4, active=False)
    assert cs.canValid and not cs.canTimeout
    cc = structs.CarControl(enabled=False, latActive=False, longActive=False)
    _, frames = ci.apply(cc.as_reader(), now + 2)
    assert all(frame[2] == 0 for frame in frames)
    sent.extend(frames)
  assert any(frame[0] == 0x180 for frame in sent)


@pytest.mark.parametrize('word', [0xC150, 0xC151])
def test_removed_camera_native_has_no_forwarding_or_camera_transmit(word):
  safety = libsafety_py.libsafety
  release = safety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) != 0
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, word) == 0
  safety.init_tests()
  if release and word == 0xC151:
    assert not safety.safety_config_valid()
    return
  for bus in (0, 1, 2):
    for address in range(0x800):
      assert safety.safety_fwd_hook(bus, address) == -1
  packer = CANPacker(DBC[CAR.CHEVROLET_VOLT_CAMERA][Bus.pt])
  frame = packer.make_can_msg('PSCMStatus', 2, {})
  assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))


@pytest.mark.parametrize('alpha', [False, True])
def test_connected_camera_retains_its_status_delivery(alpha):
  cp = camera_params(alpha=alpha)
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  sent = []
  for tick in range(12):
    now = 1_000_000_000 + tick * 40_000_000
    cs, _ = feed_camera(ci, packer, now, counter=tick % 4, active=False)
    assert cs.canValid and not cs.canTimeout
    _, frames = ci.apply(structs.CarControl().as_reader(), now + 2)
    sent.extend(frames)
  assert any(frame[0] == 0x184 and frame[2] == 2 for frame in sent)
