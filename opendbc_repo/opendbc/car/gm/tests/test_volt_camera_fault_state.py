"""A missing-camera ACC fault must not masquerade as engaged cruise."""
import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.gm import gmcan
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.tests.test_volt_camera_removed import feed_removed, removed_params
from opendbc.car.gm.values import DBC
from opendbc.safety.tests.libsafety import libsafety_py


@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('alternate', [False, True])
@pytest.mark.parametrize('state', [0, 1, 2, 3, 4])
def test_removed_camera_cruise_state_keeps_real_fault(alpha, alternate, state):
  cp = removed_params(alpha=alpha, alternate=alternate)
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  for tick in range(20):
    now = 1_000_000_000 + tick * 40_000_000
    feed_removed(ci, packer, now, counter=tick % 4)
    status = packer.make_can_msg('AcceleratorPedal2', 0, {'CruiseState': state})
    out = ci.update([(now + 2, [status])])
    assert out.canValid
    assert out.accFaulted == (state == 3)
    assert out.cruiseState.enabled == (state in (1, 2, 4))
    assert out.cruiseState.standstill == (state == 4)


@pytest.mark.parametrize('active', [1, 2, 4])
def test_stock_removed_camera_fault_withdraws_native_cruise_and_cancel(active):
  cp = removed_params(alpha=False)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, 0xC150) == 0
  safety.init_tests()
  safety.set_timer(1_000_000)

  def rx(name, values):
    address, data, bus = packer.make_can_msg(name, 0, values)
    assert safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))

  def cancel(counter):
    address, data, bus = gmcan.create_buttons(packer, 0, counter, 6)
    return safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))

  rx('ECMEngineStatus', {'CruiseMainOn': 1})
  rx('AcceleratorPedal2', {'CruiseState': active})
  rx('ASCMSteeringButton', {'ACCAlwaysOne': 1, 'ACCButtons': 1, 'RollingCounter': 0,
                           'SteeringButtonChecksum': 0xFF})
  assert safety.get_controls_allowed()
  assert cancel(0)
  rx('AcceleratorPedal2', {'CruiseState': 3})
  assert not safety.get_controls_allowed()
  rx('ASCMSteeringButton', {'ACCAlwaysOne': 1, 'ACCButtons': 1, 'RollingCounter': 1,
                           'SteeringButtonChecksum': 0xFF + 0x4EF})
  assert not cancel(1)
  # A subsequent physical active state can recover; a fault is not a latch exemption.
  rx('AcceleratorPedal2', {'CruiseState': active})
  assert safety.get_controls_allowed()
  rx('ASCMSteeringButton', {'ACCAlwaysOne': 1, 'ACCButtons': 1, 'RollingCounter': 2,
                           'SteeringButtonChecksum': 0xFF + 2 * 0x4EF})
  safety.set_timer(1_040_001)
  assert cancel(2)
