import pytest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.toyota.tests import test_pedal_caller_native as helpers

source_matched_native = helpers.source_matched_native


class Physical:
  def __init__(self, word, experience=32):
    self.native = libsafety_py.libsafety
    self.native.init_tests()
    self.native.set_alternative_experience(experience)
    assert self.native.set_safety_hooks(structs.CarParams.SafetyModel.toyota, word) == 0
    self.native.set_aol_test_heartbeat(True)
    self.packer = CANPacker('toyota_nodsu_pt_generated')
    self.now = 1_000_000
    for _ in range(20):
      self.tick()

  def tick(self, *, lka=1, lta=1, bad=None, skip_main=False):
    self.now += 10_000
    self.native.set_timer(self.now)
    frames = helpers.physical(self.packer, cruise=False)
    frames = [f for f in frames if f[0] not in (0x260, 0x262) and not (skip_main and f[0] == 0x1D3)]
    frames += [self.packer.make_can_msg('STEER_TORQUE_SENSOR', 0, {'STEER_ANGLE': .0573}),
               self.packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': lka, 'LTA_STATE': lta})]
    for address, data, bus in frames:
      if address == 0x262 and bad == 'checksum':
        data = data[:-1] + bytes([data[-1] ^ 1])
      elif address == 0x262 and bad == 'length':
        data = data[:-1]
      self.native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
    self.native.safety_tick()
    self.native.aol_set_host_request(3)

  def angle(self):
    address, data, bus = self.packer.make_can_msg('STEERING_LTA', 0,
      {'STEER_REQUEST': 1, 'STEER_REQUEST_2': 1, 'STEER_ANGLE_CMD': .0573, 'TORQUE_WIND_DOWN': 100})
    return libsafety_py.make_CANPacket(address, bus, data)


@pytest.mark.parametrize('word', [0x449, 0x649, 0x1449, 0x1649])
@pytest.mark.parametrize('fault', ['lta', 'lta_zero', 'lka', 'checksum', 'length', 'main_expiry'])
def test_angle_native_lateral_only_and_fault_withdrawal(word, fault):
  case = Physical(word)
  try:
    assert not case.native.get_controls_allowed()
    assert case.native.aol_get_permission_mask() == 1
    assert case.native.safety_tx_hook(case.angle())
    address, data, bus = case.packer.make_can_msg('ACC_CONTROL', 0, {'ACCEL_CMD': .8})
    assert not case.native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
    if fault == 'main_expiry':
      for _ in range(34):
        case.tick(skip_main=True)
    else:
      case.tick(lta=0 if fault == 'lta_zero' else 9 if fault == 'lta' else 1, lka=9 if fault == 'lka' else 1,
                bad=fault if fault in ('checksum', 'length') else None)
    assert case.native.aol_get_permission_mask() == 0
    assert not case.native.safety_tx_hook(case.angle())
    for _ in range(20):
      case.tick()
    assert case.native.aol_get_permission_mask() == 1
    assert not case.native.get_controls_allowed()
  finally:
    case.native.set_alternative_experience(0)
    case.native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)


@pytest.mark.parametrize('word', [0x448, 0x44A, 0x648, 0x64A, 0x1448, 0x144A, 0x1648, 0x164A])
def test_angle_adjacent_profiles_fail_closed(word):
  case = Physical(word)
  try:
    assert case.native.aol_get_permission_mask() == 0
    assert not case.native.safety_tx_hook(case.angle())
  finally:
    case.native.set_alternative_experience(0)
    case.native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
