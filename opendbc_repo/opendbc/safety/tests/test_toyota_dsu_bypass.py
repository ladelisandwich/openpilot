"""Actual camera ACC, physical MAIN/EPS and native TX/forwarding ownership."""

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.toyota import dsu_bypass
from opendbc.car.toyota.values import CAR, DBC
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers

source_matched_native = helpers.source_matched_native
REPRESENTATIVES = (CAR.TOYOTA_RAV4H, CAR.LEXUS_ES, CAR.TOYOTA_CAMRY, CAR.TOYOTA_PRIUS, CAR.TOYOTA_COROLLA, CAR.TOYOTA_PRIUS_V, CAR.LEXUS_IS, CAR.LEXUS_GS_F)


class Sources:
  def __init__(self, candidate, stock=False):
    self.candidate, self.stock = candidate, stock
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_alternative_experience(32)
    assert self.safety.set_safety_hooks(structs.CarParams.SafetyModel.toyota, dsu_bypass.PROFILES[candidate] | (0x200 if stock else 0)) == 0
    self.safety.set_aol_test_heartbeat(True)
    self.packer = CANPacker(DBC[candidate][Bus.pt])
    self.now = 1_000_000
    for _ in range(30):
      self.tick()

  def rx(self, frame):
    a, d, b = frame
    return bool(self.safety.safety_rx_hook(libsafety_py.make_CANPacket(a, b, d)))

  def tick(self, fault=None, cruise=True):
    self.now += 10_000
    self.safety.set_timer(self.now)
    fields = {
      'cruise': cruise,
      'sdsu': None,
      'main': fault != 'main-off',
      'eps': 9 if fault == 'eps-state' else 1,
      'gear': 16 if fault == 'reverse' else 0,
      'gas': fault == 'gas',
      'brake': fault == 'brake',
    }
    frames = helpers.physical(self.packer, **fields)
    if self.candidate in dsu_bypass.ALT_MAIN_CARS:
      frames += [self.packer.make_can_msg('DSU_CRUISE', 0, {'MAIN_ON': fields['main'], 'SET_SPEED': 90})]
    frames += [
      self.packer.make_can_msg('ACC_CONTROL', 2, {'ACC_TYPE': 1, 'ACC_MALFUNCTION': fault == 'acc-fault'}),
      self.packer.make_can_msg('PRE_COLLISION', 2, {'PRECOLLISION_ACTIVE': fault == 'aeb', 'FORCE': -20 if fault == 'aeb' else 0}),
    ]
    selected = (
      0x343
      if fault and fault.startswith('acc-')
      else (0x365 if self.candidate in dsu_bypass.ALT_MAIN_CARS else 0x1D3)
      if fault and fault.startswith('main-')
      else 0x262
    )
    for a, d, b in frames:
      if fault and a == selected and b == (2 if selected == 0x343 else 0):
        if fault.endswith('-drop'):
          continue
        if fault.endswith('-dlc'):
          d = d[:-1]
        elif fault.endswith('-crc'):
          d = d[:-1] + bytes([d[-1] ^ 1])
      accepted = self.rx((a, d, b))
      if fault and fault.endswith('-crc') and a == selected and b == (2 if selected == 0x343 else 0):
        assert not accepted
      else:
        assert accepted, (hex(a), d.hex(), b)
    if fault == 'mixed-pedal':
      self.rx((0x201, bytes(6), 0))
    self.safety.safety_tick()
    self.safety.aol_set_host_request(3)

  def accel(self, value):
    return self.rx_tx('ACC_CONTROL', {'ACCEL_CMD': value, 'PERMIT_BRAKING': 1, 'RELEASE_STANDSTILL': value != 0})

  def rx_tx(self, name, fields):
    a, d, b = self.packer.make_can_msg(name, 0, fields)
    return bool(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d)))

  def close(self):
    self.safety.set_alternative_experience(0)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize('stock', (False, True))
def test_exact_native_health_permission_accel_and_forwarding(candidate, stock):
  src = Sources(candidate, stock)
  try:
    assert src.safety.safety_config_valid()
    assert src.safety.aol_get_permission_mask() == (1 if stock else 3)
    assert src.accel(0.05) == (not stock)
    assert src.safety.safety_fwd_hook(2, 0x343) == (0 if stock else -1)
    assert src.safety.safety_fwd_hook(2, 0x4CB) == 0
    assert src.safety.safety_fwd_hook(2, 0x283) == 0
  finally:
    src.close()


@pytest.mark.parametrize('candidate', REPRESENTATIVES)
@pytest.mark.parametrize(
  'fault',
  ('acc-drop', 'acc-dlc', 'acc-crc', 'acc-fault', 'main-off', 'eps-drop', 'eps-dlc', 'eps-crc', 'eps-state', 'reverse', 'gas', 'brake', 'aeb', 'mixed-pedal'),
)
def test_independent_sources_and_physical_vetoes(candidate, fault):
  src = Sources(candidate)
  try:
    assert src.accel(0.05)
    for _ in range(41):  # Strictly exceed the existing 400ms EPS lease.
      src.tick(fault)
    assert not src.accel(0.05), fault
    if fault != 'mixed-pedal':
      for _ in range(30):
        src.tick(cruise=False)
      for _ in range(30):
        src.tick()
      assert src.accel(0.05), fault
    else:
      src.tick(cruise=False)
      src.tick()
      assert not src.accel(0.05)
  finally:
    src.close()
