"""Actual MEB native, CAN-FD CRC generations, stock/off and independent axes."""
import unittest
import pytest
from opendbc.car import structs
from opendbc.can.dbc import DBC
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py


class _MebOwner:
  def __init__(self, gen2, stock, alt_gear):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_alternative_experience(32)
    self.word = 8 | (2 if gen2 else 0) | (4 if stock else 0) | (16 if alt_gear else 0)
    self.safety.set_safety_hooks(34, self.word)
    self.safety.set_aol_test_heartbeat(True)
    self.packer = CANPackerSafety('vw_meb_2024_generated' if gen2 else 'vw_meb_generated')
    self.dbc = DBC(self.packer.dbc_name) if hasattr(self.packer, 'dbc_name') else DBC('vw_meb_2024_generated' if gen2 else 'vw_meb_generated')
    self.time = 1_000_000
    self.stock, self.alt_gear = stock, alt_gear

  def packet(self, name, fields, bus=0):
    return self.packer.make_can_msg_safety(name, bus, fields)

  def feed(self, engaged=False, brake=False, gear=8, eps=2, main=True, set_button=False, cancel=False, omit=None, moving=True):
    self.time += 10_000
    self.safety.set_timer(self.time)
    data = [('Gateway_73' if self.alt_gear else 'Getriebe_11', {'GE_Fahrstufe': gear}),
            ('LH_EPS_03', {'EPS_Lenkmoment': 0}), ('Motor_14', {'MO_Fahrer_bremst': int(brake)}),
            ('GRA_ACC_01', {'GRA_Tip_Setzen': int(set_button), 'GRA_Abbrechen': int(cancel)}),
            ('QFK_01', {'LatCon_HCA_Status': eps, 'Curvature': 0}),
            ('ESP_21', {'ESP_v_Signal': 72 if moving else 0}),
            ('ESC_51', {name + '_Radgeschw': 72 if moving else 0 for name in ('VL', 'VR', 'HL', 'HR')}),
            ('Motor_51', {'TSK_Status': 3 if engaged else 2 if main else 0, 'Accel_Pedal_Pressure': 0})]
    for name, fields in data:
      if name != omit:
        self.safety.safety_rx_hook(self.packet(name, fields))
    self.safety.safety_tick()
    self.safety.aol_set_host_request(3)

  def close(self):
    self.safety.init_tests()
    self.safety.set_alternative_experience(0)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)


@pytest.mark.parametrize('gen2', (False, True))
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('alt_gear', (False, True))
def test_actual_meb_native_full_engagement_brake_reverse_and_long_off(gen2, stock, alt_gear):
  owner = _MebOwner(gen2, stock, alt_gear)
  try:
    for _ in range(15):
      owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1  # Current cold MAIN arms LAT only.
    for address in (0x14D, 0x300, 0x26B):
      assert owner.safety.safety_fwd_hook(2, address) == (0 if stock else -1)
    if stock:
      owner.feed(engaged=True)
    else:
      owner.feed(set_button=True)
      owner.feed(set_button=False)
    assert owner.safety.aol_get_permission_mask() == (1 if stock else 3)
    assert owner.safety.safety_tx_hook(owner.packet('HCA_03', {'Curvature': 0, 'Power': 4, 'RequestStatus': 4, 'HighSendRate': 1}))
    active = {'ACC_Sollbeschleunigung_02': .5, 'ACC_Status_ACC': 3, 'ACC_Anforderung_HMS': 0}
    assert bool(owner.safety.safety_tx_hook(owner.packet('ACC_18', active))) == (not stock)
    owner.feed(brake=True)
    assert owner.safety.aol_get_permission_mask() == 1
    assert not owner.safety.get_controls_allowed()
    assert not owner.safety.safety_tx_hook(owner.packet('ACC_18', active))
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    for gear in (6, 7):
      owner.feed(gear=gear)
      assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    owner.feed(eps=7)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    owner.feed(eps=3)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
def test_meb_gear_crc_counter_and_wrong_bus_withdraw(gen2):
  owner = _MebOwner(gen2, False, False)
  try:
    for _ in range(15):
      owner.feed()
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
    frame = owner.packet('Getriebe_11', {'GE_Fahrstufe': 8})
    frame[0].data[0] ^= 255
    owner.safety.safety_rx_hook(frame)
    assert owner.safety.aol_get_permission_mask() == 0
    for _ in range(40):
      owner.feed(omit='Getriebe_11')
      owner.safety.safety_rx_hook(owner.packet('Getriebe_11', {'GE_Fahrstufe': 8}, 1))
    assert owner.safety.aol_get_permission_mask() == 0
    owner.safety.set_aol_test_heartbeat(False)
    assert owner.safety.aol_get_permission_mask() == 0
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
@pytest.mark.parametrize('stock', (False, True))
def test_meb_current_main_never_grants_long_or_recovers_long_automatically(gen2, stock):
  owner = _MebOwner(gen2, stock, False)
  try:
    for _ in range(12):
      owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    for active in (
      {'ACC_Sollbeschleunigung_02': .5, 'ACC_Status_ACC': 3},
      {'ACC_Sollbeschleunigung_02': 0, 'ACC_Status_ACC': 4},
      {'ACC_Sollbeschleunigung_02': 3.01, 'ACC_Anforderung_HMS': 1},
      {'ACC_Sollbeschleunigung_02': 3.01, 'ACC_Anforderung_HMS': 4},
      {'ACC_Sollbeschleunigung_02': 3.01, 'ACC_Anfahren': 1},
      {'ACC_Sollbeschleunigung_02': 3.01, 'ACC_Anhalten': 1},
    ):
      assert not owner.safety.safety_tx_hook(owner.packet('ACC_18', active))
    owner.feed(set_button=True, engaged=stock)
    owner.feed(engaged=stock)
    assert owner.safety.aol_get_permission_mask() == (1 if stock else 3)
    owner.feed(gear=6)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    owner.feed(eps=3)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed(set_button=True, engaged=stock)
    owner.feed(engaged=stock)
    assert owner.safety.aol_get_permission_mask() == (1 if stock else 3)
    owner.feed(cancel=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
@pytest.mark.parametrize('alt_gear', (False, True))
@pytest.mark.parametrize('source', ('LH_EPS_03', 'Motor_14', 'GRA_ACC_01', 'QFK_01', 'ESP_21', 'ESC_51', 'Motor_51', 'gear'))
def test_meb_each_required_source_lease_withdraws_without_rearming(gen2, alt_gear, source):
  owner = _MebOwner(gen2, False, alt_gear)
  try:
    for _ in range(12):
      owner.feed()
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
    omitted = ('Gateway_73' if alt_gear else 'Getriebe_11') if source == 'gear' else source
    for _ in range(120):
      owner.feed(omit=omitted)
    assert owner.safety.aol_get_permission_mask() == 0
    for _ in range(12):
      owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
def test_meb_permanent_fault_held_button_requires_fresh_press(gen2):
  owner = _MebOwner(gen2, False, False)
  try:
    for _ in range(12):
      owner.feed()
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
    owner.feed(eps=3, set_button=True)
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
    owner.safety.set_aol_test_heartbeat(False)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.safety.set_aol_test_heartbeat(True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
def test_meb_main_off_and_counter_replay_cannot_restore_long(gen2):
  owner = _MebOwner(gen2, False, False)
  try:
    for _ in range(12):
      owner.feed()
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
    owner.feed(main=False)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    frame = owner.packet('Getriebe_11', {'GE_Fahrstufe': 8})
    owner.safety.safety_rx_hook(frame)
    owner.safety.safety_rx_hook(frame)
    assert owner.safety.aol_get_permission_mask() == 0
    for _ in range(12):
      owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
@pytest.mark.parametrize('initial_eps', (1, 3, 6))
def test_meb_initial_eps_unavailability_and_reverse_disabled_follow_real_parser_semantics(gen2, initial_eps):
  owner = _MebOwner(gen2, False, False)
  try:
    for _ in range(15):
      owner.feed(eps=initial_eps)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    owner.feed(gear=6, eps=0)
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 1
    owner.feed(eps=0)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0  # DISABLED in Drive is permanent.
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
def test_meb_fault_after_original_initialization_window_cannot_rearm_from_held_main(gen2):
  owner = _MebOwner(gen2, False, False)
  try:
    for _ in range(650):
      owner.feed(eps=3)
      assert owner.safety.aol_get_permission_mask() == 0
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 0
    owner.feed(set_button=True)
    owner.feed()
    assert owner.safety.aol_get_permission_mask() == 3
  finally:
    owner.close()


@pytest.mark.parametrize('gen2', (False, True))
def test_meb_stop_hold_resume_requires_current_ordinary_long_grant(gen2):
  owner = _MebOwner(gen2, False, False)
  try:
    for _ in range(12):
      owner.feed()
    owner.feed(set_button=True)
    owner.feed()
    owner.feed(moving=False)
    assert owner.safety.aol_get_permission_mask() == 3
    for hold in (1, 4):
      frame = owner.packet('ACC_18', {'ACC_Sollbeschleunigung_02': 3.01, 'ACC_Status_ACC': 3,
                                     'ACC_Anforderung_HMS': hold})
      assert owner.safety.safety_tx_hook(frame)
    assert not owner.safety.safety_tx_hook(owner.packet('ACC_18', {'ACC_Sollbeschleunigung_02': 3.01,
                                                               'ACC_Anforderung_HMS': 7}))
    owner.feed(brake=True, moving=False)
    owner.feed(moving=False)
    assert owner.safety.aol_get_permission_mask() == 1
    for hold in (1, 4):
      assert not owner.safety.safety_tx_hook(owner.packet('ACC_18', {'ACC_Sollbeschleunigung_02': 3.01,
                                                                  'ACC_Status_ACC': 3, 'ACC_Anforderung_HMS': hold}))
    assert owner.safety.safety_tx_hook(owner.packet('ACC_18', {'ACC_Sollbeschleunigung_02': 3.01,
                                                            'ACC_Status_ACC': 2, 'ACC_Anforderung_HMS': 5}))
  finally:
    owner.close()


class TestMebNativeUnittest(unittest.TestCase):
  """Execute the same physical-source assertions in the native unittest coverage gate."""
  def test_all_native_profiles(self):
    for gen2 in (False, True):
      for stock in (False, True):
        for alt in (False, True):
          test_actual_meb_native_full_engagement_brake_reverse_and_long_off(gen2, stock, alt)
        test_meb_current_main_never_grants_long_or_recovers_long_automatically(gen2, stock)
      test_meb_gear_crc_counter_and_wrong_bus_withdraw(gen2)
      test_meb_permanent_fault_held_button_requires_fresh_press(gen2)
      test_meb_main_off_and_counter_replay_cannot_restore_long(gen2)
      test_meb_stop_hold_resume_requires_current_ordinary_long_grant(gen2)
      test_meb_fault_after_original_initialization_window_cannot_rearm_from_held_main(gen2)
      for initial_eps in (1, 3, 6):
        test_meb_initial_eps_unavailability_and_reverse_disabled_follow_real_parser_semantics(gen2, initial_eps)

  def test_all_native_source_leases(self):
    for gen2 in (False, True):
      for alt in (False, True):
        for source in ('LH_EPS_03', 'Motor_14', 'GRA_ACC_01', 'QFK_01', 'ESP_21', 'ESC_51', 'Motor_51', 'gear'):
          test_meb_each_required_source_lease_withdraws_without_rearming(gen2, alt, source)

  def test_invalid_profile_and_unarmed_set_resume_are_denied(self):
    safety = libsafety_py.libsafety
    try:
      for experience, word in ((0, 8), (32, 4), (0, 0x8000)):
        safety.init_tests()
        safety.set_alternative_experience(experience)
        safety.set_safety_hooks(structs.CarParams.SafetyModel.volkswagenMeb, word)
        self.assertFalse(safety.safety_tx_hook(libsafety_py.make_CANPacket(0x12B, 0, bytes(8))))
      for gen2 in (False, True):
        owner = _MebOwner(gen2, True, False)
        for _ in range(15):
          owner.feed()
        self.assertFalse(safety.get_controls_allowed())
        self.assertFalse(safety.safety_tx_hook(owner.packet('GRA_ACC_01', {'GRA_Tip_Setzen': 1})))
        self.assertTrue(safety.safety_tx_hook(owner.packet('GRA_ACC_01', {'GRA_Abbrechen': 1})))
        owner.close()
    finally:
      safety.init_tests()
      safety.set_alternative_experience(0)
      safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
