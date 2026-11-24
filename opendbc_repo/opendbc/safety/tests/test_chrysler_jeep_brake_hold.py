"""Exact opt-in Jeep native retention, physical permission and copied payload."""

import unittest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.chrysler.brake_hold import create_das_3_command
from opendbc.safety.tests.libsafety import libsafety_py


class TestJeepBrakeHoldSafety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety

  def select(self, word=4):
    self.safety.init_tests()
    self.safety.set_alternative_experience(0)
    self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.chrysler, word), 0)
    self.packer = CANPacker('chrysler_pacifica_2017_hybrid_generated')
    self.stock = {
      'ACC_AVAILABLE': 1,
      'ACC_ACTIVE': 1,
      'ACC_STANDSTILL': 1,
      'ACC_DECEL': -2.0,
      'ENGINE_TORQUE_REQUEST': 13.25,
      'DISABLE_FUEL_SHUTOFF': 1,
      'COLLISION_BRK_PREP': 1,
      'COUNTER': 0,
    }

  def packet(self, frame):
    return libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])

  def gear(self, value):
    return self.packer.make_can_msg('GEAR', 0, {'PRNDL': value})

  def feed(self, tick, *, active=True, main=True, gas=0.0, brake=0, speed=0.0, fault=0, gear=4, omit_gear=False):
    self.safety.set_timer(tick * 10_000)
    self.stock.update(ACC_ACTIVE=int(active), ACC_AVAILABLE=int(main), ACC_FAULTED=fault, COUNTER=tick % 16)
    if not omit_gear:
      self.safety.safety_rx_hook(self.packet(self.gear(gear)))
    for name, values in [
      ('EPS_2', {}),
      ('ESP_1', {'Brake_Pedal_State': brake}),
      ('SPEED_1', {'SPEED_LEFT': speed, 'SPEED_RIGHT': speed}),
      ('ECM_5', {'Accelerator_Position': gas}),
      ('DAS_3', self.stock),
    ]:
      self.assertTrue(self.safety.safety_rx_hook(self.packet(self.packer.make_can_msg(name, 0, values))))
    self.safety.safety_tick()

  def ready(self):
    self.select()
    for tick in range(20):
      self.feed(tick)
    self.assertTrue(self.safety.get_controls_allowed())
    self.feed(20, active=False)
    self.assertTrue(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed())

  def hold(self, offset=2, decel=-2.0):
    return create_das_3_command(self.packer, offset, decel, self.stock)

  def test_physical_acc_braking_arms_retention_and_disengaged_resume(self):
    self.ready()
    for offset in (2, 3):
      self.assertTrue(self.safety.safety_tx_hook(self.packet(self.hold(offset))))
    button = self.packer.make_can_msg('CRUISE_BUTTONS', 0, {'ACC_Resume': 1})
    self.assertTrue(self.safety.safety_tx_hook(self.packet(button)))
    for name in ('ACC_Cancel', 'ACC_Resume'):
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.packer.make_can_msg('CRUISE_BUTTONS', 0, {name: 1, 'ACC_Distance_Dec': 1}))))

  def test_driver_main_stale_and_unarmed_withdrawal(self):
    for change in ('gas', 'brake', 'speed', 'main', 'stale', 'unarmed'):
      self.ready()
      if change == 'stale':
        self.safety.set_timer(1_200_001)
        self.safety.safety_tick()
      elif change == 'unarmed':
        self.select()
        self.feed(20, active=False)
      else:
        self.feed(21, active=False, **{change: {'gas': 1.0, 'brake': 1, 'speed': 1.0, 'main': False}[change]})
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())), change)
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.packer.make_can_msg('CRUISE_BUTTONS', 0, {'ACC_Resume': 1}))), change)

  def test_decel_counter_checksum_bus_length_and_copied_field_denials(self):
    self.ready()
    for decel in (-3.0, -0.5):
      self.assertTrue(self.safety.safety_tx_hook(self.packet(self.hold(decel=decel))))
    for decel in (-3.01, -0.49, 0.0, 1.0):
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold(decel=decel))))
    for offset in (0, 1, 4):
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold(offset))))
    address, data, bus = self.hold()
    for invalid_bus, invalid_data in ((1, data), (2, data), (0, data[:7]), (0, data + b'0000')):
      self.assertFalse(self.safety.safety_tx_hook(self.packet((address, invalid_data, invalid_bus))))
    for signal in ('ACC_GO', 'ACC_STANDSTILL', 'ENGINE_TORQUE_REQUEST_MAX', 'ACC_BRK_PREP'):
      fields = self.stock.copy()
      fields.update(
        ACC_AVAILABLE=1,
        ACC_ACTIVE=1,
        ACC_GO=0,
        ACC_STANDSTILL=0,
        ACC_DECEL_REQ=1,
        ACC_BRK_PREP=0,
        ENGINE_TORQUE_REQUEST_MAX=0,
        GR_MAX_REQ=2,
        COUNTER=(fields['COUNTER'] + 2) % 16,
      )
      fields[signal] = 1
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.packer.make_can_msg('DAS_3', 0, fields))), signal)
    changed = bytearray(data)
    changed[-1] ^= 1
    self.assertFalse(self.safety.safety_tx_hook(self.packet((address, bytes(changed), bus))))

  def test_permission_word_and_lifecycle_isolation(self):
    for word in (0, 1, 2, 3, 5, 6, 7, 8, 12):
      self.select(word)
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())), word)
    self.ready()
    self.select()
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))

  def test_physical_cancel_and_crc_health_revoke_retention(self):
    self.ready()
    button = self.packer.make_can_msg('CRUISE_BUTTONS', 0, {'ACC_Cancel': 1, 'ACC_Distance_Dec': 1})
    self.safety.safety_rx_hook(self.packet(button))
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
    for tick in range(21, 25):
      self.feed(tick, active=False)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
    for tick in range(25, 28):
      self.feed(tick, active=True)
    self.feed(28, active=False)
    self.assertTrue(self.safety.safety_tx_hook(self.packet(self.hold())))
    address, data, bus = self.packer.make_can_msg('DAS_3', 0, self.stock)
    invalid = bytearray(data)
    invalid[-1] ^= 1
    self.assertFalse(self.safety.safety_rx_hook(self.packet((address, bytes(invalid), bus))))
    self.safety.safety_tick()
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))

  def test_crc_valid_inactive_acc_fault_releases_hold_until_reengagement(self):
    for fault in (1, 2, 3):
      with self.subTest(fault=fault):
        self.ready()
        self.assertTrue(self.safety.safety_tx_hook(self.packet(self.hold())))
        self.feed(21, active=False, fault=fault)
        self.assertTrue(self.safety.safety_config_valid())
        self.assertFalse(self.safety.get_controls_allowed())
        resume = self.packer.make_can_msg('CRUISE_BUTTONS', 0, {'ACC_Resume': 1})
        self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
        self.assertFalse(self.safety.safety_tx_hook(self.packet(resume)))
        self.feed(22, active=False, fault=0)
        self.assertTrue(self.safety.safety_config_valid())
        self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
        self.assertFalse(self.safety.safety_tx_hook(self.packet(resume)))
        self.feed(23, active=True, fault=0)
        self.assertTrue(self.safety.get_controls_allowed())
        self.feed(24, active=False, fault=0)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertTrue(self.safety.safety_tx_hook(self.packet(self.hold())))
        self.assertTrue(self.safety.safety_tx_hook(self.packet(resume)))

  def test_native_reverse_pause_and_fresh_drive_reengagement(self):
    self.ready()
    self.feed(21, active=False, gear=2)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
    self.feed(22, active=False, gear=4)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
    self.feed(23, active=True, gear=4)
    self.feed(24, active=False, gear=4)
    self.assertTrue(self.safety.safety_tx_hook(self.packet(self.hold())))

  def test_native_gear_and_das_source_expire_independently(self):
    self.ready()
    for tick in range(21, 33):
      self.feed(tick, active=False, omit_gear=True)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
    self.ready()
    self.safety.set_timer(300_001)
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))

  def test_native_bad_gear_counter_wrong_bus_and_commanded_cancel(self):
    self.ready()
    address, data, bus = self.gear(1)
    self.safety.safety_rx_hook(self.packet((address, data, 1)))
    self.assertTrue(self.safety.safety_tx_hook(self.packet(self.hold())))
    invalid = bytearray(data)
    invalid[3] ^= 0x20
    self.safety.safety_rx_hook(self.packet((address, bytes(invalid), bus)))
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
    self.ready()
    cancel = self.packer.make_can_msg('CRUISE_BUTTONS', 0, {'ACC_Cancel': 1})
    self.assertTrue(self.safety.safety_tx_hook(self.packet(cancel)))
    self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))

  def test_repeated_invalid_gear_does_not_renew_permission(self):
    self.ready()
    address, data, bus = self.gear(4)
    self.safety.safety_rx_hook(self.packet((address, data, bus)))
    # Same counter is invalid, including when repeated after invalidation.
    for _ in range(3):
      self.safety.safety_rx_hook(self.packet((address, data, bus)))
      self.assertFalse(self.safety.safety_tx_hook(self.packet(self.hold())))
