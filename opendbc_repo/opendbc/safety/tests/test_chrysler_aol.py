"""Actual Chrysler native permission + unchanged controller/CAN limit regressions."""

import unittest
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, DBC
from opendbc.car.chrysler.aol_contract import CHRYSLER_AOL_CARS
from opendbc.car.chrysler.chryslercan import create_lkas_command
from opendbc.safety.tests.libsafety import libsafety_py


class TestChryslerAolSafety(unittest.TestCase):
  def select(self, candidate=CAR.CHRYSLER_PACIFICA_2018, *, experience=32):
    self.candidate = candidate
    self.ram = candidate == CAR.RAM_1500_5TH_GEN
    self.cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, False)
    if experience == 32 and candidate == CAR.CHRYSLER_PACIFICA_2019_HYBRID:
      self.cp.safetyConfigs[0].safetyParam |= 8
    self.cp.alternativeExperience = experience
    self.ci = CarInterface(self.cp)
    self.ci.update([])
    self.packer = CANPacker(DBC[candidate][Bus.pt])
    self.native = libsafety_py.libsafety
    self.native.init_tests()
    self.native.set_alternative_experience(experience)
    result = self.native.set_safety_hooks(self.cp.safetyConfigs[0].safetyModel.raw, self.cp.safetyConfigs[0].safetyParam)
    if result != 0:
      self.assertTrue(self.cp.dashcamOnly)
      self.assertEqual(result, -1)  # release excludes an existing debug-only model
    self.native.set_aol_test_heartbeat(True)
    self.now = 0

  def packet(self, frame):
    return libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])

  def feed(self, tick, *, active=False, main=True, brake=False, gear=None, temporary=False, permanent=False, cancel=False, omit=(), repeat_gear=False):
    self.now = tick * 10_000_000
    self.native.set_timer(tick * 10_000)
    if gear is None:
      gear = 4
    values = {
      'DAS_3': {'ACC_AVAILABLE': int(main), 'ACC_ACTIVE': int(active)},
      'ESP_1': {'Brake_Pedal_State': int(brake)},
      'EPS_2': {'EPS_TORQUE_MOTOR': self.ci.CC.apply_torque_last, 'LKAS_STATE': 4 if permanent else 1, 'LKAS_TEMPORARY_FAULT': int(temporary)},
      'EPS_3': {'DASM_FAULT': int(temporary)},
      'GEAR': {'PRNDL': gear},
      'Transmission_Status': {'Gear_State': gear},
      'SPEED_1': {'SPEED_LEFT': 20.0, 'SPEED_RIGHT': 20.0},
      'ESP_8': {'Vehicle_Speed': 14.75 * 3.6},
      'CRUISE_BUTTONS': {'ACC_Cancel': int(cancel)},
    }
    frames = []
    for parser in self.ci.can_parsers.values():
      for address in sorted(parser.addresses):
        name = parser.dbc.addr_to_msg[address].name
        if (parser.bus, name) in omit:
          continue
        fields = dict(values.get(name, {}))
        if repeat_gear and name in ('GEAR', 'Transmission_Status'):
          fields['COUNTER'] = 0
        frames.append(self.packer.make_can_msg(name, parser.bus, fields))
    self.ci.update([(self.now, frames)])
    for frame in frames:
      self.native.safety_rx_hook(self.packet(frame))
    self.native.safety_tick()
    return frames

  def arm(self, candidate=CAR.CHRYSLER_PACIFICA_2018):
    self.select(candidate)
    for tick in range(1, 21):
      self.feed(tick)
    self.native.aol_set_host_request(0)
    self.assertEqual(self.native.aol_get_permission_mask(), 0)  # no current driver request
    self.feed(21, active=True)
    self.assertTrue(self.native.get_controls_allowed())
    self.native.aol_set_host_request(1)
    self.assertEqual(self.native.aol_get_permission_mask(), 1)
    self.feed(22, active=False, brake=True)
    self.native.aol_set_host_request(1)
    self.assertFalse(self.native.get_controls_allowed())
    self.assertEqual(self.native.aol_get_permission_mask(), 1)

  def test_all_active_platforms_arm_then_brake_retains_only_native_lateral(self):
    for candidate in CHRYSLER_AOL_CARS:
      with self.subTest(candidate=candidate):
        self.arm(candidate)
        self.assertFalse(self.native.get_controls_allowed())
        self.native.aol_set_host_request(3)
        self.assertEqual(self.native.aol_get_request_mask(), 1)
        self.assertEqual(self.native.aol_get_permission_mask(), 1)
        self.native.aol_set_host_request(2)
        self.assertEqual(self.native.aol_get_permission_mask(), 0)
        self.assertEqual(self.native.aol_get_request_mask(), 0)

  def test_reverse_temporary_fault_and_fresh_drive_never_restore_long(self):
    for candidate in (CAR.CHRYSLER_PACIFICA_2018, CAR.RAM_1500_5TH_GEN):
      self.arm(candidate)
      self.feed(23, gear=1 if self.ram else 2)
      self.native.aol_set_host_request(1)
      self.assertEqual(self.native.aol_get_permission_mask(), 0)
      self.assertFalse(self.native.safety_tx_hook(self.packet(create_lkas_command(self.packer, self.cp, 3, True))))
      self.feed(24, gear=4)
      self.assertEqual(self.native.aol_get_permission_mask(), 1)
      self.feed(25, temporary=True)
      self.assertEqual(self.native.aol_get_permission_mask(), 0)
      self.feed(26, temporary=False)
      self.assertEqual(self.native.aol_get_permission_mask(), 1)
      self.assertFalse(self.native.get_controls_allowed())

  def test_main_cancel_permanent_and_heartbeat_revoke_native_arm(self):
    for reason in ('main', 'cancel', 'permanent', 'heartbeat'):
      self.arm()
      if reason == 'heartbeat':
        self.native.set_aol_test_heartbeat(False)
      else:
        self.feed(23, **{'main': False} if reason == 'main' else {reason: True})
      self.assertEqual(self.native.aol_get_permission_mask(), 0, reason)
      self.native.set_aol_test_heartbeat(True)
      self.feed(24, main=reason != 'main')
      self.native.aol_set_host_request(0 if reason == 'heartbeat' else 1)
      self.assertEqual(self.native.aol_get_permission_mask(), 0, reason)
      if reason == 'main':
        # Non-Hybrid default MAIN mode admits a new physical MAIN cycle.
        # This does not excuse a still-current cancel/permanent-fault inhibit.
        self.feed(25, main=True)
        self.assertEqual(self.native.aol_get_permission_mask(), 1)


  def test_repeated_gear_missing_eps_wrong_bus_source_and_host_expiry_deny(self):
    for candidate in (CAR.CHRYSLER_PACIFICA_2018, CAR.RAM_1500_5TH_GEN):
      for reason in ('gear', 'repeat', 'eps', 'lease'):
        self.arm(candidate)
        gear_name = 'Transmission_Status' if self.ram else 'GEAR'
        eps_name = 'EPS_3' if self.ram else 'EPS_2'
        for tick in range(23, 57):
          self.feed(tick, omit=((0, gear_name),) if reason == 'gear' else ((0, eps_name),) if reason == 'eps' else (), repeat_gear=reason == 'repeat')
          if reason != 'lease':
            self.native.aol_set_host_request(1)
        self.assertEqual(self.native.aol_get_permission_mask(), 0, (candidate, reason))
        # A wrong-bus gear frame cannot renew the missing physical source.
        if reason == 'gear':
          frame = self.packer.make_can_msg(gear_name, 1, {'Gear_State' if self.ram else 'PRNDL': 4})
          self.native.safety_rx_hook(self.packet(frame))
          self.assertEqual(self.native.aol_get_permission_mask(), 0)

  def test_actual_selected_controller_ramp_keeps_both_native_limits(self):
    for candidate in CHRYSLER_AOL_CARS:
      self.arm(candidate)
      command = structs.CarControl(latActive=True)
      command.actuators.torque = 1.0
      self.ci.CC.frame = 202
      positive = []
      for tick in range(23, 180):
        self.feed(tick)
        self.native.aol_set_host_request(1)
        _, frames = self.ci.apply(command.as_reader(), self.now)
        for frame in frames:
          self.assertTrue(self.native.safety_tx_hook(self.packet(frame)), (candidate, tick, frame))
        if self.ci.CC.apply_torque_last:
          positive.append(tick)
      self.assertTrue(positive, candidate)
      self.assertLessEqual(abs(self.ci.CC.apply_torque_last), self.ci.CC.params.STEER_MAX)

  def test_off_and_dashcam_profiles_do_not_negotiate(self):
    for candidate, experience in ((CAR.CHRYSLER_PACIFICA_2018, 0), (CAR.RAM_1500_5TH_GEN, 0), (CAR.RAM_HD_5TH_GEN, 32), (CAR.JEEP_CHEROKEE_5TH_GEN, 32)):
      self.select(candidate, experience=experience)
      self.native.aol_set_host_request(3)
      self.assertEqual(self.native.aol_get_request_mask(), 0)
      self.assertEqual(self.native.aol_get_permission_mask(), 0)

  def test_ram_optional_crc_and_counter_denials(self):
    self.arm(CAR.RAM_1500_5TH_GEN)
    for name in ('Transmission_Status', 'EPS_3'):
      address, data, bus = self.packer.make_can_msg(name, 0, {'Gear_State': 4} if name == 'Transmission_Status' else {})
      invalid = bytearray(data)
      invalid[-1] ^= 1
      self.native.safety_rx_hook(self.packet((address, bytes(invalid), bus)))
      self.assertEqual(self.native.aol_get_permission_mask(), 0)
      self.feed(23)

  def test_main_following_stock_words_and_hybrid_native_set_guard(self):
    for candidate in CHRYSLER_AOL_CARS:
      self.select(candidate)
      for tick in range(1, 21):
        self.feed(tick)
      self.native.aol_set_host_request(1)
      self.assertEqual(self.native.aol_get_permission_mask(), 0 if candidate == CAR.CHRYSLER_PACIFICA_2019_HYBRID else 1)
      self.assertFalse(self.native.get_controls_allowed())
    # The Hybrid's real current MAIN gesture cannot substitute for its first SET.
    self.select(CAR.CHRYSLER_PACIFICA_2019_HYBRID)
    for tick in range(1, 21):
      self.feed(tick)
    button = self.packer.make_can_msg('CRUISE_BUTTONS', 0, {'ACC_OnOff': 1})
    self.native.safety_rx_hook(self.packet(button))
    self.native.aol_set_host_request(1)
    self.assertEqual(self.native.aol_get_permission_mask(), 0)

  def test_allocated_guard_word_with_wrong_marker_cannot_fall_back_to_stock(self):
    self.select(CAR.CHRYSLER_PACIFICA_2019_HYBRID, experience=0)
    self.assertEqual(self.native.set_safety_hooks(9, 8), 0)
    frame = create_lkas_command(self.packer, self.cp, 0, False)
    self.assertFalse(self.native.safety_tx_hook(self.packet(frame)))

  def test_cancel_with_still_active_stock_requires_a_fresh_set_epoch(self):
    for candidate in CHRYSLER_AOL_CARS:
      with self.subTest(candidate=candidate):
        self.select(candidate)
        for tick in range(1, 21):
          self.feed(tick)
        self.feed(21, active=True)
        self.native.aol_set_host_request(1)
        self.assertEqual(self.native.aol_get_permission_mask(), 1)
        self.feed(22, active=True, cancel=True)
        self.assertEqual(self.native.aol_get_permission_mask(), 0)
        self.feed(23, active=True)
        self.native.aol_set_host_request(1)
        self.assertEqual(self.native.aol_get_permission_mask(), 0)
        self.feed(24, active=False)
        self.feed(25, active=True)
        self.native.aol_set_host_request(1)
        self.assertEqual(self.native.aol_get_permission_mask(), 1)
        self.feed(26, active=False, brake=True)
        self.assertEqual(self.native.aol_get_permission_mask(), 1)
        self.assertFalse(self.native.get_controls_allowed())

  def test_malformed_physical_buttons_require_fresh_engagement(self):
    for candidate in CHRYSLER_AOL_CARS:
      for fault in ('checksum', 'short', 'long'):
        with self.subTest(candidate=candidate, fault=fault):
          self.arm(candidate)
          address, data, bus = self.packer.make_can_msg('CRUISE_BUTTONS', 0, {})
          if fault == 'checksum':
            invalid = bytearray(data)
            invalid[-1] ^= 1
            data = bytes(invalid)
          elif fault == 'short':
            data = data[:2]
          else:
            data += b'\x00'
          self.native.safety_rx_hook(self.packet((address, data, bus)))
          self.assertEqual(self.native.aol_get_permission_mask(), 0)
          self.assertFalse(self.native.safety_tx_hook(self.packet(create_lkas_command(self.packer, self.cp, 3, True))))
          # Fresh healthy source traffic alone cannot undo the physical inhibit.
          self.feed(23)
          self.native.aol_set_host_request(1)
          self.assertEqual(self.native.aol_get_permission_mask(), 0)
          self.assertFalse(self.native.get_controls_allowed())
          self.feed(24, active=True)
          self.native.aol_set_host_request(1)
          self.assertEqual(self.native.aol_get_permission_mask(), 1)
          self.feed(25, active=False, brake=True)
          self.assertEqual(self.native.aol_get_permission_mask(), 1)
          self.assertFalse(self.native.get_controls_allowed())
