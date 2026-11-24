import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.carstate import CarState
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, DBC, SubaruSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py


CARS = (CAR.SUBARU_FORESTER_2022,)


class TestSubaruForesterAngle(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety

  @staticmethod
  def packet(frame):
    return libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])

  @staticmethod
  def params(car, *, alpha=False, release=False):
    return CarInterface.get_params(car, {0: {}, 1: {}, 2: {}}, [], alpha, release, False)

  def mode(self, cp, *, extra_flags=0):
    cfg = cp.safetyConfigs[0]
    self.assertEqual(self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam | extra_flags), 0)
    self.safety.init_tests()
    self.safety.set_timer(1_000_000)

  @staticmethod
  def sources(car, *, brake=False, cruise=True, speed=30, main=True, torque=0):
    packer = CANPacker(DBC[car][Bus.pt])
    inputs: tuple[tuple[str, int, dict[str, float]], ...] = (
      ('Throttle', 0, {}),
      ('Steering_Torque', 0, {'Steer_Torque_Sensor': torque}),
      ('Steering_2', 0, {'Steering_Angle': 0}),
      ('Wheel_Speeds', 0, {'FL': speed, 'FR': speed, 'RL': speed, 'RR': speed}),
      ('Brake_Status', 0, {'Brake': int(brake)}),
      ('ES_Status', 2, {'Cruise_Activated': int(cruise)}),
      ('ES_DashStatus', 2, {'Cruise_On': int(main)}),
      ('Transmission', 0, {'Gear': 121}),
      ('Dashlights', 0, {}),
      ('BodyInfo', 0, {}),
      ('ES_Distance', 2, {}),
      ('ES_LKAS_State', 2, {}),
      ('ES_Brake', 2, {}),
    )
    return packer, [packer.make_can_msg(name, bus, values) for name, bus, values in inputs]

  def controller(self, car, cp, frames, *, active=True):
    state = CarState(cp)
    parsers = state.get_can_parsers(cp)
    state.update(parsers)
    for parser in parsers.values():
      parser.update([(1_000_000_000, frames)])
    state.out = state.update(parsers).as_reader()
    self.assertEqual(state.out.gearShifter, structs.CarState.GearShifter.drive)
    self.assertGreater(state.out.vEgoRaw, 0)

    command = structs.CarControl()
    command.enabled = True
    command.latActive = active
    command.actuators.steeringAngleDeg = 1.0
    _, sends = CarController(DBC[car], cp).update(command.as_reader(), state, 1_100_000_000)
    steer = next(frame for frame in sends if frame[0] == 0x124)
    self.assertEqual(steer[2], 0)
    self.assertEqual(bool(steer[1][1] & 0x10), active and state.out.cruiseState.available and
                     state.out.cruiseState.enabled and not state.out.brakePressed)
    return state, steer, sends

  def arm(self, cp, frames):
    self.mode(cp)
    required = {0x40, 0x119, 0x11A, 0x13A, 0x13C, 0x222, 0x321}
    for frame in frames:
      if frame[0] in required:
        self.assertTrue(self.safety.safety_rx_hook(self.packet(frame)), hex(frame[0]))
    self.safety.safety_tick()
    self.assertTrue(self.safety.safety_config_valid())
    self.assertTrue(self.safety.get_controls_allowed())

  def test_stock_packed_host_to_native_and_inactive(self):
    for car in CARS:
      for release in (False, True):
        with self.subTest(car=car, release=release):
          cp = self.params(car, alpha=True, release=release)
          self.assertFalse(cp.dashcamOnly)
          self.assertFalse(cp.openpilotLongitudinalControl)
          self.assertFalse(cp.alphaLongitudinalAvailable)
          self.assertEqual(cp.steerControlType, structs.CarParams.SteerControlType.angle)
          packer, frames = self.sources(car)
          state, steer, sends = self.controller(car, cp, frames)
          self.assertTrue(state.out.cruiseState.enabled)
          self.assertNotEqual(steer[1][5:7], b'\x00\x00')
          self.arm(cp, frames)
          self.assertTrue(self.safety.safety_tx_hook(self.packet(steer)))
          self.assertEqual({frame[0] for frame in sends}, {0x124, 0x321, 0x322})
          self.assertFalse(self.safety.safety_tx_hook(self.packet((0x122, bytes(8), 0))))
          self.assertFalse(self.safety.safety_tx_hook(self.packet((0x124, steer[1], 1))))
          self.assertFalse(self.safety.safety_tx_hook(self.packet((0x124, steer[1][:-1], 0))))
          self.assertFalse(self.safety.safety_tx_hook(self.packet(packer.make_can_msg('ES_Distance', 2, {}))))
          cancel = structs.CarControl()
          cancel.cruiseControl.cancel = True
          _, cancel_sends = CarController(DBC[car], cp).update(cancel.as_reader(), state, 1_100_000_000)
          distance = next(frame for frame in cancel_sends if frame[0] == 0x221)
          self.assertEqual(distance[2], 0)
          self.assertTrue(self.safety.safety_tx_hook(self.packet(distance)))

          _, inactive, _ = self.controller(car, cp, frames, active=False)
          self.assertTrue(self.safety.safety_tx_hook(self.packet(inactive)))
          brake_frames = [packer.make_can_msg('Brake_Status', 0, {'Brake': 1}) if f[0] == 0x13C else f for f in frames]
          brake_state, brake_steer, _ = self.controller(car, cp, brake_frames)
          self.assertTrue(brake_state.out.brakePressed)
          self.assertFalse(brake_steer[1][1] & 0x10)
          self.assertTrue(self.safety.safety_rx_hook(self.packet(brake_frames[4])))
          self.assertFalse(self.safety.get_controls_allowed())
          self.assertTrue(self.safety.safety_tx_hook(self.packet(brake_steer)))
          self.assertFalse(self.safety.safety_tx_hook(self.packet(steer)))

  def test_required_health_and_raw_long_conflict(self):
    for car in CARS:
      cp = self.params(car)
      _, frames = self.sources(car)
      required = [f for f in frames if f[0] in {0x40, 0x119, 0x11A, 0x13A, 0x13C, 0x222, 0x321}]
      for missing in required:
        with self.subTest(car=car, missing=hex(missing[0])):
          self.mode(cp)
          for frame in required:
            if frame is not missing:
              self.safety.safety_rx_hook(self.packet(frame))
          self.safety.safety_tick()
          self.assertFalse(self.safety.safety_config_valid())
      self.arm(cp, frames)
      _, steer, _ = self.controller(car, cp, frames)
      invalid_option = 0x200 if car == CAR.SUBARU_LEGACY_2025 else 0x100
      for flags in (SubaruSafetyFlags.LONG, 0x20, invalid_option):
        with self.subTest(car=car, flags=flags):
          self.mode(cp, extra_flags=flags)
          for frame in required:
            self.safety.safety_rx_hook(self.packet(frame))
          self.safety.safety_tick()
          self.assertFalse(self.safety.safety_tx_hook(self.packet(steer)))
      for raw_param in (0x50, 0x30, 0x31, 0x111):
        with self.subTest(car=car, raw_param=hex(raw_param)):
          self.assertEqual(self.safety.set_safety_hooks(structs.CarParams.SafetyModel.subaru, raw_param), 0)
          self.safety.init_tests()
          for frame in required:
            self.safety.safety_rx_hook(self.packet(frame))
          self.safety.safety_tick()
          self.assertFalse(self.safety.safety_tx_hook(self.packet(steer)))
      self.mode(cp)
      for frame in required:
        if frame[0] == 0x222:
          frame = (frame[0], frame[1][:-1] + bytes([frame[1][-1] ^ 1]), frame[2])
        self.safety.safety_rx_hook(self.packet(frame))
      self.safety.safety_tick()
      self.assertFalse(self.safety.safety_config_valid())

      for source in required:
        for bad in ((source[0], source[1], (source[2] + 1) % 3),
                    (source[0], source[1][:-1], source[2])):
          with self.subTest(car=car, source=hex(source[0]), bad=bad[2:]):
            self.mode(cp)
            for frame in required:
              self.safety.safety_rx_hook(self.packet(bad if frame is source else frame))
            self.safety.safety_tick()
            self.assertFalse(self.safety.safety_config_valid())

      self.arm(cp, frames)
      self.safety.set_timer(10_000_000)
      self.safety.safety_tick()
      self.assertFalse(self.safety.safety_config_valid())

      self.mode(cp)
      for frame in required:
        self.safety.safety_rx_hook(self.packet(frame))
      for _ in range(6):
        self.safety.safety_rx_hook(self.packet(next(f for f in required if f[0] == 0x222)))
      self.safety.safety_tick()
      self.assertFalse(self.safety.safety_config_valid())

  def test_stock_main_driver_and_cruise_handoff(self):
    for car in CARS:
      cp = self.params(car)
      for override in ({'main': False}, {'cruise': False}, {'brake': True}, {'torque': 250}):
        with self.subTest(car=car, override=override):
          _, frames = self.sources(car, **override)
          self.mode(cp)
          for frame in frames:
            self.safety.safety_rx_hook(self.packet(frame))
          state, steer, _ = self.controller(car, cp, frames)
          if override.get('torque'):
            # The host deliberately waits for a second steering period before
            # cutting a request on persistent driver input.
            command = structs.CarControl()
            command.enabled = command.latActive = True
            command.actuators.steeringAngleDeg = 1.0
            controller = CarController(DBC[car], cp)
            controller.update(command.as_reader(), state, 1_100_000_000)
            controller.update(command.as_reader(), state, 1_110_000_000)
            _, sends = controller.update(command.as_reader(), state, 1_120_000_000)
            steer = next(frame for frame in sends if frame[0] == 0x124)
          self.assertFalse(steer[1][1] & 0x10)
          self.assertTrue(self.safety.safety_tx_hook(self.packet(steer)))

      _, frames = self.sources(car)
      self.arm(cp, frames)
      _, steer, _ = self.controller(car, cp, frames)
      self.assertTrue(self.safety.safety_tx_hook(self.packet(steer)))
      _, main_off = self.sources(car, main=False)
      dash = next(frame for frame in main_off if frame[0] == 0x321)
      self.assertTrue(self.safety.safety_rx_hook(self.packet(dash)))
      self.assertFalse(self.safety.safety_tx_hook(self.packet(steer)))

  def test_forwarding_and_relay(self):
    for car in CARS:
      cp = self.params(car)
      self.mode(cp)
      self.assertEqual(self.safety.safety_fwd_hook(0, 0x124), 2)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x124), -1)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x321), -1)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x322), -1)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x221), 0)
      self.assertEqual(self.safety.safety_fwd_hook(1, 0x124), -1)
      _, frames = self.sources(car)
      _, steer, _ = self.controller(car, cp, frames)
      self.arm(cp, frames)
      self.assertTrue(self.safety.safety_tx_hook(self.packet(steer)))
      self.safety.set_relay_malfunction(True)
      self.assertFalse(self.safety.safety_tx_hook(self.packet(steer)))
      self.assertEqual(self.safety.safety_fwd_hook(0, 0x124), -1)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x221), -1)

  def test_host_native_angle_rate_matrix(self):
    for car in CARS:
      for speed in (5, 30, 80):
        for target in (-650., 650.):
          with self.subTest(car=car, speed=speed, target=target):
            cp = self.params(car)
            _, frames = self.sources(car, speed=speed)
            state = CarState(cp)
            parsers = state.get_can_parsers(cp)
            state.update(parsers)
            for parser in parsers.values():
              parser.update([(1_000_000_000, frames)])
            state.out = state.update(parsers).as_reader()
            self.arm(cp, frames)
            controller = CarController(DBC[car], cp)
            command = structs.CarControl()
            command.enabled = command.latActive = True
            command.actuators.steeringAngleDeg = target
            seen = []
            for i in range(120):
              self.safety.set_timer(1_000_000 + i * 10_000)
              _, sends = controller.update(command.as_reader(), state, 1_000_000_000 + i * 10_000_000)
              for frame in sends:
                if frame[0] == 0x124:
                  self.assertTrue(self.safety.safety_tx_hook(self.packet(frame)),
                                  f'{car.value} {speed=} {target=} {i=} {frame[1].hex()}')
                  seen.append(frame)
            self.assertGreater(len(seen), 50)
            self.assertNotEqual(seen[-1][1][5:7], b'\x00\x00')

  def test_own_vehicle_model_and_exact_configuration(self):
    from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
    cp = self.params(CARS[0])
    self.assertEqual(cp.safetyConfigs[0].safetyParam, 16)
    self.assertAlmostEqual(cp.mass, 1704.)
    self.assertAlmostEqual(calc_slip_factor(VehicleModel(cp)), -0.0006281956135115129, places=13)
    self.assertFalse(cp.openpilotLongitudinalControl or cp.alphaLongitudinalAvailable)

  def test_firmware_complete_partial_and_mixed(self):
    from opendbc.car.subaru.fingerprints import FW_VERSIONS
    car = CARS[0]
    rows = [(ecu, addr, sub, values[0]) for (ecu, addr, sub), values in FW_VERSIONS[car].items()]

    def records(entries):
      return [structs.CarParams.CarFw(ecu=ecu, address=addr, subAddress=0 if sub is None else sub, fwVersion=value, brand='subaru')
              for ecu, addr, sub, value in entries]

    exact, matches = match_fw_to_car(records(rows), '00000000000000000', log=False)
    self.assertTrue(exact)
    self.assertEqual(matches, {car})
    for partial in (rows[:1], rows[1:2]):
      _, matches = match_fw_to_car(records(partial), '00000000000000000', log=False)
      self.assertNotIn(car, matches)
    foreign = next((ecu, addr, sub, value) for sibling, firmware in FW_VERSIONS.items() if sibling != car
                   for (ecu, addr, sub), values in firmware.items() for value in values
                   if (ecu, addr, sub) in FW_VERSIONS[car] and value not in FW_VERSIONS[car][(ecu, addr, sub)])
    _, matches = match_fw_to_car(records([*rows, foreign]), '00000000000000000', log=False)
    self.assertNotIn(car, matches)

  def test_profile_switch_rebinds_every_required_angle_bus(self):
    from opendbc.safety.tests.test_subaru_gen2_angle_pair import TestSubaruGen2AnglePair
    required = {0x40, 0x119, 0x11A, 0x13A, 0x13C, 0x222, 0x321}
    sequence = (CAR.SUBARU_FORESTER_2022, CAR.SUBARU_CROSSTREK_2025, CAR.SUBARU_LEGACY_2025,
                CAR.SUBARU_FORESTER_2022, CAR.SUBARU_ASCENT_2023, CAR.SUBARU_CROSSTREK_2025,
                CAR.SUBARU_FORESTER_2022)
    for candidate in sequence:
      cp = self.params(candidate)
      _, frames = TestSubaruGen2AnglePair.sources(candidate)
      self.arm(cp, frames)
      for missing in (frame for frame in frames if frame[0] in required):
        with self.subTest(candidate=candidate, wrong_bus=hex(missing[0])):
          self.mode(cp)
          for frame in frames:
            if frame[0] in required:
              bad = (frame[0], frame[1], (frame[2] + 1) % 3) if frame is missing else frame
              self.safety.safety_rx_hook(self.packet(bad))
          self.safety.safety_tick()
          self.assertFalse(self.safety.safety_config_valid())
      self.arm(cp, frames)
