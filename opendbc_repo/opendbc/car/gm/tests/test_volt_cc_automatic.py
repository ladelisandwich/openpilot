"""Source-qualified Volt CC discovery reaches the existing parser/controller/native owner."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.car_helpers import can_fingerprint
from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.tests import test_volt_cc_gas as cc_fixture
from opendbc.car.gm.values import CAR, DBC, GMFlags
from opendbc.safety.tests.libsafety import libsafety_py


class TestVoltCcAutomatic(unittest.TestCase):
  def discovered(self, release=False, *, variant=0):
    variants = FINGERPRINTS[CAR.CHEVROLET_VOLT_CC]
    self.assertEqual(len(variants), 2)
    messages = [SimpleNamespace(address=address, dat=bytes(length), src=0)
                for address, length in variants[variant].items()]
    selected, observed = can_fingerprint(lambda **_kwargs: [messages])
    self.assertEqual(selected, CAR.CHEVROLET_VOLT_CC)
    cp = CarInterface.get_params(selected, observed, [], False, release, False)
    return cp, CarInterface(cp), CANPacker(DBC[cp.carFingerprint][Bus.pt])

  def test_actual_discovery_finalizes_existing_owner_and_release_denial(self):
    for release in (False, True):
      cp, ci, _ = self.discovered(release)
      self.assertEqual(cp.carFingerprint, CAR.CHEVROLET_VOLT_CC)
      self.assertEqual(cp.openpilotLongitudinalControl, not release)
      self.assertEqual(cp.dashcamOnly, release)
      self.assertEqual(bool(cp.flags & GMFlags.CC_LONG), not release)
      self.assertEqual(ci.CC.volt_cc_profile, not release)
      if not release:
        self.assertEqual(cp.safetyConfigs[0].safetyParam, 20)
      # Physical structure comes from this car, not another GM tune.
      reference = cc_fixture.fixture()[0]
      self.assertEqual(cp.mass, reference.mass)
      self.assertEqual(cp.wheelbase, reference.wheelbase)
      self.assertEqual(cp.lateralTuning.to_dict(), reference.lateralTuning.to_dict())

  def test_discovered_owner_emits_real_set_and_native_enforces_credit(self):
    # Reuse the existing real receive/ADC/gear/button sequence; replace only discovery.
    probe = cc_fixture.TestVoltCcGas()
    safety = libsafety_py.libsafety
    release = safety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) != 0
    # Reinitialize the actual GM profile after detecting build flavor.
    with patch.object(cc_fixture, 'fixture', return_value=self.discovered()):
      _, ci, _, now = probe.prepare()
    ci.CC.frame = 156
    _, packets = ci.apply(probe.command().as_reader(), now + 1)
    cancel_or_set = [packet for packet in packets if packet[0] == 0x1E1]
    self.assertEqual(len(cancel_or_set), 1)
    packet = cancel_or_set[0]
    self.assertEqual(packet[2], 0)
    self.assertEqual((packet[1][5] >> 4) & 7, 3)
    self.assertEqual(cc_fixture.native('tx', packet, now // 1000), not release)
    self.assertFalse(cc_fixture.native('tx', packet, now // 1000))

  def test_automatic_card_factory_and_controls_preserve_disabled_startup(self):
    import json
    import os

    from opendbc.car import car_helpers
    from opendbc.car.can_definitions import CanData
    from openpilot.common.params import Params
    from openpilot.common.prefix import OpenpilotPrefix
    from openpilot.selfdrive.car.card import Car
    from openpilot.selfdrive.controls.controlsd import Controls
    from openpilot.starpilot.lateral.tests.test_lane_runtime import feed
    from openpilot.starpilot.vehicle_selection import encode

    physical = FINGERPRINTS[CAR.CHEVROLET_VOLT_CC][0]
    packets = [CanData(address, bytes(length), 0) for address, length in physical.items()]

    def receive(wait_for_one=False):
      return [packets] if wait_for_one else []

    def discover(*args, **kwargs):
      return car_helpers.get_car(receive, *args[1:], **kwargs)
    with OpenpilotPrefix(), patch.dict(os.environ, {'SKIP_FW_QUERY': '1', 'FINGERPRINT': '', 'SIMULATION': '1', 'REPLAY': '1'}):
      settings = Params()
      settings.put_bool('OpenpilotEnabledToggle', True, block=True)
      settings.put_bool('IsReleaseBranch', False, block=True)
      settings.put('VehicleSelection', json.loads(encode(None)), block=True)
      with patch('openpilot.selfdrive.car.card.messaging.recv_one_retry', return_value=SimpleNamespace(can=[1])), \
           patch('openpilot.selfdrive.car.card.get_car', side_effect=discover):
        card = Car()
      self.assertEqual(card.CP.carFingerprint, CAR.CHEVROLET_VOLT_CC)
      self.assertEqual(card.CP.fingerprintSource, structs.CarParams.FingerprintSource.can)
      self.assertTrue(card.CI.CC.volt_cc_profile)
      self.assertFalse(card.CP.dashcamOnly)
      controls = Controls()
      self.assertEqual(controls.CP.carFingerprint, card.CP.carFingerprint)
      self.assertEqual(controls.CP.safetyConfigs[0].safetyParam, 20)
      feed(controls, 1_000_000_000, 0, active=False, enabled=False, can_valid=False, can_timeout=True)
      command, _ = controls.state_control()
      self.assertFalse(command.enabled)
      self.assertFalse(command.latActive)
      self.assertFalse(command.longActive)

  def test_automatic_pedal_signature_keeps_distinct_brake_and_off_owner(self):
    from opendbc.car.gm.startup_preferences import prepare_disable_longitudinal
    from opendbc.car.gm.values import volt_cc_pedal_profile
    for release in (False, True):
      cp, ci, _ = self.discovered(release, variant=1)
      self.assertFalse(cp.dashcamOnly)
      self.assertFalse(cp.pcmCruise)
      self.assertFalse(cp.flags & GMFlags.CC_LONG)
      self.assertTrue(cp.flags & GMFlags.PEDAL_LONG)
      self.assertTrue(cp.flags & GMFlags.NO_ACCELERATOR_POS_MSG)
      self.assertEqual(cp.openpilotLongitudinalControl, not release)
      self.assertEqual(cp.safetyConfigs[0].safetyParam, 0xE613 if release else 0xE603)
      self.assertIsNotNone(volt_cc_pedal_profile(cp))
      self.assertIsNotNone(ci.CC.volt_cc_pedal_profile)
      prepare_disable_longitudinal(cp, True)
      self.assertFalse(cp.openpilotLongitudinalControl)
      self.assertEqual(cp.safetyConfigs[0].safetyParam, 0xE613)

  def test_discovered_pedal_runs_existing_actual_parser_and_wire_sequence(self):
    from opendbc.can import CANPacker, CANParser
    from opendbc.car import Bus
    from opendbc.car.gm import gmcan
    from opendbc.car.gm.tests.test_cc_gateway_stock import pt_frames
    from openpilot.common.params import Params
    from openpilot.common.prefix import OpenpilotPrefix
    from openpilot.selfdrive.car.car_events import CarEvents, EventName
    from openpilot.starpilot.controller_extensions import configure_controller
    for be, removed, radar, release in ((False, True, False, False), (False, True, False, True)):
      with self.subTest(be=be, removed=removed, radar=radar, release=release), OpenpilotPrefix():
        params = Params()
        params.put_bool('OpenpilotEnabledToggle', True, block=True)
        cp = self.discovered(release, variant=1)[0]
        ci = CarInterface(cp)
        configure_controller(ci, params)
        packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
        output = CANParser(DBC[cp.carFingerprint][Bus.pt], [('GAS_COMMAND', float('nan'))], 0)
        command = structs.CarControl(enabled=True, longActive=not release)
        command.actuators.accel = .5
        loopback = []
        for tick in range(104):
          now = 1_000_000_000 + tick * 10_000_000
          frames = [frame for frame in pt_frames(packer, counter=tick % 4, acc_cruise=6) if frame[0] not in (0xBE, 0xF1)]
          frames.append(packer.make_can_msg('ECMAcceleratorPos' if be else 'EBCMBrakePedalPosition', 0, {}))
          frames += [packer.make_can_msg('ECMCruiseControl', 0, {'CruiseActive': 1, 'CruiseSetSpeed': 80}),
                     packer.make_can_msg('EBCMRegenPaddle', 0, {})]
          if not removed:
            frames += [packer.make_can_msg('ASCMLKASteeringCmd', 2, {'RollingCounter': tick % 4}),
                       packer.make_can_msg('AEBCmd', 2, {})]
          if not release and tick % 2 == 0:
            sensor = bytearray.fromhex('0279012a0000')
            sensor[4] = tick // 2 % 16
            sensor[5] = gmcan.pedal_crc(sensor)
            frames.append((0x201, bytes(sensor), 0))
          state = ci.update([(now, frames + loopback)])
          actuators, tx = ci.apply(command.as_reader(), now)
          loopback = [(addr, data, 128) for addr, data, _ in tx if addr == 0x180]
          keepalives = [addr for addr, _, _ in tx if addr in (0x409, 0x40A)]
          if release:
            self.assertEqual(keepalives, [])
          if tick == 100:
            self.assertTrue(state.canValid)
            self.assertEqual(keepalives, [0x409, 0x40A] if removed and not release else [])
          if tick == 60:
            self.assertTrue(state.canValid)
            self.assertTrue(state.cruiseState.enabled)
            self.assertAlmostEqual(state.cruiseState.speed, 80 / 3.6, places=4)
            self.assertFalse(state.cruiseState.nonAdaptive)
            events = CarEvents(cp).update(state.as_reader(), structs.CarState().as_reader(), command.as_reader())
            self.assertNotIn(EventName.wrongCruiseMode, events.names)
            self.assertFalse(state.accFaulted)
            if release:
              self.assertFalse(any(a == 0x200 for a, _, _ in tx))
            else:
              pedal = next(frame for frame in tx if frame[0] == 0x200)
              output.update([(now, [pedal])])
              self.assertGreater(actuators.gas, 0.)
              self.assertEqual(actuators.brake, 0.)
              self.assertAlmostEqual(output.vl['GAS_COMMAND']['GAS_COMMAND'] / 255., actuators.gas, delta=.001)
            if removed:
              self.assertNotIn(0x180, ci.can_parsers[Bus.pt].vl)
              self.assertNotIn(0x180, ci.can_parsers[Bus.cam].vl)

  def test_automatic_pedal_card_respects_saved_longitudinal_off(self):
    import json
    import os

    from opendbc.car import car_helpers
    from opendbc.car.can_definitions import CanData
    from openpilot.common.params import Params
    from openpilot.common.prefix import OpenpilotPrefix
    from openpilot.selfdrive.car.card import Car
    from openpilot.selfdrive.controls.controlsd import Controls
    from openpilot.starpilot.lateral.tests.test_lane_runtime import feed
    from openpilot.starpilot.vehicle_selection import encode

    physical = FINGERPRINTS[CAR.CHEVROLET_VOLT_CC][1]
    packets = [CanData(address, bytes(length), 0) for address, length in physical.items()]

    def receive(wait_for_one=False):
      return [packets] if wait_for_one else []

    def discover(*args, **kwargs):
      return car_helpers.get_car(receive, *args[1:], **kwargs)
    with OpenpilotPrefix(), patch.dict(os.environ, {'SKIP_FW_QUERY': '1', 'FINGERPRINT': '', 'SIMULATION': '1', 'REPLAY': '1'}):
      settings = Params()
      settings.put_bool('OpenpilotEnabledToggle', True, block=True)
      settings.put_bool('IsReleaseBranch', False, block=True)
      settings.put_bool('DisableOpenpilotLongitudinal', True, block=True)
      settings.put('VehicleSelection', json.loads(encode(None)), block=True)
      with patch('openpilot.selfdrive.car.card.messaging.recv_one_retry', return_value=SimpleNamespace(can=[1])), \
           patch('openpilot.selfdrive.car.card.get_car', side_effect=discover):
        card = Car()
      self.assertEqual(card.CP.carFingerprint, CAR.CHEVROLET_VOLT_CC)
      self.assertEqual(card.CP.fingerprintSource, structs.CarParams.FingerprintSource.can)
      self.assertIsNotNone(card.CI.CC.volt_cc_pedal_profile)
      self.assertFalse(card.CP.openpilotLongitudinalControl)
      self.assertFalse(card.CP.dashcamOnly)
      controls = Controls()
      self.assertEqual(controls.CP.carFingerprint, card.CP.carFingerprint)
      self.assertEqual(controls.CP.safetyConfigs[0].safetyParam, 0xE613)
      feed(controls, 1_000_000_000, 0, active=False, enabled=False, can_valid=False, can_timeout=True)
      command, _ = controls.state_control()
      self.assertFalse(command.enabled)
      self.assertFalse(command.latActive)
      self.assertFalse(command.longActive)


if __name__ == '__main__':
  unittest.main()
