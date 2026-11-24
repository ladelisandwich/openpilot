"""Actual CP/selected intent plus parser, Controls and native admission regressions."""

from dataclasses import replace
import unittest
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, DBC
from opendbc.car.chrysler.aol_contract import CHRYSLER_AOL_CARS, qualified
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.vehicle import policy_for, create_intent, native_matches_cp, native_bootstrap_supported
from openpilot.starpilot.car.chrysler.intent import ChryslerCardIntent


def cp_for(candidate, *, enabled=True, alpha=False, release=False, bypass=False, bsm=False, eps=b'68310000', stacks=0):
  fp = gen_empty_fingerprint()
  if stacks & 1:
    fp[0][221] = 8
  if stacks & 2:
    fp[0][650] = 8
  if bypass:
    fp[0][0x4FF] = 8
  if bsm:
    fp[0][0x2D0] = 6
  fw = [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, fwVersion=eps)]
  cp = CarInterface.get_params(candidate, fp, fw, alpha, release, False)
  if enabled:
    cp.safetyConfigs[0].safetyParam |= policy_for(cp).safety_param_addition
    cp.alternativeExperience = 32
  return cp


def state(**kw):
  cs = structs.CarState(canValid=True, vEgo=25.0, gearShifter='drive')
  cs.cruiseState.available = True
  for key, value in kw.items():
    setattr(cs, key, value)
  return cs


SETTINGS = AolSettings(True, 0.0, 9, 9, (0, 0, 0), (0, 0, 0))


class TestChryslerAol(unittest.TestCase):
  def test_actual_factory_all_config_axes_and_exact_native_words(self):
    for candidate in CAR:
      for alpha in (False, True):
        for release in (False, True):
          for eps in (b'68310000', b'68410000'):
            for bypass, bsm in ((False, False), (True, False), (False, True), (True, True)):
              cp = cp_for(candidate, alpha=alpha, release=release, bypass=bypass, bsm=bsm, eps=eps)
              expected = candidate in CHRYSLER_AOL_CARS
              with self.subTest(candidate=candidate, alpha=alpha, release=release, eps=eps, bypass=bypass, bsm=bsm):
                self.assertEqual(qualified(cp, marked_only=True), expected)
                self.assertEqual(native_bootstrap_supported(cp), expected)
                self.assertEqual(policy_for(cp).runtime_supported, expected)
                self.assertEqual(native_matches_cp(cp, int(cp.safetyConfigs[0].safetyModel.raw), cp.safetyConfigs[0].safetyParam), expected)
                self.assertTrue(cp.pcmCruise)
                self.assertFalse(cp.openpilotLongitudinalControl or cp.alphaLongitudinalAvailable)
                self.assertGreater(cp.mass, candidate.config.specs.mass)
                if expected:
                  self.assertIsInstance(create_intent(cp, SETTINGS, policy_for(cp)), ChryslerCardIntent)
                  cp.passive = True
                  self.assertFalse(qualified(cp))

  def test_bootstrap_rejects_off_passive_wrong_native_and_longitudinal(self):
    for candidate in CHRYSLER_AOL_CARS:
      self.assertFalse(native_bootstrap_supported(cp_for(candidate, enabled=False)))
      for field in ('passive', 'notCar', 'dashcamOnly', 'openpilotLongitudinalControl', 'alphaLongitudinalAvailable'):
        cp = cp_for(candidate)
        setattr(cp, field, True)
        self.assertFalse(native_bootstrap_supported(cp))
      cp = cp_for(candidate)
      cp.safetyConfigs[0].safetyParam = 65535
      self.assertFalse(native_bootstrap_supported(cp))

  def test_cold_main_on_does_not_arm_but_real_full_activation_does(self):
    for candidate in CHRYSLER_AOL_CARS:
      intent = create_intent(cp_for(candidate), SETTINGS, policy_for(cp_for(candidate)))
      cs = state()
      intent.update(cs, now_ns=1_000_000_000, fault_active=False)
      self.assertFalse(intent.output(cs)[0])
      for stamp, enabled, active in ((1_010_000_000, False, False), (1_020_000_000, True, False)):
        intent.observe_full_engagement(cs, enabled=enabled, active=active, source_ns=stamp, observation_valid=True, now_ns=stamp, fault_active=False)
      self.assertFalse(intent.allowed_latch)
      intent.observe_full_engagement(cs, enabled=True, active=True, source_ns=1_030_000_000, observation_valid=True, now_ns=1_030_000_000, fault_active=False)
      self.assertTrue(intent.output(cs)[0])
      cs.brakePressed = True
      intent.update(cs, now_ns=1_040_000_000, fault_active=False)
      self.assertTrue(intent.output(cs)[0])
      cs.gearShifter = 'reverse'
      intent.update(cs, now_ns=1_050_000_000, fault_active=False)
      self.assertTrue(intent.allowed_latch)
      self.assertFalse(intent.output(cs)[0])
      cs.gearShifter = 'drive'
      cs.brakePressed = False
      cs.steerFaultTemporary = True
      intent.update(cs, now_ns=1_060_000_000, fault_active=False)
      self.assertTrue(intent.allowed_latch)
      cs.steerFaultTemporary = False
      self.assertTrue(intent.output(cs)[0])
      cs.steerFaultPermanent = True
      intent.update(cs, now_ns=1_070_000_000, fault_active=True)
      self.assertFalse(intent.allowed_latch)

  def test_explicit_off_main_off_and_hybrid_set_guard(self):
    cp = cp_for(CAR.CHRYSLER_PACIFICA_2019_HYBRID)
    intent = create_intent(cp, SETTINGS, policy_for(cp))
    cs = state()
    intent.update(cs, now_ns=1_000_000_000, fault_active=False)
    cs.buttonEvents = [structs.CarState.ButtonEvent(type='mainCruise', pressed=True)]
    intent.update(cs, now_ns=1_010_000_000, fault_active=False)
    self.assertFalse(intent.allowed_latch)
    # Starting during an already-enabled stream is not a new engagement.
    cs.buttonEvents = []
    intent.observe_full_engagement(cs, enabled=True, active=True, source_ns=1_020_000_000, observation_valid=True, now_ns=1_020_000_000, fault_active=False)
    self.assertFalse(intent.allowed_latch)
    intent.settings = replace(SETTINGS, enabled=False)
    intent.update(cs, now_ns=1_030_000_000, fault_active=False)
    self.assertFalse(intent.allowed_latch)
    self.assertFalse(intent.output(cs)[0])
    intent.settings = SETTINGS
    cs.cruiseState.available = False
    intent.update(cs, now_ns=1_040_000_000, fault_active=False)
    self.assertFalse(intent.allowed_latch)

  def test_real_parser_main_cancel_and_no_new_chrysler_lkas_activation(self):
    for candidate in (CAR.CHRYSLER_PACIFICA_2018, CAR.RAM_1500_5TH_GEN):
      cp = cp_for(candidate, stacks=3 if candidate == CAR.RAM_1500_5TH_GEN else 0)
      ci = CarInterface(cp)
      ci.update([])
      packer = CANPacker(DBC[candidate][Bus.pt])

      def ingest(tick, packer=packer, ci=ci, **values):
        frames = [packer.make_can_msg(name, 0, fields) for name, fields in values.items()]
        return ci.update([(1_000_000_000 + tick * 10_000_000, frames)])

      ingest(0, CRUISE_BUTTONS={'ACC_OnOff': 0})
      cs = ingest(1, CRUISE_BUTTONS={'ACC_OnOff': 1})
      self.assertTrue(any(e.type == 'mainCruise' and e.pressed for e in cs.buttonEvents))
      cs = ingest(2, CRUISE_BUTTONS={'ACC_OnOff': 0, 'ACC_Cancel': 1})
      self.assertTrue(any(e.type == 'cancel' and e.pressed for e in cs.buttonEvents))
      self.assertFalse(policy_for(cp).lkas_button_supported)
      self.assertFalse(any(e.type == 'lkas' for e in cs.buttonEvents))

  def test_off_keeps_ordinary_native_configuration_and_old_parser_events(self):
    cp = cp_for(CAR.RAM_1500_5TH_GEN, enabled=False)
    self.assertTrue(policy_for(cp).settings_supported)
    self.assertFalse(policy_for(cp).ordinary_axis_ack_required or policy_for(cp).full_axis_runtime_required)
    self.assertEqual((cp.safetyConfigs[0].safetyParam, cp.alternativeExperience), (1, 0))
    ci = CarInterface(cp)
    ci.update([])
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    cs = ci.update([(1_000_000_000, [packer.make_can_msg('CRUISE_BUTTONS', 0, {'ACC_Cancel': 1, 'ACC_OnOff': 1})])])
    self.assertFalse(any(e.type in ('cancel', 'mainCruise', 'lkas') for e in cs.buttonEvents))

  def test_actual_controller_pause_stock_long_requires_fresh_card_owner_and_never_resumes(self):
    cp = cp_for(CAR.RAM_1500_5TH_GEN)
    ci = CarInterface(cp)
    ci.update([])
    ci.CS.out.canValid = True
    ci.CS.out.canTimeout = False
    ci.CS.out.cruiseState.enabled = True
    ci.CS.out.gearShifter = 'drive'
    ci.CS.out.vEgo = 20.0
    intent = create_intent(cp, replace(SETTINGS, distance_actions=(4, 0, 0)), policy_for(cp))
    intent.observe_physical_samples(ci.CS, now_ns=1_000_000_000)
    cs = state()
    cs.cruiseState.enabled = True
    intent.update(cs, now_ns=1_000_000_000, fault_active=False)
    cs.buttonEvents = [structs.CarState.ButtonEvent(type='gapAdjustCruise', pressed=True)]
    intent.update(cs, now_ns=1_010_000_000, fault_active=False)
    cs.buttonEvents = [structs.CarState.ButtonEvent(type='gapAdjustCruise', pressed=False)]
    intent.update(cs, now_ns=1_020_000_000, fault_active=False)
    self.assertTrue(intent.pause_longitudinal)
    command = structs.CarControl(enabled=True)
    ci.CC.frame = 202
    _, frames = ci.apply(command.as_reader(), 1_020_000_000)
    buttons = [data for address, data, bus in frames if address == 0xB1]
    self.assertTrue(buttons)
    self.assertTrue(all(data[0] == 1 for data in buttons))
    ci.CC.frame = 210
    _, frames = ci.apply(command.as_reader(), 1_050_000_001)
    self.assertFalse(any(address == 0xB1 for address, _, _ in frames))
    intent.pause_longitudinal = False
    cs.buttonEvents = []
    intent.update(cs, now_ns=1_060_000_000, fault_active=False)
    ci.CC.frame = 218
    _, frames = ci.apply(command.as_reader(), 1_060_000_000)
    self.assertFalse(any(address == 0xB1 for address, _, _ in frames))

  def test_forced_torque_does_not_remove_classic_aol_or_change_its_baseline(self):
    cp = cp_for(CAR.JEEP_GRAND_CHEROKEE)
    self.assertTrue(qualified(cp))
    cp.lateralTuning.init('torque')
    CarInterface.configure_torque_tune(cp.carFingerprint, cp.lateralTuning)
    before = cp.lateralTuning.to_dict()
    self.assertTrue(qualified(cp))
    self.assertTrue(policy_for(cp).runtime_supported)
    self.assertEqual(cp.lateralTuning.to_dict(), before)

  def test_dom_main_following_mode_and_hybrid_set_are_distinct(self):
    for candidate in CHRYSLER_AOL_CARS:
      cp = cp_for(candidate)
      settings = replace(SETTINGS, lkas_action=0, main_action=0)
      intent = create_intent(cp, settings, policy_for(cp))
      cs = state()
      intent.update(cs, now_ns=1_000_000_000, fault_active=False)
      self.assertEqual(intent.allowed_latch, candidate != CAR.CHRYSLER_PACIFICA_2019_HYBRID)
      if candidate != CAR.CHRYSLER_PACIFICA_2019_HYBRID:
        cs.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
        intent.update(cs, now_ns=1_010_000_000, fault_active=False)
        cs.buttonEvents = []
        intent.update(cs, now_ns=1_020_000_000, fault_active=False)
        self.assertFalse(intent.allowed_latch)

  def test_ram_switch_fingerprints_do_not_enable_unproven_lkas_subscriptions(self):
    for stacks in range(4):
      cp = cp_for(CAR.RAM_1500_5TH_GEN, stacks=stacks)
      ci = CarInterface(cp)
      ci.update([])
      pt = ci.can_parsers[Bus.pt]
      self.assertNotIn(221, pt.addresses)
      self.assertNotIn(650, pt.addresses)
      self.assertFalse(policy_for(cp).lkas_button_supported)
      packers = {bus: CANPacker(parser.dbc.name) for bus, parser in ci.can_parsers.items()}
      for tick in range(20):
        frames = [
          packers[bus].make_can_msg(parser.dbc.addr_to_msg[address].name, parser.bus, {})
          for bus, parser in ci.can_parsers.items()
          for address in sorted(parser.addresses)
        ]
        cs = ci.update([(1_000_000_000 + tick * 10_000_000, frames)])
      self.assertTrue(cs.canValid, (stacks, pt.addresses))
    for bus, length in ((1, 8), (2, 8), (0, 7), (0, 4)):
      fp = gen_empty_fingerprint()
      fp[bus][221] = length
      fp[bus][650] = length
      cp = CarInterface.get_params(CAR.RAM_1500_5TH_GEN, fp, [], False, False, False)
      self.assertFalse(int(cp.flags) & 48)

  def test_current_stock_off_never_sends_stale_resume_in_optional_axis_mode(self):
    cp = cp_for(CAR.RAM_1500_5TH_GEN)
    ci = CarInterface(cp)
    ci.update([])
    ci.CS.out.canValid = True
    ci.CS.out.cruiseState.enabled = False
    command = structs.CarControl(enabled=True)
    command.cruiseControl.resume = True
    for paused in (False, True):
      ci.CS.aol_pause_longitudinal = paused
      ci.CS.aol_pause_longitudinal_stamp_ns = 1_000_000_000
      for now in (1_000_000_000, 1_030_000_001):
        ci.CC.frame += 20
        _, frames = ci.apply(command.as_reader(), now)
        self.assertFalse(any(address == 0xB1 and data[0] == 0x10 for address, data, _ in frames))


  def test_cold_readiness_preserves_intent_only_until_calibration_is_ready(self):
    from openpilot.cereal import log
    from openpilot.selfdrive.selfdrived.events import Events
    from openpilot.starpilot.aol.vehicle import intent_disarming_fault
    cp = cp_for(CAR.JEEP_GRAND_CHEROKEE)
    settings = replace(SETTINGS, lkas_action=0, main_action=0)
    intent = create_intent(cp, settings, policy_for(cp))
    cs = state()
    cold = Events()
    cold.add(log.OnroadEvent.EventName.calibrationIncomplete)
    fault = intent_disarming_fault(cp, cold.to_msg(), cs, startup_calibration_pending=True)
    self.assertFalse(fault)
    intent.update(cs, now_ns=1_000_000_000, fault_active=fault)
    intent.observe_calibration(cs, events=cold.to_msg())
    self.assertTrue(intent.startup_calibration_pending)
    ready = Events()
    intent.observe_calibration(cs, events=ready.to_msg())
    self.assertFalse(intent.startup_calibration_pending)
    self.assertTrue(intent.allowed_latch)
    self.assertFalse(intent_disarming_fault(cp, cold.to_msg(), cs, startup_calibration_pending=False))
    # A later SOFT calibration warning pauses commands without erasing intent.
    intent.update(cs, now_ns=1_010_000_000, fault_active=False)
    self.assertTrue(intent.allowed_latch)
    # A true immediate fault still requires a new driver gesture.
    intent.update(cs, now_ns=1_020_000_000, fault_active=True)
    self.assertFalse(intent.allowed_latch)
    intent.update(cs, now_ns=1_030_000_000, fault_active=False)
    self.assertFalse(intent.allowed_latch)

  def test_soft_readiness_and_true_immediate_calibration_remain_distinct(self):
    from openpilot.cereal import log
    from openpilot.selfdrive.selfdrived.events import Events
    from openpilot.starpilot.aol.vehicle import intent_disarming_fault
    cp = cp_for(CAR.JEEP_GRAND_CHEROKEE)
    cs = state()
    for name in (log.OnroadEvent.EventName.calibrationInvalid, log.OnroadEvent.EventName.calibrationRecalibrating,
                 log.OnroadEvent.EventName.doorOpen, log.OnroadEvent.EventName.canError):
      events = Events()
      events.add(name)
      self.assertEqual(intent_disarming_fault(cp, events.to_msg(), cs, startup_calibration_pending=True),
                       any(event.immediateDisable for event in events.to_msg()), name)
    event = log.OnroadEvent.new_message(name=log.OnroadEvent.EventName.calibrationIncomplete, immediateDisable=True)
    self.assertTrue(intent_disarming_fault(cp, [event], cs, startup_calibration_pending=True))
    cs.steerFaultPermanent = True
    self.assertTrue(intent_disarming_fault(cp, [], cs, startup_calibration_pending=True))


  def test_initial_calibration_wait_rejects_fault_identity_and_active_epoch(self):
    from types import SimpleNamespace
    from openpilot.cereal import log
    from openpilot.selfdrive.selfdrived.events import Events
    from openpilot.starpilot.car.chrysler.aol import cold_calibration_intent_current
    for reason in ('critical', 'availabilityPause'):
      cp = cp_for(CAR.JEEP_GRAND_CHEROKEE)
      cs = state()
      events = Events()
      events.add(log.OnroadEvent.EventName.calibrationIncomplete)
      axis = SimpleNamespace(intentProducerSessionId='card-1', intentIntegrityEpoch=0,
        desiredLongitudinal=False, longitudinalActive=False, lateralActive=False,
        desiredLateral=False, observedMonoTime=2_000_000_000,
        faultReason=reason, faultEventMonoTime=1_900_000_000)
      native = SimpleNamespace(pandaInventory=(object(),), requestedLongitudinal=False,
        longitudinalAllowed=False, requestedLateral=False, lateralAllowed=False)
      class Peers:
        seen = valid = alive = {'onroadEvents': True}
        logMonoTime = {'onroadEvents': 1_900_000_000}
        def __getitem__(self, _, events=events):
          return events.to_msg()
      feedback = SimpleNamespace(failed=False, ever_lateral_active=False, signature=None, cold_main_wait_ns=0)
      def observe(feedback=feedback, cp=cp, cs=cs, axis=axis, native=native, Peers=Peers):
        return cold_calibration_intent_current(feedback, Peers(), cp, cs, axis, native, 2_010_000_000,
          pending=True, producer='card-1')
      self.assertTrue(observe())
      for obj, field, value in ((feedback, 'failed', True), (feedback, 'ever_lateral_active', True),
          (axis, 'intentProducerSessionId', 'restarted-card'), (axis, 'desiredLongitudinal', True),
          (native, 'requestedLongitudinal', True), (native, 'lateralAllowed', True),
          (cs, 'doorOpen', True), (cs, 'steeringDisengage', True)):
        previous = getattr(obj, field)
        setattr(obj, field, value)
        self.assertFalse(observe(), field)
        setattr(obj, field, previous)
      for name in (log.OnroadEvent.EventName.canError, log.OnroadEvent.EventName.calibrationInvalid,
                   log.OnroadEvent.EventName.doorOpen):
        events.add(name)
        self.assertFalse(observe(), name)
        events.clear()
        events.add(log.OnroadEvent.EventName.calibrationIncomplete)
      Peers.logMonoTime['onroadEvents'] = 1
      self.assertFalse(observe())
