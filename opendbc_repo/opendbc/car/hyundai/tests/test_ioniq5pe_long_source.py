import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.hyundai import ioniq5pe_long_sender as sender
from opendbc.car.hyundai import ev9_long_sender
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.ioniq5pe_longitudinal import PELongitudinalPolicy, Request, LongCtrlState, lateral_request_allowed
from opendbc.car import structs
from unittest.mock import patch


PE_BODIES = {
  0x160: '0000000000000000fffc0100a8001000',
  0x1DA: '0000002200010000000000000000000000000000000000000000000000000000',
  0x1EA: '000000080000000000000000000000ff000000000000000000000000000f0f00',
  0x200: '00000014401b0000',
  0x345: '0000001500560000',
  0x161: '0000000000000000c0fff0c003000040000000000000000000ff000000000000',
  0x162: '0000002700000000c0ff00000000000000000000000000000000000000000000',
  0x1BA: '00000000000000880200000000000000000100000000000f',
  0x1E5: '00000000000000000000220200000080',
  0x1E0: '00000002000000000000000000000000',
  0x38C: '000000f79f000000000000000000000000000000000000000000000000000000',
}
PERIODS = {0x160: 2, 0x1DA: 100, 0x1EA: 5, 0x200: 5, 0x345: 20, 0x1E0: 5, 0x38C: 20}
HEARTBEAT = '000000000000000000000000ff1fffff55ffff00a8000000'


def independent_crc(address, payload):

  crc = 0
  for byte in bytes(payload[2:]) + address.to_bytes(2, 'little'):
    crc ^= byte << 8
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
  return crc ^ {8: 0x5F29, 16: 0x041D, 24: 0x819D, 32: 0x9F5B}[len(payload)]


class TestIoniq5PELongSource(unittest.TestCase):
  def setUp(self):
    self.cp = SimpleNamespace(carFingerprint=CAR.HYUNDAI_IONIQ_5_PE)
    self.can = SimpleNamespace(ECAN=1, ACAN=0, CAM=2)
    self.packer = CANPacker('hyundai_canfd_generated')

  def assert_crc(self, message):
    address, payload, _ = message
    self.assertEqual(int.from_bytes(payload[:2], 'little'), independent_crc(address, payload))

  def scc(self, accel=1.0, *, enabled=True, override=False, stop=False, visible=True):
    return sender.create_ccnc_acc_control(self.packer, self.can, enabled, accel, stop, stop, override,
                                         80.0, 1, 25.0, -2.0, visible, 10.0)

  def test_all_eleven_frozen_neutral_bodies_and_integrity(self):
    for address, frozen in PE_BODIES.items():
      for counter in (0, 1, 255, 256, 511):
        with self.subTest(address=hex(address), counter=counter):
          message = sender._create_ccnc_adrv_message(self.cp.carFingerprint, address, 1, counter)
          expected = bytearray.fromhex(frozen)
          expected[2] = counter & 255
          expected[:2] = independent_crc(address, expected).to_bytes(2, 'little')
          self.assertEqual(tuple(message), (address, bytes(expected), 1))

  def test_pe_and_ev9_distinct_bodies_are_not_interchanged(self):
    for address in (0x160, 0x1DA, 0x200, 0x162, 0x1E5, 0x38C):
      pe = sender._create_ccnc_adrv_message(CAR.HYUNDAI_IONIQ_5_PE, address, 1, 0)
      ev9 = ev9_long_sender._create_ccnc_adrv_message(CAR.KIA_EV9, address, 1, 0)
      self.assertNotEqual(pe[1][3:], ev9[1][3:])

  def test_pe_recipe_rejects_wrong_identity(self):
    with self.assertRaises(KeyError):
      sender._create_ccnc_adrv_message(CAR.KIA_EV9, 0x160, 1, 0)

  def test_periodic_output_201_frames_uses_frozen_periods_and_counters(self):
    out = SimpleNamespace(vCruiseCluster=80)
    hud = SimpleNamespace(leadDistanceBars=2)
    for frame in range(201):
      messages = sender.create_ccnc_adrv_messages(self.packer, self.cp, self.can, frame, True, True,
                                                 hud, out, True, True, True, False, False)
      expected = {address for address, period in PERIODS.items() if frame % period == 0}
      if frame % 5 == 0:
        expected |= {0x161, 0x162, 0x1BA, 0x1E5}
      self.assertEqual({message[0] for message in messages}, expected)
      self.assertEqual(len(messages), len(expected))
      for message in messages:
        address, payload, bus = message
        self.assertEqual(bus, 1)
        self.assertEqual(payload[2], (frame // PERIODS.get(address, 5)) & 255)
        self.assert_crc(message)

  def test_heartbeat_all_pedals_and_counter_wrap_keep_pe_reserved_body(self):
    for brake in (False, True):
      for gas in (False, True):
        for counter in (0, 255, 256):
          message = sender.create_radar_heartbeat(counter, brake, gas)
          expected = bytearray.fromhex(HEARTBEAT)
          expected[2] = counter & 255
          expected[4] |= int(brake)
          expected[22] |= int(gas)
          expected[:2] = independent_crc(0x100, expected).to_bytes(2, 'little')
          self.assertEqual(tuple(message), (0x100, bytes(expected), 0))

  def test_heartbeat_cannot_use_ev9_body(self):
    self.assertNotEqual(sender.create_radar_heartbeat(0, False, False)[1],
                        ev9_long_sender.create_radar_heartbeat(0, False, False)[1])

  def test_scc_preserves_current_acceleration_bounds_in_actual_payload(self):
    for requested, expected in ((100, 2.0), (-100, -3.5), (1.25, 1.25)):
      message = self.scc(requested)
      address, data, bus = message
      raw = (((data[17] & 7) << 8) | data[16]) - 1023
      value = ((data[18] << 4) | (data[17] >> 4)) - 1023
      self.assertEqual((address, bus), (0x1A0, 1))
      self.assertAlmostEqual(raw / 100, expected)
      self.assertAlmostEqual(value / 100, expected)
      self.assert_crc(message)

  def test_disabled_override_and_stop_zero_both_acceleration_fields(self):
    for args in ({'enabled': False}, {'override': True}, {'stop': True}):
      _, data, _ = self.scc(2.0, **args)
      self.assertEqual((((data[17] & 7) << 8) | data[16]) - 1023, 0)
      self.assertEqual(((data[18] << 4) | (data[17] >> 4)) - 1023, 0)

  def test_direct_angle_and_nondrive_inactive_transport(self):
    active = sender.create_angle_adas_cmd(self.packer, self.can, 1.0, True, 0.5)
    self.assertEqual((active[0], active[2], len(active[1])), (0xCB, 1, 24))
    self.assertEqual((active[1][3] >> 4) & 15, 2)
    inactive = sender.create_inactive_angle_steering_messages(self.packer, self.can, 2.0)
    self.assertEqual([(p[0], p[2]) for p in inactive], [(0x12A, 1), (0xCB, 1)])
    self.assertEqual((inactive[1][1][3] >> 4) & 15, 1)
    for message in [active, *inactive]:
      self.assert_crc(message)

  def test_paired_source_keeps_factory_stock_and_requires_explicit_developer_startup_choice(self):
    from opendbc.car.hyundai.ioniq5pe_startup import PEPendingStartup
    for alpha in (False, True):
      for release in (False, True):
        cp = params(alpha=alpha, release=release)
        before = cp.to_dict()
        self.assertFalse(cp.openpilotLongitudinalControl)
        self.assertEqual(cp.alphaLongitudinalAvailable, not release)
        self.assertEqual(cp.safetyConfigs[0].safetyParam, 0x5491)
        self.assertFalse(CarInterface.startup_required(cp))
        requested = alpha and not release
        owner = CarInterface.startup_owner(cp, (lambda **kwargs: [], lambda frames: None), requested=requested)
        if requested:
          self.assertIsInstance(owner, PEPendingStartup)
        else:
          self.assertIsNone(owner)
        self.assertEqual(cp.to_dict(), before)

  @staticmethod
  def update_policy(policy, frame=0, **kwargs):
    options = dict(active=True, override=False, accel=1.0, speed=0.0, measured_accel=0.0,
                   control_state=LongCtrlState.starting, last_sent_accel=0.0)
    options.update(kwargs)
    return policy.update(frame, **options)

  def test_pe_uses_original_20hz_helper_with_pe_flags(self):
    from opendbc.car.hyundai.ev9_longitudinal import update_ioniq_6_longitudinal_tuning
    policy = PELongitudinalPolicy()
    with patch('opendbc.car.hyundai.ioniq5pe_longitudinal.update_ioniq_6_longitudinal_tuning',
               wraps=update_ioniq_6_longitudinal_tuning) as helper:
      first = self.update_policy(policy)
      self.assertAlmostEqual(first.accel, 0.288)
      for frame in range(1, 5):
        self.assertEqual(self.update_policy(policy, frame), first)
      self.update_policy(policy, 5)
      self.assertEqual(helper.call_count, 2)
      for call in helper.call_args_list:
        self.assertEqual(call.kwargs, {'low_speed_stop_brake_cap': True, 'ev9': False})

  def test_pe_high_speed_stop_tracks_directly_without_ev9_decel_policy(self):
    policy = PELongitudinalPolicy()
    request = self.update_policy(policy, accel=-3.0, speed=10.0, control_state=LongCtrlState.stopping)

    self.assertAlmostEqual(request.accel, -0.36)
    self.assertEqual((request.jerk_upper, request.jerk_lower), (3.0, 5.0))
    next_request = self.update_policy(policy, 1, accel=-3.0, speed=10.0,
                                      control_state=LongCtrlState.stopping, last_sent_accel=-0.36)
    self.assertAlmostEqual(next_request.accel, -0.72)

  def test_pe_original_stop_latch_and_six_sample_release(self):
    policy = PELongitudinalPolicy()
    request = Request(-0.2, True, 1., 1.)
    self.assertEqual(policy.finish_scc(request, enabled=True, override=False, stopping=True, speed=0.48), request)
    self.assertFalse(policy.stop.stop_request)
    self.assertEqual(policy.finish_scc(request, enabled=True, override=False, stopping=True, speed=0.47).accel, 0.)
    for sample in range(1, 179):
      policy.finish_scc(request, enabled=True, override=False, stopping=True, speed=0.0)
      self.assertEqual(policy.stop.cruise_standstill, sample >= 178)
    for sample in range(1, 8):
      policy.finish_scc(request, enabled=True, override=False, stopping=False, speed=0.2)
      self.assertEqual(policy.stop.stop_request, sample <= 6)

  def test_pe_policy_resets_on_disengagement_override_off_and_stays_bounded(self):
    policy = PELongitudinalPolicy()
    for options in ({'active': False}, {'override': True}, {'control_state': LongCtrlState.off}):
      self.update_policy(policy)
      request = self.update_policy(policy, 1, **options)
      self.assertEqual(request.accel, 0.)
      self.assertEqual(policy.tuning.accel_last, 0.)
    last = 0.
    for frame in range(500):
      request = self.update_policy(policy, frame, accel=100., speed=10.,
                                   control_state=LongCtrlState.pid, last_sent_accel=last)
      if frame % 2 == 0:
        request = policy.finish_scc(request, enabled=True, override=False, stopping=False, speed=10.)
        last = request.accel
      self.assertGreaterEqual(request.accel, -3.5)
      self.assertLessEqual(request.accel, 2.0)

  def test_pe_standstill_and_host_faults_cannot_request_direct_angle(self):
    control = SimpleNamespace(enabled=True)
    state = SimpleNamespace(canValid=True, canTimeout=False, standstill=False,
                            gearShifter=structs.CarState.GearShifter.drive, brakePressed=False,
                            gasPressed=False, steerFaultTemporary=False, steerFaultPermanent=False)
    self.assertTrue(lateral_request_allowed(control, state, False))
    for field in ('standstill', 'canTimeout', 'brakePressed', 'gasPressed', 'steerFaultTemporary', 'steerFaultPermanent'):
      setattr(state, field, True)
      self.assertFalse(lateral_request_allowed(control, state, False))
      setattr(state, field, False)
    self.assertFalse(lateral_request_allowed(control, state, True))
    state.canValid = False
    self.assertFalse(lateral_request_allowed(control, state, False))
