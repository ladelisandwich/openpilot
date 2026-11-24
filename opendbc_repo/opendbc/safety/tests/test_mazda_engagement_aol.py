"""Actual Mazda RX -> ordinary grant -> brake -> retained AOL -> TX/FWD."""
import unittest

from opendbc.safety.tests import test_mazda_stock_aol as stock


class TestMazdaEngagementAol(unittest.TestCase):
  configure = stock.TestMazdaStockAol.configure
  packet = stock.TestMazdaStockAol.packet
  tearDown = stock.TestMazdaStockAol.tearDown

  def frame(self, *, active=False, brake=False, main=True, reverse=False, temporary=False, permanent=False, omit=None, request=1):
    self.time = (self.time + 10_000) & 0xFFFFFFFF
    self.safety.set_timer(self.time)
    sources = (
      ('CRZ_BTNS', 0, {'RES': 0, 'SET_M': 0, 'CAN_OFF': 0}),
      ('STEER_TORQUE', 0, {'STEER_TORQUE_SENSOR': 0}),
      ('ENGINE_DATA', 0, {'SPEED': 80, 'PEDAL_GAS': 0}),
      ('PEDALS', 0, {'BRAKE_ON': int(brake)}),
      ('GEAR', 0, {'GEAR': 2 if reverse else 4}),
      ('STEER_RATE', 0, {'LKAS_BLOCK': int(temporary)}),
      ('CAM_LKAS', 2, {'ERR_BIT_1': int(permanent), 'LKAS_REQUEST': 0}),
      ('CRZ_CTRL', 0, {'CRZ_AVAILABLE': int(main), 'CRZ_ACTIVE': int(active)}),
    )
    for name, bus, values in sources:
      if name != omit:
        self.safety.safety_rx_hook(self.packet(name, values, bus))
    self.safety.safety_tick()
    if request is not None:
      self.safety.aol_set_host_request(request)

  def warm(self, **kwargs):
    # These tests isolate ordinary cruise engagement; no bootstrap LAT desire.
    kwargs.setdefault('request', 0)
    for _ in range(20):
      self.frame(**kwargs)

  def test_accepted_cold_main_on_engagement_survives_brake_without_sampled_button(self):
    self.configure()
    self.warm()
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.frame(active=True)
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.aol_set_host_request(3)
    self.assertEqual(self.safety.aol_get_permission_mask(), 3)
    self.frame(brake=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertTrue(self.safety.safety_tx_hook(self.packet('CAM_LKAS', {'LKAS_REQUEST': 10})))
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x243), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x440), -1)
    self.safety.aol_set_host_request(3)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)  # Long never resumes.
    self.safety.aol_set_host_request(0)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x243), 0)

  def test_startup_active_has_no_new_engagement_token(self):
    self.configure()
    self.warm(active=True)
    self.frame(brake=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertFalse(self.safety.safety_tx_hook(self.packet('CAM_LKAS', {'LKAS_REQUEST': 10})))

  def test_temporary_restrictions_pause_without_destroying_accepted_intent(self):
    self.configure()
    self.warm()
    self.frame(active=True)
    self.frame(brake=True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    for restriction in ({'reverse': True}, {'temporary': True}):
      self.frame(**restriction)
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
      self.frame()
      self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_fault_and_source_loss_require_new_healthy_engagement(self):
    for fault in ('heartbeat', 'permanent', 'source'):
      with self.subTest(fault=fault):
        self.configure()
        self.warm()
        self.frame(active=True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)
        if fault == 'heartbeat':
          self.safety.set_aol_test_heartbeat(False)
          self.assertEqual(self.safety.aol_get_permission_mask(), 0)
          self.safety.set_aol_test_heartbeat(True)
        elif fault == 'permanent':
          self.frame(active=True, permanent=True)
          self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        else:
          for _ in range(40):
            self.frame(active=True, omit='CAM_LKAS')
          self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.warm(active=True)
        # Ordinary cruise may still be active; it must not disguise a new AOL grant.
        self.frame(brake=True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.frame()  # Fresh healthy inactive cruise, pedals released.
        self.frame(active=True)
        self.frame(brake=True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_ae0_ordinary_engagement_never_gains_independent_permission(self):
    self.configure(experience=0)
    self.warm()
    self.frame(active=True)
    self.assertTrue(self.safety.get_controls_allowed())
    self.frame(brake=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertFalse(self.safety.safety_tx_hook(self.packet('CAM_LKAS', {'LKAS_REQUEST': 10})))

  def test_cold_grant_requires_heartbeat_and_post_grant_request(self):
    self.configure()
    self.safety.set_aol_test_heartbeat(False)
    self.warm()
    self.frame(active=True, request=None)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.assertFalse(self.safety.safety_tx_hook(self.packet('CAM_LKAS', {'LKAS_REQUEST': 10})))
    self.safety.set_aol_test_heartbeat(True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)  # Request predates physical grant.
    self.safety.aol_set_host_request(0)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.time = (self.time + 1) & 0xFFFFFFFF
    self.safety.set_timer(self.time)
    self.safety.aol_set_host_request(1)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)
    self.frame(brake=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_pending_grant_cannot_extend_original_timeout_or_rearm_from_held_active(self):
    self.configure()
    self.safety.set_aol_test_heartbeat(False)
    self.warm()
    self.frame(active=True)
    for _ in range(31):
      self.frame(active=True)  # Fresh CAN and requests do not renew physical-grant time.
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.safety.set_aol_test_heartbeat(True)
    self.frame(active=True)
    self.frame(brake=True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.frame()
    self.frame(active=True)
    self.frame(brake=True)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_pending_main_and_button_grants_use_same_output_gate(self):
    for source in ('main', 'button'):
      with self.subTest(source=source):
        self.configure()
        self.safety.set_aol_test_heartbeat(False)
        self.warm()
        if source == 'main':
          self.frame(main=False)
          self.frame(main=True, request=None)
        else:
          self.time += 10_000
          self.safety.set_timer(self.time)
          self.safety.safety_rx_hook(self.packet('CRZ_BTNS', {'RES': 1}))
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.safety.set_aol_test_heartbeat(True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.time = (self.time + 1) & 0xFFFFFFFF
        self.safety.set_timer(self.time)
        self.safety.aol_set_host_request(1)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)
        self.safety.set_aol_test_heartbeat(False)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.safety.set_aol_test_heartbeat(True)
        self.frame()
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def prepare_cold_at(self, last_inactive_time):
    self.configure()
    self.safety.set_aol_test_heartbeat(False)
    self.time = (last_inactive_time - 200_000) & 0xFFFFFFFF
    self.warm(request=None)
    self.assertEqual(self.time, last_inactive_time)
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def physical_grant_at(self, timestamp):
    self.time = timestamp & 0xFFFFFFFF
    self.safety.set_timer(self.time)
    self.safety.safety_rx_hook(self.packet('CRZ_CTRL', {'CRZ_AVAILABLE': 1, 'CRZ_ACTIVE': 1}))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.aol_get_permission_mask(), 0)

  def test_equal_stamp_before_or_after_grant_never_acknowledges_pending(self):
    for order in ('before', 'after'):
      with self.subTest(order=order):
        self.prepare_cold_at(1_500_000)
        if order == 'before':
          self.safety.aol_set_host_request(1)
        self.physical_grant_at(1_500_000)
        if order == 'after':
          self.safety.aol_set_host_request(1)
        self.safety.set_aol_test_heartbeat(True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.assertFalse(self.safety.safety_tx_hook(self.packet('CAM_LKAS', {'LKAS_REQUEST': 10})))
        self.time += 1
        self.safety.set_timer(self.time)
        self.safety.aol_set_host_request(1)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)
        self.assertTrue(self.safety.safety_tx_hook(self.packet('CAM_LKAS', {'LKAS_REQUEST': 10})))

  def test_request_one_microsecond_before_and_after_grant_across_timer_wrap(self):
    for grant in (1_500_000, 0xFFFFFFFF, 0):
      with self.subTest(grant=grant):
        self.prepare_cold_at((grant - 1) & 0xFFFFFFFF)
        self.safety.aol_set_host_request(1)  # One microsecond BEFORE grant.
        self.physical_grant_at(grant)
        self.safety.set_aol_test_heartbeat(True)
        self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.time = (grant + 1) & 0xFFFFFFFF
        self.safety.set_timer(self.time)
        self.safety.aol_set_host_request(1)  # One microsecond AFTER, including wrap.
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)
        self.safety.safety_rx_hook(self.packet('PEDALS', {'BRAKE_ON': 1}))
        self.assertFalse(self.safety.get_controls_allowed())
        self.safety.aol_set_host_request(3)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_equal_stamp_request_stays_denied_without_a_later_request(self):
    self.prepare_cold_at(1_500_000)
    self.safety.aol_set_host_request(1)
    self.physical_grant_at(1_500_000)
    self.safety.set_aol_test_heartbeat(True)
    for elapsed in (0, 1, 10, 999):
      self.time = 1_500_000 + elapsed
      self.safety.set_timer(self.time)
      self.assertEqual(self.safety.aol_get_permission_mask(), 0)
    self.safety.aol_set_host_request(1)
    self.assertEqual(self.safety.aol_get_permission_mask(), 1)

  def test_strict_ack_keeps_original_pending_deadline_boundary(self):
    for age in (300_000, 300_001):
      with self.subTest(age=age):
        self.prepare_cold_at(1_500_000)
        self.physical_grant_at(1_500_000)
        for _ in range(30):
          self.frame(active=True)  # Refresh real RX and requests, never the grant.
          self.assertEqual(self.safety.aol_get_permission_mask(), 0)
        self.time = 1_500_000 + age
        self.safety.set_timer(self.time)
        # Withdraw ordinary controls through real brake RX before observing the
        # retained token, so ordinary controls cannot mask expired-pending failure.
        self.safety.safety_rx_hook(self.packet('PEDALS', {'BRAKE_ON': 1}))
        self.assertFalse(self.safety.get_controls_allowed())
        self.safety.set_aol_test_heartbeat(True)
        self.safety.aol_set_host_request(1)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1 if age == 300_000 else 0)
        self.assertEqual(self.safety.aol_get_permission_mask(), 1 if age == 300_000 else 0)
