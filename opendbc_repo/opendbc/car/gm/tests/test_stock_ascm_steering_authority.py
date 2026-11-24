import unittest

from opendbc.car import structs
from opendbc.car.gm.values import CAR, GMFlags
from opendbc.car.gm.steering_authority import SteeringAuthority, monitored_profile


def params():
  # Stock-longitudinal ASCM profile from route5a; no private vehicle data.
  cp = structs.CarParams.new_message(
    brand='gm',
    carFingerprint=str(CAR.CHEVROLET_VOLT_ASCM),
    flags=16,
    networkLocation='fwdCamera',
    pcmCruise=True,
    openpilotLongitudinalControl=False,
    alphaLongitudinalAvailable=False,
    radarUnavailable=True,
    minSteerSpeed=3.1292800903320312,
    alternativeExperience=32,
  )
  cp.safetyConfigs = [{'safetyModel': 'gm', 'safetyParam': 0x605}]
  return cp


class TestStockAscmSteeringAuthority(unittest.TestCase):
  def setUp(self):
    self.cp = params()
    self.monitor = SteeringAuthority(self.cp)

  def step(self, tick, status=1, active=True, pause=None, missing=False, malformed=False):
    ns = 1_000_000_000 + tick * 10_000_000
    cs = structs.CarState.new_message(
      canValid=True,
      canTimeout=False,
      vEgo=17.0,
      standstill=False,
      gearShifter='low',
      steeringPressed=pause == 'driver',
      brakePressed=pause == 'brake',
      regenBraking=pause == 'paddle',
    )
    cs.cruiseState.available = pause != 'main'
    if pause == 'stopped':
      cs.standstill = True
      cs.vEgo = 0.0
    if pause == 'slow':
      cs.vEgo = 2.0
    if pause == 'cancel':
      cs.buttonEvents = [structs.CarState.ButtonEvent(pressed=True, type='cancel')]
    cs.steerFaultTemporary = status == 2
    cs.steerFaultPermanent = status == 3
    packets = [(0xC9, b'\x00' * 8, 0), (0xBE, b'\x00' * 6, 0), (0xBD, b'\x00' * 7, 0), (0x1E1, b'\x00' * 7, 0)]
    if tick % 10 == 0 and not missing:
      packets.append((0x184, b'\x00' if malformed else bytes([status << 3]) + b'\x00' * 7, 0))
    self.monitor.observe([(ns, packets)])
    self.monitor.update(cs, ns)
    self.monitor.command(ns, active)
    return cs

  def prime(self):
    for tick in range(40):
      self.step(tick)
    self.assertTrue(self.monitor.seen_active)

  def test_exact_recorded_profile_enabled(self):
    self.assertTrue(self.monitor.enabled)

  def test_both_brake_sources_radar_and_bsm_variants(self):
    for word in (0x205, 0x605, 0xA05, 0xE05):
      for bsm in (0, 16):
        with self.subTest(word=word, bsm=bsm):
          cp = params()
          cp.safetyConfigs[0].safetyParam = word
          cp.radarUnavailable = not bool(word & 0x800)
          cp.flags = bsm
          self.assertTrue(monitored_profile(cp))

  def test_neighbor_profiles_and_wrong_owners_refused(self):
    for field, value in [
      ('brand', 'hyundai'),
      ('carFingerprint', str(CAR.CHEVROLET_BOLT_EUV)),
      ('networkLocation', 'gateway'),
      ('passive', True),
      ('dashcamOnly', True),
      ('notCar', True),
      ('pcmCruise', False),
      ('openpilotLongitudinalControl', True),
      ('flags', int(GMFlags.PEDAL_LONG)),
      ('flags', int(GMFlags.NO_CAMERA)),
      ('radarUnavailable', False),
    ]:
      with self.subTest(field=field):
        cp = params()
        setattr(cp, field, value)
        self.assertFalse(monitored_profile(cp))
    for word in (5, 0x1005, 0xC163, 0xC151, 0x4087, 0x605 ^ 0x10, 0x605 ^ 0x100, 0x605 ^ 0x8000):
      with self.subTest(word=word):
        cp = params()
        cp.safetyConfigs[0].safetyParam = word
        self.assertFalse(monitored_profile(cp))

  def test_unresponsive_eps_latches_and_healthy_feedback_cannot_auto_clear(self):
    self.prime()
    for tick in range(40, 81):
      cs = self.step(tick, status=0)
    self.assertTrue(self.monitor.latched)
    self.assertTrue(cs.steerFaultTemporary)
    for tick in range(81, 120):
      cs = self.step(tick, status=1)
    self.assertTrue(self.monitor.latched)
    self.assertTrue(cs.steerFaultTemporary)

  def test_driver_pauses_and_positive_recovery(self):
    for pause in ('brake', 'paddle', 'cancel', 'main', 'driver', 'stopped', 'slow'):
      with self.subTest(pause=pause):
        self.monitor = SteeringAuthority(self.cp)
        self.prime()
        for tick in range(40, 80):
          self.step(tick, status=0, active=False, pause=pause)
        self.assertFalse(self.monitor.latched)
        for tick in range(80, 120):
          self.step(tick, status=1)
        self.assertFalse(self.monitor.latched)
        self.assertTrue(self.monitor.seen_active)

  def test_stale_eps_feed_remains_a_fault(self):
    self.prime()
    for tick in range(40, 90):
      cs = self.step(tick, missing=True, active=tick < 70)
    self.assertTrue(self.monitor.latched)
    self.assertTrue(cs.steerFaultTemporary)

  def test_malformed_eps_remains_a_fault(self):
    self.prime()
    for tick in range(40, 80):
      cs = self.step(tick, malformed=True)
    self.assertTrue(self.monitor.latched)
    self.assertTrue(cs.steerFaultTemporary)

  def test_factory_fault_flags_preserved(self):
    for status in (2, 3):
      with self.subTest(status=status):
        self.monitor = SteeringAuthority(self.cp)
        self.prime()
        for tick in range(40, 80):
          cs = self.step(tick, status=status, active=False)
        self.assertEqual(cs.steerFaultTemporary, status == 2)
        self.assertEqual(cs.steerFaultPermanent, status == 3)

  def test_main_cycle_uses_neutral_and_fresh_rearm(self):
    self.prime()
    for tick in range(40, 81):
      self.step(tick, status=0)
    self.assertTrue(self.monitor.latched)
    for tick in range(81, 86):
      self.step(tick, status=0, active=False, pause='main')
    self.assertTrue(self.monitor.latched)
    self.step(86, status=0, active=False)
    self.assertFalse(self.monitor.latched)
    for tick in range(87, 120):
      self.step(tick, status=1)
    self.assertFalse(self.monitor.latched)
    self.assertTrue(self.monitor.seen_active)


if __name__ == '__main__':
  unittest.main(verbosity=2)
