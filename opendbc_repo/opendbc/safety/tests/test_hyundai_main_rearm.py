import unittest

import pytest

from opendbc.can import CANPacker
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests import test_hyundai_classic_long_aol as classic_fixture


class _ClassicSources(unittest.TestCase):
  setUp = classic_fixture.TestHyundaiClassicLongAol.setUp
  feed = classic_fixture.TestHyundaiClassicLongAol.feed
  rx = classic_fixture.TestHyundaiClassicLongAol.rx


class Sources:
  def __init__(self, profile):
    self.profile = profile
    self.safety = libsafety_py.libsafety
    self.now = 1_000_000
    self.counter = 0
    self.next_health_tick = 1_000_000
    if profile == 'elantra':
      self.classic = _ClassicSources(methodName='runTest')
      self.classic.setUp()
      word, model = 0x800E, CarParams.SafetyModel.hyundai
      self.packer = self.classic.packer
    else:
      self.classic = None
      word, model = (0x5495 if profile == 'pe' else 0x5C95), CarParams.SafetyModel.hyundaiCanfd
      self.packer = CANPacker('hyundai_canfd_generated')
    self.safety.init_tests()
    self.safety.set_alternative_experience(0)
    assert self.safety.set_safety_hooks(model, word) == 0
    self.safety.set_timer(self.now)

  def packet(self, name, bus, values):
    address, data, _ = self.packer.make_can_msg(name, bus, values)
    return libsafety_py.make_CANPacket(address, bus, data)

  def frame(self, button=0, *, adaptive=False, normal=False, require_admission=True):
    if self.classic is not None:
      assert not normal
      self.classic.feed(word=0x800E, button=button, main=adaptive)
      stamp = self.classic.now - 10_000
    else:
      self.counter += 1
      self.safety.set_timer(self.now)
      entries = (
        ('ACCELERATOR', {'GEAR': 5}),
        ('TCS', {'DriverBraking': 0}),
        ('WHEEL_SPEEDS', dict.fromkeys(('WHL_SpdFLVal', 'WHL_SpdFRVal', 'WHL_SpdRLVal', 'WHL_SpdRRVal'), 40.)),
        ('MDPS', {'MDPS_EstStrAnglVal': 0., 'MDPS_ADAS_AciFltSig_Lv2': 0,
                  'MDPS_StrTqSnsrVal': 0., 'MDPS_OutTqVal': 0.}),
        ('CRUISE_BUTTONS', {'CRUISE_BUTTONS': button, 'ADAPTIVE_CRUISE_MAIN_BTN': int(adaptive),
                            'NORMAL_CRUISE_MAIN_BTN': int(normal)}),
      )
      for name, values in entries:
        count = self.counter % (16 if name == 'CRUISE_BUTTONS' else 256)
        accepted = self.safety.safety_rx_hook(self.packet(name, 1, dict(values, COUNTER=count)))
        if require_admission:
          assert accepted, name
      self.now += 20_000
      stamp = self.now - 20_000
    if stamp >= self.next_health_tick:
      self.safety.safety_tick()
      self.next_health_tick += 1_000_000
    if require_admission:
      assert self.safety.safety_config_valid()
      assert not self.safety.safety_test_toyota_rx_summary_invalid()
      assert not self.safety.get_relay_malfunction()

  def ready(self):
    for _ in range(6):
      self.frame()
    self.frame(adaptive=True)
    self.frame()
    self.frame(2)
    self.frame()
    assert self.safety.get_controls_allowed()
    assert self.safety.safety_tx_hook(self.active_command())

  def active_command(self):
    if self.classic is not None:
      return self.packer.make_can_msg_safety('SCC12', 0, {'aReqRaw': .1, 'aReqValue': .1})
    return self.packet('LFA_ALT', 1, {'ADAS_ActvACISta': 0, 'ADAS_ActvACILvl2Sta': 2,
                                    'ADAS_StrAnglReqVal': .1, 'ADAS_ACIAnglTqRedcGainVal': 0})


class TestHkgMainDebug:
  @pytest.mark.parametrize('profile,main', [('pe', 'adaptive'), ('pe', 'normal'), ('ev9', 'adaptive'),
                                         ('ev9', 'normal'), ('elantra', 'adaptive')])
  @pytest.mark.parametrize('button', [1, 2])
  def test_main_withdraws_and_held_enable_cannot_cross_neutral(self, profile, main, button):
    source = Sources(profile)
    source.ready()
    source.frame(button)
    for values in ({main: True}, {main: True}, {}):
      source.frame(button, **values)
      assert not source.safety.get_controls_allowed()
      assert not source.safety.safety_tx_hook(source.active_command())
    source.frame()
    assert not source.safety.get_controls_allowed()
    source.frame(button)
    assert not source.safety.get_controls_allowed()
    source.frame()
    tracked_off = main == 'adaptive' and profile != 'pe'
    assert bool(source.safety.get_controls_allowed()) == (not tracked_off)
    if tracked_off:
      source.frame(adaptive=True)
      source.frame()
      source.frame(button)
      source.frame()
      assert source.safety.get_controls_allowed()

  @pytest.mark.parametrize('profile', ['pe', 'ev9', 'elantra'])
  def test_only_original_main_tracked_profiles_require_main_before_set(self, profile):
    source = Sources(profile)
    for _ in range(6):
      source.frame()
    source.frame(2)
    source.frame()
    assert bool(source.safety.get_controls_allowed()) == (profile == 'pe')

  @pytest.mark.parametrize('profile', ['pe', 'ev9', 'elantra'])
  @pytest.mark.parametrize('button', [1, 2])
  def test_cancel_requires_neutral_before_a_new_enable_release(self, profile, button):
    source = Sources(profile)
    source.ready()
    source.frame(button)
    source.frame(4)
    assert not source.safety.get_controls_allowed()
    source.frame(button)
    source.frame()
    assert not source.safety.get_controls_allowed()
    source.frame(button)
    source.frame()
    assert source.safety.get_controls_allowed()


class TestHkgMainRelease:
  @pytest.mark.parametrize('profile', ['pe', 'ev9', 'elantra'])
  def test_physical_main_set_cancel_cannot_admit_release_long(self, profile):
    source = Sources(profile)
    assert not source.safety.safety_config_valid()
    for _ in range(6):
      source.frame(require_admission=False)
    for button, main in ((0, True), (0, False), (2, False), (0, False), (4, False),
                         (1, False), (0, False), (0, True), (0, False), (2, False), (0, False)):
      source.frame(button, adaptive=main, require_admission=False)
      assert not source.safety.get_controls_allowed()
      assert not source.safety.safety_tx_hook(source.active_command())
