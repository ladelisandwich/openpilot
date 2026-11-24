"""MQB DEBUG OP-long uses a fresh native SET/RES release; release firmware denies the profile."""
import unittest
import pytest
from opendbc.car import structs
from opendbc.safety.tests import test_volkswagen_stock_aol as stock_tests
from opendbc.safety.tests.libsafety import libsafety_py


def configure(word):
  debug = libsafety_py.libsafety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) == 0
  owner = stock_tests.TestVolkswagenStockAol()
  owner.configure(word)
  owner.manual = word == 25
  return owner, debug


def engage(owner, button):
  assert owner.safety.safety_rx_hook(owner.packet('GRA_ACC_01', {button: 1}))
  owner.feed()
  owner.safety.aol_set_host_request(3)


@pytest.mark.parametrize('word', (9, 25))
@pytest.mark.parametrize('button', ('GRA_Tip_Setzen', 'GRA_Tip_Wiederaufnahme'))
def test_longitudinal_needs_ordinary_native_grant_and_never_resumes_after_brake(word, button):
  owner, debug = configure(word)
  try:
    for _ in range(12):
      owner.feed()
    owner.safety.aol_set_host_request(3)
    assert owner.safety.aol_get_permission_mask() == (1 if debug else 0)
    if not debug:
      assert not owner.safety.safety_tx_hook(owner.packet('ACC_06', {'ACC_Sollbeschleunigung_02': 0.5}))
      assert not owner.safety.safety_tx_hook(owner.packet('HCA_01', {'HCA_01_LM_Offset': 4, 'HCA_01_Sendestatus': 1}))
    else:
      engage(owner, button)
      assert owner.safety.get_controls_allowed()
      assert owner.safety.aol_get_permission_mask() == 3
      assert owner.safety.safety_tx_hook(owner.packet('ACC_06', {'ACC_Sollbeschleunigung_02': 0.5}))
      owner.feed(moving=False)
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 2  # Ordinary automatic stop-and-go remains available.
      owner.feed(brake=True)
      owner.safety.aol_set_host_request(3)
      assert not owner.safety.get_controls_allowed()
      assert owner.safety.aol_get_permission_mask() == 1
      owner.feed()
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 1
      assert not owner.safety.safety_tx_hook(owner.packet('ACC_06', {'ACC_Sollbeschleunigung_02': 0.5}))
      assert owner.safety.safety_tx_hook(owner.packet('ACC_06', {'ACC_Sollbeschleunigung_02': 3.01}))
      engage(owner, button)
      assert owner.safety.aol_get_permission_mask() == 3
  finally:
    owner.tearDown()


@pytest.mark.parametrize('word', (9, 25))
def test_reverse_and_permanent_eps_require_fresh_longitudinal_press(word):
  owner, debug = configure(word)
  try:
    for _ in range(12):
      owner.feed()
    if not debug:
      assert owner.safety.aol_get_permission_mask() == 0
    else:
      engage(owner, 'GRA_Tip_Setzen')
      owner.feed(gear=False)
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 0
      owner.feed()
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 1
      engage(owner, 'GRA_Tip_Wiederaufnahme')
      assert owner.safety.aol_get_permission_mask() == 3
      owner.feed(permanent=True)
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 0
      owner.feed()
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 0
      engage(owner, 'GRA_Tip_Setzen')
      assert owner.safety.aol_get_permission_mask() == 3
  finally:
    owner.tearDown()


@pytest.mark.parametrize('word', (9, 25))
def test_press_held_across_permanent_fault_cannot_regrant_on_release(word):
  owner, debug = configure(word)
  try:
    for _ in range(12):
      owner.feed()
    if debug:
      engage(owner, 'GRA_Tip_Setzen')
      original = owner.packet

      def held_packet(name, fields, bus=0):
        if name == 'GRA_ACC_01':
          fields = {**fields, 'GRA_Tip_Setzen': 1}
        return original(name, fields, bus)
      owner.packet = held_packet
      owner.feed(permanent=True)
      owner.feed()
      owner.packet = original
      owner.feed()
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 0
      engage(owner, 'GRA_Tip_Setzen')
      assert owner.safety.aol_get_permission_mask() == 3
    else:
      assert not owner.safety.safety_tx_hook(owner.packet('ACC_06', {'ACC_Sollbeschleunigung_02': 0.5}))
  finally:
    owner.tearDown()


@pytest.mark.parametrize('word', (9, 25))
def test_expired_host_and_source_lease_cannot_restore_longitudinal_from_stale_controls_allowed(word):
  owner, debug = configure(word)
  try:
    for _ in range(12):
      owner.feed()
    if debug:
      engage(owner, 'GRA_Tip_Setzen')
      owner.time += 300_001
      owner.safety.set_timer(owner.time)
      assert owner.safety.aol_get_permission_mask() == 0
      for _ in range(12):
        owner.feed()
      owner.safety.aol_set_host_request(3)
      assert owner.safety.aol_get_permission_mask() == 0
      assert not owner.safety.safety_tx_hook(owner.packet('ACC_06', {'ACC_Sollbeschleunigung_02': 0.5}))
      engage(owner, 'GRA_Tip_Setzen')
      assert owner.safety.aol_get_permission_mask() == 3
    else:
      assert owner.safety.aol_get_permission_mask() == 0
  finally:
    owner.tearDown()


class TestVolkswagenLongUnittest(unittest.TestCase):
  """Keep the actual native engagement assertions in unittest's coverage gate too."""
  def test_long_lifecycle(self):
    for word in (9, 25):
      for button in ('GRA_Tip_Setzen', 'GRA_Tip_Wiederaufnahme'):
        test_longitudinal_needs_ordinary_native_grant_and_never_resumes_after_brake(word, button)
      test_reverse_and_permanent_eps_require_fresh_longitudinal_press(word)
      test_press_held_across_permanent_fault_cannot_regrant_on_release(word)
      test_expired_host_and_source_lease_cannot_restore_longitudinal_from_stale_controls_allowed(word)
