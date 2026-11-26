"""The card's Mazda hooks: the hybrid longitudinal mode switch on the distance-increase button, the horn-tap bookmark,
and modeld's wiper gate."""
from types import SimpleNamespace

from openpilot.common.test import OpenpilotTestCase
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.starpilot.car.mazda.hybrid_buttons import HornBookmarks, MazdaCardHooks, mazda_hybrid_buttons_active, planner_experimental
from openpilot.starpilot.car.mazda.wipers import WASHER_BLOCK_S, MazdaWiperGate


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)


HYBRID = SimpleNamespace(brand="mazda", flags=int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR |
                                                  MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG))
STOCK = SimpleNamespace(brand="mazda", flags=int(MazdaSafetyFlags.GEN1))


def press(hooks, frames, **state):
  for _ in range(frames):
    hooks.update(SimpleNamespace(distance_increase_button=1, **state))
  hooks.update(SimpleNamespace(distance_increase_button=0, **state))


class TestMazdaCardHooks(OpenpilotTestCase):
  def test_active_only_with_hybrid_long(self):
    self.assertTrue(mazda_hybrid_buttons_active(HYBRID))
    self.assertFalse(mazda_hybrid_buttons_active(STOCK))
    self.assertFalse(mazda_hybrid_buttons_active(SimpleNamespace(brand="toyota", flags=HYBRID.flags)))

  def test_long_press_toggles_experimental_mode_and_a_tap_only_leaves_it(self):
    params = FakeParams(ExperimentalMode=False)
    hooks = MazdaCardHooks(HYBRID, params, long_press_frames=50)
    press(hooks, 10)                      # a tap in MRCC does nothing
    self.assertFalse(params.get_bool("ExperimentalMode"))
    press(hooks, 60)                      # a long press: emulation
    self.assertTrue(params.get_bool("ExperimentalMode"))
    press(hooks, 10)                      # a tap in emulation: back to MRCC
    self.assertFalse(params.get_bool("ExperimentalMode"))
    press(hooks, 60)
    press(hooks, 60)                      # and a long press toggles back out again
    self.assertFalse(params.get_bool("ExperimentalMode"))

  def test_planner_mode_is_the_master_choice(self):
    # emulation (ExperimentalMode on) always plans in Experimental Mode, whatever Conditional Mode decided, and MRCC
    # never does: StarPilot 6's forced CEStatus, so the planner and the car controller's ACC master always agree
    for conditional in (False, True):
      self.assertTrue(planner_experimental(conditional, True))
      self.assertFalse(planner_experimental(conditional, False))

  def test_no_mode_switch_without_hybrid_long(self):
    params = FakeParams(ExperimentalMode=False)
    hooks = MazdaCardHooks(STOCK, params)
    press(hooks, 100)
    self.assertFalse(params.get_bool("ExperimentalMode"))

  def test_one_bookmark_per_horn_press_when_enabled(self):
    on = HornBookmarks(True)
    self.assertEqual(sum(on.update(bool(horn)) for horn in (0, 1, 1, 1, 0, 0, 1, 0)), 2)
    off = HornBookmarks(False)
    self.assertEqual(sum(off.update(bool(horn)) for horn in (0, 1, 0)), 0)

  def test_card_never_publishes_bookmarks(self):
    # the UI owns userBookmark: a second publisher in card would raise on send and take card down
    import inspect
    from openpilot.starpilot.car.mazda import hybrid_buttons
    self.assertNotIn("pub_sock", inspect.getsource(hybrid_buttons))

  def test_stale_button_never_switches_the_mode(self):
    params = FakeParams(ExperimentalMode=False)
    hooks = MazdaCardHooks(HYBRID, params, long_press_frames=50)
    for _ in range(60):
      hooks.update(SimpleNamespace(distance_increase_button=1))
    for _ in range(20):                   # CAN drops out with the button reading pressed
      hooks.update(SimpleNamespace(distance_increase_button=1), can_valid=False)
    hooks.update(SimpleNamespace(distance_increase_button=0))   # recovery: not a long-press release
    self.assertFalse(params.get_bool("ExperimentalMode"))


class TestMazdaWiperGate(OpenpilotTestCase):
  def test_blocks_while_wiping_and_after_the_washer(self):
    gate = MazdaWiperGate(active=True)
    now = 10 * 10**9
    self.assertFalse(gate.blocks_auto_lane_change(now))
    gate.update(wipers_active=True, washer_active=False, now_ns=now)
    self.assertTrue(gate.blocks_auto_lane_change(now))
    gate.update(wipers_active=False, washer_active=False, now_ns=now)
    self.assertFalse(gate.blocks_auto_lane_change(now))
    gate.update(wipers_active=False, washer_active=True, now_ns=now)
    gate.update(wipers_active=False, washer_active=False, now_ns=now + 10**9)
    self.assertTrue(gate.blocks_auto_lane_change(now + int((WASHER_BLOCK_S - 0.1) * 10**9)))
    self.assertFalse(gate.blocks_auto_lane_change(now + int((WASHER_BLOCK_S + 0.1) * 10**9)))

  def test_inert_when_the_toggle_is_off(self):
    gate = MazdaWiperGate(active=False)
    gate.update(wipers_active=True, washer_active=True, now_ns=0)
    self.assertFalse(gate.blocks_auto_lane_change(0))
