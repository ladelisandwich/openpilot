"""The Mazda TI's steering smoothing, carried by the hook modeld and controlsd already call, set by the driver."""
from types import SimpleNamespace

from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.starpilot.car.hyundai.model_smoothing import seconds as hook_seconds
from openpilot.starpilot.car.mazda.model_smoothing import (DEFAULT_SECONDS, MAX_SECONDS, MIN_SECONDS, PARAM, clamp, options,
                                                          saved_seconds, seconds)

TI = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.RADAR_EMULATION)


class FakeParams:
  def __init__(self, value=None):
    self.value = value

  def get(self, key, return_default=False):
    assert key == PARAM
    return DEFAULT_SECONDS if self.value is None and return_default else self.value


def cp(brand="mazda", flags=TI, **kw):
  return SimpleNamespace(brand=brand, flags=flags, passive=kw.get("passive", False), dashcamOnly=kw.get("dashcamOnly", False),
                         notCar=False)


def test_range_and_default():
  assert options() == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10]
  assert (MIN_SECONDS, DEFAULT_SECONDS, MAX_SECONDS) == (0.01, 0.05, 0.10)


def test_saved_value_is_clamped_to_an_option():
  assert clamp(0.07) == 0.07
  assert clamp(0.074) == 0.07 and clamp(0.076) == 0.08
  assert clamp(0.0) == 0.01 and clamp(-1) == 0.01 and clamp(0.5) == 0.10
  assert clamp(None) == clamp("x") == clamp(float("nan")) == DEFAULT_SECONDS
  assert saved_seconds(FakeParams()) == DEFAULT_SECONDS
  assert saved_seconds(FakeParams(0.02)) == 0.02


def test_torque_interceptor_gets_the_setting_through_the_shared_hook(monkeypatch):
  assert seconds(cp(), FakeParams(0.03)) == 0.03
  assert seconds(cp(), FakeParams()) == DEFAULT_SECONDS
  from openpilot.starpilot.car.mazda import model_smoothing
  monkeypatch.setattr(model_smoothing, "saved_seconds", lambda params: 0.08)
  assert hook_seconds(cp()) == 0.08   # modeld and controlsd call this with CarParams only


def test_stock_gen1_and_inactive_configs_keep_upstream_timing():
  assert seconds(cp(flags=int(MazdaSafetyFlags.GEN1)), FakeParams(0.1)) == 0.
  assert seconds(cp(passive=True), FakeParams(0.1)) == 0.
  assert seconds(cp(dashcamOnly=True), FakeParams(0.1)) == 0.
  assert hook_seconds(cp(flags=int(MazdaSafetyFlags.GEN1))) == 0.
  assert seconds(cp(brand="toyota"), FakeParams(0.1)) == 0.
