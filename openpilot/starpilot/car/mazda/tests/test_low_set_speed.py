"""Mazda Lower Minimum Set Speed: openpilot owns the set speed while its emulated radar is the ACC master."""
from types import SimpleNamespace

from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import IMPERIAL_INCREMENT, V_CRUISE_MIN, VCruiseHelper
from openpilot.starpilot.car.mazda.low_set_speed import PARAM, LowSetSpeed

ButtonType = car.CarState.ButtonEvent.Type
RE = int(MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR | MazdaSafetyFlags.RADAR_EMULATION)
HYBRID = RE | int(MazdaSafetyFlags.HYBRID_LONG)
PCM_MPH = 19.0


class FakeParams:
  def __init__(self, on):
    self.on = on

  def get_bool(self, key):
    assert key == PARAM
    return self.on


def cp(flags=HYBRID, brand="mazda", long=True):
  return SimpleNamespace(brand=brand, flags=flags, openpilotLongitudinalControl=long)


def radar(emulating):
  return SimpleNamespace(hybrid_radar=SimpleNamespace(emulating=emulating))


def test_ownership_rule():
  hook = LowSetSpeed(cp(), FakeParams(True))
  assert hook.owns_set_speed(radar(True))
  assert not hook.owns_set_speed(radar(False))      # MRCC is the master: the car's 19 mph floor
  assert not hook.owns_set_speed(None)
  assert LowSetSpeed(cp(RE), FakeParams(True)).owns_set_speed(None)    # plain radar emulation: always
  assert not LowSetSpeed(cp(), FakeParams(False)).owns_set_speed(radar(True))
  assert not LowSetSpeed(cp(int(MazdaSafetyFlags.GEN1)), FakeParams(True)).owns_set_speed(radar(True))
  assert not LowSetSpeed(cp(long=False), FakeParams(True)).owns_set_speed(radar(True))
  assert not LowSetSpeed(cp(brand="toyota"), FakeParams(True)).owns_set_speed(radar(True))


def car_state(buttons=(), standstill=False, available=True):
  return car.CarState(vEgo=5.0, cruiseState={"available": available, "speed": PCM_MPH * CV.MPH_TO_MS,
                                             "speedCluster": PCM_MPH * CV.MPH_TO_MS, "standstill": standstill},
                      buttonEvents=[car.CarState.ButtonEvent(type=t, pressed=p) for t, p in buttons])


def tap(helper, button, **kw):
  helper.update_v_cruise(car_state([(button, True)], **kw), enabled=True, is_metric=False)
  helper.update_v_cruise(car_state([(button, False)], **kw), enabled=True, is_metric=False)


def test_pcm_speed_until_owner_then_steps_below_19_mph():
  helper = VCruiseHelper(car.CarParams(pcmCruise=True, openpilotLongitudinalControl=True))
  helper.update_v_cruise(car_state(), enabled=True, is_metric=False)
  tap(helper, ButtonType.decelCruise)               # not owner: the PCM's speed, buttons do nothing here
  assert round(helper.v_cruise_kph * CV.KPH_TO_MPH, 1) == PCM_MPH

  helper.owner_set_speed = True
  helper.update_v_cruise(car_state(), enabled=True, is_metric=False)   # seeded from the PCM's speed
  assert round(helper.v_cruise_kph * CV.KPH_TO_MPH, 1) == PCM_MPH
  tap(helper, ButtonType.decelCruise)
  assert round(helper.v_cruise_kph * CV.KPH_TO_MPH) == 18
  assert helper.v_cruise_cluster_kph == helper.v_cruise_kph
  for _ in range(30):
    tap(helper, ButtonType.decelCruise)
  assert helper.v_cruise_kph == V_CRUISE_MIN        # about 5 mph, and no lower
  tap(helper, ButtonType.accelCruise)
  assert abs(helper.v_cruise_kph - (V_CRUISE_MIN + IMPERIAL_INCREMENT)) < 1.0

  low = helper.v_cruise_kph
  tap(helper, ButtonType.accelCruise, standstill=True)  # RES at a stop resumes, it does not step
  assert helper.v_cruise_kph == low

  helper.owner_set_speed = False                    # back to MRCC: the PCM's speed again
  helper.update_v_cruise(car_state(), enabled=True, is_metric=False)
  assert round(helper.v_cruise_kph * CV.KPH_TO_MPH, 1) == PCM_MPH


def test_owner_rising_edge_reseeds_from_the_pcm():
  helper = VCruiseHelper(car.CarParams(pcmCruise=True, openpilotLongitudinalControl=True))
  helper.owner_set_speed = True
  helper.update_v_cruise(car_state(), enabled=True, is_metric=False)
  for _ in range(5):
    tap(helper, ButtonType.decelCruise)
  helper.owner_set_speed = False
  helper.update_v_cruise(car_state(), enabled=True, is_metric=False)
  helper.owner_set_speed = True                     # a new emulation stint starts from the car's speed, not the old low one
  helper.update_v_cruise(car_state(), enabled=True, is_metric=False)
  assert round(helper.v_cruise_kph * CV.KPH_TO_MPH, 1) == PCM_MPH
