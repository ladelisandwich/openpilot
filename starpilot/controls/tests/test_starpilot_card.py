from types import SimpleNamespace

import pytest

from opendbc.car.chrysler.values import CAR as CHRYSLER_CAR

from openpilot.common.params import ParamKeyType
from openpilot.starpilot.common.favorite_slots import (
  FAVORITE_ACTION_ACCEL_COUNTER,
  FAVORITE_ACTION_DISTANCE_INCREASE,
  FAVORITE_ACTION_TRAFFIC_MODE_COUNTER,
  FAVORITE_ACTION_TOGGLE_TRAFFIC_MODE,
  FAVORITE_SLOTS_PARAM,
)
from openpilot.starpilot.controls import starpilot_card as spc


class FakeParams:
  def __init__(self, *args, **kwargs):
    self._store = {}
    self._types = {
      FAVORITE_SLOTS_PARAM: ParamKeyType.JSON,
      "RedneckCruise": ParamKeyType.BOOL,
    }

  def get(self, key):
    return self._store.get(key)

  def get_bool(self, key):
    return bool(self._store.get(key, False))

  def put_bool(self, key, value):
    self._store[key] = bool(value)

  def put(self, key, value):
    self._store[key] = value

  def get_int(self, key, default=0):
    return int(self._store.get(key, default))

  def put_int(self, key, value):
    self._store[key] = int(value)

  def put_bool_nonblocking(self, key, value):
    self.put_bool(key, value)

  def get_type(self, key):
    return self._types.get(key, ParamKeyType.STRING)


class FakeSM(dict):
  def __init__(self, *args, updated=None, **kwargs):
    super().__init__(*args, **kwargs)
    self.updated = updated or {}


def make_sm():
  return FakeSM({
    "carControl": SimpleNamespace(longActive=False),
    "selfdriveState": SimpleNamespace(active=False, alertType=[], experimentalMode=False),
    "starpilotSelfdriveState": SimpleNamespace(alertType=[]),
    "starpilotPlan": SimpleNamespace(lateralCheck=True),
    "liveCalibration": SimpleNamespace(calPerc=100),
  }, updated={"starpilotPlan": False})


def make_toggles(**overrides):
  defaults = {
    "always_on_lateral": False,
    "always_on_lateral_lkas": False,
    "always_on_lateral_main": False,
    "always_on_lateral_pause_speed": 0.0,
    "bookmark_via_cancel": False,
    "bookmark_via_cancel_long": False,
    "bookmark_via_cancel_very_long": False,
    "experimental_mode_via_cancel": False,
    "experimental_mode_via_cancel_long": False,
    "experimental_mode_via_cancel_very_long": False,
    "force_coast_via_cancel": False,
    "force_coast_via_cancel_long": False,
    "force_coast_via_cancel_very_long": False,
    "bookmark_via_lkas": False,
    "conditional_experimental_mode": False,
    "experimental_mode_via_lkas": False,
    "force_coast_via_lkas": False,
    "pulse_and_glide_available": False,
    "pulse_and_glide_via_cancel": False,
    "pulse_and_glide_via_cancel_long": False,
    "pulse_and_glide_via_cancel_very_long": False,
    "pulse_and_glide_via_lkas": False,
    "lkas_allowed_for_aol": False,
    "main_cruise_aol_toggle": False,
    "main_cruise_slc_adopt": False,
    "pause_lateral_via_lkas": False,
    "pause_longitudinal_via_lkas": False,
    "speed_limit_controller": False,
    "switchback_mode_via_lkas": False,
    "traffic_mode_via_lkas": False,
  }
  defaults.update(overrides)
  return SimpleNamespace(**defaults)


def test_pulse_and_glide_requires_developer_access_and_active_longitudinal(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  sm = make_sm()
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  toggles = make_toggles(
    pulse_and_glide_available=True,
    pulse_and_glide_via_lkas=True,
  )

  card.handle_button_event("lkas", sm, toggles)
  assert card.pulse_and_glide is False

  sm["carControl"].longActive = True
  card.handle_button_event("lkas", sm, toggles)
  assert card.pulse_and_glide is True

  car_state = make_car_state(gas_pressed=True)
  result = card.update(car_state, starpilot_car_state, sm, toggles)
  assert result.pulseAndGlide is False


def test_pulse_and_glide_consumes_native_cancel_when_mapped(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  sm = make_sm()
  sm["carControl"].longActive = True
  toggles = make_toggles(
    pulse_and_glide_available=True,
    pulse_and_glide_via_cancel=True,
  )
  starpilot_car_state = SimpleNamespace(distancePressed=False, cancelPressed=True)

  press = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.cancel, pressed=True)])
  card.update(press, starpilot_car_state, sm, toggles)
  assert press.buttonEvents == []

  starpilot_car_state.cancelPressed = False
  release = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.cancel, pressed=False)])
  result = card.update(release, starpilot_car_state, sm, toggles)

  assert card.pulse_and_glide is True
  assert result.pulseAndGlide is True
  assert release.buttonEvents == []


def test_pulse_and_glide_consumes_lkas_when_mapped(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  sm = make_sm()
  sm["carControl"].longActive = True
  toggles = make_toggles(
    pulse_and_glide_available=True,
    pulse_and_glide_via_lkas=True,
  )
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)])

  result = card.update(car_state, starpilot_car_state, sm, toggles)

  assert card.pulse_and_glide is True
  assert result.pulseAndGlide is True
  assert car_state.buttonEvents == []


def test_pulse_and_glide_long_cancel_consumes_release_after_threshold(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  sm = make_sm()
  sm["carControl"].longActive = True
  toggles = make_toggles(
    pulse_and_glide_available=True,
    pulse_and_glide_via_cancel_long=True,
  )
  starpilot_car_state = SimpleNamespace(distancePressed=False, cancelPressed=True)

  for frame in range(card.long_press_threshold):
    button_events = [SimpleNamespace(type=spc.ButtonType.cancel, pressed=True)] if frame == 0 else []
    card.update(make_car_state(button_events=button_events), starpilot_car_state, sm, toggles)

  assert card.pulse_and_glide is True

  starpilot_car_state.cancelPressed = False
  release = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.cancel, pressed=False)])
  card.update(release, starpilot_car_state, sm, toggles)

  assert card.pulse_and_glide is True
  assert release.buttonEvents == []


def make_car_state(available=False, enabled=False, button_events=None, brake_pressed=False, gas_pressed=False):
  return SimpleNamespace(
    buttonEvents=button_events or [],
    cruiseState=SimpleNamespace(available=available, enabled=enabled),
    gearShifter=spc.GearShifter.drive,
    brakePressed=brake_pressed,
    gasPressed=gas_pressed,
    standstill=False,
    vEgo=15.0,
  )


def make_wrapped_button_event(button_type, pressed):
  return SimpleNamespace(type=SimpleNamespace(raw=int(button_type)), pressed=pressed)


@pytest.mark.parametrize(
  ("car_fingerprint", "expect_normalized_release"),
  (
    (spc.HYUNDAI_CAR.HYUNDAI_ELANTRA_HEV_2024, True),
    (spc.HYUNDAI_CAR.HYUNDAI_ELANTRA_2024, False),
  ),
)
def test_distance_release_normalization_is_limited_to_reported_elantra_hybrid(
    monkeypatch, tmp_path, car_fingerprint, expect_normalized_release,
):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=car_fingerprint),
    SimpleNamespace(alternativeExperience=0),
  )
  toggles = make_toggles(
    experimental_mode_via_distance=False,
    bookmark_via_distance=False,
    force_coast_via_distance=False,
    pulse_and_glide_via_distance=False,
    pause_lateral_via_distance=False,
    pause_longitudinal_via_distance=False,
    switchback_mode_via_distance=False,
  )
  sm = make_sm()
  starpilot_car_state = SimpleNamespace(distancePressed=True)

  card.update(make_car_state(), starpilot_car_state, sm, toggles)

  starpilot_car_state.distancePressed = False
  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.unknown, pressed=False)])
  card.update(car_state, starpilot_car_state, sm, toggles)

  assert any(
    be.type == spc.ButtonType.gapAdjustCruise and not be.pressed
    for be in car_state.buttonEvents
  ) is expect_normalized_release


def test_honda_lkas_button_can_toggle_always_on_lateral(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="honda"), SimpleNamespace(alternativeExperience=0))

  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral_lkas=True, lkas_allowed_for_aol=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.pauseLateral is False


def test_hyundai_lkas_button_can_start_aol_before_normal_engagement(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(available=True, button_events=[
    SimpleNamespace(type=spc.ButtonType.decelCruise, pressed=True),
    SimpleNamespace(type=spc.ButtonType.lkas, pressed=True),
  ])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, always_on_lateral_lkas=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = [SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)]
  ret = card.update(car_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is False
  assert ret.pauseLateral is False


def test_sonata_hybrid_lkas_button_can_start_aol_before_normal_engagement(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.HYUNDAI_SONATA_HYBRID),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(available=False, enabled=False, button_events=[SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, always_on_lateral_lkas=True, lkas_allowed_for_aol=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_sonata_hybrid_preserves_aol_latch_across_reverse(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.HYUNDAI_SONATA_HYBRID),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, always_on_lateral_lkas=True, lkas_allowed_for_aol=True)

  enabled_state = make_car_state(available=False, enabled=False, button_events=[SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)])
  ret = card.update(enabled_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  reverse_state = make_car_state(available=False, enabled=False)
  reverse_state.gearShifter = spc.GearShifter.reverse
  ret = card.update(reverse_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is False

  drive_state = make_car_state(available=False, enabled=False)
  ret = card.update(drive_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_hyundai_aol_does_not_auto_start_from_cruise_availability(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  ret = card.update(make_car_state(available=True), SimpleNamespace(distancePressed=False), make_sm(),
                    make_toggles(always_on_lateral=True, always_on_lateral_lkas=True))

  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False


def test_genesis_g90_main_aol_can_start_before_set(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.GENESIS_G90),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  ret = card.update(make_car_state(available=True), SimpleNamespace(distancePressed=False), make_sm(),
                    make_toggles(always_on_lateral=True, always_on_lateral_main=True))

  assert card.hyundai_aol_needs_engagement is False
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_other_legacy_hyundai_main_aol_still_waits_for_set(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.GENESIS_G80),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  ret = card.update(make_car_state(available=True), SimpleNamespace(distancePressed=False), make_sm(),
                    make_toggles(always_on_lateral=True, always_on_lateral_main=True))

  assert card.hyundai_aol_needs_engagement is True
  assert ret.alwaysOnLateralEnabled is False


def test_legacy_hyundai_main_aol_waits_for_main_button_permission(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.HYUNDAI_ELANTRA_2021),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )
  sm = make_sm()
  toggles = make_toggles(
    always_on_lateral=True,
    always_on_lateral_main=True,
    lkas_allowed_for_aol=True,
    main_cruise_aol_toggle=True,
  )
  starpilot_car_state = SimpleNamespace(distancePressed=False)

  ret = card.update(make_car_state(available=True), starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False

  ret = card.update(
    make_car_state(available=True, button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)]),
    starpilot_car_state,
    sm,
    toggles,
  )
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_nissan_main_aol_can_start_before_normal_engagement(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="nissan"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  ret = card.update(make_car_state(available=True), SimpleNamespace(distancePressed=False), make_sm(),
                    make_toggles(always_on_lateral=True, always_on_lateral_main=True))

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_hyundai_canfd_lkas_button_can_toggle_aol_before_engagement(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", flags=spc.HyundaiFlags.CANFD),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(available=True, button_events=[SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, always_on_lateral_lkas=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_hyundai_canfd_lkas_button_wrapped_enum_can_toggle_aol(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", flags=spc.HyundaiFlags.CANFD),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(available=True, button_events=[make_wrapped_button_event(spc.ButtonType.lkas, True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, always_on_lateral_lkas=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = [make_wrapped_button_event(spc.ButtonType.lkas, True)]
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False


def test_kia_forte_non_scc_main_cruise_button_toggles_aol_immediately(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(
      brand="hyundai",
      carFingerprint=spc.HYUNDAI_CAR.KIA_FORTE_2021_NON_SCC,
      flags=spc.HyundaiFlags.NON_SCC,
    ),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, main_cruise_aol_toggle=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = []
  car_state.cruiseState.available = False
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_genesis_g90_main_cruise_button_toggles_aol_immediately(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.GENESIS_G90),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, main_cruise_aol_toggle=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = []
  car_state.cruiseState.available = False
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_genesis_g70_main_cruise_button_waits_for_cruise_availability(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.GENESIS_G70_2020),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, main_cruise_aol_toggle=True)

  card.update(make_car_state(), starpilot_car_state, sm, toggles)
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False

  car_state.buttonEvents = []
  car_state.cruiseState.available = True
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = [SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)]
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = []
  car_state.cruiseState.available = False
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False


def test_legacy_hyundai_main_cruise_button_toggles_aol_immediately(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai", carFingerprint=spc.HYUNDAI_CAR.HYUNDAI_PALISADE),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, main_cruise_aol_toggle=True)

  card.update(make_car_state(), starpilot_car_state, sm, toggles)
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = [SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)]
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False


def test_hyundai_main_cruise_button_toggles_aol_immediately(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, main_cruise_aol_toggle=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = [SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)]
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False


def test_hyundai_main_cruise_button_wrapped_enum_can_toggle_aol(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  car_state = make_car_state()
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, main_cruise_aol_toggle=True)

  card.update(car_state, starpilot_car_state, sm, toggles)
  car_state.buttonEvents = [make_wrapped_button_event(spc.ButtonType.mainCruise, True)]
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = []
  car_state.cruiseState.available = False
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.buttonEvents = [make_wrapped_button_event(spc.ButtonType.mainCruise, True)]
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False


def test_hyundai_lda_platform_main_aol_waits_for_engagement_without_lkas_mapping(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )
  car_state = make_car_state(available=True)
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, always_on_lateral_main=True, lkas_allowed_for_aol=True)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is False

  sm["selfdriveState"].active = True
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralEnabled is True


def test_honda_mapped_main_cruise_button_keeps_immediate_toggle(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="honda"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )
  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)])
  toggles = make_toggles(always_on_lateral=True, main_cruise_aol_toggle=True, lkas_allowed_for_aol=True)

  ret = card.update(car_state, SimpleNamespace(distancePressed=False), make_sm(), toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_hyundai_main_cruise_button_adopts_slc_when_assigned_to_slc(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="hyundai"), SimpleNamespace(alternativeExperience=0))

  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.mainCruise, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  toggles = make_toggles(always_on_lateral=True, always_on_lateral_lkas=True,
                         main_cruise_slc_adopt=True, speed_limit_controller=True)

  initial_allowed = card.always_on_lateral_allowed
  card.update(car_state, starpilot_car_state, sm, toggles)

  assert card.always_on_lateral_allowed is initial_allowed
  assert card.params_memory.get_bool("SLCAdoptSpeedLimit") is True


def test_honda_lkas_button_pauses_lateral_when_cruise_is_active(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="honda"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )
  card.always_on_lateral_allowed = True

  car_state = make_car_state(available=True, enabled=True, button_events=[SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)])
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  sm = make_sm()
  sm["selfdriveState"].active = True
  toggles = make_toggles(always_on_lateral_lkas=True, lkas_allowed_for_aol=True)
  card.prev_active = True

  ret = card.update(car_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is False
  assert ret.pauseLateral is True

  ret = card.update(car_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.pauseLateral is False


def test_honda_main_aol_follows_cruise_main_without_manual_aol_button_mapping(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="honda"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  toggles = make_toggles(always_on_lateral_main=True, lkas_allowed_for_aol=True)
  ret = card.update(make_car_state(available=True), SimpleNamespace(distancePressed=False), make_sm(), toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_hyundai_main_aol_persists_after_brake_disengage_without_manual_aol_button_mapping(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="hyundai"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  sm = make_sm()
  toggles = make_toggles(always_on_lateral_main=True)
  starpilot_car_state = SimpleNamespace(distancePressed=False)

  sm["selfdriveState"].active = True
  enabled_state = make_car_state(available=True, enabled=True)
  ret = card.update(enabled_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  sm["selfdriveState"].active = False
  disengaged_state = make_car_state(available=True, enabled=False)
  ret = card.update(disengaged_state, starpilot_car_state, sm, toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_non_button_aol_platform_keeps_main_aol_when_main_cruise_is_mapped(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="gm"),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  toggles = make_toggles(always_on_lateral_main=True, main_cruise_aol_toggle=True)
  ret = card.update(make_car_state(available=True), SimpleNamespace(distancePressed=False), make_sm(), toggles)

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_main_aol_still_follows_cruise_main_for_other_platforms(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="toyota", carFingerprint="TOYOTA_TEST", pcmCruise=True),
                           SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL))

  ret = card.update(make_car_state(available=True), SimpleNamespace(distancePressed=False), make_sm(),
                    make_toggles(always_on_lateral_main=True))

  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True


def test_pacifica_hybrid_main_aol_waits_for_set_press(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(
    SimpleNamespace(brand="chrysler", carFingerprint=CHRYSLER_CAR.CHRYSLER_PACIFICA_2019_HYBRID, pcmCruise=True),
    SimpleNamespace(alternativeExperience=spc.ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL),
  )

  sm = make_sm()
  toggles = make_toggles(always_on_lateral_main=True)
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  car_state = make_car_state(available=True, enabled=False)

  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False

  car_state.cruiseState.enabled = True
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.cruiseState.enabled = False
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is True
  assert ret.alwaysOnLateralEnabled is True

  car_state.cruiseState.available = False
  ret = card.update(car_state, starpilot_car_state, sm, toggles)
  assert ret.alwaysOnLateralAllowed is False
  assert ret.alwaysOnLateralEnabled is False


def test_conditional_chill_wheel_override_cycles_manual_state(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  sm = make_sm()
  toggles = make_toggles(conditional_chill_mode=True)

  sm["selfdriveState"].experimentalMode = True
  card.handle_experimental_mode(sm, toggles)
  assert card.params_memory.get_int("CCStatus") == spc.CCStatus["USER_CHILL"]

  card.handle_experimental_mode(sm, toggles)
  assert card.params_memory.get_int("CCStatus") == spc.CCStatus["OFF"]


def test_cancel_button_short_press_can_run_independent_mapping(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  sm = make_sm()
  toggles = make_toggles(bookmark_via_cancel=True)
  starpilot_car_state = SimpleNamespace(distancePressed=False, cancelPressed=False)

  card.update(make_car_state(), starpilot_car_state, sm, toggles)
  assert card.params_memory.get_int("WheelButtonBookmarkCounter") == 0

  starpilot_car_state.cancelPressed = True
  ret = card.update(make_car_state(), starpilot_car_state, sm, toggles)
  assert ret.cancelLongPressed is False
  assert ret.cancelVeryLongPressed is False
  assert card.params_memory.get_int("WheelButtonBookmarkCounter") == 0

  starpilot_car_state.cancelPressed = False
  ret = card.update(make_car_state(), starpilot_car_state, sm, toggles)
  assert ret.cancelLongPressed is False
  assert ret.cancelVeryLongPressed is False
  assert card.params_memory.get_int("WheelButtonBookmarkCounter") == 1


def test_lkas_button_press_creates_bookmark(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="toyota"), SimpleNamespace(alternativeExperience=0))
  car_state = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.lkas, pressed=True)])

  card.update(car_state, SimpleNamespace(distancePressed=False), make_sm(), make_toggles(bookmark_via_lkas=True))

  assert card.params_memory.get_int("WheelButtonBookmarkCounter") == 1


def test_favorite_wheel_action_toggles_hidden_onroad_slot(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  card.params.put("RedneckCruise", False)
  card.params.put(FAVORITE_SLOTS_PARAM, [
    {"enabled": True, "show_onroad": False, "key": "RedneckCruise", "label": "Redneck Cruise"},
  ])

  card.handle_button_event("lkas", make_sm(), make_toggles(favorite_1_via_lkas=True))

  assert card.params.get_bool("RedneckCruise") is True
  assert card.params_memory.get_bool("StarPilotTogglesUpdated") is True


def test_favorite_wheel_action_can_press_virtual_resume(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  card.params.put(FAVORITE_SLOTS_PARAM, [
    {"enabled": True, "show_onroad": False, "key": FAVORITE_ACTION_DISTANCE_INCREASE, "label": "Distance + / RES"},
  ])

  card.handle_button_event("lkas", make_sm(), make_toggles(favorite_1_via_lkas=True))

  assert card.params_memory.get_int(FAVORITE_ACTION_ACCEL_COUNTER) == 1


def test_favorite_action_toggles_traffic_mode_when_longitudinal_control_is_active(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  card.params.put(FAVORITE_SLOTS_PARAM, [
    {"enabled": True, "show_onroad": True, "key": FAVORITE_ACTION_TOGGLE_TRAFFIC_MODE, "label": "Toggle Traffic Mode"},
  ])

  sm = make_sm()
  sm["carControl"].longActive = True
  card.handle_button_event("lkas", sm, make_toggles(favorite_1_via_lkas=True))

  card.update(make_car_state(), SimpleNamespace(distancePressed=False), sm, make_toggles())

  assert card.traffic_mode_enabled is True
  assert card.params_memory.get_int(FAVORITE_ACTION_TRAFFIC_MODE_COUNTER) == 1


def test_favorite_traffic_mode_action_is_consumed_when_not_active(monkeypatch, tmp_path):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)

  card = spc.StarPilotCard(SimpleNamespace(brand="gm"), SimpleNamespace(alternativeExperience=0))
  card.params.put(FAVORITE_SLOTS_PARAM, [
    {"enabled": True, "show_onroad": True, "key": FAVORITE_ACTION_TOGGLE_TRAFFIC_MODE, "label": "Toggle Traffic Mode"},
  ])

  sm = make_sm()
  card.handle_button_event("lkas", sm, make_toggles(favorite_1_via_lkas=True))
  card.update(make_car_state(), SimpleNamespace(distancePressed=False), sm, make_toggles())

  assert card.traffic_mode_enabled is False
  assert card._favorite_traffic_mode_counter == 1


MAZDA_HYBRID_FLAGS = 256 | 512  # MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG


def make_distance_toggles(**overrides):
  distance = {}
  for suffix in ("distance", "distance_long", "distance_very_long"):
    for action in ("experimental_mode", "bookmark", "force_coast", "pulse_and_glide", "pause_lateral",
                   "pause_longitudinal", "switchback_mode", "traffic_mode"):
      distance[f"{action}_via_{suffix}"] = False
  distance.update(overrides)
  return make_toggles(**distance)


def make_mazda_hybrid_card(monkeypatch, tmp_path, flags=MAZDA_HYBRID_FLAGS):
  monkeypatch.setattr(spc, "Params", FakeParams)
  monkeypatch.setattr(spc, "is_FrogsGoMoo", lambda: False)
  monkeypatch.setattr(spc, "ERROR_LOGS_PATH", tmp_path)
  return spc.StarPilotCard(SimpleNamespace(brand="mazda", flags=flags), SimpleNamespace(alternativeExperience=0))


def press_distance(card, frames, toggles, sm=None):
  """Hold the distance button for `frames` card cycles, then release it. Returns the release
  frame's car state, and whether the card reported a long press while it was held."""
  sm = sm or make_sm()
  starpilot_car_state = SimpleNamespace(distancePressed=False)
  long_pressed = False
  for i in range(frames):
    starpilot_car_state.distancePressed = True
    events = [SimpleNamespace(type=spc.ButtonType.gapAdjustCruise, pressed=True)] if i == 0 else []
    card.update(make_car_state(button_events=events), starpilot_car_state, sm, toggles)
    long_pressed |= starpilot_car_state.distanceLongPressed
  starpilot_car_state.distancePressed = False
  release = make_car_state(button_events=[SimpleNamespace(type=spc.ButtonType.gapAdjustCruise, pressed=False)])
  card.update(release, starpilot_car_state, sm, toggles)
  card.update(make_car_state(), starpilot_car_state, sm, toggles)
  return release, long_pressed


def has_distance_release(car_state):
  return any(be.type == spc.ButtonType.gapAdjustCruise and not be.pressed for be in car_state.buttonEvents)


def test_mazda_hybrid_buttons_need_both_flags(monkeypatch, tmp_path):
  assert make_mazda_hybrid_card(monkeypatch, tmp_path).mazda_hybrid
  assert not make_mazda_hybrid_card(monkeypatch, tmp_path, flags=256).mazda_hybrid
  assert not make_mazda_hybrid_card(monkeypatch, tmp_path, flags=512).mazda_hybrid


def test_mazda_hybrid_distance_button_with_conditional_experimental(monkeypatch, tmp_path):
  card = make_mazda_hybrid_card(monkeypatch, tmp_path)
  toggles = make_distance_toggles(conditional_experimental_mode=True)

  def ce_status():
    return card.params_memory.get_int("CEStatus")

  # Short tap in standard (MRCC): its normal job, the release still reaches the personality change
  release, _ = press_distance(card, 10, toggles)
  assert ce_status() == spc.CEStatus["OFF"]
  assert has_distance_release(release)

  # Long press: forced experimental (emulation), and only that -- no personality step
  release, long_pressed = press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  assert ce_status() == spc.CEStatus["USER_OVERRIDDEN"]
  assert not has_distance_release(release)
  assert not long_pressed

  # Short tap in forced experimental: back to standard (MRCC), the tap is used up
  release, _ = press_distance(card, 10, toggles)
  assert ce_status() == spc.CEStatus["OFF"]
  assert not has_distance_release(release)

  # Long press toggles back and forth
  press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  assert ce_status() == spc.CEStatus["USER_OVERRIDDEN"]
  release, _ = press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  assert ce_status() == spc.CEStatus["OFF"]
  assert not has_distance_release(release)


def test_mazda_hybrid_automatic_ce_trigger_counts_as_standard(monkeypatch, tmp_path):
  card = make_mazda_hybrid_card(monkeypatch, tmp_path)
  toggles = make_distance_toggles(conditional_experimental_mode=True)
  card.params_memory.put_int("CEStatus", spc.CEStatus["LEAD"])

  release, _ = press_distance(card, 10, toggles)
  assert card.params_memory.get_int("CEStatus") == spc.CEStatus["LEAD"]
  assert has_distance_release(release)

  press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  assert card.params_memory.get_int("CEStatus") == spc.CEStatus["USER_OVERRIDDEN"]


def test_mazda_hybrid_mode_persists_when_asked(monkeypatch, tmp_path):
  card = make_mazda_hybrid_card(monkeypatch, tmp_path)
  toggles = make_distance_toggles(conditional_experimental_mode=True)
  card.params.put_bool("PersistExperimentalState", True)

  press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  assert card.params.get_int("PersistedCEStatus") == spc.CEStatus["USER_OVERRIDDEN"]
  press_distance(card, 10, toggles)
  assert card.params.get_int("PersistedCEStatus") == spc.CEStatus["OFF"]


def test_mazda_hybrid_distance_button_without_conditional_experimental(monkeypatch, tmp_path):
  card = make_mazda_hybrid_card(monkeypatch, tmp_path)
  toggles = make_distance_toggles()

  press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  assert card.params.get_bool("ExperimentalMode")
  release, _ = press_distance(card, 10, toggles)
  assert not card.params.get_bool("ExperimentalMode")
  assert not has_distance_release(release)


def test_mazda_hybrid_very_long_press_leaves_the_mode_alone(monkeypatch, tmp_path):
  card = make_mazda_hybrid_card(monkeypatch, tmp_path)
  toggles = make_distance_toggles(conditional_experimental_mode=True, bookmark_via_distance_very_long=True)

  release, _ = press_distance(card, card.very_long_press_threshold + 10, toggles)
  assert card.params_memory.get_int("CEStatus") == spc.CEStatus["OFF"]
  assert card.params_memory.get_int("WheelButtonBookmarkCounter") == 1
  assert not has_distance_release(release)


def test_mazda_hybrid_forced_experimental_tap_skips_the_short_press_action(monkeypatch, tmp_path):
  card = make_mazda_hybrid_card(monkeypatch, tmp_path)
  toggles = make_distance_toggles(conditional_experimental_mode=True, bookmark_via_distance=True)

  press_distance(card, 10, toggles)
  assert card.params_memory.get_int("WheelButtonBookmarkCounter") == 1

  press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  press_distance(card, 10, toggles)
  assert card.params_memory.get_int("WheelButtonBookmarkCounter") == 1
  assert card.params_memory.get_int("CEStatus") == spc.CEStatus["OFF"]


def test_mazda_hybrid_distance_button_is_stock_in_safe_mode(monkeypatch, tmp_path):
  card = make_mazda_hybrid_card(monkeypatch, tmp_path)
  toggles = make_distance_toggles(conditional_experimental_mode=True, safe_mode=True)

  release, long_pressed = press_distance(card, spc.CRUISE_LONG_PRESS + 10, toggles)
  assert card.params_memory.get_int("CEStatus") == spc.CEStatus["OFF"]
  assert has_distance_release(release)
  assert long_pressed
