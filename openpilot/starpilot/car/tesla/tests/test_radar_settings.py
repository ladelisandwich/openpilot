import pytest

from openpilot.starpilot.car.tesla.radar_settings import read_radar_settings


@pytest.mark.parametrize("raw,valid", ((b"0", True), (b"-0.25", True), (b"nan", False), (b"inf", False),
                                    (b"1e300", False), (b"malformed", False)))
def test_offset_is_finite_and_representable_in_startup_context(tmp_path, raw, valid):
  class Saved:
    def get_param_path(self, name):
      return str(tmp_path / name)

  (tmp_path / "NAPRadarEnabled").write_bytes(b"1")
  (tmp_path / "NAPRadarOffset").write_bytes(raw)
  settings = read_radar_settings(Saved())
  assert settings.valid == valid


def test_nosecone_cannot_select_radar_without_radar_enabled(tmp_path):
  class Saved:
    def get_param_path(self, name):
      return str(tmp_path / name)

  (tmp_path / "NAPRadarBehindNosecone").write_bytes(b"1")
  assert not read_radar_settings(Saved()).valid


@pytest.mark.parametrize("raw", (b"nan", b"1e300", b"malformed"))
def test_disabled_stock_ignores_irrelevant_offset(tmp_path, raw):
  class Saved:
    def get_param_path(self, name):
      return str(tmp_path / name)

  (tmp_path / "NAPRadarOffset").write_bytes(raw)
  settings = read_radar_settings(Saved())
  assert settings.valid and not settings.enabled and settings.offset == 0.0


def test_invalid_snapshot_cannot_fall_back_to_stock_even_if_admitted_flag_is_true():
  from opendbc.car import structs
  from opendbc.car.tesla.tests.test_preap_stock import params
  from openpilot.starpilot.car.tesla.preap_preferences import prepare_stock
  from openpilot.starpilot.car.tesla.radar_settings import RadarSettings

  cp = params()
  prepare_stock(cp, True, RadarSettings(False, False, 0.0, False))
  assert cp.dashcamOnly and cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput


def test_startup_reads_one_radar_snapshot(monkeypatch, tmp_path):
  from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences
  from openpilot.starpilot.car.tesla import radar_settings
  from openpilot.starpilot.car.tesla.radar_settings import RadarSettings

  class Saved:
    def get_param_path(self, name):
      return str(tmp_path / name)

  calls = []

  def read_once(params):
    calls.append(params)
    assert len(calls) == 1
    return RadarSettings(False, False, 0.0, True)

  monkeypatch.setattr(radar_settings, "read_radar_settings", read_once)
  preferences = VehicleStartupPreferences.read(Saved(), enabled=False)
  assert preferences.tesla_preap_stock and len(calls) == 1


def test_canonical_document_is_authoritative_over_legacy_hardware_choices(tmp_path):
  from openpilot.starpilot.car.tesla.radar_settings import DOCUMENT_KEY, encode

  class Saved:
    def get_param_path(self, name):
      return str(tmp_path / name)

  (tmp_path / "NAPRadarEnabled").write_bytes(b"1")
  (tmp_path / "NAPRadarBehindNosecone").write_bytes(b"1")
  (tmp_path / DOCUMENT_KEY).write_bytes(encode("off", 0.25))
  settings = read_radar_settings(Saved())
  assert settings.valid and not settings.enabled and settings.offset == 0.25
  (tmp_path / DOCUMENT_KEY).write_bytes(b"{invalid")
  assert not read_radar_settings(Saved()).valid


@pytest.mark.parametrize('raw', (b'{"version":1,"mode":"front","mode":"off","offset":0}',
                                b'{"version":1,"mode":"front","offset":0,"extra":1}',
                                b'{"version":true,"mode":"front","offset":0}'))
def test_unknown_or_ambiguous_document_does_not_admit_radar(tmp_path, raw):
  from openpilot.starpilot.car.tesla.radar_settings import DOCUMENT_KEY

  class Saved:
    def get_param_path(self, name):
      return str(tmp_path / name)

  (tmp_path / DOCUMENT_KEY).write_bytes(raw)
  assert not read_radar_settings(Saved()).valid
