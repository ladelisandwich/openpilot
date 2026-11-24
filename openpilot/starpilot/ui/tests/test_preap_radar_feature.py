from opendbc.car.tesla.tests.test_preap_stock import params
from openpilot.common.params import Params
from openpilot.starpilot.car.tesla.radar_settings import DOCUMENT_KEY, read_radar_settings
from openpilot.starpilot.galaxy.settings import AuthorityContext, SettingsGateway
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest


def owner(saved, cp, parked=True):
  return FeatureSettingsOwner(saved, lambda group: parked and group == "parked_preferences",
    vehicle_fingerprint=lambda: cp.carFingerprint, vehicle_params=lambda: cp)


def request(row, value):
  return FeatureSettingsRequest(row.key, row.source, value, confirmation=True,
    vehicle_fingerprint=row.vehicle_fingerprint, capability=row.capability, dependencies=row.dependencies)


def test_shared_owner_changes_nosecone_to_off_atomically_and_preserves_legacy(tmp_path):
  saved, cp = Params(str(tmp_path)), params()
  saved.put_bool("NAPRadarEnabled", True, block=True)
  saved.put_bool("NAPRadarBehindNosecone", True, block=True)
  feature = owner(saved, cp)
  row = next(r for r in feature.snapshot("vehicle", parked=True, system_long=False, lateral_context=False, metric=True).rows
             if r.key == "preap_radar:mode")
  assert row.value == "Behind nosecone" and row.available
  assert feature.apply(request(row, "Off"))
  assert read_radar_settings(saved).valid and not read_radar_settings(saved).enabled
  assert saved.get_bool("NAPRadarEnabled") and saved.get_bool("NAPRadarBehindNosecone")
  assert saved.get(DOCUMENT_KEY) is not None


def test_onroad_and_changed_source_cannot_commit_hardware_choice(tmp_path):
  saved, cp = Params(str(tmp_path)), params()
  feature = owner(saved, cp)
  row = feature.preap_radar.rows()[0]
  change = request(row, "Front")
  saved.put_bool("NAPRadarEnabled", True, block=True)
  assert not feature.apply(change)
  row = owner(saved, cp, parked=False).preap_radar.rows()[0]
  assert not row.available


def test_galaxy_vehicle_page_uses_same_radar_setup_owner(tmp_path):
  saved, cp = Params(str(tmp_path)), params()

  class Context:
    def sample(self):
      return AuthorityContext(True, cp, b"preap-cp")

  gateway = SettingsGateway(saved, Context(), clock=lambda: 10.0)
  page = gateway.page("vehicle", "session", b"generation")
  index = next(i for i, row in enumerate(page["rows"]) if row["label"] == "Hardware radar")
  intent = gateway.preview(page["view"], index, 1, "session", b"generation")
  assert gateway.confirm(intent["intent"], "session", b"generation")
  settings = read_radar_settings(saved)
  assert settings.valid and settings.enabled and not settings.behind_nosecone
