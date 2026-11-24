from pathlib import Path
import tempfile

from openpilot.common.params import Params, ParamKeyType
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
from openpilot.starpilot.galaxy.settings import PAGES


def test_nudge_is_default_on_persistent_and_editable_without_a_vehicle():
  with tempfile.TemporaryDirectory() as directory:
    params = Params(directory)
    assert params.get_type('ExperimentalSpeedNudge') == ParamKeyType.BOOL
    assert params.get_default_value('ExperimentalSpeedNudge') is True
    owner = FeatureSettingsOwner(params, lambda group: group == 'preferences', vehicle_fingerprint=lambda: None)
    state = owner.snapshot('profiles', parked=False, system_long=False, lateral_context=False, metric=False)
    row = next(row for row in state.rows if row.key == 'ExperimentalSpeedNudge')
    assert row.label == 'Experimental Speed Nudge'
    assert row.value == row.default_value == 'On'
    assert row.available and row.vehicle_fingerprint is None
    assert owner.apply(FeatureSettingsRequest(row.key, row.source, 'Off'))
    assert Params(directory).get('ExperimentalSpeedNudge') is False
    assert Path(params.get_param_path(row.key)).read_bytes() == b'0'
    assert not owner.apply(FeatureSettingsRequest(row.key, row.source, 'On'))
    assert owner.apply(FeatureSettingsRequest(row.key, b'0', 'On'))
    assert Path(params.get_param_path(row.key)).read_bytes() == b'1'


def test_personality_editors_are_grouped_once_and_retain_their_routes():
  with tempfile.TemporaryDirectory() as directory:
    owner = FeatureSettingsOwner(Params(directory), lambda group: True, vehicle_fingerprint=lambda: None)
    def state(page):
      return owner.snapshot(page, parked=True, system_long=False, lateral_context=False, metric=False)
    hub = state('hub')
    assert [row.page for row in hub.rows if row.page == 'personalities'] == ['personalities']
    assert not any(row.page in ('aggressive', 'standard', 'relaxed', 'traffic') for row in hub.rows)
    grouped = state('personalities')
    assert grouped.title == 'Driving Personalities'
    assert [row.page for row in grouped.rows] == ['aggressive', 'standard', 'relaxed', 'traffic']
    assert 'personalities' in PAGES
