from dataclasses import replace
from opendbc.car import gen_empty_fingerprint
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR
from openpilot.common.params import Params
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import row_change


def test_software_parked_row_confirmation_and_authority(tmp_path):
  params = Params(str(tmp_path))
  cp = CarInterface.get_params(CAR.SUBARU_IMPREZA_2020, gen_empty_fingerprint(), [], False, False, False)
  permitted = [True]
  owner = FeatureSettingsOwner(params, lambda group: permitted[0], vehicle_fingerprint=lambda: cp.carFingerprint,
                               vehicle_params=lambda: cp, configuration_vehicle=lambda: True)
  row = next(row for row in owner.snapshot('vehicle', parked=True, system_long=False, lateral_context=True, metric=True).rows
             if row.key == 'SubaruRedneckCruise')
  assert row.available and row.value == 'Off'
  request = row_change(row)
  assert request is not None and not owner.apply(request)
  permitted[0] = False
  assert not owner.apply(replace(request, confirmation=True))
  permitted[0] = True
  assert owner.apply(replace(request, confirmation=True))
  assert params.get_bool('SubaruRedneckCruise')
  assert not owner.apply(replace(request, confirmation=True))
  assert cp.safetyConfigs[0].safetyParam == 0 and not cp.openpilotLongitudinalControl
