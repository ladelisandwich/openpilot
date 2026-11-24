"""Real shared native/Galaxy scalar persistence and commit-time authority."""
from dataclasses import replace
import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR
from openpilot.common.params import Params
from openpilot.starpilot.galaxy.settings import AuthorityContext, SettingsGateway
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import row_change


class Context:
  def __init__(self, cp):
    self.cp, self.parked = cp, True

  def sample(self):
    reader = self.cp.as_reader()
    return AuthorityContext(self.parked, reader, reader.as_builder().to_bytes())


def setup(tmp_path):
  params = Params(str(tmp_path))
  params.put_bool('DeveloperUI', True, block=True)
  params.put_bool('SafeMode', False, block=True)
  cp = CarInterface.get_params(CAR.SUBARU_OUTBACK_2023, gen_empty_fingerprint(), [], False, False, False)
  context = Context(cp)
  owner = FeatureSettingsOwner(params, lambda _: context.parked, vehicle_fingerprint=lambda: context.cp.carFingerprint,
                               vehicle_params=lambda: context.cp.as_reader(), configuration_vehicle=lambda: True)
  return params, context, owner


def native_row(owner, parked=True):
  return next(row for row in owner.snapshot('vehicle', parked=parked, system_long=False, lateral_context=True, metric=True).rows
              if row.key == 'LatSmoothSeconds')


def test_native_parked_scalar_zero_off_and_exact_source(tmp_path):
  params, context, owner = setup(tmp_path)
  row = native_row(owner)
  assert row.available and row.value == '0.4' and row.minimum == 0. and row.maximum == 2. and row.step == .005
  request = row_change(row)
  assert request is not None
  assert owner.apply(replace(request, value='0'))
  assert params.get('LatSmoothSeconds') == 0.
  assert not owner.apply(request)
  row = native_row(owner)
  request = row_change(row)
  assert request is not None
  context.parked = False
  assert not native_row(owner, parked=False).available and not owner.apply(request)
  assert params.get('LatSmoothSeconds') == 0.


@pytest.mark.parametrize('change', ['developer', 'safe', 'vehicle', 'scalar'])
def test_native_commit_rechecks_all_dependencies(tmp_path, change):
  params, context, owner = setup(tmp_path)
  request = row_change(native_row(owner))
  assert request is not None
  if change == 'developer':
    params.put_bool('DeveloperUI', False, block=True)
  elif change == 'safe':
    params.put_bool('SafeMode', True, block=True)
  elif change == 'vehicle':
    context.cp.steerRatio += 1.
  else:
    params.put('LatSmoothSeconds', 1., block=True)
  assert not owner.apply(request)


def test_actual_galaxy_editor_and_native_share_saved_source(tmp_path):
  params, context, owner = setup(tmp_path)
  gateway = SettingsGateway(params, context)
  try:
    for value in ('2', '0'):
      page = gateway.page('vehicle', 'subaru-session', b'epoch')
      index, row = next((i, row) for i, row in enumerate(page['rows']) if row['label'] == 'Subaru model curvature smoothing')
      assert row['available'] and row['step'] == .005
      intent = gateway.preview(page['view'], index, 0, 'subaru-session', b'epoch', value=value)
      assert gateway.confirm(intent['intent'], 'subaru-session', b'epoch')
      assert native_row(owner).value == value
    page = gateway.page('vehicle', 'subaru-session', b'epoch')
    index = next(i for i, row in enumerate(page['rows']) if row['label'] == 'Subaru model curvature smoothing')
    intent = gateway.preview(page['view'], index, 0, 'subaru-session', b'epoch', value='1')
    context.parked = False
    assert not gateway.confirm(intent['intent'], 'subaru-session', b'epoch')
    assert params.get('LatSmoothSeconds') == 0.
  finally:
    gateway.close()



def test_developer_gate_preserves_saved_tune_and_uses_next_drive_default(tmp_path):
  from openpilot.starpilot.car.subaru.smoothing import maximum_for
  from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences
  params, context, owner = setup(tmp_path)
  params.put('LatSmoothSeconds', 2., block=True)
  row = next(row for row in owner.snapshot('vehicle', parked=True, system_long=False, lateral_context=True, metric=True).rows
             if row.key == 'DeveloperUI')
  request = row_change(row)
  assert request is not None and not owner.apply(request)
  before = context.cp.to_dict()
  assert owner.apply(replace(request, confirmation=True))
  assert not params.get_bool('DeveloperUI') and params.get('LatSmoothSeconds') == 2.
  assert not native_row(owner).available
  choice = VehicleStartupPreferences.read(params, enabled=True)
  assert maximum_for(context.cp, choice.subaru_smoothing) == .4
  assert context.cp.to_dict() == before
  assert not owner.apply(replace(request, confirmation=True))
