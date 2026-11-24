"""The HKG healthy-stop exception cannot widen a Ford native denial."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.ford.interface import CarInterface
from openpilot.starpilot.aol.vehicle import policy_for
from openpilot.starpilot.car.hyundai.aol import expected_stopped_lateral_denial
from openpilot.starpilot.tests.ford_lifecycle_helpers import FORD_FACTORIES


@pytest.mark.parametrize('identity,base', FORD_FACTORIES)
@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('manual,offset', [(False, 0), (True, 0), (False, 4), (True, 4)])
def test_hkg_stop_exception_never_qualifies_ford(identity, base, alpha, manual, offset, monkeypatch):
  fp = gen_empty_fingerprint()
  if not manual:
    fp[offset][0x5A] = 8
  fp[offset + 2].update({0x3D6: 8, 0x186: 8})
  cp = CarInterface.get_params(identity, fp, [], alpha, True, False)
  policy = policy_for(cp)
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  assert cp.safetyConfigs[-1].safetyParam == (base | int(alpha) | (256 if manual else 0))
  transport = Mock(side_effect=AssertionError('Ford must reject the HKG-only exception before native transport'))
  monkeypatch.setattr('openpilot.starpilot.aol.transport_pause._configured_transport_native', transport)
  assert not expected_stopped_lateral_denial(cp, SimpleNamespace(), SimpleNamespace(), sm=None, axis_session_id='ford', now_ns=100)
  transport.assert_not_called()
  from openpilot.starpilot.car.hyundai.aol import source_deadline_supported
  from opendbc.car.hyundai.canfd_stock_aol import qualified as hkg_stock_qualified
  assert not source_deadline_supported(cp)
  assert not hkg_stock_qualified(cp, marked_only=True)
  from openpilot.starpilot.car.volkswagen.aol import qualified as vw_qualified, temporary_eps_restriction
  assert not vw_qualified(cp, marked_only=True)
  assert not temporary_eps_restriction(cp, SimpleNamespace())

  from openpilot.starpilot.car.subaru.smoothing import prepare, install, read_frozen, control_maximum, seconds, filter_action
  before = cp.to_dict()
  params = Mock()
  assert prepare(cp, object()).to_dict() == before
  assert install(cp, b'ford', object(), params) is None
  assert read_frozen(cp, params) == 0. and control_maximum(SimpleNamespace(CP=cp)) == 0.
  assert not params.mock_calls
  action = SimpleNamespace(desiredCurvature=.001, desiredAcceleration=.5)
  previous = SimpleNamespace(desiredCurvature=.002)
  for speed in (0., .3, 5., 20.):
    tau = seconds(speed, 0.)
    assert tau == 0.
    filter_action(action, previous, speed, tau)
    assert vars(action) == {'desiredCurvature': .001, 'desiredAcceleration': .5}
