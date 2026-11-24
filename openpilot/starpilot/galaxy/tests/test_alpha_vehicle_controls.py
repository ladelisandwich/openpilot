"""Galaxy projects the existing startup Alpha opt-in through its vehicle settings gateway."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.galaxy.settings import AuthorityContext, ContextSource, SettingsChanged, SettingsGateway

KEY = 'AlphaLongitudinalEnabled'


def fixture(identity='HYUNDAI_IONIQ_6'):
  params = Params()
  fingerprint = gen_empty_fingerprint()
  if identity == 'HYUNDAI_IONIQ_6':
    fingerprint[2].update({0x50: 16, 0x2A4: 24})
    fingerprint[1].update({0x1CF: 8, 0x1AA: 16, 0x35: 32, 0x175: 24, 0xA0: 24, 0xEA: 24,
                           0x1BA: 24, 0x1E5: 16, 0x36A: 16})
    fingerprint[0][0x3A5] = 24
  cp = interfaces[identity].get_params(identity, fingerprint, [], False, False, False)
  state = SimpleNamespace(parked=True, cp=cp, raw=cp.to_bytes())
  gateway = SettingsGateway(params, Mock(spec=ContextSource, sample=lambda: AuthorityContext(state.parked, state.cp, state.raw)), clock=lambda: 10)
  return params, state, gateway


def alpha_row(gateway):
  page = gateway.page('vehicle', 'session', b'generation')
  index = next(i for i, row in enumerate(page['rows']) if row['label'] == 'Alpha Longitudinal')
  return page, index


@pytest.mark.parametrize('identity', ['HYUNDAI_IONIQ_6', 'HYUNDAI_SONATA', 'HONDA_CIVIC_BOSCH'])
def test_alpha_current_supported_config_stays_visible_and_can_enable_then_disable(identity):
  with OpenpilotPrefix():
    params, state, gateway = fixture(identity)
    try:
      original = state.cp.to_dict()
      for expected in ('On', 'Off'):
        page, index = alpha_row(gateway)
        assert page['rows'][index]['available']
        intent = gateway.preview(page['view'], index, 0, 'session', b'generation', value=expected)
        assert intent['proposed'] == expected
        assert 'Emergency Braking' in intent['question']
        assert gateway.confirm(intent['intent'], 'session', b'generation')
        assert params.get_bool(KEY) == (expected == 'On')
        assert params.get_bool('OnroadCycleRequested')
        assert state.cp.to_dict() == original
    finally:
      gateway.close()


@pytest.mark.parametrize('change', ['release', 'unknown', 'unsupported', 'dashcam', 'notCar'])
def test_alpha_stale_enabled_preference_does_not_make_wrong_config_supported(change):
  with OpenpilotPrefix():
    params, state, gateway = fixture()
    try:
      params.put_bool(KEY, True, block=True)
      if change == 'release':
        params.put_bool('IsReleaseBranch', True, block=True)
      elif change == 'unknown':
        state.cp.carFingerprint = 'UNSUPPORTED_CAR'
      elif change == 'unsupported':
        state.cp = interfaces['TOYOTA_COROLLA_TSS2'].get_non_essential_params('TOYOTA_COROLLA_TSS2')
      else:
        setattr(state.cp, change if change == 'notCar' else 'dashcamOnly', True)
      state.raw = state.cp.to_bytes()
      page = gateway.page('vehicle', 'session', b'generation')
      assert not any(row['label'] == 'Alpha Longitudinal' for row in page['rows'])
      assert params.get_bool(KEY)
    finally:
      gateway.close()


def test_alpha_parked_authority_revocation_rejects_pending_save():
  with OpenpilotPrefix():
    params, state, gateway = fixture()
    try:
      page, index = alpha_row(gateway)
      intent = gateway.preview(page['view'], index, 0, 'session', b'generation', value='On')
      state.parked = False
      with pytest.raises(SettingsChanged):
        gateway.confirm(intent['intent'], 'session', b'generation')
      assert not params.get_bool(KEY)
      assert not params.get_bool('OnroadCycleRequested')
      page, index = alpha_row(gateway)
      assert not page['rows'][index]['available']
    finally:
      gateway.close()
