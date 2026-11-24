from unittest.mock import Mock, patch

import pytest

from openpilot.starpilot.bluetooth.native_owner import NativeBluetoothOwner
from openpilot.starpilot.bluetooth.owner import AdapterBusyElsewhere, BluetoothOwner, BluetoothRejected


@pytest.mark.parametrize('operation, fields', [
  ('scan', {}), ('stop_scan', {}), ('pair', {'address': 'AA:BB:CC:DD:EE:FF', 'session': ('large', 'one')}),
  ('connect', {'address': 'AA:BB:CC:DD:EE:FF'}), ('disconnect', {'address': 'AA:BB:CC:DD:EE:FF'}),
  ('forget', {'address': 'AA:BB:CC:DD:EE:FF'}), ('power', {'enabled': False}),
])
def test_busy_native_owner_delegates_all_controls(operation, fields):
  owner = NativeBluetoothOwner(lambda: True)
  call = Mock(return_value=(200, {'devices': []}))
  with patch.object(BluetoothOwner, 'request', side_effect=AdapterBusyElsewhere('AA owns radio')), \
       patch('openpilot.starpilot.galaxy_bluetooth.phone_ui.galaxy_call', return_value=call) as factory:
    assert owner.request(operation, **fields) == {'devices': []}
    payload = {'operation': operation, **{k: v for k, v in fields.items() if k != 'session'}}
    call.assert_called_once_with('POST', '/api/bluetooth/action', payload)
    owner.snapshot(session=('large', 'one'))
    call.assert_called_with('GET', '/api/bluetooth/status')
    factory.assert_called_once()


def test_pairing_poll_response_and_close_use_the_same_galaxy_session():
  owner = NativeBluetoothOwner(lambda: True)
  call = Mock(return_value=(200, {'devices': [], 'pairing': {'state': 'pairing'}}))
  with patch.object(BluetoothOwner, 'request', side_effect=AdapterBusyElsewhere('busy')) as local, \
       patch('openpilot.starpilot.galaxy_bluetooth.phone_ui.galaxy_call', return_value=call) as factory:
    owner.request('pair', address='AA:BB:CC:DD:EE:FF', session=('large', 'one'))
    owner.snapshot(session=('large', 'one'))
    owner.request('pairing_response', prompt_id='a' * 32, accepted=True, value='', session=('large', 'one'))
    call.assert_called_with('POST', '/api/bluetooth/action',
                            {'operation': 'pairing_response', 'promptId': 'a' * 32, 'accepted': True, 'value': ''})
    owner.close()
    call.assert_called_with('POST', '/api/bluetooth/action', {'operation': 'cancel_pair'})
    local.assert_called_once()
    factory.assert_called_once()
    assert owner.galaxy is None


@pytest.mark.parametrize('error', [BluetoothRejected('park first', code='park_required'),
                                   BluetoothRejected('Pairing is in progress', code='busy')])
def test_local_denials_never_switch_to_galaxy(error):
  owner = NativeBluetoothOwner(lambda: True)
  with patch.object(BluetoothOwner, 'request', side_effect=error), \
       patch('openpilot.starpilot.galaxy_bluetooth.phone_ui.galaxy_call') as factory:
    with pytest.raises(BluetoothRejected):
      owner.request('scan')
    factory.assert_not_called()
  assert owner.galaxy is None


def test_delegated_page_open_never_scans():
  owner = NativeBluetoothOwner(lambda: True)
  owner.galaxy = Mock(return_value=(200, {'devices': []}))
  assert owner.request('scan', delegate=False) == {'devices': []}
  owner.galaxy.assert_called_once_with('GET', '/api/bluetooth/status')


def test_denials_do_not_retry_through_a_different_owner():
  owner = NativeBluetoothOwner(lambda: True)
  with patch.object(BluetoothOwner, 'request', side_effect=BluetoothRejected('park first', code='park_required')), \
       patch('openpilot.starpilot.galaxy_bluetooth.phone_ui.galaxy_call') as factory:
    with pytest.raises(BluetoothRejected, match='park first'):
      owner.request('power', enabled=False)
    factory.assert_not_called()
  call = Mock(return_value=(409, {'code': 'busy'}))
  owner.galaxy = call
  with patch.object(BluetoothOwner, 'request') as local:
    with pytest.raises(BluetoothRejected):
      owner.request('scan')
    local.assert_not_called()
    call.assert_called_once()
