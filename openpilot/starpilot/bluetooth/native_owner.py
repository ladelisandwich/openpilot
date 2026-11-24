"""Native settings use Galaxy when Android Auto already owns the Bluetooth adapter."""
from openpilot.starpilot.bluetooth.owner import AdapterBusyElsewhere, BluetoothOwner, BluetoothRejected, BluetoothUnavailable


class NativeBluetoothOwner(BluetoothOwner):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.galaxy = None
    self.delegated_pairing = False
    self.delegated_scan = False

  def _galaxy(self, operation, **kwargs):
    if self.galaxy is None:
      from openpilot.starpilot.galaxy_bluetooth.phone_ui import galaxy_call
      self.galaxy = galaxy_call()  # Keep the same authenticated session for prompts and polling.
    if operation == 'status':
      status, result = self.galaxy('GET', '/api/bluetooth/status')
    else:
      payload = {'operation': operation}
      for key, value in kwargs.items():
        if key in ('address', 'enabled', 'prompt_id', 'accepted') and value is not None:
          payload['promptId' if key == 'prompt_id' else key] = value
      if operation == 'pairing_response':
        payload['value'] = kwargs.get('value', '')
      status, result = self.galaxy('POST', '/api/bluetooth/action', payload)
    if status == 409:
      raise BluetoothRejected('Bluetooth change was not accepted', code=result.get('code', 'busy'))
    if status != 200 or not isinstance(result.get('devices'), list):
      raise BluetoothUnavailable('Galaxy Bluetooth service is unavailable')
    if operation in ('pair', 'accept_pair'):
      self.delegated_pairing = True
    elif operation == 'cancel_pair':
      self.delegated_pairing = False
    if operation == 'scan':
      self.delegated_scan = True
    elif operation in ('stop_scan', 'pair', 'accept_pair'):
      self.delegated_scan = False
    return result

  def request(self, operation, *, delegate=True, **kwargs):
    session = kwargs.get('session')
    if session is not None and not self.session_valid(session):
      raise BluetoothRejected('Pairing session expired', code='session_expired')
    if self.galaxy is None:
      try:
        return super().request(operation, **kwargs)
      except AdapterBusyElsewhere:
        if not delegate:
          return super().snapshot(session=session)
    elif not delegate:
      return self.snapshot(session=session)
    return self._galaxy(operation, **kwargs)

  def snapshot(self, *, session=None):
    if self.galaxy is not None:
      return self._galaxy('status')
    return super().snapshot(session=session)

  def close(self):
    try:
      if self.galaxy is not None:
        for operation, needed in (('cancel_pair', self.delegated_pairing), ('stop_scan', self.delegated_scan)):
          if needed:
            try:
              self._galaxy(operation)
            except (BluetoothRejected, BluetoothUnavailable, OSError, RuntimeError):
              pass  # The daemon also bounds scans and cancels expired pairing sessions.
    finally:
      self.galaxy = None
      self.delegated_pairing = self.delegated_scan = False
      super().close()
