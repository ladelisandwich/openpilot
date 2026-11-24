import errno
import socket
import unittest
from unittest.mock import Mock, patch

from openpilot.starpilot.system.starport import bt_sockets


class TestBluetoothSocketFallback(unittest.TestCase):
  ADDRESS = '50:5A:65:8B:8B:82'

  def routes(self):
    return ((bt_sockets.connect_rfcomm, 1), (bt_sockets.connect_l2cap, 1))

  def capabilities(self):
    return patch.multiple(socket, AF_BLUETOOTH=31, BTPROTO_RFCOMM=3, create=True)

  def test_native_success_does_not_retry(self):
    with self.capabilities(), patch.object(bt_sockets, '_native') as native, patch.object(bt_sockets, '_libc_connect') as libc:
      for connect, port in self.routes():
        self.assertIs(connect(self.ADDRESS, port, 4), native.return_value)
      self.assertEqual(native.call_count, 2)
      libc.assert_not_called()

  def test_unimplemented_connect_uses_same_protocol_and_timeout(self):
    with self.capabilities(), patch.object(bt_sockets, '_native', side_effect=OSError(errno.ENOSYS, 'Function not implemented')), \
         patch.object(bt_sockets.time, 'sleep'), patch.object(bt_sockets, '_libc_connect') as libc:
      for connect, port in self.routes():
        self.assertIs(connect(self.ADDRESS.lower(), port, 4), libc.return_value)
      self.assertEqual(libc.call_args_list[0].args, (3, socket.SOCK_STREAM, self.ADDRESS, 1, 4))
      self.assertEqual(libc.call_args_list[1].args, (0, socket.SOCK_SEQPACKET, self.ADDRESS, 1, 4))

  def test_unimplemented_connect_retries_natively_before_falling_back(self):
    ok = Mock()
    with self.capabilities(), patch.object(bt_sockets, '_native', side_effect=[OSError(errno.ENOSYS, 'x'), ok]) as native, \
         patch.object(bt_sockets.time, 'sleep') as sleep, patch.object(bt_sockets, '_libc_connect') as libc:
      self.assertIs(bt_sockets.connect_rfcomm(self.ADDRESS, 1), ok)
    self.assertEqual(native.call_count, 2)
    sleep.assert_called_once_with(bt_sockets.ENOSYS_RETRY_S)
    libc.assert_not_called()

  def test_real_connection_failures_do_not_retry(self):
    for code in (errno.EHOSTDOWN, errno.ECONNREFUSED, errno.ETIMEDOUT, errno.EPERM, errno.EAFNOSUPPORT):
      with self.subTest(errno=code), self.capabilities(), \
           patch.object(bt_sockets, '_native', side_effect=OSError(code, 'connection failed')), \
           patch.object(bt_sockets, '_libc_connect') as libc:
        for connect, port in self.routes():
          with self.assertRaises(OSError) as caught:
            connect(self.ADDRESS, port)
          self.assertEqual(caught.exception.errno, code)
        libc.assert_not_called()

  def test_fallback_failure_propagates_without_loop(self):
    with self.capabilities(), patch.object(bt_sockets, '_native', side_effect=OSError(errno.ENOSYS, 'unimplemented')) as native, \
         patch.object(bt_sockets.time, 'sleep'), \
         patch.object(bt_sockets, '_libc_connect', side_effect=OSError(errno.EHOSTDOWN, 'host down')) as libc:
      with self.assertRaises(OSError) as caught:
        bt_sockets.connect_rfcomm(self.ADDRESS, 1)
      self.assertEqual(caught.exception.errno, errno.EHOSTDOWN)
      self.assertEqual(native.call_count, bt_sockets.ENOSYS_RETRIES + 1)
      libc.assert_called_once()

  def test_native_failure_closes_original_descriptor(self):
    original = Mock()
    original.connect.side_effect = OSError(errno.ENOSYS, 'unimplemented')
    with self.capabilities(), patch.object(socket, 'socket', return_value=original), patch.object(bt_sockets.time, 'sleep'), \
         patch.object(bt_sockets, '_libc_connect') as libc:
      self.assertIs(bt_sockets.connect_rfcomm(self.ADDRESS, 1), libc.return_value)
    self.assertEqual(original.close.call_count, bt_sockets.ENOSYS_RETRIES + 1)

  def test_missing_python_capability_uses_libc(self):
    with patch.object(bt_sockets, 'socket') as python_socket, patch.object(bt_sockets, '_libc_connect') as libc:
      del python_socket.AF_BLUETOOTH
      self.assertIs(bt_sockets.connect_rfcomm(self.ADDRESS, 1), libc.return_value)

  def test_invalid_inputs_never_open_socket(self):
    with patch.object(bt_sockets, '_native') as native, patch.object(bt_sockets, '_libc_connect') as libc:
      for address, channel in (('bad', 1), (self.ADDRESS, 0), (self.ADDRESS, 31)):
        with self.assertRaises(ValueError):
          bt_sockets.connect_rfcomm(address, channel)
      native.assert_not_called()
      libc.assert_not_called()
