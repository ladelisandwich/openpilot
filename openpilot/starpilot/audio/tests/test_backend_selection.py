from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from openpilot.selfdrive.ui import soundd
from openpilot.starpilot.audio import native_playback


def device():
  return SimpleNamespace(_terminate=Mock(), _initialize=Mock(), OutputStream=Mock(return_value=object()))


def daemon():
  obj = soundd.Soundd.__new__(soundd.Soundd)
  obj.native_audio = None
  return obj


def test_host_backend_reports_python_without_attempting_native():
  obj, sd = daemon(), device()
  with patch.object(soundd, 'PC', True), patch.object(native_playback, 'NativePlayback') as ctor:
    assert soundd.Soundd.get_stream.__wrapped__(obj, sd) is sd.OutputStream.return_value
  assert (obj.audio_backend, obj.audio_fallback_reason) == ('python', 'host')
  ctor.assert_not_called()
  assert sd.OutputStream.call_args.kwargs == {'channels': 1, 'samplerate': 48000, 'callback': obj.callback, 'blocksize': 4096}


def test_native_backend_reports_native_without_python_stream():
  obj, sd = daemon(), device()
  playback = Mock()
  with patch.object(soundd, 'PC', False), patch.object(native_playback, 'NativePlayback', return_value=playback):
    assert soundd.Soundd.get_stream.__wrapped__(obj, sd) is playback.stream.return_value
  assert obj.native_audio is playback
  assert (obj.audio_backend, obj.audio_fallback_reason) == ('native', None)
  sd.OutputStream.assert_not_called()
  playback.close.assert_not_called()


@pytest.mark.parametrize('exception', (OSError, AttributeError, ValueError, MemoryError))
def test_constructor_failure_preserves_python_audio_and_bounded_reason(exception):
  obj, sd = daemon(), device()
  with patch.object(soundd, 'PC', False), patch.object(native_playback, 'NativePlayback', side_effect=exception('private arbitrary detail')):
    assert soundd.Soundd.get_stream.__wrapped__(obj, sd) is sd.OutputStream.return_value
  assert obj.native_audio is None
  assert (obj.audio_backend, obj.audio_fallback_reason) == ('python', 'initialization:' + exception.__name__)


@pytest.mark.parametrize('exception', (AttributeError, TypeError, ValueError))
def test_native_stream_unsupported_preserves_close_and_fallback(exception):
  obj, sd = daemon(), device()
  playback = Mock()
  playback.stream.side_effect = exception('private arbitrary detail')
  with patch.object(soundd, 'PC', False), patch.object(native_playback, 'NativePlayback', return_value=playback):
    assert soundd.Soundd.get_stream.__wrapped__(obj, sd) is sd.OutputStream.return_value
  playback.close.assert_called_once_with()
  assert (obj.audio_backend, obj.audio_fallback_reason) == ('python', 'stream:' + exception.__name__)


def test_native_backend_unhandled_failure_closes_and_remains_an_error():
  obj, sd = daemon(), device()
  playback = Mock()
  playback.stream.side_effect = OSError('backend open failed')
  with patch.object(soundd, 'PC', False), patch.object(native_playback, 'NativePlayback', return_value=playback), pytest.raises(OSError):
    soundd.Soundd.get_stream.__wrapped__(obj, sd)
  playback.close.assert_called_once_with()
  sd.OutputStream.assert_not_called()
