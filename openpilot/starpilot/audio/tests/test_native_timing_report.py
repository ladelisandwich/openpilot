from types import SimpleNamespace
from unittest.mock import patch

from openpilot.starpilot.audio.native_playback import NativePlayback


def playback(*, frozen=1, count=21, words=None, underflows=1, clock_errors=0):
  obj = NativePlayback.__new__(NativePlayback)
  obj.owner = SimpleNamespace(output_underflow_count=underflows)
  obj.timing_reported = False
  obj.handle, obj.rate, obj.buffer = 1, 48000, 4096
  calls = []
  def read(handle, values, capacity):
    calls.append((handle, capacity))
    header = (1, 22, 32, 25, 21, frozen, 5, count, 0, clock_errors)
    for index, value in enumerate(header):
      values[index] = value
    for row in range(min(count, 32)):
      values[10 + row * 22] = 5 + row
    return 10 + 22 * count if words is None else words
  obj.timing_getter = read
  return obj, calls


def test_healthy_or_old_native_library_does_not_poll_or_log():
  obj, calls = playback(underflows=0)
  with patch('openpilot.common.swaglog.cloudlog.error') as error:
    obj.report_underflow_timing()
    obj.owner.output_underflow_count = 1
    obj.timing_getter = None
    obj.report_underflow_timing()
  assert not calls
  error.assert_not_called()


def test_waits_for_recovery_then_logs_one_bounded_trace():
  obj, calls = playback(frozen=0)
  with patch('openpilot.common.swaglog.cloudlog.error') as error:
    obj.report_underflow_timing()
    error.assert_not_called()
    assert not obj.timing_reported
    obj.timing_getter = playback()[0].timing_getter
    obj.report_underflow_timing()
    obj.report_underflow_timing()
  assert len(calls) == 1
  error.assert_called_once()
  trace = error.call_args.args[0]
  assert trace['event'] == 'soundd_underflow_timing' and trace['schema'] == 1
  assert trace['sample_rate'] == 48000 and trace['block_frames'] == 4096
  assert len(trace['columns']) == 22 and trace['columns'][1:5] == ('entry_mono_ns', 'entry_cpu_ns', 'exit_mono_ns', 'exit_cpu_ns')
  assert len(trace['header_columns']) == 10 and trace['header_columns'][8] == 'callback_aborts'
  assert trace['header'] == [1, 22, 32, 25, 21, 1, 5, 21, 0, 0]
  assert len(trace['rows']) == 21 and all(len(row) == 22 for row in trace['rows'])
  assert [row[0] for row in trace['rows']] == list(range(5, 26))


def test_unavailable_snapshot_retries_and_clock_errors_remain_explicit():
  obj, calls = playback(words=0)
  with patch('openpilot.common.swaglog.cloudlog.error') as error:
    obj.report_underflow_timing()
    assert not obj.timing_reported
    obj.timing_getter = playback(clock_errors=3)[0].timing_getter
    obj.report_underflow_timing()
  assert len(calls) == 1
  assert error.call_args.args[0]['header'][9] == 3


def test_invalid_native_metadata_is_an_error_without_unbounded_rows():
  for count, words in ((33, 736), (21, 999)):
    obj, _ = playback(count=count, words=words)
    with patch('openpilot.common.swaglog.cloudlog.error') as error:
      obj.report_underflow_timing()
      obj.report_underflow_timing()
    error.assert_called_once_with({'event': 'soundd_underflow_timing', 'error': 'invalid_native_diagnostic'})


def test_closing_stream_preserves_partial_underflow_trace():
  obj, _ = playback(frozen=0)
  with patch('openpilot.common.swaglog.cloudlog.error') as error:
    obj.report_underflow_timing(closing=True)
  error.assert_called_once()
  assert error.call_args.args[0]['header'][5] == 0
  assert error.call_args.args[0]['rows']


def test_closing_stream_reports_native_abort_without_claiming_an_underflow():
  obj, _ = playback(frozen=0, underflows=0)
  read = obj.timing_getter
  def aborted(handle, values, capacity):
    words = read(handle, values, capacity)
    values[4], values[8] = 0, 1
    return words
  obj.timing_getter = aborted
  with patch('openpilot.common.swaglog.cloudlog.error') as error:
    obj.report_underflow_timing(closing=True)
  header = error.call_args.args[0]['header']
  assert header[4] == 0 and header[8] == 1


def test_compiled_native_trace_reaches_service_logger_with_exact_pcm(tmp_path):
  import ctypes
  import subprocess
  from pathlib import Path

  source = Path(__file__).parents[1] / 'native/audio_mixer.c'
  library = tmp_path / 'mixer.so'
  subprocess.run(['cc', '-std=c11', '-O2', '-shared', '-fPIC', str(source), '-o', str(library)],
                 check=True, capture_output=True, timeout=15)
  lib = ctypes.CDLL(str(library))
  lib.audio_create.restype = ctypes.c_void_p
  lib.audio_destroy.argtypes = [ctypes.c_void_p]
  lib.audio_callback_address.restype = ctypes.c_size_t
  lib.audio_publish_bank.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_float)),
                                    ctypes.POINTER(ctypes.c_uint), ctypes.c_uint, ctypes.c_uint64]
  lib.audio_command.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_int,
                               ctypes.c_float, ctypes.c_uint64, ctypes.c_uint64, ctypes.c_uint64]
  lib.audio_prepare.argtypes = [ctypes.c_void_p]
  lib.audio_diagnostic_snapshot.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64), ctypes.c_uint]
  lib.audio_diagnostic_snapshot.restype = ctypes.c_uint
  obj, _ = playback()
  obj.handle = lib.audio_create()
  assert obj.handle
  obj.timing_getter = lib.audio_diagnostic_snapshot
  samples = (ctypes.c_float * 4)(.25, -.5, .75, -1.)
  bank = (ctypes.POINTER(ctypes.c_float) * 1)(samples)
  lengths = (ctypes.c_uint * 1)(4)
  output = (ctypes.c_float * 4)()
  callback = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong,
                             ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p)(lib.audio_callback_address())
  try:
    assert lib.audio_publish_bank(obj.handle, bank, lengths, 1, 1)
    assert lib.audio_command(obj.handle, 1, 0, -1, 0, .5, 1, 1, 0)
    assert lib.audio_prepare(obj.handle)
    with patch('openpilot.common.swaglog.cloudlog.error') as error:
      for index in range(25):
        assert callback(None, output, 4, None, 4 if index == 20 else 0, obj.handle) == 0
        assert list(output) == [.125, -.25, .375, -.5]
        obj.report_underflow_timing()
      obj.report_underflow_timing()
    error.assert_called_once()
    trace = error.call_args.args[0]
    assert trace['header'][:8] == [1, 22, 32, 25, 21, 1, 5, 21]
    assert trace['rows'][16][9] == 4 and trace['rows'][16][16] == 1
    assert all(row[19] == 0 for row in trace['rows'])  # Missing host clock metadata remains explicit.
  finally:
    lib.audio_destroy(obj.handle)
