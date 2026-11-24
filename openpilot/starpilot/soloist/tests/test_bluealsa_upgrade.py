from pathlib import Path
import runpy
import shutil
import subprocess

import pytest

from openpilot.starpilot.soloist import bluealsa_runtime as runtime, phone_profile, phone_audio, backend


def test_bundle_is_complete_checksummed_arm64():
  runtime.validate()
  assert (runtime.ROOT / 'UPSTREAM_COMMIT').read_text().strip() == '1935d6dcb8975f2d7a51aaafe61538d157224623'


def test_capture_and_playback_select_matching_private_plugins():
  capture = phone_audio.capture_config('11:22:33:44:55:66')
  playback = backend.alsa_config('11:22:33:44:55:66')
  for config in (capture, playback):
    assert runtime.alsa_plugins() in config
    assert 'softvol "true"' in config
  assert 'volume "100+"' in playback  # saved state must never leave Spotify quiet or muted
  assert 'volume' not in capture  # phone audio level was already right; leave it to the phone
  assert 'hwcompat "silence"' in capture
  assert 'hwcompat' not in playback


def test_checksum_failure_rejects_incomplete_upgrade(tmp_path):
  shutil.copytree(runtime.ROOT, tmp_path / 'bundle', ignore=shutil.ignore_patterns('build', '__pycache__'))
  root = tmp_path / 'bundle'
  (root / 'lib/libasound_module_pcm_bluealsa.so').write_bytes(b'broken')
  with pytest.raises(RuntimeError, match='checksum'):
    runtime.validate(root)


def test_old_commit_restart_restores_stock_daemon_and_removes_upgrade(tmp_path, monkeypatch):
  checkout = tmp_path / 'checkout'
  checkout.mkdir()
  def git(*args):
    return subprocess.run(['git', '-C', str(checkout), *args], check=True, capture_output=True, text=True).stdout.strip()
  git('init', '-q')
  git('config', 'user.name', 'Rollback test')
  git('config', 'user.email', 'rollback@example.invalid')
  old = Path(__file__).with_name('fixtures') / 'phone_profile_4_1_1.txt'
  shutil.copyfile(old, checkout / 'phone_profile.py')
  git('add', '.')
  git('commit', '-qm', '4.1.1')
  previous_commit = git('rev-parse', 'HEAD')
  bundle = checkout / 'bluealsa'
  shutil.copytree(runtime.ROOT, bundle, ignore=shutil.ignore_patterns('build', '__pycache__'))
  git('add', '.')
  git('commit', '-qm', '5.0.0')
  monkeypatch.setattr(runtime, 'ROOT', bundle)
  legacy = tmp_path / 'run/systemd/system/bluealsa.service.d/90-starpilot-phone.conf'
  calls = []
  def run(*args):
    calls.append(args)
    if args[0] == 'show':
      return '{ argv[]=/usr/bin/bluealsa -p a2dp-source ; }'
    return ''
  phone_profile.configure('acquire', legacy, run)
  override, launcher = runtime.paths(legacy)
  launch = runpy.run_path(str(launcher))['select']
  assert launch(bundle, legacy, override, lambda: None)[0] == str(bundle / 'bin/bluealsad')
  # Actual checkout removes tracked v5 bundle; /run intentionally survives.
  git('checkout', '-q', previous_commit)
  assert not (bundle / 'VERSION').exists()
  old_module = runpy.run_path(str(checkout / 'phone_profile.py'))
  selected = []
  def old_run(*args):
    calls.append(args)
    if args[0] == 'try-restart':
      selected.append(launch(bundle, legacy, override, lambda: calls.append(('daemon-reload',))))
    return ''
  old_module['configure']('release', legacy, old_run)
  assert selected == [['/usr/bin/bluealsa', '-p', 'a2dp-source']]
  assert not legacy.exists() and not override.exists() and not launcher.exists()
  old_module['configure']('acquire', legacy, run)
  assert legacy.read_text() == old_module['CONTENTS']
  assert not override.exists()  # old code stays on the system daemon/plugins


def test_failed_start_rolls_back_both_overrides(tmp_path):
  legacy = tmp_path / 'system/bluealsa.service.d/90-starpilot-phone.conf'
  calls = []
  def run(*args):
    calls.append(args)
    if args[0] == 'show':
      return '{ argv[]=/usr/bin/bluealsa -p a2dp-source ; }'
    if args[0] == 'restart':
      raise RuntimeError('start failed')
    return ''
  with pytest.raises(RuntimeError, match='start failed'):
    phone_profile.configure('acquire', legacy, run)
  assert not legacy.exists()
  assert all(not p.exists() for p in runtime.paths(legacy))
  assert calls[-1] == ('try-restart', 'bluealsa.service')


def test_spotify_bluetooth_owns_upgrade_until_output_stops(tmp_path, monkeypatch):
  from openpilot.starpilot.soloist import preferences, supervisor
  from openpilot.starpilot.soloist.paths import Layout
  from openpilot.starpilot.soloist.tests.test_runtime import make_supervisor, ready_layout
  layout = Layout(tmp_path / 'music')
  ready_layout(layout, monkeypatch)
  preferences.save(preferences.Preferences(output='bluetooth', output_address='11:22:33:44:55:66'), layout)
  calls, children = [], []
  monkeypatch.setattr(phone_audio, 'profile', calls.append)
  monkeypatch.setattr(supervisor, '_kill_group', lambda process: None)
  sup, now = make_supervisor(layout, children)
  sup.output.ready = lambda address: True
  sup.tick()
  assert calls == ['acquire'] and sup.phone_profile and len(children) == 2
  now[0] += 300
  sup.tick()
  assert calls == ['acquire']  # pairing timeout must not downgrade a playing output
  sup.stop_children()
  assert calls == ['acquire', 'release'] and not sup.phone_profile


def test_spotify_never_starts_v5_plugin_against_v4_after_activation_failure(tmp_path, monkeypatch):
  from openpilot.starpilot.soloist import preferences
  from openpilot.starpilot.soloist.paths import Layout
  from openpilot.starpilot.soloist.tests.test_runtime import make_supervisor, ready_layout
  layout = Layout(tmp_path / 'music')
  ready_layout(layout, monkeypatch)
  preferences.save(preferences.Preferences(output='bluetooth', output_address='11:22:33:44:55:66'), layout)
  def failed(action):
    raise RuntimeError('upgrade unavailable')
  monkeypatch.setattr(phone_audio, 'profile', failed)
  children = []
  sup, now = make_supervisor(layout, children)
  sup.tick()
  assert not children and not sup.phone_profile
  assert sup.state == 'audio_unavailable' and sup.retry_at > now[0]


def test_daemon_uses_separate_persistent_storage():
  binary = (runtime.ROOT / 'bin/bluealsad').read_bytes()
  assert b'/data/starpilot/bluealsa5/lib/bluealsa\0' in binary
  assert b'/var/lib/bluealsa\0' not in binary


def test_bundle_artifacts_are_not_git_ignored():
  result = subprocess.run(['git', '-C', str(runtime.ROOT.parents[3]), 'check-ignore',
                           *(str(runtime.ROOT / name) for name in runtime.ARTIFACTS)], check=False, capture_output=True)
  assert result.returncode == 1  # all artifacts can travel with the upgrade commit
