import tarfile
from pathlib import Path
from types import SimpleNamespace

import zstandard as zstd

from openpilot.starpilot.common import starpilot_backups as sb
from openpilot.starpilot.common import starpilot_functions as sf


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get_int(self, key, return_default=False, default=0):
    return int(self.values.get(key, default))

  def get_bool(self, key, default=False):
    return bool(self.values.get(key, default))

  def put(self, key, value):
    self.values[key] = value


def make_install(root: Path) -> Path:
  install = root / "data" / "openpilot"
  (install / "selfdrive").mkdir(parents=True)
  (install / "selfdrive" / "car.py").write_text("print('drive')\n")
  (install / ".git" / "objects").mkdir(parents=True)
  (install / ".git" / "objects" / "pack").write_bytes(b"x" * 4096)
  (install / ".git" / "HEAD").write_text("ref: refs/heads/main\n")
  (install / "opendbc_repo").mkdir()
  (install / "opendbc_repo" / ".git").write_text("gitdir: ../.git/modules/opendbc\n")   # a submodule's .git file
  (install / "opendbc_repo" / "safety.h").write_text("// safety\n")
  return install


def members(archive: Path) -> list[str]:
  with open(archive, "rb") as f, zstd.ZstdDecompressor().stream_reader(f) as reader, tarfile.open(fileobj=reader, mode="r|") as tar:
    return [m.name for m in tar]


def backup(tmp_path, monkeypatch, abort=None):
  install = make_install(tmp_path / "root")
  backups = tmp_path / "backups"
  monkeypatch.setattr(sb, "BASEDIR", str(install))
  monkeypatch.setattr(sb, "STARPILOT_BACKUPS", backups)
  meta = SimpleNamespace(channel="mazda-long-ti1-testing", openpilot=SimpleNamespace(git_commit="cb65fce0123"))
  params = FakeParams()
  done = sb.backup_starpilot(meta, params, abort=abort)
  return done, backups, params, install


def test_install_backup_leaves_out_the_git_history_and_keeps_the_rest(tmp_path, monkeypatch):
  done, backups, params, install = backup(tmp_path, monkeypatch)
  assert done
  archive = backups / "cb65fce0123_mazda-long-ti1-testing_auto.tar.zst"
  names = members(archive)
  assert not any(".git" in n.split("/") for n in names), names
  root_rel = str(install).lstrip("/")
  assert f"{root_rel}/selfdrive/car.py" in names and f"{root_rel}/opendbc_repo/safety.h" in names
  assert params.values["MinimumBackupSize"] == archive.stat().st_size
  assert not list(backups.glob("*in_progress*"))


def test_install_backup_restores_where_system_backups_puts_it(tmp_path, monkeypatch):
  # The System Backups restore clears /data/openpilot and extracts at /: the archive must hold data/openpilot/...
  install = make_install(tmp_path / "src")
  backups = tmp_path / "backups"
  backups.mkdir()
  destination = backups / "abc_channel_auto"
  sb.create_backup(install, destination, "ok", "fail", FakeParams(), compressed=True, arcname="data/openpilot",
                   member_filter=sb._without_git)
  names = members(backups / "abc_channel_auto.tar.zst")
  assert "data/openpilot/selfdrive/car.py" in names
  assert all(n == "data/openpilot" or n.startswith("data/openpilot/") for n in names), names


def test_install_backup_stops_when_a_drive_starts_and_leaves_nothing_behind(tmp_path, monkeypatch):
  monkeypatch.setattr(sb.time, "monotonic", iter(range(10_000)).__next__)   # every check is "due"
  done, backups, params, _ = backup(tmp_path, monkeypatch, abort=lambda: True)
  assert not done
  assert list(backups.iterdir()) == []
  assert "MinimumBackupSize" not in params.values


def test_install_backup_is_made_once_per_commit(tmp_path, monkeypatch):
  done, backups, _, _ = backup(tmp_path, monkeypatch)
  assert done
  first = backups / "cb65fce0123_mazda-long-ti1-testing_auto.tar.zst"
  mtime = first.stat().st_mtime_ns
  meta = SimpleNamespace(channel="mazda-long-ti1-testing", openpilot=SimpleNamespace(git_commit="cb65fce0123"))
  assert sb.backup_starpilot(meta, FakeParams(), abort=lambda: True)    # already there: nothing to do, not "stopped"
  assert first.stat().st_mtime_ns == mtime


def test_wait_offroad_needs_the_car_off_for_the_whole_hold():
  clock = {"t": 0.0}
  onroad = {0.0: False, 5.0: False, 10.0: True, 15.0: False}   # off, a drive starts at 10 s, ends at 15 s

  class P:
    def get_bool(self, key):
      assert key == "IsOnroad"
      return [v for t, v in sorted(onroad.items()) if t <= clock["t"]][-1]

  def sleep(s):
    clock["t"] += s

  sf.wait_offroad(P(), 60.0, poll_s=5.0, sleep=sleep, clock=lambda: clock["t"])
  assert clock["t"] == 75.0   # off again from 15 s, then a full 60 s off


def test_another_commits_backup_from_today_is_replaced(tmp_path, monkeypatch):
  done, backups, _, _ = backup(tmp_path, monkeypatch)
  assert done
  old = backups / "cb65fce0123_mazda-long-ti1-testing_auto.tar.zst"
  meta = SimpleNamespace(channel="mazda-long-ti1-testing", openpilot=SimpleNamespace(git_commit="0123456789ab"))
  assert sb.backup_starpilot(meta, FakeParams())
  assert not old.exists() and (backups / "0123456789ab_mazda-long-ti1-testing_auto.tar.zst").exists()
