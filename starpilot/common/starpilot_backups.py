#!/usr/bin/env python3
import datetime
import shutil
import tarfile
import time
import zstandard as zstd

from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths

from openpilot.starpilot.common.starpilot_utilities import delete_file
from openpilot.starpilot.common.starpilot_variables import EXCLUDED_KEYS, STARPILOT_BACKUPS, TOGGLE_BACKUPS


class BackupAborted(Exception):
  pass


def _without_git(info: tarfile.TarInfo):
  # The git history (.git, and the .git file of any submodule) is the bulk of the install and is not needed to
  # run the backed-up code; a restore keeps the device's own history (see the System Backups restore).
  return None if ".git" in info.name.split("/") else info


def backup_starpilot(build_metadata, params, abort=None) -> bool:
  """Back up the install for this commit, once. Returns False when `abort` stopped it part-way (try again later)."""
  maximum_backups = 3
  cleanup_backups(STARPILOT_BACKUPS, maximum_backups)

  today = datetime.datetime.now().date()
  for backup in STARPILOT_BACKUPS.glob("*_auto.tar.zst"):
    if backup.name.endswith(f"_{build_metadata.channel}_auto.tar.zst"):
      if datetime.datetime.fromtimestamp(backup.stat().st_mtime).date() == today:
        # today's backup of another commit; the names carry the full commit, so compare the full commit (with
        # [:6] here nothing ever matched, and every boot deleted and re-made the current commit's backup)
        if not backup.name.startswith(f"{build_metadata.openpilot.git_commit}_"):
          delete_file(backup, report=False)

  _, _, free = shutil.disk_usage(STARPILOT_BACKUPS)
  minimum_backup_size = params.get_int("MinimumBackupSize", return_default=True, default=0)
  if free > minimum_backup_size * maximum_backups:
    destination = STARPILOT_BACKUPS / f"{build_metadata.openpilot.git_commit}_{build_metadata.channel}_auto"
    # Stored under its own path (data/openpilot/...), like a System Backups backup, so the restore there, which
    # extracts at / over a cleared /data/openpilot, puts it back where it came from.
    return create_backup(Path(BASEDIR), destination, "Successfully backed up StarPilot!", "Failed to backup StarPilot...", params,
                         minimum_backup_size, compressed=True, arcname=str(Path(BASEDIR)).lstrip("/"), member_filter=_without_git,
                         abort=abort)
  return True


def backup_toggles(params, boot_run=False):
  params_backup = Params(f"{Paths.shm_path()}/params_backup", return_defaults=True)

  changes_found = False
  for key in params.all_keys():
    current_value = params.get(key)
    if current_value is None:
      continue

    if boot_run:
      params_backup.put(key, current_value)
      changes_found = True
    elif current_value != params_backup.get(key):
      params_backup.put(key, current_value)
      changes_found |= key.decode("utf-8") not in EXCLUDED_KEYS

  maximum_backups = 5
  cleanup_backups(TOGGLE_BACKUPS, maximum_backups)

  if not changes_found or boot_run:
    if not changes_found:
      print("Toggles are identical to the previous backup. Aborting...")
    return

  params.put_bool("GalaxyUploadPending", True)

  destination = TOGGLE_BACKUPS / f"{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_auto"
  create_backup(Path(params_backup.get_param_path()), destination, "Successfully backed up toggles!", "Failed to backup toggles...", params)


def cleanup_backups(directory, limit):
  directory.mkdir(parents=True, exist_ok=True)

  for backup in directory.glob("*_in_progress*"):
    delete_file(backup, report=False)

  backups = sorted(directory.glob("*_auto*"), key=lambda f: f.stat().st_mtime, reverse=True)
  for oldest_backup in backups[limit:]:
    delete_file(oldest_backup, report=False)


def create_backup(backup, destination, success_message, fail_message, params, minimum_backup_size=0, compressed=False,
                  arcname=None, member_filter=None, abort=None, abort_check_s=0.5) -> bool:
  """Returns False only when `abort` (checked about every abort_check_s while archiving) stopped it."""
  final_destination = destination.parent / f"{destination.name}.tar.zst" if compressed else destination

  if final_destination.exists():
    print("Backup already exists. Aborting...")
    return True

  if compressed:
    compressed_temp = destination.parent / f"{destination.name}_in_progress.tar.zst"
    last_check = time.monotonic()

    def member(info):
      nonlocal last_check
      if abort is not None and time.monotonic() - last_check >= abort_check_s:
        last_check = time.monotonic()
        if abort():
          raise BackupAborted
      return member_filter(info) if member_filter is not None else info

    try:
      with open(compressed_temp, "wb") as f_out:
        cctx = zstd.ZstdCompressor()
        with cctx.stream_writer(f_out) as compressor:
          with tarfile.open(fileobj=compressor, mode="w") as tar:
            try:
              tar.add(backup, arcname=arcname or destination.name, filter=member)
            except OSError:
              pass
    except BackupAborted:
      delete_file(compressed_temp, report=False)
      print("Backup paused: the car is on. It will start again once the car is off.")
      return False

    compressed_temp.rename(final_destination)

    compressed_backup_size = final_destination.stat().st_size
    if minimum_backup_size == 0 or compressed_backup_size < minimum_backup_size:
      params.put("MinimumBackupSize", int(compressed_backup_size))
  else:
    in_progress_destination = destination.parent / f"{destination.name}_in_progress"

    shutil.copytree(backup, in_progress_destination, symlinks=True)

    in_progress_destination.rename(destination)

  print(success_message)
  return True
