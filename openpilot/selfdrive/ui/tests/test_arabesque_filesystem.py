"""Real file operations in temporary roots; never modify device or developer files."""

import errno
import os
from threading import Event

import pytest

from openpilot.selfdrive.ui.layouts.arabesque.tools import filesystem
from openpilot.selfdrive.ui.layouts.arabesque.tools.filesystem import Entry, FileSystem, PREVIEW_BYTES


@pytest.fixture
def storage(tmp_path):
  protected = tmp_path / 'managed'
  protected.mkdir()
  return FileSystem((tmp_path,), (protected,), lambda: True)


def entry(path):
  return Entry(path, path.lstat())


def test_browse_sort_preview_and_links(storage, tmp_path):
  (tmp_path / 'z-folder').mkdir()
  (tmp_path / 'café.txt').write_text('Hello, 世界\n')
  (tmp_path / '.hidden').touch()
  (tmp_path / 'binary').write_bytes(b'\x00\xff')
  (tmp_path / 'link').symlink_to(tmp_path / 'café.txt')
  os.mkfifo(tmp_path / 'pipe')
  path, items = storage.listing(tmp_path, Event())
  assert path == tmp_path
  assert [item.path.name for item in items] == ['managed', 'z-folder', '.hidden', 'binary', 'café.txt', 'link', 'pipe']
  assert storage.preview(entry(tmp_path / 'café.txt'), Event()) == 'Hello, 世界\n'
  assert 'unavailable' in storage.preview(entry(tmp_path / 'binary'), Event())
  assert 'Symbolic link' in storage.preview(entry(tmp_path / 'link'), Event())
  assert 'unavailable' in storage.preview(entry(tmp_path / 'pipe'), Event())


def test_preview_limit_keeps_partial_unicode_valid(storage, tmp_path):
  path = tmp_path / 'large.txt'
  path.write_text('a' * (PREVIEW_BYTES - 1) + '界' * 100)
  preview = storage.preview(entry(path), Event())
  assert preview.startswith('aaa') and preview.endswith('Preview limited to 256 KiB.')
  assert len(preview) < PREVIEW_BYTES + 100


def test_create_rename_copy_move_and_delete_tree(storage, tmp_path):
  storage.change('mkdir', Event(), destination=tmp_path, name='folder')
  source = tmp_path / 'folder'
  (source / 'nested').mkdir()
  (source / 'nested' / 'café.txt').write_text('data')
  destination = tmp_path / 'destination'
  destination.mkdir()
  storage.change('copy', Event(), entry=entry(source), destination=destination)
  assert (destination / 'folder' / 'nested' / 'café.txt').read_text() == 'data'
  storage.change('rename', Event(), entry=entry(destination / 'folder'), name='renamed')
  storage.change('move', Event(), entry=entry(destination / 'renamed'), destination=tmp_path)
  assert not (destination / 'renamed').exists()
  assert (tmp_path / 'renamed' / 'nested' / 'café.txt').read_text() == 'data'
  storage.change('delete', Event(), entry=entry(tmp_path / 'renamed'))
  assert not (tmp_path / 'renamed').exists()
  assert (source / 'nested' / 'café.txt').read_text() == 'data'


@pytest.mark.parametrize('name', ['', '.', '..', '../escape', 'a/b', '\x00', '界' * 86])
def test_invalid_names_do_not_create_files(storage, tmp_path, name):
  before = set(tmp_path.iterdir())
  with pytest.raises(ValueError):
    storage.change('mkdir', Event(), destination=tmp_path, name=name)
  assert set(tmp_path.iterdir()) == before


@pytest.mark.parametrize('action', ['copy', 'move', 'rename'])
def test_existing_destination_is_never_replaced(storage, tmp_path, action):
  source = tmp_path / 'source'
  source.write_text('original')
  destination = tmp_path / 'target'
  destination.mkdir()
  existing = destination / 'source' if action != 'rename' else tmp_path / 'existing'
  existing.write_text('keep me')
  with pytest.raises(FileExistsError):
    storage.change(action, Event(), entry=entry(source), destination=destination, name='existing')
  assert source.read_text() == 'original'
  assert existing.read_text() == 'keep me'
  assert not list(destination.glob('.arabesque-*'))


def test_destination_created_at_commit_wins_without_replacement(storage, tmp_path, monkeypatch):
  source = tmp_path / 'source'
  source.write_text('original')
  destination = tmp_path / 'destination'
  destination.mkdir()
  rename = filesystem._rename

  def race(*args):
    (destination / 'source').write_text('new arrival')
    rename(*args)

  monkeypatch.setattr(filesystem, '_rename', race)
  with pytest.raises(FileExistsError):
    storage.change('copy', Event(), entry=entry(source), destination=destination)
  assert (destination / 'source').read_text() == 'new arrival'
  assert source.read_text() == 'original'
  assert not list(destination.glob('.arabesque-*'))


@pytest.mark.parametrize('failure', ['cancel', 'permission', 'full', 'source-change'])
def test_unfinished_copy_does_not_publish_or_destroy_source(storage, tmp_path, monkeypatch, failure):
  source = tmp_path / 'source'
  source.write_bytes(b'a' * 2_000_000)
  destination = tmp_path / 'destination'
  destination.mkdir()
  cancelled = Event()
  copy = storage._copy

  def stop(*args):
    copy(*args)
    if failure == 'cancel':
      cancelled.set()
    elif failure == 'permission':
      storage._permitted = lambda: False
    elif failure == 'full':
      raise OSError(errno.ENOSPC, 'No space left on device')
    else:
      source.write_bytes(b'changed')

  monkeypatch.setattr(storage, '_copy', stop)
  with pytest.raises(OSError):
    storage.change('copy', cancelled, entry=entry(source), destination=destination)
  assert source.exists()
  assert not list(destination.iterdir())


def test_copy_refuses_links_and_cleans_readonly_staging(storage, tmp_path):
  source = tmp_path / 'source'
  source.mkdir()
  readonly = source / 'a-readonly'
  readonly.mkdir()
  (readonly / 'data').write_text('keep')
  readonly.chmod(0o555)
  (source / 'z-link').symlink_to(tmp_path / 'managed')
  destination = tmp_path / 'destination'
  destination.mkdir()
  with pytest.raises(ValueError, match='links'):
    storage.change('copy', Event(), entry=entry(source), destination=destination)
  assert not list(destination.iterdir())
  readonly.chmod(0o755)


def test_protected_locations_and_ancestors_cannot_be_modified(storage, tmp_path):
  protected = tmp_path / 'managed' / 'keep'
  protected.write_text('managed data')
  with pytest.raises(PermissionError):
    storage.change('delete', Event(), entry=entry(protected))
  with pytest.raises(PermissionError):
    storage.change('mkdir', Event(), destination=protected.parent, name='no')
  assert not storage.writable(tmp_path, recursive=True)
  assert protected.read_text() == 'managed data'
  assert storage.preview(entry(protected), Event()) == 'managed data'


def test_symlink_parent_and_parent_traversal_cannot_escape_policy(storage, tmp_path):
  alias = tmp_path / 'alias'
  alias.symlink_to(tmp_path / 'managed', target_is_directory=True)
  with pytest.raises(OSError):
    storage.change('mkdir', Event(), destination=alias, name='no')
  with pytest.raises(ValueError):
    storage.change('mkdir', Event(), destination=tmp_path / 'child' / '..' / 'managed', name='no')
  assert not list((tmp_path / 'managed').iterdir())


def test_changed_source_and_changed_destination_are_rejected(storage, tmp_path, monkeypatch):
  source = tmp_path / 'source'
  source.write_text('original')
  selected = entry(source)
  source.unlink()
  source.symlink_to(tmp_path / 'managed')
  with pytest.raises(OSError, match='changed'):
    storage.change('delete', Event(), entry=selected)
  destination = tmp_path / 'destination'
  destination.mkdir()
  check = storage._write
  checks = 0

  def swap(path, *args, **kwargs):
    nonlocal checks
    check(path, *args, **kwargs)
    checks += 1
    if checks == 2:
      destination.rename(tmp_path / 'old')
      destination.mkdir()

  monkeypatch.setattr(storage, '_write', swap)
  # Swap after the destination descriptor was opened.
  with pytest.raises(OSError, match='changed'):
    storage.change('mkdir', Event(), destination=destination, name='new')
  assert not list((tmp_path / 'old').iterdir())
  assert not list(destination.iterdir())


def test_cancel_and_offroad_gate_cover_reads_and_writes(storage, tmp_path):
  cancelled = Event()
  cancelled.set()
  with pytest.raises(InterruptedError):
    storage.listing(tmp_path, cancelled)
  storage._permitted = lambda: False
  with pytest.raises(PermissionError):
    storage.change('mkdir', Event(), destination=tmp_path, name='no')


def test_copying_folder_into_itself_is_rejected(storage, tmp_path):
  source = tmp_path / 'source'
  source.mkdir()
  child = source / 'nested'
  child.mkdir()
  with pytest.raises(ValueError, match='outside'):
    storage.change('copy', Event(), entry=entry(source), destination=child)
  assert not list(child.iterdir())


def test_copy_cancels_between_file_blocks(storage, tmp_path, monkeypatch):
  source = tmp_path / 'source'
  source.write_bytes(b'a' * 3_000_000)
  destination = tmp_path / 'destination'
  destination.mkdir()
  check = storage._check
  checks = 0

  def cancel_during_copy(cancelled):
    nonlocal checks
    checks += 1
    if checks == 5:
      cancelled.set()
    check(cancelled)

  monkeypatch.setattr(storage, '_check', cancel_during_copy)
  with pytest.raises(InterruptedError):
    storage.change('copy', Event(), entry=entry(source), destination=destination)
  assert source.stat().st_size == 3_000_000
  assert not list(destination.iterdir())


def test_delete_stops_on_authority_loss_and_does_not_follow_links(storage, tmp_path, monkeypatch):
  source = tmp_path / 'source'
  source.mkdir()
  protected = tmp_path / 'managed' / 'keep'
  protected.write_text('keep')
  (source / 'alias').symlink_to(protected)
  (source / 'other').write_text('other')
  unlink = filesystem.os.unlink

  def lose_authority(*args, **kwargs):
    unlink(*args, **kwargs)
    storage._permitted = lambda: False

  monkeypatch.setattr(filesystem.os, 'unlink', lose_authority)
  with pytest.raises(PermissionError):
    storage.change('delete', Event(), entry=entry(source))
  assert protected.read_text() == 'keep'
  assert len(list(source.iterdir())) == 1


def test_cross_filesystem_move_fails_without_deleting_source(storage, tmp_path, monkeypatch):
  from types import SimpleNamespace
  from unittest.mock import Mock
  source = tmp_path / 'source'
  source.write_text('keep')
  destination = tmp_path / 'destination'
  destination.mkdir()
  monkeypatch.setattr(filesystem.ctypes, 'CDLL', lambda *args, **kwargs: SimpleNamespace(renameat2=Mock(return_value=-1)))
  monkeypatch.setattr(filesystem.ctypes, 'get_errno', lambda: errno.EXDEV)
  with pytest.raises(OSError, match='Copy the item instead'):
    storage.change('move', Event(), entry=entry(source), destination=destination)
  assert source.read_text() == 'keep' and not list(destination.iterdir())


@pytest.mark.parametrize('action', ['mkdir', 'copy', 'move'])
def test_missing_destination_preserves_type_error_without_mutation(storage, tmp_path, action):
  source = tmp_path / 'source'
  source.write_text('keep')
  before = sorted(path.name for path in tmp_path.iterdir())
  with pytest.raises(TypeError, match='destination path'):
    storage.change(action, Event(), entry=entry(source), name='new')
  assert source.read_text() == 'keep'
  assert sorted(path.name for path in tmp_path.iterdir()) == before


@pytest.mark.parametrize('name', ['', '..', 'nested/name', '\x00'])
def test_missing_mkdir_destination_still_validates_name_first(storage, tmp_path, name):
  before = sorted(path.name for path in tmp_path.iterdir())
  with pytest.raises(ValueError, match='Use a name'):
    storage.change('mkdir', Event(), name=name)
  assert sorted(path.name for path in tmp_path.iterdir()) == before
