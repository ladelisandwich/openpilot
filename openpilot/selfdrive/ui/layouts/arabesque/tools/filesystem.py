"""Local file operations: pinned directories, no replacement, and cancellable copies."""

from collections.abc import Callable
from contextlib import contextmanager
import codecs
import ctypes
from dataclasses import dataclass
import errno
import os
from pathlib import Path
import stat
from threading import Event
from uuid import uuid4


_DIR_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
PREVIEW_BYTES = 256 * 1024


@contextmanager
def _directory(path: Path):
  if not path.is_absolute() or '..' in path.parts:
    raise ValueError('Choose an absolute location without parent traversal.')
  fd = os.open('/', _DIR_FLAGS)
  try:
    for part in path.parts[1:]:
      child = os.open(part, _DIR_FLAGS, dir_fd=fd)
      os.close(fd)
      fd = child
    yield fd
  finally:
    os.close(fd)


def _identity(info):
  return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns


def _attached(fd, path):
  with _directory(path) as current:
    before, after = os.fstat(fd), os.fstat(current)
    if (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino):
      raise OSError('This folder changed. Refresh and try again.')


def _rename(source_fd, source, destination_fd, destination):
  # shortcut: atomic moves require Linux renameat2; add an equivalent when supporting another OS.
  rename = getattr(ctypes.CDLL(None, use_errno=True), 'renameat2', None)
  if rename is None:
    raise OSError('Atomic moves are unavailable on this system.')
  rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
  rename.restype = ctypes.c_int
  if rename(source_fd, os.fsencode(source), destination_fd, os.fsencode(destination), 1):
    error = ctypes.get_errno()
    if error == errno.EXDEV:
      raise OSError('Move between filesystems is unavailable. Copy the item instead.')
    raise OSError(error, os.strerror(error), destination)


@dataclass(frozen=True)
class Entry:
  path: Path
  info: os.stat_result

  @property
  def directory(self):
    return stat.S_ISDIR(self.info.st_mode)


class FileSystem:
  def __init__(self, writable: tuple[Path, ...], protected: tuple[Path, ...], permitted: Callable[[], bool]):
    self._writable = tuple(path.resolve() for path in writable)
    self._protected = tuple(path.resolve() for path in protected)
    self._permitted = permitted

  def writable(self, path: Path, *, recursive=False):
    return (any(path.is_relative_to(root) for root in self._writable) and path not in self._writable and
            not any(path.is_relative_to(root) or recursive and root.is_relative_to(path) for root in self._protected))

  def _check(self, cancelled: Event):
    if cancelled.is_set():
      raise InterruptedError('Operation cancelled.')
    if not self._permitted():
      raise PermissionError('Files is available only while parked.')

  def _write(self, path: Path, cancelled: Event, *, recursive=False):
    self._check(cancelled)
    if not self.writable(path, recursive=recursive):
      raise PermissionError('This location is read-only in Files.')

  @staticmethod
  def _current(fd, entry: Entry):
    if _identity(os.stat(entry.path.name, dir_fd=fd, follow_symlinks=False)) != _identity(entry.info):
      raise OSError('This item changed. Refresh and try again.')

  @staticmethod
  def _name(name: str):
    if not name or name in ('.', '..') or '/' in name or '\x00' in name or len(os.fsencode(name)) > 255:
      raise ValueError('Use a name of 1–255 bytes without a slash.')
    return name

  def listing(self, path: Path, cancelled: Event):
    self._check(cancelled)
    path = path.resolve(strict=True)
    result = []
    with _directory(path) as fd, os.scandir(fd) as entries:
      for entry in entries:
        self._check(cancelled)
        try:
          result.append(Entry(path / entry.name, entry.stat(follow_symlinks=False)))
        except FileNotFoundError:
          continue
    return path, sorted(result, key=lambda item: (not item.directory, item.path.name.casefold()))

  def preview(self, entry: Entry, cancelled: Event):
    self._check(cancelled)
    with _directory(entry.path.parent) as parent:
      self._current(parent, entry)
      if stat.S_ISLNK(entry.info.st_mode):
        return 'Symbolic link → ' + os.readlink(entry.path.name, dir_fd=parent)
      if not stat.S_ISREG(entry.info.st_mode):
        return 'Preview is unavailable for this file type.'
      fd = os.open(entry.path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent)
      with os.fdopen(fd, 'rb') as opened:
        if _identity(os.fstat(opened.fileno())) != _identity(entry.info):
          raise OSError('This item changed. Refresh and try again.')
        data = opened.read(PREVIEW_BYTES + 1)
      self._check(cancelled)
    try:
      text = codecs.getincrementaldecoder('utf-8')().decode(data[:PREVIEW_BYTES], final=len(data) <= PREVIEW_BYTES)
    except UnicodeDecodeError:
      return 'Preview is unavailable for this file type.'
    if '\x00' in text:
      return 'Preview is unavailable for this file type.'
    return text + ('\n\nPreview limited to 256 KiB.' if len(data) > PREVIEW_BYTES else '')

  def change(self, action: str, cancelled: Event, *, entry: Entry | None = None, destination: Path | None = None, name=''):
    self._check(cancelled)
    if action not in ('mkdir', 'rename', 'copy', 'move', 'delete'):
      raise ValueError('Unknown file action.')
    if action == 'mkdir':
      name = self._name(name)
      if destination is None:
        raise TypeError('A destination path is required.')
      target = destination / name
      self._write(target, cancelled)
      with _directory(destination) as fd:
        self._write(target, cancelled)
        _attached(fd, destination)
        os.mkdir(target.name, dir_fd=fd)
      return
    if entry is None or not (entry.directory or stat.S_ISREG(entry.info.st_mode)):
      raise ValueError('Links and special files are read-only in Files.')
    with _directory(entry.path.parent) as source:
      self._current(source, entry)
      if action == 'delete':
        self._write(entry.path, cancelled, recursive=True)
        self._remove(source, entry.path.name, entry.path, cancelled)
        return
      if action == 'rename':
        target = entry.path.parent / self._name(name)
      else:
        if destination is None:
          raise TypeError('A destination path is required.')
        target = destination / entry.path.name
      self._write(target, cancelled)
      if target == entry.path and action in ('rename', 'move'):
        return
      if entry.directory and target.is_relative_to(entry.path):
        raise ValueError('Choose a destination outside this folder.')
      with _directory(target.parent) as output:
        try:
          os.stat(target.name, dir_fd=output, follow_symlinks=False)
        except FileNotFoundError:
          pass
        else:
          raise FileExistsError(errno.EEXIST, os.strerror(errno.EEXIST), target.name)
        if action in ('rename', 'move'):
          self._write(entry.path, cancelled, recursive=True)
          self._current(source, entry)
          self._write(target, cancelled)
          _attached(source, entry.path.parent)
          _attached(output, target.parent)
          _rename(source, entry.path.name, output, target.name)
          return
        temporary = '.arabesque-' + uuid4().hex + '.partial'
        try:
          self._copy(source, entry.path.name, output, temporary, cancelled)
          self._current(source, entry)
          self._write(target, cancelled)
          _attached(output, target.parent)
          _rename(output, temporary, output, target.name)
        finally:
          # Cleanup is independent of UI authority; only this operation owns the unpublished item.
          self._remove(output, temporary, target.parent / temporary, None)

  def _copy(self, source, name, output, target, cancelled):
    self._check(cancelled)
    info = os.stat(name, dir_fd=source, follow_symlinks=False)
    if stat.S_ISDIR(info.st_mode):
      incoming = os.open(name, _DIR_FLAGS, dir_fd=source)
      try:
        if _identity(os.fstat(incoming)) != _identity(info):
          raise OSError('Source changed during copy.')
        os.mkdir(target, mode=0o700, dir_fd=output)
        outgoing = os.open(target, _DIR_FLAGS, dir_fd=output)
        try:
          with os.scandir(incoming) as children:
            for child in children:
              self._copy(incoming, child.name, outgoing, child.name, cancelled)
          os.fchmod(outgoing, stat.S_IMODE(info.st_mode) & 0o777)
          os.utime(outgoing, ns=(info.st_atime_ns, info.st_mtime_ns))
          if _identity(os.fstat(incoming)) != _identity(info):
            raise OSError('Source changed during copy.')
        finally:
          os.close(outgoing)
      finally:
        os.close(incoming)
    elif stat.S_ISREG(info.st_mode):
      incoming = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=source)
      with os.fdopen(incoming, 'rb') as opened:
        if _identity(os.fstat(incoming)) != _identity(info):
          raise OSError('Source changed during copy.')
        outgoing = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600, dir_fd=output)
        with os.fdopen(outgoing, 'wb') as copied:
          while block := opened.read(1024 * 1024):
            self._check(cancelled)
            copied.write(block)
          copied.flush()
          os.fchmod(outgoing, stat.S_IMODE(info.st_mode) & 0o777)
          os.utime(outgoing, ns=(info.st_atime_ns, info.st_mtime_ns))
          os.fsync(outgoing)
        if _identity(os.fstat(incoming)) != _identity(info):
          raise OSError('Source changed during copy.')
    else:
      raise ValueError('Folders containing links or special files cannot be copied.')

  def _remove(self, parent, name, path, cancelled):
    try:
      info = os.stat(name, dir_fd=parent, follow_symlinks=False)
    except FileNotFoundError:
      return
    if cancelled is not None:
      self._write(path, cancelled, recursive=True)
      _attached(parent, path.parent)
      if info.st_dev != os.fstat(parent).st_dev:
        raise PermissionError('Mounted filesystems are read-only in Files.')
    if stat.S_ISDIR(info.st_mode):
      fd = os.open(name, _DIR_FLAGS, dir_fd=parent)
      try:
        opened = os.fstat(fd)
        if (opened.st_dev, opened.st_ino) != (info.st_dev, info.st_ino):
          raise OSError('This folder changed. Refresh and try again.')
        if cancelled is None:
          os.fchmod(fd, 0o700)
        with os.scandir(fd) as entries:
          for entry in entries:
            self._remove(fd, entry.name, path / entry.name, cancelled)
      finally:
        os.close(fd)
      if cancelled is not None:
        self._write(path, cancelled, recursive=True)
        _attached(parent, path.parent)
      os.rmdir(name, dir_fd=parent)
    else:
      os.unlink(name, dir_fd=parent)
