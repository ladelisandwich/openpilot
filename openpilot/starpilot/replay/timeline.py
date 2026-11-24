"""Replay-local history pages and palette; never used for driving decisions."""

from bisect import bisect_right
import math
import os
from pathlib import Path
import stat

from openpilot.starpilot.replay.client import parse


LEGEND = {
  'chill': {'label': 'Chill', 'rgb': [22, 127, 64]},
  'experimental': {'label': 'Experimental', 'rgb': [218, 111, 37]},
  'aol': {'label': 'AOL', 'rgb': [10, 186, 181]},
  'error': {'label': 'Alert', 'rgb': [201, 34, 49]},
  'bookmark': {'label': 'Bookmark', 'rgb': [255, 214, 0]},
}
KINDS = tuple(LEGEND)
MAX_ROWS, PAGE_ROWS, MAX_PAGE, MAX_FILE = 65536, 256, 65536, 8 * 1024 * 1024


def _identity(info):
  if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o600 or
      info.st_nlink != 1 or not 0 < info.st_size <= MAX_FILE):
    raise ValueError('Timeline history is unavailable')
  return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns


def file_identity(path: Path):
  fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC)
  try:
    return _identity(os.fstat(fd))
  finally:
    os.close(fd)


def row_key(row):
  return row[0], row[1], KINDS.index(row[2])


def validate_rows(rows, ranges, previous=None, ends=None):
  if type(rows) is not list or len(rows) > PAGE_ROWS:
    raise ValueError('Invalid timeline page')
  ends = {} if ends is None else ends
  for row in rows:
    if (type(row) is not list or len(row) != 3 or type(row[2]) is not str or row[2] not in LEGEND or
        any(type(t) not in (int, float) or not math.isfinite(t) or not 0 <= t <= 604800 for t in row[:2]) or
        (row[0] != row[1] if row[2] == 'bookmark' else row[0] >= row[1]) or
        not any(lo <= row[0] <= row[1] <= hi for lo, hi in ranges)):
      raise ValueError('Invalid timeline interval')
    if previous is not None and row_key(row) <= row_key(previous):
      raise ValueError('Unordered timeline history')
    layer = row[2] if row[2] in ('error', 'bookmark') else 'mode'
    if row[0] < ends.get(layer, -1) or layer == 'bookmark' and row[0] == ends.get(layer):
      raise ValueError('Overlapping timeline history')
    ends[layer] = row[1]
    previous = row
  return previous, ends


def read_page(path: Path, cursor, identity, ranges):
  if type(cursor) is not int or not 0 <= cursor < identity[2]:
    raise ValueError('Invalid timeline cursor')
  fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC)
  with os.fdopen(fd, 'rb') as source:
    if _identity(os.fstat(source.fileno())) != identity:
      raise ValueError('Timeline history changed')
    if cursor and os.pread(source.fileno(), 1, cursor - 1) != b'\n':
      raise ValueError('Invalid timeline cursor')
    source.seek(cursor)
    raw = source.readline(MAX_PAGE + 1)
    if len(raw) > MAX_PAGE or not raw.endswith(b'\n'):
      raise ValueError('Incomplete timeline page')
    value = parse(raw, MAX_PAGE)
    if (set(value) != {'schemaVersion', 'rowOffset', 'totalRows', 'source', 'rows'} or value['schemaVersion'] != 1 or
        type(value['source']) is not str or value['source'] not in ('qlog', 'rlog', 'mixed') or
        any(type(value[key]) is not int for key in ('rowOffset', 'totalRows')) or
        not 0 <= value['rowOffset'] <= value['totalRows'] <= MAX_ROWS):
      raise ValueError('Invalid timeline page')
    validate_rows(value['rows'], ranges)
    count = value['rowOffset'] + len(value['rows'])
    next_cursor = source.tell() if source.tell() < identity[2] else None
    if (count > value['totalRows'] or (next_cursor is None) != (count == value['totalRows']) or
        not value['rows'] and (value['totalRows'] != 0 or cursor != 0) or
        _identity(os.fstat(source.fileno())) != identity or file_identity(path) != identity):
      raise ValueError('Incomplete timeline history')
    return {**value, 'nextCursor': next_cursor, 'legend': LEGEND}


class TimelineHistory:
  """Assemble pages atomically; seeks reuse the same session history."""

  def __init__(self, owner, session):
    self.owner, self.session = owner, session
    self.cursor = 0
    self.pending = []
    self.rows = None
    self.total = self.source = None
    self.previous, self.ends = None, {}
    self.layers = {}

  def append(self, page, ranges):
    if (page.get('ownerId') != self.owner or page.get('sessionId') != self.session or page.get('schemaVersion') != 1 or
        type(page.get('rowOffset')) is not int or page['rowOffset'] != len(self.pending) or
        type(page.get('totalRows')) is not int or not 0 <= page['totalRows'] <= MAX_ROWS or
        page.get('source') not in ('qlog', 'rlog', 'mixed') or page.get('legend') != LEGEND or
        self.total is not None and (page['totalRows'] != self.total or page['source'] != self.source)):
      raise ValueError('Invalid timeline history')
    self.previous, self.ends = validate_rows(page.get('rows'), ranges, self.previous, self.ends)
    next_cursor = page.get('nextCursor')
    if (next_cursor is not None and (type(next_cursor) is not int or not self.cursor < next_cursor < MAX_FILE) or
        len(self.pending) + len(page['rows']) > page['totalRows'] or
        (next_cursor is None) != (len(self.pending) + len(page['rows']) == page['totalRows']) or
        not page['rows'] and page['totalRows'] != 0):
      raise ValueError('Incomplete timeline history')
    self.total, self.source = page['totalRows'], page['source']
    self.pending.extend(page['rows'])
    self.cursor = next_cursor
    if next_cursor is None:
      self.rows, self.pending = self.pending, []
      for kind in ('mode', 'error'):
        rows = [row for row in self.rows if (row[2] == 'error') == (kind == 'error') and row[2] != 'bookmark']
        self.layers[kind] = ([row[0] for row in rows], rows)

  def entry(self, position):
    if self.rows is None:
      return None
    for kind in ('error', 'mode'):
      times, rows = self.layers[kind]
      i = bisect_right(times, position) - 1
      if i >= 0 and rows[i][0] <= position < rows[i][1]:
        return LEGEND[rows[i][2]]
    return None

  def label(self, position):
    if self.rows is None:
      return 'Loading history'
    entry = self.entry(position)
    return entry['label'] if entry else 'Neutral'
