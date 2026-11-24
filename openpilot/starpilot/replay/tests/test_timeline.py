"""History is optional, bounded, session-owned metadata, not playback state."""
import json
import os

import pytest

from openpilot.starpilot.replay.timeline import LEGEND, TimelineHistory, file_identity, read_page


OWNER, SESSION = 'a' * 32, 'b' * 32


def history_file(path, rows):
  pages = []
  for offset in range(0, max(1, len(rows)), 256):
    pages.append(json.dumps({'schemaVersion': 1, 'rowOffset': offset, 'totalRows': len(rows),
                             'source': 'qlog', 'rows': rows[offset:offset + 256]}, separators=(',', ':')) + '\n')
  path.write_text(''.join(pages))
  path.chmod(0o600)
  return file_identity(path)


def test_paged_history_is_atomic_and_seek_independent(tmp_path):
  path = tmp_path / 'timeline.jsonl'
  rows = [[i / 100, (i + 1) / 100, 'chill' if i % 2 else 'experimental'] for i in range(300)]
  identity = history_file(path, rows)
  history = TimelineHistory(OWNER, SESSION)
  page = read_page(path, 0, identity, [[0, 3]])
  history.append({**page, 'ownerId': OWNER, 'sessionId': SESSION}, [[0, 3]])
  assert history.rows is None and history.label(1) == 'Loading history'
  assert history.entry(1) is None
  page = read_page(path, history.cursor, identity, [[0, 3]])
  history.append({**page, 'ownerId': OWNER, 'sessionId': SESSION}, [[0, 3]])
  assert history.rows == rows and history.cursor is None
  assert history.label(2.995) == 'Chill'
  assert history.label(.005) == 'Experimental'
  assert history.source == 'qlog'


def test_alert_bookmark_and_mode_layers(tmp_path):
  path = tmp_path / 'timeline.jsonl'
  rows = [[0, 1, 'chill'], [.2, .8, 'error'], [.4, .4, 'bookmark'], [1, 2, 'aol']]
  identity = history_file(path, rows)
  history = TimelineHistory(OWNER, SESSION)
  page = read_page(path, 0, identity, [[0, 2]])
  history.append({**page, 'ownerId': OWNER, 'sessionId': SESSION}, [[0, 2]])
  assert history.label(.4) == 'Alert'
  assert history.entry(.4) is LEGEND['error']
  assert history.label(.8) == 'Chill'
  assert history.entry(.8) is LEGEND['chill']
  assert history.label(1) == 'AOL'
  assert history.label(2) == 'Neutral'
  assert history.entry(2) is None


@pytest.mark.parametrize('cursor', [-1, True, 1, 10000000])
def test_invalid_cursor(tmp_path, cursor):
  path = tmp_path / 'timeline.jsonl'
  identity = history_file(path, [])
  with pytest.raises(ValueError):
    read_page(path, cursor, identity, [[0, 1]])


@pytest.mark.parametrize('rows', [
  [[0, 1, 'invented']], [[0, float('inf'), 'error']], [[0, 2, 'chill']],
  [[0, 1, 'bookmark']], [[0, 0, 'error']], [[0, .7, 'chill'], [.4, .8, 'aol']],
])
def test_bad_rows_are_rejected(tmp_path, rows):
  path = tmp_path / 'timeline.jsonl'
  identity = history_file(path, rows)
  with pytest.raises(ValueError):
    read_page(path, 0, identity, [[0, 1]])


def test_empty_and_replaced_or_truncated_artifact(tmp_path):
  path = tmp_path / 'timeline.jsonl'
  identity = history_file(path, [])
  page = read_page(path, 0, identity, [[0, 1]])
  assert page['rows'] == [] and page['nextCursor'] is None and page['legend'] == LEGEND
  with pytest.raises(ValueError):
    read_page(path, identity[2], identity, [[0, 1]])
  replacement = tmp_path / 'other'
  history_file(replacement, [])
  replacement.replace(path)
  with pytest.raises(ValueError):
    read_page(path, 0, identity, [[0, 1]])
  path.write_bytes(path.read_bytes().rstrip(b'\n'))
  with pytest.raises(ValueError):
    read_page(path, 0, file_identity(path), [[0, 1]])


def test_symlink_and_bad_permissions_are_rejected(tmp_path):
  path = tmp_path / 'timeline.jsonl'
  history_file(path, [])
  link = tmp_path / 'link'
  link.symlink_to(path)
  with pytest.raises(OSError):
    file_identity(link)
  os.chmod(path, 0o644)
  with pytest.raises(ValueError):
    file_identity(path)


def test_late_page_cannot_change_session_or_count(tmp_path):
  path = tmp_path / 'timeline.jsonl'
  identity = history_file(path, [])
  page = read_page(path, 0, identity, [[0, 1]])
  for change in ({'sessionId': 'c' * 32}, {'rowOffset': 1}, {'nextCursor': 0}, {'totalRows': 1}):
    with pytest.raises(ValueError):
      TimelineHistory(OWNER, SESSION).append({**page, 'ownerId': OWNER, 'sessionId': SESSION, **change}, [[0, 1]])
