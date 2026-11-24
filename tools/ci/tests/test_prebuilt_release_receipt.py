import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tools.ci.check_prebuilt_receipt import ReceiptError, check_receipt

ROOT = Path(__file__).resolve().parents[3]


class TestPublishedReceipt(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.repo = Path(self.temp.name)
    self.environment = {name: value for name, value in os.environ.items()
                        if name not in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE')}
    self.git('init', '-b', 'release')
    self.git('config', 'user.name', 'Receipt Test')
    self.git('config', 'user.email', 'receipt@example.invalid')
    path = self.repo / 'openpilot/common/prebuilt_manifest.py'
    path.parent.mkdir(parents=True)
    shutil.copy2(ROOT / 'openpilot/common/prebuilt_manifest.py', path)
    spec = importlib.util.spec_from_file_location('fixture_prebuilt_manifest', path)
    assert spec is not None and spec.loader is not None
    self.manifest = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(self.manifest)
    (self.repo / 'prebuilt').touch()
    (self.repo / 'native.so').write_bytes(b'original native artifact')
    (self.repo / 'native.cc').write_text('int original_source;\n')
    self.receipt()

  def git(self, *args):
    return subprocess.check_output(['git', '-c', 'gc.auto=0', '-c', 'maintenance.auto=false', '-C', str(self.repo), *args],
                                   env=self.environment, text=True, stderr=subprocess.PIPE).strip()

  def commit(self):
    self.git('add', '.')
    self.git('commit', '-m', 'fixture')
    return self.git('rev-parse', 'HEAD')

  def receipt(self):
    self.git('add', '.')
    entries = []
    for record in self.git('ls-files', '--stage', '-z').split('\0'):
      if record:
        metadata, name = record.split('\t', 1)
        mode, blob, stage = metadata.split()
        assert stage == '0'
        entries.append((name, mode, blob))
    (self.repo / 'prebuilt.json').write_text(json.dumps({'schema_version': 1,
      'protected_inventory_sha256': self.manifest.inventory_digest(entries)})+'\n')
    return self.commit()

  def test_matching_committed_release_passes(self):
    self.assertEqual(check_receipt(self.repo), self.git('rev-parse', 'HEAD'))

  def test_stale_source_and_artifact_each_fail(self):
    for name in ('native.cc', 'native.so'):
      with self.subTest(name=name):
        (self.repo / name).write_bytes(b'changed')
        self.commit()
        with self.assertRaisesRegex(ReceiptError, 'no matching build receipt'):
          check_receipt(self.repo)
        self.receipt()

  def test_ui_only_runtime_change_keeps_existing_contract(self):
    path = self.repo / 'openpilot/starpilot/ui/onroad.py'
    path.parent.mkdir(parents=True)
    path.write_text('updated_ui = True\n')
    commit = self.commit()
    self.assertEqual(check_receipt(self.repo), commit)

  def test_protected_test_change_also_requires_receipt(self):
    path = self.repo / 'tools/ci/tests/example.py'
    path.parent.mkdir(parents=True)
    path.write_text('changed_fixture = True\n')
    self.commit()
    with self.assertRaises(ReceiptError):
      check_receipt(self.repo)

  def test_checks_committed_tree_not_dirty_working_bytes(self):
    commit = self.git('rev-parse', 'HEAD')
    (self.repo / 'native.so').write_bytes(b'uncommitted')
    self.assertEqual(check_receipt(self.repo), commit)

  def test_missing_marker_or_receipt_fails(self):
    for name in ('prebuilt.json', 'prebuilt'):
      with self.subTest(name=name):
        (self.repo / name).unlink()
        self.commit()
        with self.assertRaises(ReceiptError):
          check_receipt(self.repo)
        (self.repo / name).touch()
        self.receipt()

  def test_old_valid_revision_remains_valid_after_bad_release(self):
    previous = self.git('rev-parse', 'HEAD')
    (self.repo / 'native.cc').write_text('new source without receipt\n')
    self.commit()
    self.assertEqual(check_receipt(self.repo, previous), previous)
    with self.assertRaises(ReceiptError):
      check_receipt(self.repo)
