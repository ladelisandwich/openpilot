import pytest
from pathlib import Path

from openpilot.common.params import Params
from openpilot.starpilot.state_migration import prepare_manager_start
from openpilot.starpilot.system.starport.settings import enabled, set_enabled


@pytest.fixture
def saved_settings(tmp_path):
  params = Params(str(tmp_path / 'params'))
  storage = tmp_path / 'storage'
  prepare_manager_start(params, storage)
  return params, storage


@pytest.mark.parametrize('value', [False, True])
def test_legacy_enabled_value_migrates_at_startup(saved_settings, value):
  params, storage = saved_settings
  legacy = Path(params.get_param_path('AndroidAutoEnabled'))
  legacy.write_bytes(b'1' if value else b'0')

  prepare_manager_start(params, storage, auto_migrate=True)

  assert enabled(params) is value
  assert params.get('StarPortEnabled') is value
  assert not legacy.exists()
  assert b'AndroidAutoEnabled' not in params.all_keys()
  assert enabled(params) is value


def test_new_disabled_value_wins_over_stale_legacy_value(saved_settings):
  params, storage = saved_settings
  params.put_bool('StarPortEnabled', False, block=True)
  legacy = Path(params.get_param_path('AndroidAutoEnabled'))
  legacy.write_bytes(b'1')

  prepare_manager_start(params, storage, auto_migrate=True)

  assert not enabled(params)
  assert not legacy.exists()


def test_normal_reads_and_writes_do_not_migrate(saved_settings):
  params, _ = saved_settings
  legacy = Path(params.get_param_path('AndroidAutoEnabled'))
  legacy.write_bytes(b'1')

  assert not enabled(params)

  set_enabled(params, False)

  assert params.get('StarPortEnabled') is False
  assert legacy.read_bytes() == b'1'


def test_invalid_legacy_value_is_not_enabled(saved_settings):
  params, storage = saved_settings
  legacy = Path(params.get_param_path('AndroidAutoEnabled'))
  legacy.write_bytes(b'invalid')

  prepare_manager_start(params, storage, auto_migrate=True)

  assert not enabled(params)
  assert not legacy.exists()
