"""Prius four-byte regressions remain mandatory in serial and native CI."""
import ast
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
CALLER = 'openpilot/starpilot/car/toyota/tests/test_prius_advert_callers.py'
NATIVE = 'opendbc_repo/opendbc/safety/tests/test_toyota_prius_advert.py'
FACTORY = 'openpilot/starpilot/car/toyota/tests/test_prius_advert_filter.py'


def constant(name):
  module = ast.parse((ROOT / 'tools/ci/run_vehicle_tests.py').read_text())
  return next(ast.literal_eval(node.value) for node in module.body if isinstance(node, ast.Assign)
              and any(isinstance(target, ast.Name) and target.id == name for target in node.targets))


def test_prius_advert_mandatory_native_and_joined_targets():
  lifecycle = constant('AOL_LIFECYCLE_TARGETS')
  assert all(lifecycle.count(path) == 1 for path in (CALLER, NATIVE, FACTORY))
  assert constant('RELEASE_TARGETS').count(NATIVE) == 1
  assert (ROOT / NATIVE).name.startswith('test_')


def test_prius_advert_serial_group_bounds_and_timing():
  custom = json.loads((ROOT / 'tools/ci/custom_pytest_workloads.json').read_text())
  lifecycle = json.loads((ROOT / 'tools/ci/lifecycle_workloads.json').read_text())
  assert custom['independent_functions'][CALLER]['seconds_per_case'] == 4.0
  assert lifecycle['function_groups'][CALLER]['estimated_seconds_per_case'] == 4.0
  assert lifecycle['function_groups'][CALLER]['independent_parameters']
  assert custom['independent_max_cases'] <= 60
  assert custom['independent_max_seconds'] <= 240
