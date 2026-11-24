"""Matrix qualification must remain mandatory in lifecycle and RELEASE CI."""
import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
CALLER = 'openpilot/starpilot/car/toyota/tests/test_matrix_stock_caller_native.py'
NATIVE = 'opendbc_repo/opendbc/safety/tests/test_toyota_matrix_stock_aol.py'


class TestMatrixRegistration(unittest.TestCase):
  def test_matrix_actual_callers_and_native_are_registered(self):
    module = ast.parse((ROOT / 'tools/ci/run_vehicle_tests.py').read_text())
    values = {node.targets[0].id: ast.literal_eval(node.value) for node in module.body
              if isinstance(node, ast.Assign) and len(node.targets) == 1
              and isinstance(node.targets[0], ast.Name)
              and node.targets[0].id in ('AOL_LIFECYCLE_TARGETS', 'RELEASE_TARGETS')}
    for collection in ('AOL_LIFECYCLE_TARGETS', 'RELEASE_TARGETS'):
      for target in (CALLER, NATIVE):
        assert values[collection].count(target) == 1
        assert (ROOT / target).is_file()
    native = ast.parse((ROOT / NATIVE).read_text())
    caller = ast.parse((ROOT / CALLER).read_text())
    for module in (native, caller):
      assert any(isinstance(node, ast.Assign) and any(isinstance(target, ast.Name)
                 and target.id == 'source_matched_native' for target in node.targets) for node in module.body)
