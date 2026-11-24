"""Native ring wrap/freeze/faults and concurrent snapshots; no output device needed."""
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize('name', ('test_diagnostic', 'test_diagnostic_concurrent'))
def test_native_diagnostic_contract(tmp_path, name):
  source = Path(__file__).with_name('native') / (name + '.c')
  executable = tmp_path / name
  subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread', str(source),
                  '-o', str(executable)], check=True, capture_output=True, timeout=15)
  subprocess.run([str(executable)], check=True, capture_output=True, timeout=20)
