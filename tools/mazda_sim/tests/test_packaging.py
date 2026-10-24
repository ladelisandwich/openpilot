"""What the containers need must actually reach a clone: the repo's .gitignore once swallowed the comma's
entrypoint (`comma*.sh`), and the comma container then could not start on anyone else's PC."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parents[1]

pytestmark = pytest.mark.skipif(shutil.which("git") is None or not (REPO / ".git").exists(), reason="needs the git checkout")


def test_entrypoints_exist():
  for dockerfile in (SIM / "docker").glob("*.Dockerfile"):
    for path in re.findall(r'"(/sim/mazda_sim/[^"]+)"', dockerfile.read_text()):
      assert (SIM / path.removeprefix("/sim/mazda_sim/")).is_file(), f"{dockerfile.name} starts {path}, which is missing"


def test_nothing_of_the_simulator_is_gitignored():
  out = subprocess.run(["git", "status", "--ignored", "--porcelain", "--", str(SIM)], cwd=REPO, capture_output=True,
                       text=True, check=True).stdout
  ignored = [line[3:] for line in out.splitlines() if line.startswith("!! ")]
  ignored = [p for p in ignored if not re.search(r"(__pycache__|\.pyc$|_cache/|\.egg-info)", p)]
  assert not ignored, f"ignored by .gitignore, so a clone would not have them: {ignored}"
