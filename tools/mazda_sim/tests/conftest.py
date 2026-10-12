import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[2]
REPO = TOOLS.parent
for p in (str(TOOLS), str(REPO / "opendbc_repo")):
  if p not in sys.path:
    sys.path.insert(0, p)
