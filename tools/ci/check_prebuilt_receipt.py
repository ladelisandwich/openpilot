"""Check the committed release inventory before device downloads use it."""
import argparse
import importlib.util
from pathlib import Path
import subprocess


class ReceiptError(RuntimeError):
  pass


def check_receipt(repo, revision='HEAD'):
  repo = Path(repo).resolve()
  def git(*args):
    return subprocess.check_output(['git', '--no-optional-locks', '-c', 'gc.auto=0', '-c', 'maintenance.auto=false',
                                    '-C', str(repo), *args], text=True, timeout=30)
  commit = git('rev-parse', '--verify', '--end-of-options', revision + '^{commit}').strip()
  if git('ls-tree', commit, '--', 'prebuilt').strip().split()[:1] != ['100644']:
    raise ReceiptError('Published device release requires a regular prebuilt marker')
  spec = importlib.util.spec_from_file_location('release_prebuilt_manifest', repo / 'openpilot/common/prebuilt_manifest.py')
  if spec is None or spec.loader is None:
    raise ReceiptError('Prebuilt manifest validator is unavailable')
  manifest = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(manifest)
  if not manifest.valid_receipt(git, commit):
    raise ReceiptError(f'Prebuilt release {commit} has no matching build receipt; validate and refresh prebuilt.json before publishing')
  return commit


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
  parser.add_argument('--revision', default='HEAD')
  args = parser.parse_args()
  try:
    commit = check_receipt(args.repo, args.revision)
  except (ReceiptError, subprocess.CalledProcessError, ValueError) as error:
    parser.exit(1, f'{error}\n')
  print(f'Committed prebuilt receipt matches {commit}')


if __name__ == '__main__':
  main()
