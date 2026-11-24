"""Checkout-local BlueALSA 5 bundle and reversible /run service selection."""
from __future__ import annotations

import hashlib
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent / 'bluealsa'
ARTIFACTS = ('bin/bluealsad', 'bin/bluealsactl', 'bin/bluealsa-aplay',
             'lib/libasound_module_pcm_bluealsa.so', 'lib/libasound_module_ctl_bluealsa.so')


def validate(root: Path | None = None) -> None:
  root = root or ROOT
  if (root / 'VERSION').read_text().strip() != '5.0.0':
    raise RuntimeError('BlueALSA 5.0.0 bundle is missing')
  records = dict(line.split(maxsplit=1)[::-1] for line in (root / 'SHA256SUMS').read_text().splitlines())
  for name in ARTIFACTS:
    data = (root / name).read_bytes()
    if records.get(name) != hashlib.sha256(data).hexdigest():
      raise RuntimeError(f'BlueALSA artifact checksum failed: {name}')
    if data[:6] != b'\x7fELF\x02\x01' or data[18:20] != b'\xb7\x00':
      raise RuntimeError(f'BlueALSA artifact is not ARM64: {name}')
    if name.startswith('bin/') and not os.access(root / name, os.X_OK):
      raise RuntimeError(f'BlueALSA executable is not executable: {name}')


def alsa_plugins(root: Path | None = None) -> str:
  """Explicit library paths affect this ALSA config only, never system defaults."""
  root = root or ROOT
  result = ''
  for kind in ('pcm', 'ctl'):
    library = str(root / 'lib' / f'libasound_module_{kind}_bluealsa.so')
    if any(c in library for c in '\n\r"\\'):
      raise RuntimeError('Invalid BlueALSA plugin path')
    result += f'{kind}_type.bluealsa {{ lib "{library}" }}\n'
  return result


def paths(legacy: Path) -> tuple[Path, Path]:
  return legacy.with_name('95-starpilot-bluealsa5.conf'), legacy.parent.parent / 'starpilot-bluealsa-launch.py'


def contents(legacy: Path, root: Path | None = None) -> str:
  root = root or ROOT
  override, launcher = paths(legacy)
  def quote(path: Path) -> str:
    value = str(path)
    if any(c in value for c in '\n\r"\\$'):
      raise RuntimeError('Invalid BlueALSA service path')
    return '"' + value.replace('%', '%%') + '"'
  return ('[Service]\nExecStart=\nExecStart=/usr/bin/python3 ' +
          ' '.join(quote(p) for p in (launcher, root, legacy, override)) + '\n')


def check_owned(legacy: Path, root: Path | None = None) -> None:
  override, _ = paths(legacy)
  if override.exists() and override.read_text() != contents(legacy, root):
    raise RuntimeError('BlueALSA 5 override was changed externally')


def install(legacy: Path, root: Path | None = None) -> None:
  root = root or ROOT
  validate(root)
  check_owned(legacy, root)
  override, launcher = paths(legacy)
  launcher.parent.mkdir(parents=True, exist_ok=True)
  temporary = launcher.with_suffix('.tmp')
  temporary.write_bytes((root / 'launcher.py').read_bytes())
  temporary.chmod(0o644)
  temporary.replace(launcher)
  override.write_text(contents(legacy, root))


def remove(legacy: Path, root: Path | None = None) -> None:
  check_owned(legacy, root)
  override, launcher = paths(legacy)
  override.unlink(missing_ok=True)
  launcher.unlink(missing_ok=True)
