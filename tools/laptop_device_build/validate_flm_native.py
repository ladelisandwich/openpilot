"""Read-only source and ABI binding check for the shipped FLM primitive extension."""
import hashlib
import json
from pathlib import Path


def validate(root: Path) -> None:
  here = root / 'openpilot/starpilot/flm'
  sources = {'_primitive_walk.pyx': here / '_primitive_walk.pyx', 'native_primitives.py': here / 'native_primitives.py',
             'SConscript': here / 'SConscript', 'SConstruct': root / 'SConstruct'}
  artifact = here / '_primitive_walk.so'
  receipt = here / '_primitive_walk.binding.json'
  if artifact.is_symlink() or receipt.is_symlink() or any(path.is_symlink() for path in sources.values()):
    raise ValueError('FLM native binding cannot use symbolic files')
  value = json.loads(receipt.read_text())
  expected = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in sources.items()}
  if type(value) is not dict or type(value.get('abi')) is not dict:
    raise ValueError('FLM native binding object schema')
  abi = value['abi']
  if (value.get('schema') != 1 or value.get('revision') != 'flm-exact-primitives-v1' or
      value.get('sources') != expected or value.get('artifact_sha256') != hashlib.sha256(artifact.read_bytes()).hexdigest() or
      abi.get('implementation') != 'cpython' or abi.get('python') != [3, 12] or abi.get('cache_tag') != 'cpython-312' or
      abi.get('machine') != 'aarch64' or abi.get('pointer_bits') != 64 or abi.get('platform') != 'linux' or
      abi.get('SOABI') != 'cpython-312-aarch64-linux-gnu'):
    raise ValueError('FLM native artifact source or target ABI binding mismatch')
