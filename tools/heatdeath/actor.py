"""Run a normal daemon, optionally placing only its own realtime entrypoint."""

import argparse
import importlib
import sys

if __package__:
  from .placement import apply
else:
  from placement import apply


def install(module, name):
  for attribute in ('config_realtime_process', 'set_core_affinity'):
    if not hasattr(module, attribute):
      continue
    original = getattr(module, attribute)

    def placed(*args, _original=original, **kwargs):
      result = _original(*args, **kwargs)
      apply(name)
      return result

    setattr(module, attribute, placed)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('name', choices=('radard', 'plannerd'))
  args = parser.parse_args()
  module = importlib.import_module('openpilot.selfdrive.controls.' + args.name)
  if hasattr(module, 'config_realtime_process'):
    install(module, args.name)
  else:
    apply(args.name)
  module.main()


if __name__ == '__main__':
  sys.dont_write_bytecode = True
  main()
