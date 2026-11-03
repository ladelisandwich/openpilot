#!/usr/bin/env python3
"""Starts the build's own manager with the comma's hardware swapped for the simulator's.

  pandad  -> harnessd     virtual panda on the harness link (the build's own panda safety code)
  camerad -> opticsd      road/wide camera frames from the simulated world
  modeld  -> gt_modeld    ground truth in place of the net (world.model == "groundtruth"); otherwise the build's
                          modeld runs, on a tinygrad device that exists on a PC
Everything else (card, controlsd, selfdrived, plannerd, locationd, ui, loggerd, ...) is the build, unmodified.
"""
from __future__ import annotations

import os
import sys

from ..common.config import SimConfig

# not on a PC, or replaced: no cameras to encode, no microphone, no driver camera model, and StarPilot's mapd,
# which ships as an aarch64 binary
BLOCKED = ["encoderd", "stream_encoderd", "micd", "dmonitoringmodeld", "dmonitoringd", "sensord", "webrtcd", "bridge", "mapd"]


def vision_device(requested: str) -> str:
  """tinygrad device for the real driving model. NVIDIA goes through OpenCL: the NVIDIA container runtime mounts
  the driver's OpenCL library, but not the CUDA toolkit's nvrtc that tinygrad's CUDA backend compiles with."""
  if requested and requested != "auto":
    return {"CPU": "CPU:LLVM"}.get(requested, requested)
  if os.path.exists("/dev/kfd"):
    return "AMD"
  if os.path.exists("/dev/nvidiactl") or os.path.exists("/dev/dri/renderD128"):
    return "CL"
  return "CPU:LLVM"


def main() -> None:
  cfg = SimConfig.load()
  os.environ.setdefault("SIMULATION", "1")
  os.environ.setdefault("NO_FAN_CONTROL", "1")
  # The Galaxy (StarPilot's web UI) runs Flask in debug mode with the auto-reloader off-device, and the reloader
  # re-executes this process's argv: a second manager. Serve it like the device does instead.
  os.environ.setdefault("SP_GALAXY_DEBUG", "0")
  os.environ.setdefault("SP_GALAXY_RELOAD", "0")
  os.environ.setdefault("SP_GALAXY_PORT", "8082")
  os.environ.setdefault("BIG", "1")     # the comma 3X's UI, not the comma four's
  os.environ.setdefault("FPS", "20")    # UI frame rate: software GL in a container
  blocked = list(BLOCKED)
  if os.environ.get("MAZDA_SIM_AUDIO", "0") != "1":
    blocked.append("soundd")  # no sound card in the container
  os.environ["BLOCK"] = ",".join([x for x in os.environ.get("BLOCK", "").split(",") if x] + blocked)
  if cfg.world.model == "vision":
    # This build's modeld asks tinygrad for an 'LLVM' device on PC, which this tinygrad no longer has.
    # Fix the device in the parent before manager forks modeld; the forked child keeps it.
    os.environ["DEV"] = vision_device(cfg.world.vision_device)
    import tinygrad.helpers  # noqa: F401
    print(f"sim_manager: driving model on tinygrad device {os.environ['DEV']}")

  from . import params_setup
  params_setup.apply(cfg)

  from openpilot.system.manager import process_config as pc
  from openpilot.system.manager.process import PythonProcess
  pc.managed_processes["pandad"] = PythonProcess("pandad", "mazda_sim.device.harnessd", pc.always_run)
  pc.managed_processes["camerad"] = PythonProcess("camerad", "mazda_sim.device.opticsd", pc.always_run)
  if cfg.world.model != "vision":
    pc.managed_processes["modeld"] = PythonProcess("modeld", "mazda_sim.device.gt_modeld", pc.only_onroad)

  from openpilot.system.manager import manager
  sys.argv = sys.argv[:1]
  manager.unblock_stdout()
  manager.main()


if __name__ == "__main__":
  main()
