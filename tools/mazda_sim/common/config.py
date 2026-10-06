"""Simulation configuration, shared by the launcher, the Mazda container and the comma container.

Stdlib only. Saved as JSON (sim.json) by the launcher and handed to both containers.
"""
from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field, fields

CONFIG_ENV = "MAZDA_SIM_CONFIG"


@dataclass
class CarOptions:
  """What is fitted to the simulated car. Each maps to the param the build reads (device/params_setup.py)."""
  fingerprint: str = "MAZDA_CX9_2021"   # CX-9 2021-23; the FW versions served come from this platform
  vin: str = "JM3TCBDY4P0600001"         # 2023 CX-9 Grand Touring pattern
  torque_interceptor: bool = True        # TorqueInterceptorEnabled
  ti_version: int = 1                    # TI1 has no RAMP_DOWN; 2+ reports it
  radar_emulation: bool = False          # RadarEmulationEnabled: openpilot silences the radar and impersonates it
  hybrid_long: bool = False              # MazdaHybridLong: MRCC or emulation, chosen with the distance button
  no_fsc: bool = False                   # NoFSC: car without a Forward Sensing Camera
  no_mrcc: bool = False                  # NoMRCC: car without radar cruise
  manual_transmission: bool = False      # ManualTransmission
  experimental_mode: bool = False        # ExperimentalMode
  lower_min_set_speed: bool = False      # LowerMinSetSpeed
  imperial: bool = True                  # IsMetric = not imperial; also the dash's set-speed step
  # Any other params the build under test should start with, e.g. {"BlendedACC": true}
  extra_params: dict = field(default_factory=dict)


@dataclass
class WorldOptions:
  world: str = "lite"                    # "lite": no rendering, any PC. "metadrive": rendered road and cameras
  track: str = "highway"                 # highway | loop | twisty | city | straight
  lanes: int = 2
  lane_width: float = 3.6
  start_speed_kph: float = 0.0
  lead: str = "none"                     # none | cruise | stopgo | brake
  lead_speed_kph: float = 80.0
  lead_gap_m: float = 45.0
  model: str = "groundtruth"             # groundtruth: modelV2 from the road itself. vision: the real driving model
  vision_device: str = "auto"            # auto | CUDA | AMD | CL | CPU  (tinygrad backend for the real model)
  dual_camera: bool = True               # also render the wide road camera
  frame_codec: str = "nv12"              # nv12 (raw, same machine) | jpeg (smaller, costs CPU)


@dataclass
class PlantOptions:
  """The physical car. Defaults reproduce the CX-9's learned lateral response (LAT_ACCEL_FACTOR 1.76 m/s^2 at
  full command, highway speed, stock LKAS and TI together). Tune these against your own logs."""
  mass_kg: float = 2050.0
  sensor_nm_per_unit: float = 0.1        # torque sensor scale: STEER_TORQUE_SENSOR / TI_TORQUE_SENSOR raw unit in Nm
  lkas_full_nm: float = 2.4              # EPS motor torque (column-equivalent) for LKAS_REQUEST = 800
  ti_inject_full_nm: float = 1.0         # sensor torque the TI fakes for a command of 800, before EPS assist
  assist_bp_kph: list = field(default_factory=lambda: [0.0, 30.0, 60.0, 100.0, 130.0])
  assist_gain: list = field(default_factory=lambda: [4.0, 3.0, 2.2, 1.6, 1.4])
  align_nm_per_mps2: float = 2.27        # column torque that holds 1 m/s^2 of lateral accel at speed
  column_friction_nm: float = 0.25
  lkas_enable_kph: float = 52.0          # stock EPS accepts LKAS above this ...
  lkas_disable_kph: float = 45.0         # ... and drops it below this
  hands_off_lockout_s: float = 15.0      # stock EPS locks LKAS out after this long without a hands-on torque
  ti_driver_over_units: int = 25         # TI hands control back above this driver torque (raw units)
  ti_driver_release_units: int = 12      # ... and resumes below this
  ti_rate_limit_units: int = 60          # TI flags a VIOL above this command step per frame


@dataclass
class BuildOptions:
  """Which openpilot build the comma container runs."""
  source: str = "git"                    # git | local
  repo: str = "https://github.com/ladelisandwich/openpilot.git"
  ref: str = "mazda-long-ti1-testing"
  local_path: str = ""                   # with source=local: a checkout on your PC, mounted read-only and copied


@dataclass
class SimConfig:
  car: CarOptions = field(default_factory=CarOptions)
  world: WorldOptions = field(default_factory=WorldOptions)
  plant: PlantOptions = field(default_factory=PlantOptions)
  build: BuildOptions = field(default_factory=BuildOptions)

  def to_dict(self) -> dict:
    return asdict(self)

  @classmethod
  def from_dict(cls, d: dict | None) -> SimConfig:
    d = d or {}
    out = cls()
    for f in fields(cls):
      section = getattr(out, f.name)
      for k, v in (d.get(f.name) or {}).items():
        if hasattr(section, k):
          setattr(section, k, v)
    return out

  def save(self, path: str) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
      json.dump(self.to_dict(), fh, indent=2)
    os.replace(tmp, path)

  @classmethod
  def load(cls, path: str | None = None) -> SimConfig:
    path = path or os.environ.get(CONFIG_ENV, "")
    if path and os.path.isfile(path):
      with open(path) as fh:
        return cls.from_dict(json.load(fh))
    return cls()
