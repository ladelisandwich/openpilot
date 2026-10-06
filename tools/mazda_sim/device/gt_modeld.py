#!/usr/bin/env python3
"""modeld with the neural network replaced by the simulator's ground truth.

The build's own modeld main loop runs unchanged: camera frame pacing, DesireHelper and lane changes, action
smoothing, and every message it publishes (modelV2, drivingModelData, cameraOdometry, starpilotModelV2) through its
own fill_model_msg. Only ModelState.run() differs: instead of a tinygrad net it returns the parsed-output
dictionary the net would have produced, built from the road ahead, the planned path and the lead, which the car
side computes (car/mazda.py, MazdaCar.truth) and harnessd relays here over UDP.

It impersonates the builtin v15 model: `action` carries the desired curvature (x v^2) and acceleration, which
is the path the car's own build takes with the stock model.
"""
from __future__ import annotations

import json
import os
import pickle
import socket
import sys

import numpy as np

GT_UDP_PORT = 7011
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
X_IDXS = np.array([192.0 * (i / 32) ** 2 for i in range(33)])
ROAD_Z = 1.22  # road surface below the device, m (z down)


class TruthFeed:
  def __init__(self, port: int = GT_UDP_PORT):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind(("127.0.0.1", port))
    self.sock.setblocking(False)
    self.latest: dict | None = None

  def poll(self) -> dict | None:
    while True:
      try:
        data, _ = self.sock.recvfrom(1 << 16)
        self.latest = json.loads(data)
      except BlockingIOError:
        return self.latest
      except (OSError, ValueError):
        return self.latest


def _template(models_dir: str, Parser) -> dict[str, np.ndarray]:
  """Every key and shape the build's own parser produces for its vision + policy model."""
  out: dict[str, np.ndarray] = {}
  for part, parse in (("vision", "parse_vision_outputs"), ("policy", "parse_policy_outputs")):
    with open(os.path.join(models_dir, f"driving_{part}_metadata.pkl"), "rb") as fh:
      meta = pickle.load(fh)
    n = max(s.stop for s in meta["output_slices"].values() if s.stop is not None) + 8
    raw = np.zeros(n, dtype=np.float32)
    sliced = {k: raw[np.newaxis, s] for k, s in meta["output_slices"].items()}
    out.update(getattr(Parser(), parse)(sliced))
  out["action"] = np.zeros((1, 2), dtype=np.float32)
  return {k: np.array(v, dtype=np.float32, copy=True) for k, v in out.items()}


def make_state_class(md):
  """Build a ModelState stand-in against the build's modeld module `md`."""
  from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
  from openpilot.selfdrive.modeld.constants import Meta, Plan
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser

  models_dir = os.path.join(os.path.dirname(md.__file__), "models")

  class GroundTruthModelState:
    def __init__(self, cam_w: int, cam_h: int, *args, **kwargs):
      self.template = _template(models_dir, Parser)
      self.truth = TruthFeed()
      self.rng = np.random.default_rng(42)
      self.policy_generation = "v15"
      self.is_v9 = self.is_v14 = False
      self.is_v15 = True
      self.mlsim = True
      self.uses_external_gpu = False
      self.can_prepare_only = False
      self.road_key, self.wide_key = "img", "big_img"
      self.vision_input_names = [self.road_key, self.wide_key]
      self.desire_key = "desire_pulse"
      # declaring action_t makes modeld pass this cycle's lateral and longitudinal action times
      self.numpy_inputs = {self.desire_key: np.zeros((1, 25, 8), dtype=np.float32),
                           "traffic_convention": np.zeros((1, 2), dtype=np.float32),
                           "action_t": np.zeros((1, 2), dtype=np.float32)}
      self.off_policy_enabled = False
      self.off_policy_numpy_inputs = {}
      self.prev_desired_curv_key = None
      self.frame_skip = 4
      md.Params().put("ModelVersion", self.policy_generation)
      md.Params().put("DrivingModelVersion", self.policy_generation)
      print("gt_modeld: ground-truth model ready (impersonating the builtin v15 model)")

    def warmup(self) -> None:
      pass

    def run(self, bufs, transforms, inputs, prepare_only):
      if prepare_only:
        return None
      tr = self.truth.poll()
      out = {k: v.copy() for k, v in self.template.items()}
      if tr is None:
        return out
      p = tr["plan"]
      # a real net is never exactly certain: small noise, and no exact zeros downstream code might divide by
      rng = self.rng
      x, y = np.array(p["x"]), np.array(p["y"]) + rng.normal(0.0, 0.01, 33)
      v, a = np.array(p["v"]), np.array(p["a"]) + rng.normal(0.0, 0.01, 33)
      yaw = np.array(p["yaw"]) + rng.normal(0.0, 2e-4, 33)
      yaw_rate = np.array(p["yawRate"]) + rng.normal(0.0, 2e-4, 33)
      y[0] = yaw[0] = 0.0
      plan = out["plan"][0]
      plan[:, Plan.POSITION] = np.stack([x, y, np.zeros_like(x)], axis=1)
      plan[:, Plan.VELOCITY] = np.stack([v * np.cos(yaw), v * np.sin(yaw), np.zeros_like(v)], axis=1)
      plan[:, Plan.ACCELERATION] = np.stack([a, np.zeros_like(a), np.zeros_like(a)], axis=1)
      plan[:, Plan.T_FROM_CURRENT_EULER] = np.stack([np.zeros_like(yaw), np.zeros_like(yaw), yaw], axis=1)
      plan[:, Plan.ORIENTATION_RATE] = np.stack([np.zeros_like(yaw), np.zeros_like(yaw), yaw_rate], axis=1)
      if "plan_stds" in out:
        out["plan_stds"][:] = 0.1

      for i, line in enumerate(tr["lanes"][:4]):
        out["lane_lines"][0, i, :, 0] = np.array(line) + rng.normal(0.0, 0.01, 33)
        out["lane_lines"][0, i, :, 1] = ROAD_Z
      if "lane_lines_stds" in out:
        out["lane_lines_stds"][:] = 0.05
      for i, prob in enumerate(tr["laneProbs"][:4]):
        out["lane_lines_prob"][0, 2 * i] = 1.0 - prob
        out["lane_lines_prob"][0, 2 * i + 1] = prob
      for i, edge in enumerate(tr["edges"][:2]):
        out["road_edges"][0, i, :, 0] = edge
        out["road_edges"][0, i, :, 1] = ROAD_Z
      if "road_edges_stds" in out:
        out["road_edges_stds"][:] = 0.1

      lead = tr.get("lead")
      out["lead_prob"][:] = 0.0
      if lead is not None:
        for i in range(out["lead"].shape[1]):
          out["lead"][0, i, :, 0] = lead["x"]
          out["lead"][0, i, :, 1] = lead["y"]
          out["lead"][0, i, :, 2] = lead["v"]
          out["lead"][0, i, :, 3] = lead["a"]
        out["lead_prob"][0, :] = lead["prob"]
        if "lead_stds" in out:
          out["lead_stds"][:] = 0.5

      out["meta"][0, :] = 0.0
      out["meta"][0, Meta.ENGAGED] = 1.0
      out["desire_state"][0, :] = 0.0
      out["desire_state"][0, 0] = 1.0
      if "desire_pred" in out:
        out["desire_pred"][0, :, :] = 0.0
        out["desire_pred"][0, :, 0] = 1.0

      v0 = float(tr["v"])
      out["pose"][0, :] = [v0, 0.0, 0.0, 0.0, 0.0, float(tr["yawRate"])]
      if "pose_stds" in out:
        out["pose_stds"][0, :] = [0.1, 0.1, 0.1, 0.01, 0.01, 0.01]
      out["road_transform"][:] = 0.0

      action_t = inputs.get("action_t") if isinstance(inputs, dict) else None
      lat_t = float(action_t[0]) if action_t is not None else 0.4
      long_t = float(action_t[1]) if action_t is not None else 0.6
      curvature = float(get_curvature_from_plan(yaw, yaw_rate, T_IDXS, v0, lat_t))
      accel = float(np.interp(long_t, T_IDXS, a))
      out["action"][0, :] = [curvature * max(1.0, v0) ** 2, accel]
      self.n_runs = getattr(self, "n_runs", 0) + 1
      if os.environ.get("MAZDA_SIM_GT_DEBUG") and self.n_runs % 20 == 0:
        print(f"gt_modeld: v={v0:.2f} laneOffset={tr.get('laneOffset')} curvature={curvature:+.5f} lat_t={lat_t:.2f} " +
              f"yaw@lat_t={np.interp(lat_t, T_IDXS, yaw):+.4f} yawRate0={yaw_rate[0]:+.4f} action={out['action'][0].tolist()}", flush=True)
      return out

  return GroundTruthModelState


def main() -> None:
  from openpilot.selfdrive.modeld import modeld as md
  state_cls = make_state_class(md)
  md.ModelState = state_cls
  md._load_model_state = lambda cam_w, cam_h, *a, **k: state_cls(cam_w, cam_h)
  md.usbgpu_present = lambda: False
  md.model_uses_external_gpu = lambda *_a, **_k: False
  md.main(demo="--demo" in sys.argv)


if __name__ == "__main__":
  main()
