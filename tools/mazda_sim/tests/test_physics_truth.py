"""Sign conventions end to end: road, vehicle, ground truth, and the MetaDrive map built from the same road."""
import math

import numpy as np
import pytest

from mazda_sim.car.mazda import MazdaCar
from mazda_sim.car.road import Road, Straight, TRACKS
from mazda_sim.car.vehicle import Vehicle
from mazda_sim.common.config import PlantOptions, SimConfig


@pytest.mark.parametrize("name", [t for t in TRACKS if t != "straight"])
def test_tracks_close(name):
  r = Road.preset(name)
  assert r.closed
  x0, y0, h0, _ = r.pose_at(0.0)
  x1, y1, h1, _ = r.pose_at(r.length - 1e-3)
  assert math.hypot(x1 - x0, y1 - y0) < 1.0
  assert abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi) < 0.02


def test_lanes_are_right_of_the_reference_line():
  r = Road.preset("straight", lanes=2, lane_width=3.6)
  assert r.lane_offset(0) == pytest.approx(-1.8) and r.lane_offset(1) == pytest.approx(-5.4)
  _, y, _, _ = r.pose_at(10.0, r.lane_offset(1))
  assert y == pytest.approx(-5.4)   # heading +x: right is -y


def test_left_torque_turns_left():
  veh = Vehicle(PlantOptions())
  veh.reset(0.0, 0.0, 0.0, 15.0)
  veh.drive_force = 600.0
  for _ in range(200):
    veh.steer.t_driver = 1.5
    veh.step(0.01)
  assert veh.steering_wheel_deg > 5.0 and veh.r > 0.02 and veh.y > 0.0


def test_ground_truth_plan_follows_a_left_corner():
  cfg = SimConfig.from_dict({"world": {"track": "loop", "start_speed_kph": 72}})
  car = MazdaCar(cfg)
  car.reset()
  gt = car.truth()
  y = np.array(gt["plan"]["y"])
  x = np.array(gt["plan"]["x"])
  assert abs(y[0]) < 0.3 and x[-1] > 50.0
  assert y[-1] < -5.0, "the loop turns left ahead: the plan must bend to negative y (device frame, y right)"
  lanes = gt["lanes"]
  assert len(lanes) == 4


def test_metadrive_blocks_fold_straights_into_curves():
  from mazda_sim.car.world_metadrive import metadrive_blocks
  road = Road.preset("loop", lanes=2, lane_width=3.6)
  blocks = metadrive_blocks(road)
  assert blocks[0] is None and blocks[1]["id"] == "S"
  curves = [b for b in blocks[1:] if b["id"] == "C"]
  assert len(curves) == 4 and all(c["dir"] == 0 for c in curves)
  # MetaDrive bends its rightmost lane center; ours is the left edge: R + 1.5 lane widths for left turns
  arc = next(b for b in road.blocks if not isinstance(b, Straight))
  assert curves[0]["radius"] == pytest.approx(arc.radius + 1.5 * 3.6)
  # a straight after each corner becomes the curve block's own straight
  assert sum(b["length"] for b in blocks[1:] if b["id"] == "S") + sum(c["length"] for c in curves) == \
    pytest.approx(sum(b.length for b in road.blocks if isinstance(b, Straight)) + 0.0, abs=0.1)
