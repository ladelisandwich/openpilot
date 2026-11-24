"""Exercise modeld's real publication statements without loading a GPU model."""
import ast
import copy
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]


def publication_program():
  tree = ast.parse((ROOT / "selfdrive/modeld/modeld.py").read_text())
  def sends(node):
    return (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and
            isinstance(node.value.func, ast.Attribute) and ast.unparse(node.value.func) == "pm.send")
  def slice_statements(statements):
    result = []
    for statement in statements:
      if sends(statement):
        result.append(copy.deepcopy(statement))
      elif isinstance(statement, ast.If):
        body = slice_statements(statement.body)
        other = slice_statements(statement.orelse)
        if body or other:
          result.append(ast.If(test=copy.deepcopy(statement.test), body=body or [ast.Pass()], orelse=other))
    return result
  main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
  loop = next(node for node in ast.walk(main) if isinstance(node, ast.While) and any(
    isinstance(child, ast.Assign) and ast.unparse(child.targets[0]) == "vipc_dropped_frames" for child in node.body))
  count = next(node for node in loop.body if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "vipc_dropped_frames")
  advance = next(node for node in loop.body if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "last_vipc_frame_id")
  program = ast.Module(body=[copy.deepcopy(count), *slice_statements(loop.body), copy.deepcopy(advance)], type_ignores=[])
  services = [node.value.args[0].value for node in ast.walk(program) if sends(node)]
  assert sorted(services) == sorted(["modelV2", "drivingModelData", "cameraOdometry", "laneChangeAssistWire"])
  return compile(ast.fix_missing_locations(program), "<actual modeld publication slice>", "exec")


class TestModeldPublication(unittest.TestCase):
  def replay(self, frames, *, model_output=True, lane_status=True):
    emitted = []
    namespace = {"max": max, "last_vipc_frame_id": frames[0] - 1, "model_output": object() if model_output else None,
                 "pm": SimpleNamespace(send=lambda service, payload: emitted.append((service, payload)))}
    program = publication_program()
    for frame in frames:
      namespace["meta_main"] = SimpleNamespace(frame_id=frame)
      for name in ("modelv2_send", "drivingdata_send", "posenet_send", "lane_status_msg"):
        namespace[name] = (frame, frame * 50_000_000) if name != "lane_status_msg" or lane_status else None
      exec(program, namespace)
      self.assertEqual(namespace["last_vipc_frame_id"], frame)
    return emitted

  def test_recorded_route15_frame_gap_recovers_on_next_frame(self):
    # Original dense route: frame 1900 was skipped; invalid odometry 1901 preceded inputsOK=false.
    emitted = self.replay([1899, 1901, 1902])
    self.assertEqual(len(emitted), 8)
    self.assertEqual({payload[0] for _, payload in emitted}, {1899, 1902})
    self.assertTrue(all(stamp == frame * 50_000_000 for _, (frame, stamp) in emitted))

  def test_contiguous_frames_keep_all_publications(self):
    self.assertEqual(len(self.replay([1899, 1900, 1901])), 12)

  def test_continuous_drops_do_not_fabricate_fresh_messages(self):
    emitted = self.replay([1899, 1901, 1903, 1905])
    self.assertEqual(len(emitted), 4)
    self.assertEqual({payload[0] for _, payload in emitted}, {1899})

  def test_missing_model_output_never_publishes(self):
    self.assertEqual(self.replay([1899, 1900, 1902], model_output=False), [])

  def test_optional_lane_status_stays_optional(self):
    emitted = self.replay([1899, 1901, 1902], lane_status=False)
    self.assertEqual(len(emitted), 6)
    self.assertNotIn("laneChangeAssistWire", {service for service, _ in emitted})

  def test_pose_validity_still_rejects_missing_calibration_and_drops(self):
    tree = ast.parse((ROOT / "selfdrive/modeld/fill_model_msg.py").read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "fill_pose_msg")
    function.returns = None
    for argument in function.args.args:
      argument.annotation = None
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), "<actual fill_pose_msg>", "exec"), namespace)
    class Array:
      def __getitem__(self, key):
        return self
      def tolist(self):
        return [1., 2., 3.]
    output = {name: Array() for name in ("pose", "wide_from_device_euler", "road_transform", "pose_stds", "wide_from_device_euler_stds", "road_transform_stds")}
    for calibration in (False, True):
      for drops in (0, 1, 3):
        with self.subTest(calibration=calibration, drops=drops):
          message = SimpleNamespace(cameraOdometry=SimpleNamespace())
          namespace["fill_pose_msg"](message, output, 1901, drops, 3958001402699, calibration)
          self.assertEqual(bool(message.valid), calibration and drops == 0)
          self.assertEqual(message.cameraOdometry.frameId, 1901)
          self.assertEqual(message.cameraOdometry.timestampEof, 3958001402699)


if __name__ == "__main__":
  unittest.main()
