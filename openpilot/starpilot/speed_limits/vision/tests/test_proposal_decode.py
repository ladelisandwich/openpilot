import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from openpilot.starpilot.speed_limits.vision import model


class TestProposalDecode(unittest.TestCase):
  @staticmethod
  def decode(predictions, threshold=0.06):
    core = model.VisionModelCore.__new__(model.VisionModelCore)
    core.detector_input_size = 256
    core.last_detector_forward_count = 0
    core.last_detector_forward_duration_s = 0.0
    core.net = SimpleNamespace(setInput=lambda _: None, forward=lambda: predictions)
    backend = SimpleNamespace(dnn=SimpleNamespace(blobFromImage=lambda *a, **kw: None))
    with patch.object(model, 'cv2', backend), patch.object(core, '_letterbox', return_value=(None, 1.0, 0.0, 0.0)):
      result = core._collect_detector_classifier_proposals_from_region(
        np.zeros((1080, 1920, 3), dtype=np.uint8), 10, 20, 1920, 1080, threshold)
    assert core.last_detector_forward_count == 1
    return result

  def test_scores_keep_row_order_supported_classes_and_first_tie(self):
    rows = np.zeros((16, 9), dtype=np.float32)
    rows[:, :4] = [1000, 300, 40, 70]
    rows[:, 4] = 0.01
    rows[0, 4:6] = 0.9
    rows[2, 6] = 0.8
    rows[4, 7] = 0.99  # An unsupported class cannot become a sign proposal.
    rows[6, 5] = 0.7
    expected = [(float(rows[i, 4 + c]), c, (990, 285, 1030, 355)) for i, c in ((0, 0), (2, 2), (6, 1))]
    self.assertEqual(self.decode(rows), expected)
    self.assertEqual(self.decode(rows.T), expected)

  def test_threshold_uses_python_float_precision(self):
    for dtype in (np.float16, np.float32, np.float64):
      with self.subTest(dtype=dtype):
        rows = np.zeros((8, 5), dtype=dtype)
        rows[:, :4] = [1000, 300, 40, 70]
        rows[:, 4] = dtype(0.06)
        score = float(rows[0, 4])
        self.assertEqual(len(self.decode(rows, score)), 8)
        self.assertEqual(self.decode(rows, np.nextafter(score, np.inf)), [])
        self.assertEqual(len(self.decode(rows, np.nextafter(score, -np.inf))), 8)

  def test_nonfinite_scores_preserve_existing_decoder_behavior(self):
    rows = np.zeros((8, 5), dtype=np.float32)
    rows[:, :4] = [1000, 300, 40, 70]
    rows[:, 4] = -np.inf
    rows[1, 4] = np.nan
    rows[5, 4] = np.inf
    result = self.decode(rows)
    self.assertEqual(len(result), 2)
    self.assertTrue(np.isnan(result[0][0]))
    self.assertEqual(result[1][0], np.inf)
    self.assertTrue(all(row[1:] == (0, (990, 285, 1030, 355)) for row in result))

  def test_invalid_rank_has_no_proposals(self):
    self.assertEqual(self.decode(np.zeros((2, 2, 2))), [])
    self.assertEqual(self.decode(np.zeros((1, 5))), [])

  def test_geometry_filters_remain_after_score_admission(self):
    rows = np.zeros((8, 5), dtype=np.float32)
    rows[:, :4] = [1000, 300, 40, 70]
    rows[:, 4] = 0.9
    rows[1, 2] = 5
    rows[2, 3] = 5
    rows[3, 0] = 100
    rows[4, 1] = 1000
    rows[5, 2:4] = 1500
    result = self.decode(rows)
    self.assertEqual(len(result), 3)
    self.assertTrue(all(row[2] == (990, 285, 1030, 355) for row in result))
