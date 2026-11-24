"""AOL sample coverage and interval boundaries."""

import unittest

from openpilot.starpilot.galaxy.lateral_stats import aol_statistics


class LateralStatsTest(unittest.TestCase):
  def test_unknown_evidence_is_not_zero(self):
    for samples in ([], [(1_000_000_000, True, True)],
                    [(1_000_000_000, True, True), (1_000_000_000, True, True)],
                    [(1_000_000_000, True, True), (2_100_000_000, True, True)],
                    [(1_000_000_000, True, True), (2_000_000_000, False, False)],
                    [(0, True, True), (1_000_000_000, True, True)],
                    [(1_000_000_000, True, True), (900_000_000, True, False)]):
      with self.subTest(samples=samples):
        self.assertEqual(aol_statistics(samples, 1_000_000_000, 2_000_000_000),
                         {'aolSeconds': None, 'aolPercent': None})

  def test_clip_control_intervals_to_existing_drive_span(self):
    samples = [(900_000_000, True, True), (1_500_000_000, True, True), (2_100_000_000, True, False)]
    self.assertEqual(aol_statistics(samples, 1_000_000_000, 2_000_000_000),
                     {'aolSeconds': 1.0, 'aolPercent': 100.0})

  def test_missing_boundary_coverage_is_unknown(self):
    samples = [(2_500_000_000, True, True), (3_000_000_000, True, True)]
    self.assertIsNone(aol_statistics(samples, 1_000_000_000, 3_000_000_000)['aolSeconds'])
    self.assertIsNone(aol_statistics(samples, 2_500_000_000, 4_500_000_000)['aolSeconds'])

  def test_observed_inactive_control_is_zero(self):
    samples = [(1_000_000_000, True, False), (2_000_000_000, True, False)]
    self.assertEqual(aol_statistics(samples, 1_000_000_000, 2_000_000_000), {'aolSeconds': 0.0, 'aolPercent': 0.0})
