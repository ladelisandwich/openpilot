"""Receive-only readiness checks for Ioniq 6 startup selection."""
import unittest
from opendbc.car.hyundai.ioniq6_handoff import _qualified_ioniq6_ready


class TestIoniq6ReadySelection(unittest.TestCase):
  @staticmethod
  def samples(ready=False):
    return [(1_000_000_000+i*40_000_000, 10.+i*.04, i, ready, True, 1_001_000_000+i*40_000_000) for i in range(4)]

  def test_stable_current_ready_and_not_ready_remain_distinct(self):
    for ready in (False, True):
      self.assertIs(_qualified_ioniq6_ready(self.samples(ready), 10.13), ready)

  def test_invalid_stale_missing_future_and_counter_samples_are_unknown(self):
    mutations = ((4, False), (0, 0), (0, 9_000_000_000), (5, 0), (5, 2_000_000_000), (2, 99))
    for column, value in mutations:
      with self.subTest(column=column, value=value):
        samples = self.samples()
        row = list(samples[-1])
        row[column] = value
        samples[-1] = tuple(row)
        self.assertIsNone(_qualified_ioniq6_ready(samples, 10.13))
    self.assertIsNone(_qualified_ioniq6_ready([], 10.13))
    self.assertIsNone(_qualified_ioniq6_ready(self.samples()[:3], 10.13))
    self.assertIsNone(_qualified_ioniq6_ready(self.samples(), 11.))

  def test_transition_or_queued_burst_cannot_be_interpreted_as_false(self):
    samples = self.samples()
    row = list(samples[-1])
    row[3] = True
    samples[-1] = tuple(row)
    self.assertIsNone(_qualified_ioniq6_ready(samples, 10.13))
    samples = [(stamp, 10.+i*.001, counter, ready, valid, boot) for i,(stamp,_,counter,ready,valid,boot) in enumerate(self.samples())]
    self.assertIsNone(_qualified_ioniq6_ready(samples, 10.004))
