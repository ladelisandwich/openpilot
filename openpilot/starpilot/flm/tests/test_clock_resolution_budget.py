"""Conservative clock intervals retain freshness and reject unbounded clocks."""
import unittest

from openpilot.starpilot.flm import producer_clock_equivalence as clock


def samples(*, span=100, offset=50):
  return tuple((1_000_000_000 + i * 10_000_000,
                1_000_000_000 + i * 10_000_000 + offset,
                1_000_000_000 + i * 10_000_000 + span) for i in range(3))


class TestClockResolutionBudget(unittest.TestCase):
  def test_supported_millisecond_clocks_have_a_feasible_combined_budget(self):
    witness = clock.equivalence_from_samples(samples(), 1_000_000, 1_000_000)
    assert witness.uncertainty_ns == 2_000_050
    assert witness.offset_lower_ns <= 0 <= witness.offset_upper_ns
    raw = 2_000_000_000
    assert clock.source_interval(raw, raw + 30_000_000, raw + 40_000_000, witness) == (raw - 2_000_050, raw + 2_000_050)

  def test_fine_clock_uncertainty_is_not_inflated_to_the_budget(self):
    witness = clock.equivalence_from_samples(samples(), 1, 1)
    assert witness.uncertainty_ns == 52

  def test_individual_clock_resolution_limit_is_preserved(self):
    with self.assertRaisesRegex(ValueError, 'explicit clock resolutions'):
      clock.equivalence_from_samples(samples(), 1_000_001, 1)

  def test_large_sampling_uncertainty_still_fails_closed(self):
    with self.assertRaisesRegex(ValueError, 'not bounded-equivalent'):
      clock.equivalence_from_samples(samples(span=8_000_000, offset=4_000_000), 1_000_000, 1_000_000)

  def test_separate_clock_epochs_still_fail_closed(self):
    with self.assertRaisesRegex(ValueError, 'not bounded-equivalent'):
      clock.equivalence_from_samples(samples(offset=20_000_000), 1_000_000, 1_000_000)

  def test_entire_interval_must_be_fresh_and_not_future(self):
    witness = clock.equivalence_from_samples(samples(), 1_000_000, 1_000_000)
    raw = 2_000_000_000
    with self.assertRaisesRegex(ValueError, 'freshness/future'):
      clock.source_interval(raw, raw + 1_000_000, raw + 30_000_000, witness)
    with self.assertRaisesRegex(ValueError, 'freshness/future'):
      clock.source_interval(raw, raw + 30_000_000, raw + 99_000_000, witness)

  def test_other_clock_domains_keep_the_original_limit(self):
    assert clock.clock_uncertainty_limit(clock.CONVENTION) == 3_000_000
    for domain in ('monotonic', 'boottime', 'unknown-equivalent-at-receipt'):
      assert clock.clock_uncertainty_limit(domain) == 1_000_000
