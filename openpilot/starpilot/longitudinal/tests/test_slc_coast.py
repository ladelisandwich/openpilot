"""Live lead guards for cruise comfort braking preferences."""

import unittest
from types import SimpleNamespace

from openpilot.starpilot.longitudinal.slc_coast import braking_lead_relevant


class BrakingLeadTests(unittest.TestCase):
  def test_absent_lead_does_not_veto_cruise_tuning(self):
    self.assertFalse(braking_lead_relevant(SimpleNamespace(present=False), 20.0))

  def test_close_slow_or_braking_lead_vetoes_cruise_tuning(self):
    for speed, accel, distance, expected in ((25.0, 0.0, 100.0, False),
                                           (10.0, 0.0, 100.0, True),
                                           (25.0, -0.5, 100.0, True),
                                           (25.0, 0.0, 10.0, True)):
      with self.subTest(speed=speed, accel=accel, distance=distance):
        lead = SimpleNamespace(present=True, vLead=speed, aLeadK=accel, dRel=distance)
        self.assertEqual(braking_lead_relevant(lead, 20.0), expected)

  def test_malformed_lead_is_conservative(self):
    for lead in (SimpleNamespace(present=True),
                 SimpleNamespace(present=True, vLead=float('nan'), aLeadK=0.0, dRel=100.0)):
      with self.subTest(lead=lead):
        self.assertTrue(braking_lead_relevant(lead, 20.0))


if __name__ == '__main__':
  unittest.main()
