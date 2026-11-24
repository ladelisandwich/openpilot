"""Alternate radar stock Honda with current factory/parser/Card/SD/Controls/native."""
from opendbc.car.honda.values import CAR
from openpilot.starpilot.aol.tests.test_honda_recovery_lifecycle import TestHondaCurrentCaller as CurrentCaller

class TestHondaAlternateRadar(CurrentCaller):
  def test_alt_0_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_RDX_3G_MMR, word=1, speed_mps=25.)

  def test_alt_1_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.HONDA_ODYSSEY_5G_MMR, word=1, speed_mps=25.)

  def test_alt_2_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_TLX_2G, word=0, speed_mps=25.)

  def test_alt_3_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_MDX_4G, word=512, speed_mps=25.)


  def test_mdx_alternate_brake_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_MDX_4G, word=513, alternate_brake=True)

  def test_alt_alpha_0_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_RDX_3G_MMR, word=1059, alpha=True, speed_mps=25.)

  def test_alt_alpha_1_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.HONDA_ODYSSEY_5G_MMR, word=1059, alpha=True, speed_mps=25.)

  def test_alt_alpha_2_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_TLX_2G, word=1058, alpha=True, speed_mps=25.)

  def test_alt_alpha_3_eps_reverse_recovery(self):
    self.run_current_caller(candidate=CAR.ACURA_MDX_4G, word=1570, alpha=True, speed_mps=25.)
