"""Actual radar disable/stock restore, Card, SelfdriveD, Controls and native path.

Acquired FW, physical CAN, perception, USB and input time are synthetic;
permission, parser health, CP/CS and readiness come from production paths.
"""
import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4_angle_filter_callers as helpers
source_matched_native=helpers.source_matched_native
daemon_gc_scope=helpers.daemon_gc_scope

@pytest.mark.parametrize('candidate',(CAR.TOYOTA_CHR_TSS2,CAR.TOYOTA_RAV4_TSS2_2022,CAR.TOYOTA_RAV4_TSS2_2023))
@pytest.mark.parametrize('stock',(False,True))
def test_actual_standalone_pedal_radar_disable_or_stock_restore(candidate,stock,tmp_path):
  helpers.run_case(candidate,True,stock,0,tmp_path,has_filter=False,has_pedal=True)
