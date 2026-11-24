"""Lexus DSU MAIN automatic factory through actual daemon/native RX and TX."""
import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import (
  source_matched_native as source_matched_native, daemon_gc_scope as daemon_gc_scope, run_case,
)

pytestmark = pytest.mark.usefixtures('source_matched_native', 'daemon_gc_scope')


@pytest.mark.parametrize('scenario', ('nominal', 'physical', 'faults', 'bad_dlc', 'off'))
def test_stock_lexus_actual_callers(scenario):
  run_case(True, scenario, candidate=CAR.LEXUS_IS)


def test_stock_lexus_aol_off():
  run_case(True, aol=False, candidate=CAR.LEXUS_IS)
