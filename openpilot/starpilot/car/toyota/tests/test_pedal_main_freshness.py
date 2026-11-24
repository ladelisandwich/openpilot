"""MAIN expiry must withdraw actual positive steering before native denial."""
import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests.test_pedal_caller_native import (
  source_matched_native as source_matched_native, daemon_gc_scope as daemon_gc_scope, run_case,
)

pytestmark = pytest.mark.usefixtures('source_matched_native', 'daemon_gc_scope')


@pytest.mark.parametrize('candidate', (CAR.TOYOTA_COROLLA_TSS2, CAR.TOYOTA_RAV4_TSS2))
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('scenario', ('main_dropout', 'main_bad_crc', 'main_bad_dlc'))
def test_pedal_main_source_positive_native(candidate, stock, scenario):
  run_case(stock, scenario, candidate=candidate)
