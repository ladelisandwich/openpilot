"""Actual caller/native pedal qualification for added camera-longitudinal models."""
import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_pedal_caller_native as helpers

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope

NEW_MODELS = (
  CAR.TOYOTA_ALPHARD_TSS2,
  CAR.TOYOTA_AVALON_TSS2,
  CAR.TOYOTA_CAMRY_TSS2,
  CAR.TOYOTA_HIGHLANDER_TSS2,
  CAR.TOYOTA_PRIUS_TSS2,
  CAR.TOYOTA_MIRAI,
  CAR.LEXUS_ES_TSS2,
  CAR.LEXUS_NX_TSS2,
  CAR.LEXUS_LC_TSS2,
  CAR.LEXUS_RX_TSS2,
  CAR.LEXUS_IS_TSS2,
  CAR.LEXUS_RC_TSS2,
)

@pytest.mark.parametrize('candidate', NEW_MODELS)
@pytest.mark.parametrize('stock', [False, True])
@pytest.mark.parametrize('scenario', ['physical', 'main_dropout'])
def test_added_camera_pedal_actual_callers(candidate, stock, scenario):
  helpers.run_case(stock, scenario, candidate=candidate)
