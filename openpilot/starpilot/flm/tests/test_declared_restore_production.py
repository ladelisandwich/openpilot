"""Registered producer version dispatch; this does not grant restore authority."""
import pytest
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HKG
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.starpilot.flm.learner_factory import from_evidence
from openpilot.starpilot.flm.feedback_replay import PROTOCOL as FEEDBACK_PROTOCOL
from openpilot.starpilot.flm.source_plant_fit import PROTOCOL as LEGACY_SOURCE_PROTOCOL
from openpilot.starpilot.flm.clipping_quality import ZERO_REFERENCE_QUALITY, QUALIFIED_EVENT_CLIPPING_QUALITY
from openpilot.starpilot.flm.runtime_adapter_registry import baseline_values
from openpilot.starpilot.flm.steerlearnerd import production_adapter_for_evidence
from openpilot.starpilot.flm.tests.test_production_event_contract import capture
from openpilot.starpilot.flm.tests.test_registered_controls_global import controls_for


@pytest.mark.parametrize('platform,mode', ((TOYOTA.TOYOTA_RAV4_TSS2, 'standard'),
  (HKG.HYUNDAI_IONIQ_6, 'starpilot'), (HONDA.HONDA_CIVIC, None)))
def test_actual_registered_producer_explicit_restore_version_and_legacy_default(platform, mode, protocol=None, restore_version=8):
  cp = interfaces[platform].get_non_essential_params(platform)
  with controls_for(cp, mode, True) as controls:
    evidence, source, _ = capture(controls)
    before = controls.CP.as_builder().to_bytes(), baseline_values(controls.LaC)
    live = production_adapter_for_evidence(controls.CP, evidence, ('event', 'center'), source=source,
      source_plant_protocol=protocol)
    old = from_evidence(controls.CP, evidence, ('event', 'center'), source=source)
    assert live is not None and live.replay_version == 3 and live.restore_version == restore_version
    assert live.clipping_quality == ZERO_REFERENCE_QUALITY
    assert live.source_plant_protocol == (FEEDBACK_PROTOCOL if protocol is None else protocol)
    for version in (4, 5):
      legacy = from_evidence(controls.CP, evidence, ('event', 'center'), source=source,
        event_replay_version=3, event_clipping_quality=QUALIFIED_EVENT_CLIPPING_QUALITY, event_restore_version=version)
      assert legacy is not None and legacy.restore_version == version and legacy.clipping_quality == QUALIFIED_EVENT_CLIPPING_QUALITY
    assert old is not None and old.replay_version == 2 and old.restore_version == 2
    assert from_evidence(controls.CP, evidence, ('event', 'center'), source=source, event_restore_version=4) is None
    live.select_objective(('event', 'deadband.high'))
    assert live.restore_version == restore_version
    assert (controls.CP.as_builder().to_bytes(), baseline_values(controls.LaC)) == before


@pytest.mark.parametrize('platform,mode', ((TOYOTA.TOYOTA_RAV4_TSS2, 'standard'),
  (HKG.HYUNDAI_IONIQ_6, 'starpilot'), (HONDA.HONDA_CIVIC, None)))
def test_actual_registered_producer_explicit_legacy_source_restore6(platform, mode):
  # Exercise the same real application, refusal and unchanged-controller checks
  # for the independently selected legacy source protocol as for default8.
  test_actual_registered_producer_explicit_restore_version_and_legacy_default(
    platform, mode, protocol=LEGACY_SOURCE_PROTOCOL, restore_version=6)
