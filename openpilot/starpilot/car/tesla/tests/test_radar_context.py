from dataclasses import replace
import hashlib
import math

import pytest

from opendbc.car import structs
from opendbc.car.tesla.preap.radar_profile import RadarStartupContext
from opendbc.car.tesla.tests.test_preap_radar import radar_params
from opendbc.car.tesla.radar_interface import RadarInterface
from openpilot.starpilot.car.tesla.radar_context import SCHEMA, encode_context, decode_context, install_startup_context
from openpilot.starpilot.car.tesla.radar_settings import RadarSettings


def context(cp, offset=0.25):
  raw = cp.to_bytes()
  return raw, RadarStartupContext(1, int(cp.safetyConfigs[0].safetyParam), hashlib.sha256(raw).digest(), offset)


@pytest.mark.parametrize('mount', (False, True))
def test_final_serialized_cp_context_and_reader_lifetime(mount):
  cp = radar_params(mount=mount)
  raw, frozen = context(cp)
  wire = encode_context(cp, raw, frozen)
  with structs.CarParams.from_bytes(raw) as published:
    decoded = decode_context(published, raw, wire)
    ri = RadarInterface(published)
    ri.configure_preap_radar(decoded, raw)
  assert decoded == frozen and ri.preap.radar_offset == 0.25
  assert not ri.preap.radar_off_can


@pytest.mark.parametrize('field,value', (('version', 2), ('kind', 3), ('safetyModel', 10), ('safetyParam', 6),
                                        ('carParamsSha256', bytes(32)), ('lateralOffset', math.nan),
                                        ('lateralOffset', math.inf)))
def test_unknown_or_mismatched_wire_cannot_configure_decoder(field, value):
  cp = radar_params()
  raw, frozen = context(cp)
  message = SCHEMA.new_message(version=1, kind=4, safetyModel=39, safetyParam=2,
                               carParamsSha256=frozen.car_params_sha256, lateralOffset=frozen.offset)
  setattr(message, field, value)
  assert decode_context(cp, raw, message.to_bytes()) is None


def test_missing_truncated_multisegment_or_mutated_cp_context_denies():
  cp = radar_params()
  raw, frozen = context(cp)
  wire = encode_context(cp, raw, frozen)
  for damaged in (None, b'', wire[:-8], wire + bytes(8), bytes([1]) + wire[1:], memoryview(wire)):
    assert decode_context(cp, raw, damaged) is None
  cp.carVin = '1' * 17
  assert decode_context(cp, raw, wire) is None
  with pytest.raises(ValueError):
    RadarInterface(cp).configure_preap_radar(frozen, raw)


def test_missing_context_reports_errors_and_never_decodes_tracks():
  ri = RadarInterface(radar_params())
  results = [ri.update([]) for _ in range(5)]
  assert results[-1] is not None and not results[-1].points
  assert results[-1].errors.canError and results[-1].errors.radarUnavailableTemporary


def test_immutable_startup_settings_install_and_mismatched_mount_denial():
  cp = radar_params()
  raw, _ = context(cp)
  ri = RadarInterface(cp)
  settings = RadarSettings(True, False, 0.25, True)
  frozen = install_startup_context(ri, cp, raw, settings)
  assert frozen.offset == ri.preap.radar_offset == 0.25
  with pytest.raises(ValueError):
    install_startup_context(ri, cp, raw, replace(settings, behind_nosecone=True))
  assert ri.preap.radar_offset == 0.25


def test_radar_native_receipt_requires_same_cp_namespace_and_word():
  from openpilot.starpilot.car.tesla.aol import native_accepts_cp
  cp = radar_params()
  cp.alternativeExperience = 32
  assert native_accepts_cp(cp, 39, 2)
  for model, word in ((39, 0), (39, 6), (10, 512), (10, 1536), (35, 2)):
    assert not native_accepts_cp(cp, model, word)
