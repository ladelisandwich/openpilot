from dataclasses import replace
from pathlib import Path
from opendbc.car import gen_empty_fingerprint
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR
from openpilot.common.params import Params
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import row_change


def test_stop_start_parked_row_confirmation_and_authority(tmp_path):
  params = Params(str(tmp_path))
  cp = CarInterface.get_params(CAR.SUBARU_LEGACY_2025, gen_empty_fingerprint(), [], False, False, False)
  permitted = [True]
  owner = FeatureSettingsOwner(params, lambda group: permitted[0], vehicle_fingerprint=lambda: cp.carFingerprint,
                               vehicle_params=lambda: cp, configuration_vehicle=lambda: True)
  row = next(row for row in owner.snapshot('vehicle', parked=True, system_long=False, lateral_context=True, metric=True).rows
             if row.key == 'SubaruStopStartOff')
  assert row.available and row.value == 'Off'
  request = row_change(row)
  assert request is not None and not owner.apply(request)
  permitted[0] = False
  assert not owner.apply(replace(request, confirmation=True))
  permitted[0] = True
  assert owner.apply(replace(request, confirmation=True))
  assert params.get_bool('SubaruStopStartOff')
  assert not owner.apply(replace(request, confirmation=True))
  assert cp.safetyConfigs[0].safetyParam == 145 and not cp.openpilotLongitudinalControl


def test_only_supported_identity_and_exact_saved_boolean_startup(tmp_path):
  from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences
  from opendbc.car.subaru.startup import selected
  params = Params(str(tmp_path))
  for raw, safe, expected in ((None, b'0', False), (b'0', b'0', False), (b'1', b'0', True),
                              (b'true', b'0', False), (b'1', b'1', False), (b'1', b'invalid', False)):
    if raw is None:
      params.remove('SubaruStopStartOff')
    else:
      Path(params.get_param_path('SubaruStopStartOff')).write_bytes(raw)
    Path(params.get_param_path('SafeMode')).write_bytes(safe)
    choice = VehicleStartupPreferences.read(params, enabled=True)
    assert choice.subaru_stop_start_off == expected
    assert not VehicleStartupPreferences.read(params, enabled=False).subaru_stop_start_off
    for car in CAR:
      cp = CarInterface.get_params(car, gen_empty_fingerprint(), [], True, False, False)
      before = cp.to_dict()
      choice.prepare(cp)
      assert selected(cp) == (expected and car in (CAR.SUBARU_LEGACY_2025, CAR.SUBARU_OUTBACK_2023))
      assert not cp.openpilotLongitudinalControl and not cp.alphaLongitudinalAvailable
      if car not in (CAR.SUBARU_LEGACY_2025, CAR.SUBARU_OUTBACK_2023) or not expected:
        assert cp.to_dict() == before


def test_raw_sources_keep_original_clock_duplicate_does_not_renew():
  from opendbc.can import CANPacker
  from opendbc.car.subaru.startup import StartupSources, checksum
  packer = CANPacker('subaru_global_2017_generated')
  source = StartupSources()
  for tick in (1, 2):
    inputs: tuple[tuple[str, int, dict[str, float]], ...] = (
      ('Dashlights', 1, {'COUNTER': tick}), ('Transmission', 0, {'COUNTER': tick, 'Gear': 4}),
      ('Engine_Stop_Start', 1, {'Counter': tick, 'STOP_START_STATE': 1}), ('ES_Distance', 1, {'COUNTER': tick}))
    frames = [packer.make_can_msg(name, bus, fields) for name, bus, fields in inputs]
    source.observe([(10_000_000_000 + tick * 10_000_000, frames)],
                   boot_ns=10_000_000_000 + tick * 10_000_000, mono_ns=1_000_000_000 + tick * 10_000_000)
  assert source.fresh(1.02)
  original = dict(source.frames)
  source.observe([(10_100_000_000, frames)], boot_ns=10_100_000_000, mono_ns=1_100_000_000)
  assert source.frames == original and not source.fresh(1.321)
  address, raw, bus = frames[0]
  changed = bytearray(raw)
  changed[7] ^= 1
  changed[0] = checksum(address, changed)
  source.observe([(10_110_000_000, [(address, bytes(changed), bus)])], boot_ns=10_110_000_000, mono_ns=1_110_000_000)
  assert address not in source.frames and not source.fresh(1.11)


def test_manual_press_and_fault_stop_startup_even_before_delay():
  from opendbc.can import CANPacker
  from opendbc.car.subaru.startup import StartupSources, StopStartRequest
  from opendbc.car import structs
  from types import SimpleNamespace
  packer = CANPacker('subaru_global_2017_generated')
  cases: tuple[tuple[str, int, dict[str, float]], ...] = (('Dashlights', 1, {'STOP_START': 1}),
                            ('Engine_Stop_Start', 1, {'STOP_START_STATE': 3}), ('ES_Distance', 1, {'Cruise_Fault': 1}))
  for name, bus, fields in cases:
    source = StartupSources()
    source.observe([(1_000_000_000, [packer.make_can_msg(name, bus, fields)])], boot_ns=1_000_000_000, mono_ns=1_000_000_000)
    assert source.blocked
    owner = StopStartRequest()
    state = SimpleNamespace(startup_sources=source, out=structs.CarState(standstill=True))
    assert owner.update(structs.CarControl(), state, 1.0) is None and owner.done


def test_configured_dwell_restarts_on_gap_and_never_retries_started_pulse():
  from opendbc.car.subaru.startup import StopStartRequest
  owner = StopStartRequest()
  owner.observe_configured(True, 1.0)
  owner.observe_configured(True, 1.1)
  assert owner.configured_since == 1.0
  owner.observe_configured(True, 1.3)
  assert owner.configured_since == 1.3
  owner.pulse_start = 1.4
  owner.observe_configured(False, 1.45)
  owner.observe_configured(True, 1.5)
  assert owner.done
