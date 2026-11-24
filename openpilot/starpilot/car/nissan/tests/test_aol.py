"""Nissan actual acquisition/factory/Card/parser/Controls/native optional path."""

import json
import os
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from opendbc.car import Bus, car_helpers, gen_empty_fingerprint, structs
from opendbc.car.nissan.values import CAR, DBC
from opendbc.car.nissan.nissancan import nissan_checksum
from opendbc.car.nissan.fingerprints import FW_VERSIONS
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.can import CANPacker
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.vehicle_selection import encode
from openpilot.starpilot.aol.vehicle import policy_for
from opendbc.safety.tests.libsafety import libsafety_py


@contextmanager
def factory(identity, *, enabled=True, manual=False, lkas=0, main=0, alpha=False, release=False):
  with (
    OpenpilotPrefix(),
    patch.dict(os.environ, {'SIMULATION': '1', 'AOL_REPLAY_RUNTIME': '0', 'FINGERPRINT': '', 'SKIP_FW_QUERY': '', 'DISABLE_FW_CACHE': '1'}),
  ):
    os.environ.pop('REPLAY', None)
    params = Params()
    for key, value in [
      ('AlwaysOnLateral', enabled),
      ('OpenpilotEnabledToggle', True),
      ('IsReleaseBranch', release),
      ('AlphaLongitudinalEnabled', alpha),
      ('IsOffroad', False),
    ]:
      params.put_bool(key, value, block=True)
    params.put('LKASButtonControl', lkas, block=True)
    params.put('MainCruiseButtonControl', main, block=True)
    params.put('VehicleSelection', json.loads(encode(str(identity) if manual else None)), block=True)
    fw = [
      structs.CarParams.CarFw(ecu=ecu, address=addr, subAddress=sub or 0, fwVersion=versions[0], brand='nissan')
      for (ecu, addr, sub), versions in FW_VERSIONS.get(identity, {}).items()
    ]
    if not manual:
      _, matches = match_fw_to_car(fw, 'JN1AZ0CP9CT000001', log=False)
      assert matches == {identity}, matches
    with (
      patch.object(card_module.messaging, 'recv_one_retry', return_value=SimpleNamespace(can=[1])),
      patch.object(card_module, 'can_comm_callbacks', return_value=(lambda *a, **kw: [], lambda *a, **kw: None)),
      patch.object(card_module, 'obd_callback', return_value=lambda _: None),
      patch.object(car_helpers, 'get_vin', return_value=(-1, -1, 'JN1AZ0CP9CT000001')),
      patch.object(car_helpers, 'get_present_ecus', return_value=set()),
      patch.object(car_helpers, 'get_fw_versions_ordered', return_value=fw),
      patch.object(car_helpers, 'can_fingerprint', return_value=(None, gen_empty_fingerprint())),
    ):
      card = card_module.Car()
    try:
      assert card.CP.carFingerprint == identity
      assert card.CP.fingerprintSource == ('fixed' if manual else 'fw')
      yield card, params
    finally:
      card.vehicle_startup.close()


class Frames:
  def __init__(self, ci):
    self.ci = ci
    ci.update([])
    self.packers = {bus: CANPacker(p.dbc_name) for bus, p in ci.can_parsers.items()}

  def make(self, tick, *, main=True, cruise=True, gear=4, status=5, brake=False, gas=0, lkas=0):
    result = []
    for logical, p in self.ci.can_parsers.items():
      for address in sorted(p.addresses):
        name = p.dbc.addr_to_msg[address].name
        values: dict[str, float] = {}
        if name == 'STEER_TORQUE_SENSOR':
          values = {'STEER_ANGLE': 0.0, 'STEER_TORQUE_DRIVER': 0.0, 'LKAS_STATUS': status}
        elif name == 'WHEEL_SPEEDS_REAR':
          values = {'WHEEL_SPEED_RR': 36.0, 'WHEEL_SPEED_RL': 36.0}
        elif name == 'WHEEL_SPEEDS_FRONT':
          values = {'WHEEL_SPEED_FR': 36.0, 'WHEEL_SPEED_FL': 36.0}
        elif name == 'GEARBOX':
          values = {'GEAR_SHIFTER': gear}
        elif name == 'CRUISE_STATE':
          values = {'CRUISE_ENABLED': cruise}
        elif name == 'PRO_PILOT':
          values = {'CRUISE_ON': main}
        elif name == 'CRUISE_THROTTLE':
          values = {'USER_BRAKE_PRESSED': brake, 'GAS_PEDAL': gas}
          if self.ci.CP.carFingerprint in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
            values['CRUISE_AVAILABLE'] = main
        elif name == 'GAS_PEDAL':
          values = {'GAS_PEDAL': gas}
        elif name == 'DOORS_LIGHTS' and self.ci.CP.carFingerprint not in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
          values = {'USER_BRAKE_PRESSED': brake}
        elif name in ('HUD', 'SEATBELT'):
          values = {'SEATBELT_DRIVER_LATCHED': 1}
        elif name == 'LKAS_SETTINGS':
          values = {'LKAS_ENABLED': lkas}
        result.append(self.packers[logical].make_can_msg(name, p.bus, values))
    return result


@pytest.mark.parametrize('identity', list(CAR))
@pytest.mark.parametrize('alpha,release', [(False, False), (True, False), (False, True), (True, True)])
def test_actual_factory_enrolls_nissan_aol(identity, alpha, release):
  with factory(identity, manual=identity == CAR.NISSAN_ROGUE, alpha=alpha, release=release) as (card, params):
    assert card.aol_qualified and card.aol_card_intent is not None
    assert policy_for(card.CP).normal_runtime_supported
    assert card.CP.alternativeExperience == 32
    assert not card.CP.openpilotLongitudinalControl and card.CP.pcmCruise
    assert not card.CP.alphaLongitudinalAvailable
    with structs.CarParams.from_bytes(params.get('CarParams')) as published:
      assert published.to_dict() == card.CP.to_dict()


@pytest.mark.parametrize('identity', list(CAR))
def test_explicit_off_retains_upstream(identity):
  with factory(identity, enabled=False, manual=identity == CAR.NISSAN_ROGUE) as (card, params):
    assert card.aol_card_intent is None and not card.aol_qualified
    assert card.CP.alternativeExperience == 0
    assert card.CP.safetyConfigs[0].safetyParam == (1 if identity == CAR.NISSAN_ALTIMA else 0)
    assert params.get('AlwaysOnLateral') is False
    assert Path(params.get_param_path('AlwaysOnLateral')).read_bytes() == b'0'


@pytest.mark.parametrize('identity', list(CAR))
def test_current_native_and_real_controller_boundaries(identity):
  with factory(identity, manual=identity == CAR.NISSAN_ROGUE) as (card, params):
    assert card.aol_qualified
    cp = card.CP
    ci = card.CI
    frames = Frames(ci)
    safety = libsafety_py.libsafety
    safety.set_alternative_experience(32)
    assert safety.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
    safety.init_tests()
    safety.set_alternative_experience(32)
    states = []
    # Real Card.state_update parses original CAN and receives actual native bits.
    for tick, (main, cruise, brake, gear, status, expected) in enumerate(
      [(True, True, False, 4, 5, True)] * 20
      + [
        (True, False, True, 4, 5, True),
        (True, False, False, 2, 5, False),
        (True, False, False, 3, 5, False),
        (True, False, False, 4, 9, False),
        (True, False, False, 4, 5, True),
        (False, False, False, 4, 5, False),
      ],
      1,
    ):
      now = 10_000_000_000 + tick * 10_000_000
      safety.set_timer(tick * 10_000)
      packets = frames.make(tick, main=main, cruise=cruise, brake=brake, gear=gear, status=status)
      for addr, data, bus in packets:
        safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))
      state = ci.update([(now, packets)])
      safety.set_aol_test_heartbeat(True)
      safety.aol_set_host_request(3)
      assert bool(safety.aol_get_permission_mask() & 1) == expected
      if brake:
        assert not safety.get_controls_allowed() and safety.aol_get_permission_mask() & 2 == 0
      command = structs.CarControl(enabled=False, latActive=expected)
      _, sends = ci.apply(command.as_reader(), now)
      steering = next(frame for frame in sends if frame[0] == 0x169)
      assert steering[1][7] == nissan_checksum(steering[1][:7])
      assert safety.safety_tx_hook(libsafety_py.make_CANPacket(steering[0], steering[2], steering[1]))
      assert not ({0x1C3, 0x2B0, 0x707} & {f[0] for f in sends})
      if tick >= 20:
        states.append((state.to_dict(), int(safety.aol_get_permission_mask())))
    assert states
    safety.set_timer(tick * 10_000 + 300001)
    safety.aol_set_host_request(1)
    assert safety.aol_get_permission_mask() == 0


@pytest.mark.parametrize('identity', list(CAR))
def test_intent_parser_cold_main_full_engagement_and_fault_rearm(identity):
  with factory(identity, manual=identity == CAR.NISSAN_ROGUE) as (card, _):
    assert card.aol_qualified
    frames = Frames(card.CI)
    owner = card.aol_card_intent
    for tick in range(1, 21):
      cs = card.CI.update([(tick * 10_000_000, frames.make(tick))])
    assert cs.canValid and not cs.canTimeout
    owner.update(cs.as_reader(), now_ns=210_000_000, fault_active=False)
    assert owner.allowed_latch  # Actual received cold MAIN; no artificial cycle.
    brake = card.CI.update([(220_000_000, frames.make(22, cruise=False, brake=True))])
    owner.update(brake.as_reader(), now_ns=220_000_000, fault_active=False)
    assert owner.allowed_latch
    reverse = card.CI.update([(230_000_000, frames.make(23, cruise=False, gear=2))])
    owner.update(reverse.as_reader(), now_ns=230_000_000, fault_active=False)
    assert owner.allowed_latch and not owner.output(reverse)[0]
    temp = card.CI.update([(240_000_000, frames.make(24, cruise=False, status=9))])
    owner.update(temp.as_reader(), now_ns=240_000_000, fault_active=False)
    assert owner.allowed_latch
    owner.update(cs.as_reader(), now_ns=250_000_000, fault_active=True)
    assert not owner.allowed_latch
    owner.update(cs.as_reader(), now_ns=260_000_000, fault_active=False)
    assert not owner.allowed_latch
    owner.observe_full_engagement(
      cs.as_reader(), enabled=False, active=False, source_ns=270_000_000, observation_valid=True, now_ns=270_000_000, fault_active=False
    )
    owner.observe_full_engagement(
      cs.as_reader(), enabled=True, active=True, source_ns=280_000_000, observation_valid=True, now_ns=280_000_000, fault_active=False
    )
    assert owner.allowed_latch


@pytest.mark.parametrize('identity', list(CAR))
@pytest.mark.parametrize('button', ['main', 'lkas'])
def test_received_remap_edges_do_not_replay_boot_or_held_inputs(identity, button):
  with factory(identity, manual=identity == CAR.NISSAN_ROGUE, **{button: 9}) as (card, _):
    frames = Frames(card.CI)
    owner = card.aol_card_intent

    def update(tick, **kwargs):
      state = card.CI.update([(tick * 10_000_000, frames.make(tick, cruise=False, **kwargs))])
      owner.update(state.as_reader(), now_ns=tick * 10_000_000, fault_active=False)
      return state

    for tick in range(1, 21):
      update(tick)
    assert not owner.allowed_latch
    if button == 'main':
      update(21, main=False)
      update(22, main=True)
      assert owner.allowed_latch
      update(23, main=True)
      assert owner.allowed_latch
      update(24, main=False)
      assert not owner.allowed_latch
    else:
      state = update(21, lkas=1)
      assert state.invalidLkasSetting and owner.allowed_latch
      update(22, lkas=1)
      assert owner.allowed_latch
      update(23, lkas=0)
      update(24, lkas=1)
      assert not owner.allowed_latch


@pytest.mark.parametrize('identity', list(CAR))
def test_exact_owner_denies_other_configs_and_native_guards(identity):
  with factory(identity, manual=identity == CAR.NISSAN_ROGUE) as (card, _):
    from openpilot.starpilot.car.nissan.aol import qualified

    cp = card.CP
    assert qualified(cp, marked_only=True)
    for field in ('passive', 'dashcamOnly', 'notCar', 'openpilotLongitudinalControl'):
      changed = structs.CarParams(**cp.to_dict())
      setattr(changed, field, True)
      assert not qualified(changed)
    changed = structs.CarParams(**cp.to_dict())
    changed.flags = 1
    assert not qualified(changed)
    changed = structs.CarParams(**cp.to_dict())
    changed.safetyConfigs[0].safetyParam = 2
    assert not qualified(changed)
    safety = libsafety_py.libsafety
    safety.set_alternative_experience(32)
    safety.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam)
    safety.init_tests()
    safety.set_alternative_experience(32)
    frames = Frames(card.CI)
    for tick in range(1, 21):
      safety.set_timer(tick * 10000)
      batch = frames.make(tick)
      for addr, data, bus in batch:
        safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))
      card.CI.update([(tick * 10_000_000, batch)])
    safety.set_aol_test_heartbeat(True)
    safety.aol_set_host_request(3)
    assert safety.aol_get_permission_mask() == 3
    packer = CANPacker(DBC[identity][Bus.pt])
    for angle in (601, -601, 100, -100):
      a, d, b = packer.make_can_msg('LKAS', 0, {'DESIRED_ANGLE': angle, 'LKA_ACTIVE': 1})
      assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d))
    a, d, b = packer.make_can_msg('LKAS', 0, {'DESIRED_ANGLE': 0, 'LKA_ACTIVE': 1})
    assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(a, 1, d))
    safety.aol_set_host_request(0)
    assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(a, b, d))
    safety.aol_set_host_request(1)
    safety.set_aol_test_heartbeat(False)
    assert safety.aol_get_permission_mask() == 0
    safety.set_aol_test_heartbeat(True)
    safety.set_relay_malfunction(True)
    assert safety.aol_get_permission_mask() == 0
    safety.set_relay_malfunction(False)
    # Wrong MAIN bus cannot change true to false; malformed correct-source
    # gear immediately invalidates it instead of reusing a cached Drive.
    leaf = identity in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC)
    main_bus = 0 if leaf else 2 if identity == CAR.NISSAN_ALTIMA else 1
    a, d, _ = packer.make_can_msg('CRUISE_THROTTLE' if leaf else 'PRO_PILOT', main_bus, {'CRUISE_AVAILABLE': 0} if leaf else {'CRUISE_ON': 0})
    safety.safety_rx_hook(libsafety_py.make_CANPacket(a, (main_bus + 1) % 3, d))
    assert safety.aol_get_permission_mask() & 1
    pt = 1 if identity == CAR.NISSAN_ALTIMA else 0
    safety.safety_rx_hook(libsafety_py.make_CANPacket(0x421 if leaf else 0x41F, pt, b'\x20'))
    assert safety.aol_get_permission_mask() == 0


@pytest.mark.parametrize('identity', list(CAR))
def test_temporary_eps_retention_rejects_other_vehicle_conditions(identity):
  from openpilot.starpilot.car.nissan.aol import temporary_eps_restriction
  with factory(identity, manual=identity == CAR.NISSAN_ROGUE) as (card, _):
    frames = Frames(card.CI)
    for tick in range(1, 21):
      state = card.CI.update([(tick * 10_000_000, frames.make(tick, cruise=False, status=9))])
    assert temporary_eps_restriction(card.CP, state)
    for field, value in [('canValid', False), ('canTimeout', True), ('steerFaultTemporary', False),
                         ('steerFaultPermanent', True), ('accFaulted', True), ('brakePressed', True),
                         ('gasPressed', True), ('regenBraking', True), ('doorOpen', True),
                         ('seatbeltUnlatched', True), ('invalidLkasSetting', True),
                         ('gearShifter', structs.CarState.GearShifter.reverse)]:
      bad = structs.CarState(**state.to_dict())
      setattr(bad, field, value)
      assert not temporary_eps_restriction(card.CP, bad), field
    for field in ('available', 'enabled'):
      bad = structs.CarState(**state.to_dict())
      setattr(bad.cruiseState, field, field == 'enabled')
      assert not temporary_eps_restriction(card.CP, bad), field
