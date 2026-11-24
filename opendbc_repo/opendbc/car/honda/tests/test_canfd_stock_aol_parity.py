"""Factory, Card/Controls, real parser and native Honda stock CANFD contracts."""
from unittest.mock import patch

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, HondaFlags
from openpilot.common.prefix import OpenpilotPrefix

IDENTITIES = (CAR.HONDA_CRV_6G, CAR.HONDA_PILOT_4G, CAR.HONDA_PASSPORT_4G,
              CAR.ACURA_MDX_4G_MMR, CAR.ACURA_TLX_2G_MMR)


def factory(identity, *, alt=False, cvt=False, hybrid=False, offset=0):
  fp = gen_empty_fingerprint()
  fp[offset][0x191 if cvt else 0x1A3] = 8
  if alt:
    fp[offset][0x1BE] = 3
  if hybrid:
    fp[offset][0x184] = 8
  return CarInterface.get_params(identity, fp, [], True, False, False)


@pytest.mark.parametrize('identity', IDENTITIES)
@pytest.mark.parametrize('alt', (False, True))
@pytest.mark.parametrize('cvt,hybrid', ((False, False), (True, True)))
def test_actual_factory_exact_stock_aol_admission_and_negative_profiles(identity, alt, cvt, hybrid):
  from opendbc.car.honda.stock_aol import qualified
  from openpilot.starpilot.car.honda.aol import policy_for, native_accepts_cp
  cp = factory(identity, alt=alt, cvt=cvt, hybrid=hybrid)
  assert cp.pcmCruise and not cp.openpilotLongitudinalControl
  assert not cp.alphaLongitudinalAvailable
  assert bool(cp.flags & HondaFlags.HYBRID) == hybrid
  assert cp.safetyConfigs[0].safetyParam == 16 | int(alt)
  assert qualified(cp)
  policy = policy_for(cp)
  assert policy.explicit_latch and policy.normal_runtime_supported
  assert policy.alternative_experience_addition == 32 and policy.safety_param_addition == 0
  cp.alternativeExperience = 32
  assert qualified(cp, marked_only=True)
  assert native_accepts_cp(cp, 20, 16 | int(alt))
  for field, value in (('passive', True), ('dashcamOnly', True), ('notCar', True),
                       ('transmissionType', 'manual'),
                       ('flags', int(cp.flags) | (1 << 30))):
    bad = cp.as_reader().as_builder()
    setattr(bad, field, value)
    assert not qualified(bad)
  for word in (18, 19, 80, 81, 82, 0x4010):
    bad = cp.as_reader().as_builder()
    bad.safetyConfigs[0].safetyParam = word
    assert not qualified(bad)
    assert not native_accepts_cp(bad, 20, word)
  assert qualified(factory(identity, offset=4))


@pytest.mark.parametrize('identity', IDENTITIES)
@pytest.mark.parametrize('aol', (False, True))
@pytest.mark.parametrize('release', (False, True))
def test_actual_card_final_cp_controls_startup_preserves_explicit_off(identity, aol, release, monkeypatch):
  from opendbc.car.honda.tests.test_stock_startup import exercise_startup
  exercise_startup(identity, True, release, aol, 16, monkeypatch)


@pytest.mark.parametrize('identity', (CAR.HONDA_PILOT_4G_MMR,))
def test_distinct_or_new_platform_does_not_inherit_this_owner(identity):
  from opendbc.car.honda.stock_aol import qualified
  cp = factory(identity)
  assert not qualified(cp)


def frames(ci, *, cruise=0, lkas=0, brake=False, gear=4, eps=0, main=True):
  intended = {
    'SCM_FEEDBACK': {'MAIN_ON': int(main)},
    'SCM_BUTTONS': {'CRUISE_BUTTONS': 0, 'CRUISE_SETTING': lkas},
    'ENGINE_DATA': {'XMISSION_SPEED': 45.},
    'WHEEL_SPEEDS': {f'WHEEL_SPEED_{wheel}': 45. for wheel in ('FL', 'FR', 'RL', 'RR')},
    'POWERTRAIN_DATA': {'ACC_STATUS': cruise, 'BRAKE_PRESSED': int(brake), 'BRAKE_SWITCH': int(brake), 'PEDAL_GAS': 0},
    'BRAKE_MODULE': {'BRAKE_PRESSED': int(brake)},
    'STEER_STATUS': {'STEER_STATUS': eps, 'STEER_TORQUE_SENSOR': 0},
    ci.CS.gearbox_msg: {'GEAR_SHIFTER': gear},
    'SEATBELT_STATUS': {'SEATBELT_DRIVER_LATCHED': 1, 'SEATBELT_DRIVER_LAMP': 0},
  }
  result = []
  for role, parser in ci.can_parsers.items():
    packer = CANPacker(parser.dbc.name)
    # Keep packer counters across frames; a new packer per publication is invalid.
    packer = ci._canfd_test_packers.setdefault(role, packer)
    for address in sorted(parser.addresses):
      name = parser.dbc.addr_to_msg[address].name
      values = intended.get(name, {}) if role == Bus.pt else {}
      assert values.keys() <= parser.dbc.name_to_msg[name].sigs.keys()
      result.append(packer.make_can_msg(name, parser.bus, values))
  return result


@pytest.mark.parametrize('alt', (False, True))
@pytest.mark.parametrize('offset', (0, 4))
def test_parsed_native_brake_reverse_temporary_and_real_host_controller_withdrawal(alt, offset, tmp_path):
  from opendbc.safety.tests.libsafety import libsafety_py
  from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import honda_family_case
  safety = libsafety_py.libsafety
  cp = factory(CAR.HONDA_CRV_6G, alt=alt, offset=offset)
  silent = None
  if offset:
    import shutil
    library = tmp_path / 'independent-silent-panda.so'
    shutil.copyfile(libsafety_py.libpath, library)
    silent = libsafety_py.ffi.dlopen(str(library))
    silent.init_tests()
    silent.set_alternative_experience(32)
    assert silent.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, 0) == 0
    silent.set_aol_test_heartbeat(True)
    silent.aol_set_host_request(3)
    assert silent.aol_get_permission_mask() == 0
  with OpenpilotPrefix():
    case = honda_family_case('honda', False)
    case.family_cp = cp.as_reader()
    try:
      sd, controls, drive, card, receive = case.family_callers(False)
      assert card.CP.alternativeExperience == 32
      ci = card.CI
      ci.update([])
      ci._canfd_test_packers = {}
      safety.init_tests()
      safety.set_alternative_experience(32)
      assert safety.set_safety_hooks(20, 16 | int(alt)) == 0
      safety.set_aol_test_heartbeat(True)

      def parse(tick, **kwargs):
        now = 10_000_000_000 + tick * 10_000_000
        data = frames(ci, **kwargs)
        safety.set_timer((now // 1000) & 0xFFFFFFFF)
        for address, payload, bus in data:
          assert safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus - offset, payload))
        safety.aol_set_host_request(1)
        return now, ci.update([(now, data)])

      for tick in range(-40, 0):
        parse(tick)
      parse(0, cruise=1)  # Accepted real PCM rising edge, not fabricated controlsAllowed.
      assert safety.get_controls_allowed()
      _, cs = parse(1, cruise=0, brake=True)
      assert cs.brakePressed and not safety.get_controls_allowed()
      assert safety.aol_get_permission_mask() == 1
      # Existing fixture transports exact current native values to actual SD/Controls.
      # No native permission is fabricated; host session serialization is synthetic.
      for tick in range(2, 7):
        now, cs = parse(tick)
        request, permission = safety.aol_get_request_mask(), safety.aol_get_permission_mask()
        axis, _, command = drive(tick, observed_state=structs.CarState(**cs.to_dict()),
                                 allowed=bool(permission & 1), native_request_lat=bool(request & 1),
                                 native_request_long=False, panda_controls_allowed=bool(safety.get_controls_allowed()))
        assert axis.lateralActive and command.latActive
        assert not axis.longitudinalActive and not command.longActive
        assert receive(now, axis=sd.pm.messages['aolAxisState'], event=sd.pm.messages['onroadEvents'],
                       state=structs.CarState(**cs.to_dict()))
        # Exercise Card's real initialization and apply caller, with only wire
        # publication captured. The command is the real Controls publication.
        card.sm.update_msgs(now / 1e9, [controls.pm.messages['carControl']])
        outgoing = []
        with patch('openpilot.selfdrive.car.card.time.monotonic_ns', return_value=now), \
             patch('openpilot.selfdrive.car.card.time.monotonic', return_value=now / 1e9), \
             patch.object(card, 'publish_sendcan', side_effect=lambda sends, valid, _outgoing=outgoing: _outgoing.extend(sends) if valid else None):
          card.controls_update(cs, command.as_reader())
        steering = [f for f in outgoing if f[0] == 0xE4]
        assert len(steering) == 1
        for address, payload, bus in steering:
          packet = libsafety_py.make_CANPacket(address, bus - offset, payload)
          assert safety.safety_tx_hook(packet)
          if silent is not None:
            assert not silent.safety_tx_hook(packet)
            assert silent.aol_get_permission_mask() == 0
      for tick, kwargs, expected in ((7, {'gear': 2}, 0), (8, {'eps': 6}, 0), (9, {}, 1),
                                     (10, {'eps': 7}, 0), (11, {}, 0)):
        now, cs = parse(tick, **kwargs)
        permission = safety.aol_get_permission_mask()
        assert permission == expected
        axis, _, command = drive(tick, observed_state=structs.CarState(**cs.to_dict()), allowed=bool(permission & 1),
                                 native_request_long=False, panda_controls_allowed=bool(safety.get_controls_allowed()))
        assert not command.longActive and not axis.longitudinalActive
        if not expected:
          assert not command.latActive and not axis.lateralActive
    finally:
      safety.set_alternative_experience(0)
      if silent is not None:
        silent.set_alternative_experience(0)
      case.doCleanups()


@pytest.mark.parametrize('identity', IDENTITIES)
def test_canfd_does_not_borrow_classic_can2_diagnostic_exception(identity):
  from openpilot.starpilot.aol.panda_faults import blocking_fault_mask, blocking_fault_list
  from openpilot.cereal import log
  cp = factory(identity)
  cp.alternativeExperience = 32
  assert blocking_fault_mask(1 << 3, cp)
  assert blocking_fault_list([log.PandaState.FaultType.interruptRateCan2], cp)
