"""Root-only actual factory/parser/intent/CI/native pairing; no synthetic native ACK."""
import pytest
from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.aol.intent import AolSettings, AOL_TOGGLE
from openpilot.starpilot.aol.tests.test_canfd_stock_profiles import params
from openpilot.starpilot.aol.tests.test_ev6_stock_parity import packets
from openpilot.starpilot.car.hyundai.aol import create_intent, policy_for


@pytest.mark.parametrize('identity', [CAR.KIA_EV6, CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_IONIQ_6, CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN])
@pytest.mark.parametrize('alternate', [False, True])
@pytest.mark.parametrize('incidental', ['cancel', 'mainCruise'])
def test_actual_stock_factory_parser_intent_ci_and_native_cancel_pair(identity, alternate, incidental):
  cp = params(identity, alternate=alternate)
  if identity == CAR.HYUNDAI_IONIQ_6:
    # Real I6 MRR35 radar inventory on radar_bus(CP)=0 for HDAII.
    # Missing radar correctly denies exact_ioniq6_hda2_base; never mark that CP.
    fp = gen_empty_fingerprint()
    fp[2][0x110 if alternate else 0x50] = 32 if alternate else 16
    fp[1].update({0x1cf: 8, 0x130: 32, 0x35: 32, 0x175: 24, 0xa0: 24, 0xea: 24, 0x1a0: 32})
    fp[0][0x3a5] = 24
    cp = CarInterface.get_params(identity, fp, [], False, False, False)
    assert not cp.radarUnavailable
  assert policy_for(cp).explicit_latch

  cp.safetyConfigs[-1].safetyParam |= policy_for(cp).safety_param_addition
  assert cp.safetyConfigs[-1].safetyParam == (0x891 if alternate else 0x811)
  ci = CarInterface(cp)
  ci.update([])
  owner = create_intent(cp, AolSettings(True, 0., AOL_TOGGLE, 0, (0, 0, 0), (0, 0, 0)))
  native = libsafety_py.libsafety
  native.set_alternative_experience(cp.alternativeExperience)
  assert native.set_safety_hooks(int(cp.safetyConfigs[-1].safetyModel.raw), cp.safetyConfigs[-1].safetyParam) == 0
  native.init_tests()
  native.set_alternative_experience(cp.alternativeExperience)  # init_tests resets this global.
  native.set_aol_test_heartbeat(True)
  plan = [(0, 0, 0, 0)] * 20 + [(0, 1, 0, 0), (0, 0, 0, 0), (0, 0, 2, 0), (0, 0, 0, 1)]
  plan += [(int(incidental == 'mainCruise'), 0, 4 if incidental == 'cancel' else 0, 1), (0, 0, 0, 0)]
  edge = None
  for tick, (main, lkas, button, acc) in enumerate(plan):
    now = 1_000_000 + tick * 10_000
    native.set_timer(now)
    frames = packets(ci, tick, main=main, lkas=lkas)
    # Extend the established real-parser frame fixture with actual cruise codes.
    for parser in ci.can_parsers.values():
      packer = CANPacker(parser.dbc.name)
      for address in parser.addresses:
        name = parser.dbc.addr_to_msg[address].name
        values: dict[str, float]
        if name == ci.CS.cruise_btns_msg_canfd:
          values = {'COUNTER': tick % (16 if name == 'CRUISE_BUTTONS' else 256),
                    'LDA_BTN': lkas, 'ADAPTIVE_CRUISE_MAIN_BTN': main, 'CRUISE_BUTTONS': button}
        elif name == 'MDPS':
          values = {'COUNTER': tick % 256, 'MDPS_StrTqSnsrVal': 0}
        elif name == 'SCC_CONTROL':
          values = {'COUNTER': tick % 256, 'MainMode_ACC': 1, 'ACCMode': acc}
        else:
          continue
        frames = [f for f in frames if (f[0], f[2]) != (address, parser.bus)]
        frames.append(packer.make_can_msg(name, parser.bus, values))
    for frame in frames:
      assert native.safety_rx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))
    state = ci.update([(now * 1000, frames)])
    owner.update(state, now_ns=now * 1000, fault_active=False, standard_enabled=bool(acc))
    native.aol_set_host_request(1 if owner.allowed_latch and not owner.pause_lateral else 0)
    permission = native.aol_get_permission_mask()
    if tick == 21:
      assert state.canValid and owner.allowed_latch and permission == 1
      edge = owner._last_latch_edge_ns
    if tick >= 24:
      assert owner.allowed_latch and not owner.pause_lateral and permission == 1
      assert owner._last_latch_edge_ns == edge
  assert not state.cruiseState.enabled and not native.get_controls_allowed()
  command = structs.CarControl(latActive=True, longActive=False)
  command.actuators.torque = .002
  _, outgoing = ci.apply(command.as_reader(), now * 1000)
  steering = [frame for frame in outgoing if frame[0] == (0x110 if alternate else 0x50) and frame[2] == ci.CC.CAN.ACAN]
  assert steering
  for frame in steering:
    assert native.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))
  # Actual host toggle-off reaches the native request mask; no continued steer.
  native.aol_set_host_request(0)
  assert native.aol_get_permission_mask() == 0
