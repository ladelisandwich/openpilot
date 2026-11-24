"""No-camera/no-ACC Volt: independent F1 brake source, conventional cruise only."""
import pytest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.tests.test_volt_camera_automatic import discover, recorded_fingerprint
from opendbc.car.gm.tests.test_bolt_cc import feed
from opendbc.car.gm.values import CAR, DBC, is_volt_cc_profile
from opendbc.safety.tests.libsafety import libsafety_py


def params(release=False):
  return CarInterface.get_params(CAR.CHEVROLET_VOLT_CC, recorded_fingerprint(), [], False, release, False)


def tick(ci, packer, now, *, counter=0, active=True, gas=False, brake=False, regen=False, analog=0, fault=True):
  _, frames = feed(SimpleNamespace(update=lambda messages: None), packer, now,
                   counter=counter, active=active, stock=60. / 3.6, gas=gas, brake=brake, regen=regen, speed=20.)
  frames = [m for m in frames if m[2] == 0 and m[0] not in (0xBE, 0x1C4, 0x1F5)]
  frames += [packer.make_can_msg('EBCMBrakePedalPosition', 0, {'BrakePedalPosition': analog}),
             packer.make_can_msg('AcceleratorPedal2', 0, {'CruiseState': 3 if fault else 0, 'AcceleratorPedal2': 30 if gas else 0}),
             packer.make_can_msg('ECMPRDNL2', 0, {'PRNDL2': 4})]
  return ci.update([(now, frames)]), frames


def test_recorded_noacc_collision_uses_conventional_owner():
  assert discover(recorded_fingerprint())[0] == CAR.CHEVROLET_VOLT_CC
  cp = params()
  assert not cp.dashcamOnly and is_volt_cc_profile(cp)
  assert cp.networkLocation == structs.CarParams.NetworkLocation.gateway
  assert cp.safetyConfigs[0].safetyParam == 0xC163
  assert not cp.alphaLongitudinalAvailable and cp.radarUnavailable


@pytest.mark.parametrize('active', [False, True])
@pytest.mark.parametrize('fault', [False, True])
def test_actual_f1_parser_uses_ordinary_cruise_despite_unused_acc_field(active, fault):
  cp = params()
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  for i in range(30):
    cs, _ = tick(ci, packer, 1_000_000_000 + i * 40_000_000, counter=i % 4, active=active, fault=fault)
  assert cs.canValid and not cs.canTimeout
  assert cs.cruiseState.enabled == active
  assert not cs.accFaulted and not cs.cruiseState.standstill
  assert cs.cruiseState.speed == pytest.approx(60. / 3.6)
  assert ci.CS.volt_cc_physical.sources_current(2_160_000_000)


@pytest.mark.parametrize('brake,analog,regen', [(False, 0, False), (True, 0, False), (False, 6, False), (False, 0, True)])
def test_f1_driver_inputs_are_preserved_host(brake, analog, regen):
  cp = params()
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  for i in range(30):
    now = 1_000_000_000 + i * 40_000_000
    cs, frames = tick(ci, packer, now, counter=i % 4, brake=brake, analog=analog, regen=regen)
  assert cs.brakePressed == (brake or analog >= 6)
  assert cs.regenBraking == regen


@pytest.mark.parametrize('driver', ['none', 'foot', 'analog', 'paddle', 'gas'])
def test_native_f1_stock_permission_and_driver_withdrawal(driver):
  safety = libsafety_py.libsafety
  debug = safety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) == 0
  cp = params()
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, 0xC163) == 0
  safety.init_tests()
  for i in range(30):
    now = 1_000_000_000 + i * 40_000_000
    safety.set_timer(now // 1000)
    _, frames = tick(ci, packer, now, counter=i % 4, gas=driver == 'gas', brake=driver == 'foot',
                     analog=6 if driver == 'analog' else 0, regen=driver == 'paddle')
    for address, data, bus in frames:
      safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
  if not debug:
    assert not safety.safety_config_valid()
    return
  safety.safety_tick()
  assert safety.safety_config_valid()
  assert bool(safety.get_controls_allowed()) == (driver in ('none', 'gas'))
  for address, length, bus in [(0x315, 5, 0), (0x2CB, 8, 0), (0x200, 6, 0), (0x184, 8, 2)]:
    assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, bytes(length)))
  for bus in (0, 1, 2):
    for address in (0x180, 0x184, 0x1E1, 0x315, 0x2CB):
      assert safety.safety_fwd_hook(bus, address) == -1


def test_release_f1_factory_keeps_existing_volt_cc_denial():
  cp = params(release=True)
  assert cp.dashcamOnly and not cp.openpilotLongitudinalControl
  assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput


def test_native_f1_button_slots_cannot_be_reused_or_survive_stale_sources():
  safety = libsafety_py.libsafety
  debug = safety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) == 0
  cp = params()
  ci = CarInterface(cp)
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.gm, 0xC163) == 0
  safety.init_tests()
  for i in range(40):
    now = 1_000_000_000 + i * 40_000_000
    safety.set_timer(now // 1000)
    _, frames = tick(ci, packer, now, counter=i % 4)
    for address, data, bus in frames:
      safety.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
  ci.CC.frame = 156
  command = structs.CarControl(enabled=True, latActive=True, longActive=True)
  command.hudControl.setSpeed = 25.
  command.actuators.accel = 1.
  _, frames = ci.apply(command.as_reader(), now + 1)
  assert all(bus == 0 for _, _, bus in frames)
  assert not any(address in (0x315, 0x2CB, 0x2CD, 0x200, 0x370) for address, _, _ in frames)
  buttons = [m for m in frames if m[0] == 0x1E1]
  assert len(buttons) == 1
  address, data, bus = buttons[0]
  packet = libsafety_py.make_CANPacket(address, bus, data)
  assert bool(safety.safety_tx_hook(packet)) == debug
  assert not safety.safety_tx_hook(packet)
  safety.set_timer(now // 1000 + 300_001)
  assert not safety.safety_tx_hook(packet)
  safety.set_timer(now // 1000 + 2_000_000)
  safety.safety_tick()
  assert not safety.safety_config_valid()
