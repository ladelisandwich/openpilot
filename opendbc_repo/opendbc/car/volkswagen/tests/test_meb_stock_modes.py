"""Actual preference/factory/parser/controller and saved OFF matrix for all nine MEBs."""
import pytest
from opendbc.car import Bus, structs
from opendbc.can import CANPacker, CANParser
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.meb_modes import MEB_CARS, base_word, qualified
from opendbc.car.volkswagen.values import DBC, CanBus
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences


@pytest.mark.parametrize('identity', sorted(MEB_CARS))
@pytest.mark.parametrize('disabled', (False, True))
@pytest.mark.parametrize('alt_gear', (False, True))
def test_actual_saved_off_keeps_stock_acc_and_the_selected_curvature_controller(identity, disabled, alt_gear):
  fp = {bus: {} for bus in range(8)}
  fp[1][0x13D] = 32
  if alt_gear:
    fp[0][0x3DC] = 8
  cp = CarInterface.get_params(identity, fp, [], False, False, False)
  with OpenpilotPrefix():
    params = Params()
    params.put_bool('OpenpilotEnabledToggle', True, block=True)
    params.put_bool('SafeMode', False, block=True)
    params.put_bool('DisableOpenpilotLongitudinal', disabled, block=True)
    preferences = VehicleStartupPreferences.read(params, enabled=True)
    preferences.prepare(cp, fingerprints=fp)
    ci = CarInterface(cp)
    preferences.finalize(cp)
    assert cp.openpilotLongitudinalControl == (not disabled)
    assert cp.pcmCruise == disabled
    assert cp.safetyConfigs[-1].safetyParam == base_word(cp)
    assert qualified(cp)
    ci.update([])
    packer = CANPacker(DBC[identity][Bus.pt])
    can = CanBus(cp)
    messages = [packer.make_can_msg('Getriebe_11', can.pt, {'GE_Fahrstufe': 8}),
                packer.make_can_msg('Gateway_73', can.pt, {'GE_Fahrstufe': 8}),
                packer.make_can_msg('Motor_51', can.pt, {'TSK_Status': 3}),
                packer.make_can_msg('Motor_54', can.pt, {'Engine_On': 1}),
                packer.make_can_msg('QFK_01', can.pt, {'LatCon_HCA_Status': 2}),
                packer.make_can_msg('ACC_19', can.ext, {'ACC_Wunschgeschw_02': 72, 'ACC_Limiter_Mode': 0})]
    ci.can_parsers[Bus.pt].update([(1_000_000_000, messages)])
    ci.can_parsers[Bus.cam].update([(1_000_000_000, messages)])
    state = ci.CS.update(ci.can_parsers)
    assert state.gearShifter == structs.CarState.GearShifter.drive
    assert state.cruiseState.available and state.cruiseState.enabled
    if disabled:
      assert state.cruiseState.speed == pytest.approx(20., abs=.1)
    ci.CS.out = state
    command = structs.CarControl()
    command.cruiseControl.cancel = True
    _, sent = ci.apply(command.as_reader(), 1_010_000_000)
    ids = {addr for addr, _, _ in sent}
    assert (0x14D in ids) == (not disabled)
    if disabled:
      assert not ids & {0x14D, 0x300, 0x26B}
      assert 0x12B in ids  # Actual restored stock Cancel command factory.
      observe = CANParser(DBC[identity][Bus.pt], [('GRA_ACC_01', 0)], can.ext)
      observe.update([(1_010_000_000, sent)])
      assert observe.vl['GRA_ACC_01']['GRA_Abbrechen'] == 1
      assert observe.vl['GRA_ACC_01']['COUNTER'] == (ci.CS.gra_stock_values['COUNTER'] + 1) % 16
      assert all(bus == can.ext for addr, _, bus in sent if addr == 0x12B)
