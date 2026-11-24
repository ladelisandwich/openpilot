"""Mach-E AOL must not inject UI while native safety forwards stock UI."""
import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC
from openpilot.common.prefix import OpenpilotPrefix

UI = {0x3D8, 0x18A}


def setup(*, aol=True, identity=CAR.FORD_MUSTANG_MACH_E_MK1):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x5A] = 8
  fingerprint[2].update({0x3D6: 8, 0x186: 8})
  with OpenpilotPrefix():
    cp = CarInterface.get_params(identity, fingerprint, [], False, False, False)
  controller = CarController(DBC[cp.carFingerprint], cp)
  # Card finalizes AOL qualification after the controller is constructed.
  cp.alternativeExperience = 32 if aol else 0
  state = CarState(cp)
  state.update(state.get_can_parsers(cp))
  cs = structs.CarState(canValid=True, vEgo=15., vEgoRaw=15., gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = True
  state.out = cs.as_reader()
  return controller, state


def test_mache_idle_leaves_stock_display_and_fcw_untouched_then_owns_active_ui():
  owner, state = setup()
  command = structs.CarControl()
  # Real stock UI/FCW fields remain available to be forwarded by native safety.
  state.acc_tja_status_stock_values['FcwVisblWarn_B_Rq'] = 1
  state.acc_tja_status_stock_values['Tja_D_Stat'] = 1
  for _ in range(110):
    _, frames = owner.update(command.as_reader(), state, owner.frame * 10_000_000)
    assert not UI.intersection(address for address, _, _ in frames)
  command.latActive = True
  active_ui = set()
  for _ in range(110):
    _, frames = owner.update(command.as_reader(), state, owner.frame * 10_000_000)
    active_ui.update(address for address, _, _ in frames if address in UI)
  assert active_ui == UI
  command.latActive = False
  for _ in range(110):
    _, frames = owner.update(command.as_reader(), state, owner.frame * 10_000_000)
    assert not UI.intersection(address for address, _, _ in frames)
  assert state.acc_tja_status_stock_values['FcwVisblWarn_B_Rq'] == 1


@pytest.mark.parametrize('identity,aol,expected', [(CAR.FORD_MUSTANG_MACH_E_MK1, False, UI), (CAR.FORD_F_150_MK14, True, set())])
def test_idle_ui_respects_native_display_ownership(identity, aol, expected):
  owner, state = setup(aol=aol, identity=identity)
  seen = set()
  for _ in range(110):
    _, frames = owner.update(structs.CarControl().as_reader(), state, owner.frame * 10_000_000)
    seen.update(address for address, _, _ in frames if address in UI)
  assert seen == expected
