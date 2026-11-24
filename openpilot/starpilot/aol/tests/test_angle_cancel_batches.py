import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.hyundai.canfd_angle_aol import MODELS, qualified
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_stock_angle_aol import source_profile
from opendbc.car.hyundai.values import CAR
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.tests import test_hkg_cancel_joined as parser_fixture
from openpilot.starpilot.car.hyundai.aol import create_intent


@pytest.mark.parametrize('identity', sorted(MODELS))
@pytest.mark.parametrize('action', [0, 3, 4, 9])
def test_real_angle_cancel_press_release_batch_preserves_saved_mapping(identity, action):
  cp = source_profile(CAR(identity))
  assert cp.alternativeExperience == 0
  ci = CarInterface(cp)
  cp.alternativeExperience = 32
  assert qualified(ci.CP, marked_only=True)
  settings = AolSettings(True, 0., 9, 0, (0, 0, 0), (action, 0, 0))
  owner = create_intent(ci.CP, settings)
  ci.update([])
  for tick in range(1, 125):
    state = ci.update([(10_000_000_000 + tick * 10_000_000,
                        parser_fixture.parser_frames(ci, tick, lkas=int(tick == 122)))])
    owner.update(state, now_ns=10_000_000_000 + tick * 10_000_000)
  assert state.canValid and not state.canTimeout
  assert owner.allowed_latch
  parser = ci.can_parsers[Bus.pt]
  packer = CANPacker(parser.dbc.name)
  name = ci.CS.cruise_btns_msg_canfd
  modulo = 1 << packer.dbc.name_to_msg[name].sigs['COUNTER'].size
  frames = parser_fixture.parser_frames(ci, 126)
  for counter, button in ((64, 4), (65, 0)):
    frames.append(packer.make_can_msg(name, parser.bus, {'COUNTER': counter % modulo, 'CRUISE_BUTTONS': button}))
  state = ci.update([(11_260_000_000, frames)])
  assert state.canValid and not state.canTimeout
  edges = [event.pressed for event in state.buttonEvents if str(event.type) == 'cancel']
  assert edges == [True, False]
  owner.update(state, now_ns=11_260_000_000)
  assert owner.allowed_latch == (action == 0)
  assert owner.settings == settings
