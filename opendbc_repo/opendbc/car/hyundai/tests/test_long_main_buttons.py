import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.ev9_longitudinal import candidate as ev9_candidate
from opendbc.car.hyundai.ioniq5pe_long_profile import candidate as pe_candidate
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.long_buttons import qualified
from opendbc.car.hyundai.tests.test_ioniq5pe_stock import params
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags
from opendbc.car import Bus
from opendbc.safety.tests.test_hyundai import checksum
from openpilot.cereal import log
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.selfdrived.state import StateMachine


def profile(name):
  if name == 'elantra':
    return CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, gen_empty_fingerprint(), [], True, False, False)
  identity = CAR.KIA_EV9 if name == 'ev9' else CAR.HYUNDAI_IONIQ_5_PE
  factory = ev9_candidate if name == 'ev9' else pe_candidate
  return factory(params(candidate=identity), enabled=True, is_release=False)


class Driver:
  def __init__(self, name):
    self.cp = profile(name)
    assert qualified(self.cp)
    self.ci = CarInterface(self.cp)
    self.packer = CANPacker(DBC[self.cp.carFingerprint][Bus.pt])
    self.counts = {}
    self.tick = 0
    self.events = CarEvents(self.cp)
    self.machine = StateMachine()
    self.state = self.ci.update([])
    self.last_events = None
    self.canfd = bool(self.cp.flags & HyundaiFlags.CANFD)
    self.button_name = 'CRUISE_BUTTONS' if self.canfd else 'CLU11'
    self.button_bus = 1 if self.canfd else 0
    for _ in range(120):
      self.send()
    assert self.state.canValid and not self.state.canTimeout

  def packet(self, name, bus, *, button=0, adaptive=False, normal=False):
    count = self.counts.get((name, bus), 0)
    self.counts[name, bus] = count + 1
    values = {
      'ACCELERATOR': {'GEAR': 5}, 'ELECT_GEAR': {'Elect_Gear_Shifter': 5},
      'DOORS_SEATBELTS': {'DRIVER_SEATBELT': 1}, 'CGW1': {'CF_Gway_DrvSeatBeltSw': 1},
      'MDPS': {'MDPS_StrTqSnsrVal': 0., 'MDPS_OutTqVal': 0., 'MDPS_EstStrAnglVal': 0.,
               'MDPS_ADAS_AciFltSig_Lv2': 0},
      'MDPS12': {'CR_Mdps_StrColTq': 0., 'CR_Mdps_OutTq': 0.},
      'TCS': {'ACCEnable': 0, 'ACC_REQ': 1, 'DriverBraking': 0},
      'TCS13': {'AliveCounterTCS': count % 8, 'ACCEnable': 0, 'ACC_REQ': 1, 'DriverOverride': 0},
      'EMS16': {'AliveCounter': count % 4},
      'WHL_SPD11': {'WHL_SPD_AliveCounter_LSB': count % 4, 'WHL_SPD_AliveCounter_MSB': count // 4 % 4,
                    **dict.fromkeys(('WHL_SPD_FL', 'WHL_SPD_FR', 'WHL_SPD_RL', 'WHL_SPD_RR'), 36.)},
      'WHEEL_SPEEDS': dict.fromkeys(('WHL_SpdFLVal', 'WHL_SpdFRVal', 'WHL_SpdRLVal', 'WHL_SpdRRVal'), 36.),
      'CRUISE_BUTTONS': {'CRUISE_BUTTONS': button, 'ADAPTIVE_CRUISE_MAIN_BTN': int(adaptive),
                         'NORMAL_CRUISE_MAIN_BTN': int(normal)},
      'CLU11': {'CF_Clu_AliveCnt1': count % 16, 'CF_Clu_CruiseSwState': button, 'CF_Clu_CruiseSwMain': int(adaptive)},
    }.get(name, {})
    signals = self.packer.dbc.name_to_msg[name].sigs
    if 'COUNTER' in signals:
      values = dict(values, COUNTER=count % (1 << signals['COUNTER'].size))
    frame = self.packer.make_can_msg(name, bus, values)
    if not self.canfd and frame[0] in (0x260, 0x386, 0x394):
      return checksum(frame)
    return frame

  def send(self, button=0, *, adaptive=False, normal=False, pulse=()):
    self.tick += 1
    frames = [self.packet(parser.dbc.addr_to_msg[address].name, parser.bus,
                          button=button, adaptive=adaptive, normal=normal)
              for parser in self.ci.can_parsers.values() for address in sorted(parser.addresses)]
    for pulse_adaptive, pulse_normal, pulse_button in pulse:
      frames.append(self.packet(self.button_name, self.button_bus, button=pulse_button,
                                adaptive=pulse_adaptive, normal=pulse_normal))
    previous = self.state
    self.state = self.ci.update([(self.tick * 10_000_000, frames)])
    self.last_events = self.events.update(self.state, previous, structs.CarControl())
    self.machine.update(self.last_events)
    return self.state

  def engage(self, button=2):
    self.send(adaptive=True)
    self.send()
    self.send(button)
    self.send()
    assert self.machine.state == log.SelfdriveState.OpenpilotState.enabled


@pytest.mark.parametrize('name', ['pe', 'ev9', 'elantra'])
def test_original_main_state_tracking_scope_comes_from_actual_parser(name):
  driver = Driver(name)
  assert driver.state.cruiseState.available == (name == 'pe')
  driver.engage()
  assert driver.state.cruiseState.available
  driver.send(adaptive=True)
  assert not driver.state.cruiseState.available
  assert driver.machine.state == log.SelfdriveState.OpenpilotState.disabled


@pytest.mark.parametrize('name,main', [('pe', 'adaptive'), ('pe', 'normal'), ('ev9', 'adaptive'),
                                    ('ev9', 'normal'), ('elantra', 'adaptive')])
@pytest.mark.parametrize('button', [1, 2])
def test_main_and_held_set_resume_reach_actual_events_and_state_machine(name, main, button):
  driver = Driver(name)
  driver.engage(button)
  driver.send(button)
  driver.send(button, **{main: True})
  assert driver.machine.state == log.SelfdriveState.OpenpilotState.disabled
  driver.send(button)
  driver.send()
  assert not driver.state.buttonEnable
  assert driver.machine.state == log.SelfdriveState.OpenpilotState.disabled
  if main == 'adaptive' and name != 'pe':
    driver.send(adaptive=True)
    driver.send()
  driver.send(button)
  driver.send()
  assert driver.machine.state == log.SelfdriveState.OpenpilotState.enabled


@pytest.mark.parametrize('name', ['pe', 'ev9', 'elantra'])
@pytest.mark.parametrize('kind', ['main', 'cancel'])
def test_same_batch_press_release_withdraws_without_synthetic_cancel(name, kind):
  driver = Driver(name)
  driver.engage()
  pulse = [(1, 0, 0), (0, 0, 0)] if kind == 'main' else [(0, 0, 4), (0, 0, 0)]
  driver.send(pulse=pulse)
  assert driver.machine.state == log.SelfdriveState.OpenpilotState.disabled
  assert any(str(event.type) == ('mainCruise' if kind == 'main' else 'cancel') for event in driver.state.buttonEvents)
  if kind == 'main':
    assert not any(str(event.type) == 'cancel' for event in driver.state.buttonEvents)


@pytest.mark.parametrize('name', ['pe', 'ev9', 'elantra'])
def test_enable_release_without_main_matches_original_profile_scope(name):
  driver = Driver(name)
  driver.send(2)
  driver.send()
  assert driver.state.buttonEnable == (name == 'pe')
  assert driver.machine.state == (log.SelfdriveState.OpenpilotState.enabled if name == 'pe'
                                   else log.SelfdriveState.OpenpilotState.disabled)


@pytest.mark.parametrize('name,experience', [('pe', 0), ('pe', 32), ('elantra', 0), ('ev9', 0), ('ev9', 32)])
@pytest.mark.parametrize('field,value', [('brand', 'other'), ('carFingerprint', 'foreign'),
                                       ('passive', True), ('notCar', True), ('dashcamOnly', True),
                                       ('openpilotLongitudinalControl', False), ('pcmCruise', True),
                                       ('alphaLongitudinalAvailable', False)])
def test_main_state_helper_rejects_changed_authority_contract(name, experience, field, value):
  cp = profile(name)
  cp.alternativeExperience = experience
  assert qualified(cp)
  setattr(cp, field, value)
  assert not qualified(cp)


@pytest.mark.parametrize('name,experience', [('pe', 0), ('pe', 32), ('elantra', 0), ('ev9', 0), ('ev9', 32)])
def test_main_state_helper_rejects_neighbor_native_word(name, experience):
  cp = profile(name)
  cp.alternativeExperience = experience
  assert qualified(cp)
  cp.safetyConfigs[0].safetyParam ^= 1
  assert not qualified(cp)


@pytest.mark.parametrize('name', ['pe', 'ev9', 'elantra'])
@pytest.mark.parametrize('experience', [0, 1, 32, 33, 64])
def test_main_state_helper_requires_exact_profile_experience(name, experience):
  cp = profile(name)
  cp.alternativeExperience = experience
  assert qualified(cp) == (experience == 0 or (name in ('pe', 'ev9') and experience == 32))
