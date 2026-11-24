"""Physical admission, gesture, edge handling and deterministic bounded flight."""

from dataclasses import replace
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.starpilot.ui.spacepilot.guard import ParkGate, SERVICES, read_witness
from openpilot.starpilot.ui.spacepilot.state import FiveTaps, Flight, Gate, Phase, WheelInput


class Messages(dict):
  def __init__(self, now=10_000_000_000, offset=2_000_000_000):
    super().__init__(
      carState=NS(canValid=True, canTimeout=False, standstill=True, gearShifter='park', vEgo=0., vEgoRaw=0.,
                  wheelSpeeds=NS(fl=0., fr=0., rl=0., rr=0.), gasPressed=False, cruiseState=NS(enabled=False), buttonEvents=[]),
      selfdriveState=NS(enabled=False, active=False, state='disabled', alertSize='none', alertStatus='normal', alertSound='none'),
      carControl=NS(enabled=False, latActive=False, longActive=False), deviceState=NS(started=True),
      pandaStates=[NS(pandaType='dos', harnessStatus='normal', ignitionLine=True, ignitionCan=False, controlsAllowed=False,
                      heartbeatLost=False, safetyRxChecksInvalid=False, faultStatus='none', faults=[])])
    self.valid = dict.fromkeys(SERVICES, True)
    self.alive = dict.fromkeys(SERVICES, True)
    self.recv_frame = dict.fromkeys(SERVICES, 10)
    self.recv_time = dict.fromkeys(SERVICES, now / 1e9)
    self.logMonoTime = {name: now + (offset if name == 'pandaStates' else 0) for name in SERVICES}


def parked_ui(now=10_000_000_000):
  return NS(sm=Messages(now), started=True, started_frame=1, CP=NS(notCar=False), projection_read_only=False, replay_clock=None)


def read(ui, now=10_000_000_000, *, c4=True):
  return read_witness(ui, now, now + 2_000_000_000, c4=c4)


def mature_gate():
  gate = ParkGate()
  for now in range(9_000_000_000, 10_000_000_001, 100_000_000):
    gate.update(read(parked_ui(now), now))
  assert gate.ready
  return gate


@pytest.mark.parametrize(('path', 'value'), [
  ('carState.gearShifter', gear) for gear in ('drive', 'reverse', 'neutral', 'unknown')
] + [
  ('carState.standstill', False), ('carState.vEgo', .011), ('carState.vEgoRaw', -.011),
  ('carState.wheelSpeeds.fl', .02), ('carState.wheelSpeeds.fr', .02), ('carState.wheelSpeeds.rl', .02), ('carState.wheelSpeeds.rr', .02),
  ('carState.vEgo', math.nan), ('carState.vEgoRaw', math.inf), ('carState.canValid', False), ('carState.canTimeout', True),
  ('carState.gasPressed', True), ('carState.cruiseState.enabled', True), ('selfdriveState.enabled', True),
  ('selfdriveState.active', True), ('selfdriveState.state', 'preEnabled'), ('selfdriveState.alertSize', 'small'),
  ('selfdriveState.alertStatus', 'critical'), ('selfdriveState.alertSound', 'warningImmediate'),
  ('carControl.enabled', True), ('carControl.latActive', True), ('carControl.longActive', True), ('deviceState.started', False),
])
def test_vehicle_change_immediately_revokes(path, value):
  ui, gate = parked_ui(), mature_gate()
  service, *attrs = path.split('.')
  target = ui.sm[service]
  for attr in attrs[:-1]:
    target = getattr(target, attr)
  setattr(target, attrs[-1], value)
  assert read(ui) is None
  assert not gate.update(read(ui))


@pytest.mark.parametrize(('field', 'value'), [
  ('pandaType', 'unknown'), ('harnessStatus', 'notConnected'), ('ignitionLine', False), ('controlsAllowed', True),
  ('heartbeatLost', True), ('safetyRxChecksInvalid', True), ('faultStatus', 'faultTemp'), ('faults', ['relayMalfunction']),
])
def test_physical_panda_refusals(field, value):
  ui = parked_ui()
  setattr(ui.sm['pandaStates'][0], field, value)
  assert read(ui) is None


@pytest.mark.parametrize('service', SERVICES)
@pytest.mark.parametrize('mutation', ['invalid', 'dead', 'source_old', 'source_future', 'receipt_old', 'receipt_future', 'pre_start', 'missing'])
def test_every_transport_is_required_and_has_original_clock_freshness(service, mutation):
  ui = parked_ui()
  if mutation in ('invalid', 'dead'):
    getattr(ui.sm, 'valid' if mutation == 'invalid' else 'alive')[service] = False
  elif mutation.startswith('source'):
    ui.sm.logMonoTime[service] += -2_000_000_000 if mutation == 'source_old' else 1
  elif mutation.startswith('receipt'):
    ui.sm.recv_time[service] += -2. if mutation == 'receipt_old' else 1.
  elif mutation == 'pre_start':
    ui.sm.recv_frame[service] = ui.started_frame
  else:
    del ui.sm[service]
  assert read(ui) is None


def test_real_park_required_even_when_forced_offroad_and_no_host_or_replay_bypass(monkeypatch):
  monkeypatch.setenv('SP_HOST_PARKED', '1')
  ui = parked_ui()
  assert read(ui) is not None
  assert read(ui, c4=False) is None
  ui.started = False
  assert read(ui) is None
  ui.started = True
  for field, value in [('CP', None), ('CP', NS(notCar=True)), ('projection_read_only', True), ('replay_clock', object())]:
    ui = parked_ui()
    setattr(ui, field, value)
    assert read(ui) is None


def test_continuous_park_dwell_reset_clock_resume_and_frozen_producers():
  gate = mature_gate()
  old = gate.previous
  assert old is not None
  assert not gate.update(replace(old, mono_ns=old.mono_ns + 300_000_000, boot_ns=old.boot_ns + 300_000_000))
  gate = mature_gate()
  assert not gate.update(replace(old, boot_ns=old.boot_ns + 6_000_000))
  gate = mature_gate()
  assert not gate.update(replace(old, stamps=(old.stamps[0] - 1, *old.stamps[1:])))
  gate = mature_gate()
  assert not gate.update(read(parked_ui(), 10_300_000_000))
  assert not gate.ready
  assert not gate.update(read(parked_ui(10_400_000_000), 10_400_000_000))


def test_five_distinct_taps_debounce_timeout_clock_and_reset():
  taps = FiveTaps()
  for now in (1., 1.01, 1.2, 1.21, 1.4, 1.41, 1.6):
    assert not taps.tap(now)
  assert taps.tap(1.8)
  assert taps.count == 0
  assert not taps.tap(2.)
  assert not taps.tap(3.6)
  assert taps.count == 1
  taps.reset()
  assert taps.count == 0
  for now in (10., 11.6, 13.2, 14.8, 16.4):
    assert not taps.tap(now)
  assert taps.count == 1
  assert not taps.tap(1.)
  assert taps.count == 1
  assert not taps.tap(math.nan)
  assert taps.count == 0


def test_wheel_requires_release_deduplicates_both_keys_and_repeated_holds():
  wheel = WheelInput(10)
  assert not wheel.update(11, (('accelCruise', True),), 1.)
  assert not wheel.update(12, (('accelCruise', False),), 1.1)
  assert wheel.update(13, (('accelCruise', True),), 1.3)
  assert not wheel.update(13, (('accelCruise', True),), 1.5)
  assert not wheel.update(14, (('accelCruise', True),), 1.8)
  assert not wheel.update(15, (('decelCruise', False),), 2.)
  assert wheel.update(16, (('decelCruise', True),), 2.2)
  assert not wheel.update(17, (('cancel', True), ('gapAdjustCruise', True)), 3.)
  assert not wheel.update(18, (('decelCruise', False), ('decelCruise', True)), 2.25)
  assert not wheel.update(19, (('decelCruise', True),), 3.)


def test_flight_start_collision_restart_lockout_and_cleanup():
  game = Flight(5683)
  assert game.phase == Phase.READY
  assert game.act(1.)
  assert not game.act(1.01)
  for _ in range(120):
    game.update(1 / 60)
  assert game.phase == Phase.OVER
  game.over_age = 0
  assert not game.act(3.)
  for _ in range(40):
    game.update(1 / 60)
  assert game.act(4.)
  assert game.phase == Phase.FLIGHT and game.score == 0
  game.close()
  assert not game.gates


def test_score_only_after_whole_gate_and_collision_does_not_score():
  game = Flight()
  game.act(1.)
  game.vy = 0
  game.gates = [Gate(game.SHIP_X - game.GATE_WIDTH - game.RADIUS + 1, game.y)]
  game.update(1 / 120)
  assert game.score == 1
  game.update(1 / 120)
  assert game.score == 1
  game.gates = [Gate(game.SHIP_X, game.y + game.GAP)]
  game.update(1 / 120)
  assert game.phase == Phase.OVER and game.score == game.best == 1


def test_fixed_step_60_and_120_hz_match_and_long_frame_catchup_is_bounded():
  a, b = Flight(7), Flight(7)
  a.act(1.)
  b.act(1.)
  for _ in range(30):
    a.update(1 / 60)
    b.update(1 / 120)
    b.update(1 / 120)
  assert (a.y, a.vy, a.score, a.gates) == (b.y, b.vy, b.score, b.gates)
  a.update(10.)
  b.update(.05)
  assert (a.y, a.vy, a.gates) == (b.y, b.vy, b.gates)
  before = a.y
  a.update(math.nan)
  a.update(-1.)
  assert a.y == before


def test_long_autopilot_flight_keeps_storage_bounded():
  game = Flight(5683)
  game.act(0.)
  maximum = 0
  for frame in range(30_000):
    target = next((g.center for g in game.gates if g.x + game.GATE_WIDTH > game.SHIP_X - game.RADIUS), 250.)
    if game.y > target + 13 and game.vy > 0 or game.phase == Phase.OVER:
      game.act(frame / 60)
    game.update(1 / 60)
    maximum = max(maximum, len(game.gates))
  assert maximum <= 5
  assert max(game.score, game.best) > 10
