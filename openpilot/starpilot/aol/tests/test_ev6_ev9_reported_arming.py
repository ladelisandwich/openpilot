"""EV6 05 default engagement and EV9 3b pedal/lateral regression contracts."""
import pytest

from opendbc.car import structs
from opendbc.car.hyundai.values import Buttons
from openpilot.starpilot.aol.tests import test_ev6_physical_authorization as ev6
from opendbc.safety.tests import test_hyundai_optional_release as torque
from opendbc.safety.tests import test_ev9_combined_aol as ev9


@pytest.mark.parametrize('button', [Buttons.SET_DECEL, Buttons.RES_ACCEL])
def test_ev6_default_successful_engagement_arms_lateral_after_fresh_physical_release(button):
  case = ev6.TestEv6ConfiguredDesire(methodName='runTest')
  owner = case.owner()
  case.sample(owner, [(0, 0, Buttons.NONE)])
  case.sample(owner, [(0, 0, button)])
  case.sample(owner, [(0, 0, Buttons.NONE)])
  assert not owner.allowed_latch
  cs = case.sample(owner, [], active=True)
  assert owner.allowed_latch and owner.output(cs)[0]


@pytest.mark.parametrize('word', [0x815, 0x895])
@pytest.mark.parametrize('button', [Buttons.SET_DECEL, Buttons.RES_ACCEL])
def test_torque_successful_full_engagement_keeps_only_lateral_on_brake(word, button):
  case = torque.TestHyundaiOptionalRelease(methodName='runTest')
  case.mode(word)
  assert not case.release
  case.safety.aol_set_host_request(3)
  case.release_gesture(button)
  case.safety.aol_set_host_request(3)
  assert case.safety.get_controls_allowed()
  assert case.safety.aol_get_permission_mask() == 3
  case.rx('TCS', {'DriverBraking': 1})
  assert not case.safety.get_controls_allowed()
  assert case.safety.aol_get_permission_mask() == 1
  assert case.tx(1, 1)


@pytest.mark.parametrize('pedal', ['gas', 'brake'])
@pytest.mark.parametrize('word', [0x5c95, 0x5495])
def test_ev9_physical_aol_keeps_lateral_and_denies_acceleration_during_pedal(pedal, word):
  case = ev9.TestEV9CombinedDebug(methodName='runTest')
  case.setUp()
  try:
    assert case.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, word) == 0
    case.warm()
    case.arm()
    case.tick(**{pedal: True})
    case.safety.aol_set_host_request(1)
    assert case.safety.aol_get_permission_mask() == 1
    assert case.safety.safety_tx_hook(case.angle(value=0.))
    assert not case.safety.safety_tx_hook(case.scc(1.))
  finally:
    case.doCleanups()


@pytest.mark.parametrize('boundary', ['old_ack', 'can_fault', 'real_fault', 'cancel', 'cold_held'])
def test_ev6_engagement_credit_cannot_survive_a_denied_or_unphysical_activation(boundary):
  case = ev6.TestEv6ConfiguredDesire(methodName='runTest')
  owner = case.owner()
  if boundary != 'cold_held':
    case.sample(owner, [(0, 0, Buttons.NONE)])
  case.sample(owner, [(0, 0, Buttons.SET_DECEL)])
  case.sample(owner, [(0, 0, Buttons.NONE)])
  if boundary == 'old_ack':
    owner._test_clock_ns += 200_000_001
  elif boundary == 'can_fault':
    case.sample(owner, [], active=True, valid=False)
  elif boundary == 'real_fault':
    case.sample(owner, [], active=True, fault=True)
  elif boundary == 'cancel':
    event = structs.CarState.ButtonEvent(type='cancel', pressed=True)
    case.sample(owner, [(0, 0, Buttons.CANCEL)], active=True, events=[event])
  cs = case.sample(owner, [], active=True)
  assert not owner.allowed_latch and not owner.output(cs)[0]


@pytest.mark.parametrize('word', [0x815, 0x895])
@pytest.mark.parametrize('boundary', ['no_release', 'replay', 'counter_gap', 'no_full_request', 'expired', 'cancel'])
def test_torque_full_engagement_latch_refuses_invalid_release_or_missing_full_ack(word, boundary):
  case = torque.TestHyundaiOptionalRelease(methodName='runTest')
  case.mode(word)
  case.safety.aol_set_host_request(1)
  case.buttons()
  case.buttons(Buttons.SET_DECEL)
  previous = case.counters['CRUISE_BUTTONS'] % 16
  if boundary != 'no_release':
    counter = previous if boundary == 'replay' else (previous + 2) % 16 if boundary == 'counter_gap' else None
    case.buttons(counter=counter)
  if boundary == 'expired':
    case.now += 200_001
    case.safety.set_timer(case.now)
    case.healthy()
  if boundary == 'cancel':
    case.buttons(Buttons.CANCEL)
  case.safety.aol_set_host_request(1 if boundary == 'no_full_request' else 3)
  case.rx('TCS', {'DriverBraking': 1})
  assert case.safety.aol_get_permission_mask() == 0
  assert not case.tx(1, 1)
