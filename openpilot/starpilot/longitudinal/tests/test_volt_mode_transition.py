"""Volt mode transitions reach actual factory/LongControl/Controls callers."""

from types import SimpleNamespace

import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR
from opendbc.car.gm.bolt_mode import policy_for
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.starpilot.longitudinal.extension import LongitudinalContext


def sdgm(*, alpha=True, c9=False):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  if not c9:
    fp[0][0xBE] = 6
  return CarInterface.get_params(CAR.CHEVROLET_VOLT_2019, fp, [], alpha, False, False)


def physical(speed=15.25):
  return SimpleNamespace(
    vEgo=speed, aEgo=0.72, brakePressed=False, gasPressed=False, canValid=True, canTimeout=False, cruiseState=SimpleNamespace(standstill=False)
  )


@pytest.mark.parametrize('c9', (False, True))
def test_exact_sdgm_factory_restores_dom_mode_handoff(c9):
  cp = sdgm(c9=c9)
  assert cp.openpilotLongitudinalControl and not cp.pcmCruise
  controller = LongControl(cp)
  assert controller.extension.bolt_mode is not None
  state = physical()
  for _ in range(120):
    controller.update(True, state, 0.0, False, (-4.0, 1.4), context=LongitudinalContext(experimental_mode=True, has_lead=True))
  controller.reset()
  controller.last_output_accel = 0.0
  controller.update(False, state, 0.668, False, (-4.0, 1.4), context=LongitudinalContext(experimental_mode=False))
  assert controller.extension.bolt_mode.current_mode == 'blended'
  # Recorded Mary handoff: the new positive request must follow Dom's one-second
  # mode ramp; it may not jump straight to the raw .668 m/s² request.
  output = controller.update(True, state, 0.668, False, (-4.0, 1.4), context=LongitudinalContext(experimental_mode=False, has_lead=True))
  assert controller.extension.bolt_mode.leaving_experimental
  assert controller.pid.i == 0.0
  assert output == pytest.approx(float(controller.pid.control) * 0.01)
  assert output < 0.01
  # Brake/disabled LONG cannot resume or advance its mode owner on their own.
  mode_time = controller.extension.bolt_mode.timer
  assert controller.update(False, state, 0.668, False, (-4.0, 1.4), context=LongitudinalContext(experimental_mode=True)) == 0.0
  assert controller.extension.bolt_mode.timer == mode_time


@pytest.mark.parametrize('c9', (False, True))
@pytest.mark.parametrize('fault', ('stock', 'passive', 'dashcam', 'not_car', 'wrong_word', 'wrong_identity', 'missing_sascm'))
def test_sdgm_mode_policy_never_admits_unqualified_owner(c9, fault):
  cp = sdgm(alpha=fault != 'stock', c9=c9)
  if fault == 'passive':
    cp.passive = True
  if fault == 'dashcam':
    cp.dashcamOnly = True
  if fault == 'not_car':
    cp.notCar = True
  if fault == 'wrong_word':
    cp.safetyConfigs[0].safetyParam = 0xFFFF
  if fault == 'wrong_identity':
    cp.carFingerprint = 'UNQUALIFIED_VOLT'
  if fault == 'missing_sascm':
    fp = gen_empty_fingerprint()
    fp[0][0xBE] = 6
    cp = CarInterface.get_params(CAR.CHEVROLET_VOLT_2019, fp, [], True, False, False)
  assert policy_for(cp) is None


@pytest.mark.parametrize('c9', (False, True))
def test_actual_controls_uses_only_fresh_sdgm_mode_telemetry(c9, monkeypatch):
  from openpilot.starpilot.longitudinal.tests import test_bolt_mode_transition as fixture

  case = fixture.TestBoltModeTransition(methodName='test_actual_controls_transports_only_healthy_mode_state')
  cp = sdgm(c9=c9)
  monkeypatch.setattr(fixture, 'params', lambda *_args, **_kwargs: cp)
  try:
    case.test_actual_controls_transports_only_healthy_mode_state()
  finally:
    case.doCleanups()


@pytest.mark.parametrize('identity', (CAR.CHEVROLET_VOLT, CAR.CHEVROLET_VOLT_ASCM, CAR.CHEVROLET_VOLT_CAMERA, CAR.CHEVROLET_VOLT_2019))
@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('c9', (False, True))
def test_each_volt_factory_keeps_its_own_mode_selection(identity, alpha, c9):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  fp[1][0x460] = 8
  if identity == CAR.CHEVROLET_VOLT_CAMERA:
    fp[2][0x320] = 6
  if not c9:
    fp[0][0xBE] = 6
  cp = CarInterface.get_params(identity, fp, [], alpha, False, False)
  # Gateway owns ordinary OP LONG; the other three require explicit Alpha.
  expected = identity == CAR.CHEVROLET_VOLT or alpha
  assert cp.openpilotLongitudinalControl == expected
  assert (policy_for(cp) is not None) == expected
  if expected:
    control = LongControl(cp)
    assert control.extension.bolt_mode is not None
    cs = physical()
    control.update(True, cs, -0.5, False, (-4.0, 1.4), context=LongitudinalContext(experimental_mode=True))
    assert control.extension.bolt_mode.current_mode == 'blended'
    corrupt = cp.as_reader().as_builder()
    corrupt.safetyConfigs[0].safetyParam = 0xFFFF
    assert policy_for(corrupt) is None


@pytest.mark.parametrize('topology', ('camera', 'gateway', 'ascm', 'sdgm'))
@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('be', (False, True))
@pytest.mark.parametrize('release', (False, True))
def test_pedal_volt_siblings_keep_stock_and_release_ownership(topology, alpha, be, release):
  from opendbc.car.gm.tests import test_volt_transitions as factory

  build = getattr(factory, 'volt_' + topology + '_pedal_params')
  options = {'alpha': alpha, 'be': be, 'release': release}
  if topology in ('ascm', 'sdgm'):
    options['sascm'] = True
  cp = build(**options)
  # Actual factory admission owns this distinction. A pedal observation does
  # not itself permit LONG, particularly in stock/release configurations.
  admitted = cp.openpilotLongitudinalControl and not cp.dashcamOnly and not cp.passive
  mode = policy_for(cp)
  assert (mode is not None) == admitted, (topology, alpha, be, release, cp.to_dict())
  if mode is not None:
    controller = LongControl(cp)
    controller.update(False, physical(), 1.0, False, (-4.0, 1.4), context=LongitudinalContext(experimental_mode=True))
    assert controller.last_output_accel == 0.0
    assert not controller.extension.bolt_mode.transitioning
