"""Dom4bfdfd0074f numerical behavior, applied after exact hardware enrollment."""
import pytest
from types import SimpleNamespace
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.toyota import dom_defaults, pedal
from opendbc.car.toyota.carcontroller import get_long_tune
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags, CarControllerParams
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


def factory(vehicle, hybrid, has_pedal=False):
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 8
  if has_pedal:
    fp[0][0x201] = 6
  fw = [row for row in firmware(vehicle) if row.ecu != structs.CarParams.Ecu.hybrid]
  if hybrid:
    fw.append(structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.hybrid))
  return CarInterface.get_params(vehicle, fp, fw, True, False, False)


@pytest.mark.parametrize('vehicle,hybrid,delay,stop_rate', (
  (CAR.TOYOTA_RAV4H, True, .05, .3), (CAR.TOYOTA_CAMRY, True, .05, .3),
  (CAR.TOYOTA_HIGHLANDER, False, .4, 1.), (CAR.TOYOTA_SIENNA, False, .5, 1.),
  (CAR.TOYOTA_PRIUS, True, .05, .3)))
@pytest.mark.parametrize('has_pedal', (False, True))
def test_dom_owned_factory_delay_and_stopping(vehicle, hybrid, delay, stop_rate, has_pedal):
  cp = factory(vehicle, hybrid, has_pedal)
  assert cp.openpilotLongitudinalControl
  assert cp.longitudinalActuatorDelay == pytest.approx(max(delay, .2) if has_pedal else delay)
  assert cp.stopAccel == (-1.5 if has_pedal else -2.)
  assert LongControl(cp).stopping_decel_rate == (.3 if has_pedal else stop_rate)
  if has_pedal:
    assert cp.flags & ToyotaFlags.GAS_INTERCEPTOR
    assert CarControllerParams(cp).ACCEL_MAX == 2.


@pytest.mark.parametrize('vehicle,hybrid,bypassed', ((CAR.TOYOTA_CAMRY, False, True),
  (CAR.TOYOTA_CAMRY, True, False), (CAR.TOYOTA_HIGHLANDER, False, True), (CAR.TOYOTA_SIENNA, False, False)))
def test_vehicle_pid_bypass_matches_dom_hardware_owner(vehicle, hybrid, bypassed):
  cp = factory(vehicle, hybrid)
  assert dom_defaults.bypass_pid(cp) == bypassed
  if hybrid:
    controller = get_long_tune(cp, CarControllerParams(cp))
    assert controller._k_i == ([2., 5.], [.5, .25])


@pytest.mark.parametrize('vehicle,expected', ((CAR.TOYOTA_RAV4, .11), (CAR.TOYOTA_COROLLA, .3), (CAR.LEXUS_ES, .4)))
def test_dom_pedal_launch_mapping_and_physical_override(vehicle, expected):
  cp = factory(vehicle, False, True)
  assert pedal.configured(cp)
  cc = structs.CarControl(longActive=True)
  state = structs.CarState(canValid=True, vEgo=0., standstill=False)
  cs = SimpleNamespace(out=state, pedal_ready=True)
  assert pedal.amount(cp, cc, cs, 1.4) == pytest.approx(expected)
  state.brakePressed = True
  assert pedal.amount(cp, cc, cs, 1.4) == 0.
  state.brakePressed = False
  cc.longActive = False
  assert pedal.amount(cp, cc, cs, 1.4) == 0.


def test_dom_pedal_stopping_keeps_real_lead_braking():
  assert dom_defaults.limit_interceptor_stopping_accel(-2., -.6, True, 0., False) == pytest.approx(-.68)
  assert dom_defaults.limit_interceptor_stopping_accel(-2., -.6, True, 0., True) == -2.
  assert dom_defaults.limit_interceptor_pcm_accel(-1., .4, False, 0.) > 0.
  assert dom_defaults.limit_interceptor_pcm_accel(1., -.4, False, 0.) < 0.
