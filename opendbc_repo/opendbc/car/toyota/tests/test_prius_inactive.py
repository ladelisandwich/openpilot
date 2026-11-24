"""Inactive commands stay neutral for the exact EPS47070 smart-DSU Prius profile.

Saved preferences, factory/CI/controller and packed CAN are real. Typed state and
producer fixtures cover the controller boundary independently of caller tests.
"""
import unittest

from opendbc.car import structs
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.tests.test_auto_hold import decode
from opendbc.car.toyota.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.car.toyota.prius_preferences import PriusFilterPreference


def make_ci(identity, params, *, standstill=False):
  rack = structs.CarParams.CarFw.new_message(ecu=structs.CarParams.Ecu.eps,
                                            fwVersion=b'8965B47070' + bytes(6), brand='toyota', address=0x7A1)
  cp = CarInterface.get_params(identity, {0: {0x2FF: 4}, 1: {}, 2: {}}, [rack], False, False, False)
  assert cp.safetyConfigs[0].safetyParam == 0xD049
  ci = CarInterface(cp)
  ci.CC.prius_filter_input = PriusFilterPreference(cp, params)
  state = structs.CarState(canValid=True, canTimeout=False, standstill=standstill,
                           vEgo=0. if standstill else 20., vEgoRaw=0. if standstill else 20.,
                           gearShifter=structs.CarState.GearShifter.drive)
  state.cruiseState.enabled = state.cruiseState.available = True
  state.cruiseState.speed = 25.
  state.cruiseState.standstill = standstill
  ci.CS.out = state
  ci.CS.pcm_acc_status = 8
  ci.CS.gvc = 0.
  return ci


def preferences(params, enabled=True):
  for key, value in (('OpenpilotEnabledToggle', True), ('SafeMode', False),
                     ('DisableOpenpilotLongitudinal', not enabled)):
    params.put_bool(key, value, block=True)


def emitted(ci, command, count=6):
  result=[]
  for tick in range(count):
    _, frames = ci.apply(command.as_reader(), 1_000_000_000 + tick * 10_000_000)
    result.extend(frames)
  return result


class TestPriusInactiveAxis(unittest.TestCase):
  def test_inactive_accel_zero_and_prev_reset(self):
    for car in (CAR.TOYOTA_PRIUS,CAR.TOYOTA_PRIUS_RETROFIT):
      for target in (1.,-1.):
        with self.subTest(car=car,target=target),OpenpilotPrefix():
          params = Params()
          preferences(params)
          ci=make_ci(car,params)
          ci.CC.prev_accel=.9
          command=structs.CarControl(enabled=True,longActive=False)
          command.actuators.accel=target
          command.actuators.longControlState=structs.CarControl.Actuators.LongControlState.pid
          frames=[f for f in emitted(ci,command) if f[0]==0x343]
          self.assertEqual(len(frames),2)
          for frame in frames:
            self.assertEqual(decode(ci.CP,frame,'ACC_CONTROL')['ACCEL_CMD'],0.,'inactive Prius acceleration must be neutral')
          self.assertEqual(ci.CC.prev_accel,0.,'inactive Prius acceleration must be neutral')

  def test_inactive_stop_resume_cannot_release(self):
    for car in (CAR.TOYOTA_PRIUS,CAR.TOYOTA_PRIUS_RETROFIT):
      for source_enabled,long_active in ((True,False),(False,True)):
        with self.subTest(car=car,source_enabled=source_enabled),OpenpilotPrefix():
          params = Params()
          preferences(params,source_enabled)
          ci=make_ci(car,params,standstill=True)
          command=structs.CarControl(enabled=True,longActive=long_active)
          command.cruiseControl.resume=True
          command.actuators.accel=1.
          frames=[f for f in emitted(ci,command) if f[0]==0x343]
          self.assertEqual(len(frames),2)
          for frame in frames:
            acc=decode(ci.CP,frame,'ACC_CONTROL')
            self.assertFalse(acc['RELEASE_STANDSTILL'],'inactive Prius must retain its standstill request')
            self.assertEqual(acc['ACCEL_CMD'],0.,'inactive Prius acceleration must be neutral')
