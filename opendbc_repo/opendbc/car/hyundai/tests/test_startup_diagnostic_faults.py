"""Actual Card diagnostic gate before ECU mutation, including the recorded G90 IRQ8."""
from types import SimpleNamespace
from typing import cast
from unittest.mock import patch

import pytest

from opendbc.car import structs
from opendbc.car.hyundai.g90_startup import G90Startup, Outcome
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.starpilot.vehicle_startup import VehicleStartupOwner
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.tests.test_sonata_irq_policy import recorded as sonata_recorded


def recorded(identity='GENESIS_G90'):
  cp = structs.CarParams(brand='hyundai', carFingerprint=identity, flags=0x1001000,
    openpilotLongitudinalControl=True, pcmCruise=False, alternativeExperience=32,
    safetyConfigs=[structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.hyundai, safetyParam=0xc04)])
  if identity == 'KIA_EV9':
    cp.flags = 2281712129
    cp.alternativeExperience = 0
    cp.steerControlType = 'angle'
    cp.safetyConfigs[0].safetyModel = 'hyundaiCanfd'
    cp.safetyConfigs[0].safetyParam = 0x5c95
  return cp


def card_for(cp, faults):
  card = object.__new__(Car)
  panda = SimpleNamespace(safetyModel=structs.CarParams.SafetyModel.elm327, safetyParam=1,
    controlsAllowed=False, safetyRxChecksInvalid=False, ignitionLine=True, ignitionCan=False, faults=faults)

  class Source:
    seen = valid = alive = {'pandaStates': True}
    logMonoTime = {'pandaStates': 10_000_000_000}
    recv_time = {'pandaStates': 10.}

    def __init__(self, panda):
      self.messages = {'pandaStates': [panda]}

    def __getitem__(self, name):
      return self.messages[name]

    def update(self, timeout):
      pass

  if faults is not None:
    message = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=10_000_000_000)
    message.pandaStates[0] = dict(safetyModel='elm327', safetyParam=1, controlsAllowed=False,
      safetyRxChecksInvalid=False, ignitionLine=True, ignitionCan=False, faults=faults)
    panda = message.as_reader().pandaStates[0]
  card.sm = cast(messaging.SubMaster, Source(panda))
  card.params = cast(Params, SimpleNamespace(get_bool=lambda name: False))
  card.vehicle_startup = VehicleStartupOwner(SimpleNamespace(cp=cp))
  return card


@pytest.mark.parametrize('identity', ['GENESIS_G90', 'KIA_EV9', None])
@pytest.mark.parametrize('faults,allowed', [([], True), (['interruptRateCan2'], False),
  (['interruptRateCan1'], False), (['relayMalfunction'], False), (['registerDivergent'], False),
  (['interruptRateCan2', 'relayMalfunction'], False), (None, False)])
def test_current_fault_admission_cannot_mutate_an_unexempt_vehicle(identity, faults, allowed):
  cp = recorded(identity) if identity else None
  card = card_for(cp, faults)
  with patch('time.monotonic_ns', return_value=10_000_000_000), patch('time.clock_gettime_ns', return_value=10_000_000_000):
    assert card.startup_diagnostic_admission() is allowed


def test_recorded_g90_fault_prevents_ecu_mutation():
  cp = recorded()
  before = cp.to_dict()
  card = card_for(cp, ['interruptRateCan2'])
  sends = []
  owner = G90Startup(cp, (lambda **kwargs: [], sends.extend))
  card.vehicle_startup.owner = owner
  with patch('time.monotonic_ns', return_value=10_000_000_000), patch('time.clock_gettime_ns', return_value=10_000_000_000):
    assert not card.startup_diagnostic_admission()
    fallback = owner.prepare(admission=card.startup_diagnostic_admission)
  assert owner.outcome is Outcome.STOCK_UNTOUCHED
  assert not owner.session_attempted and not owner.disable_attempted and not sends
  assert not fallback.openpilotLongitudinalControl and fallback.pcmCruise
  assert cp.to_dict() == before
  assert [str(value) for value in card.sm['pandaStates'][0].faults] == ['interruptRateCan2']


@pytest.mark.parametrize('mixed', [False, True])
def test_existing_exact_sonata_diagnostic_classification_is_preserved(mixed):
  cp = sonata_recorded()
  before = cp.to_dict()
  faults = ['interruptRateCan2'] + (['relayMalfunction'] if mixed else [])
  card = card_for(cp, faults)
  with patch('time.monotonic_ns', return_value=10_000_000_000), patch('time.clock_gettime_ns', return_value=10_000_000_000):
    assert card.startup_diagnostic_admission() is (not mixed)
  assert cp.to_dict() == before and [str(value) for value in card.sm['pandaStates'][0].faults] == faults
