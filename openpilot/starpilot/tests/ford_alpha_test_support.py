from dataclasses import replace

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.tests.test_ford_reverse_intent import NOW, PRODUCER, change, native_change, sample


def receipt(cp, source, observed, sequence, *, combined=False, gas=False, reverse=False):
  sm, native = sample(cp, source, observed, sequence, allowed=not reverse)
  controls = bool(combined or gas)
  native = native_change(
    sm,
    native,
    requestedLongitudinal=combined or gas,
    longitudinalAllowed=combined,
    pandaInventory=(replace(native.pandaInventory[0], controlsAllowed=controls),),
  )
  change(sm, 'pandaStates', lambda ps: setattr(ps[0], 'controlsAllowed', controls))
  change(sm, 'aolAxisState', lambda a: setattr(a, 'mode', 'combined' if combined else 'lateralOnly'))
  change(sm, 'aolAxisState', lambda a: setattr(a, 'desiredLongitudinal', combined or gas))
  change(sm, 'aolAxisState', lambda a: setattr(a, 'longitudinalActive', combined))
  if combined or gas:
    events = messaging.new_message('onroadEvents', int(gas), valid=True, logMonoTime=NOW + 10_000_000 if gas else NOW)
    if gas:
      events.onroadEvents[0].name = 'gasPressedOverride'
      events.onroadEvents[0].overrideLongitudinal = True
    sm.messages['onroadEvents'] = events.as_reader()
    sm.logMonoTime['onroadEvents'] = int(events.logMonoTime)
    change(sm, 'aolAxisState', lambda a: setattr(a, 'faultEventMonoTime', int(events.logMonoTime)))
  return sm, native


def baseline(*, combined):
  fp = gen_empty_fingerprint()
  fp[0][0x5A] = 8
  fp[2].update({0x3D6: 8, 0x186: 8})
  with OpenpilotPrefix():
    cp = CarInterface.get_params(CAR.FORD_MONDEO_MK5, fp, [], True, False, False)
  cp.alternativeExperience = 32
  assert cp.openpilotLongitudinalControl and cp.safetyConfigs[0].safetyParam == 11
  cs = structs.CarState(canValid=True, gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available, cs.cruiseState.enabled = True, combined
  f = TransportPauseFeedback(cp)
  sm, _ = receipt(cp, NOW, NOW, 1, combined=combined)
  assert not f.observe(sm, cp, cs, NOW, latched=True, intent_producer_session=PRODUCER)
  return cp, cs, f
