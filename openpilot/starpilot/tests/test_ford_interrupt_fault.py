"""The fault recorded on the reported Mach-E route remains blocking."""
from dataclasses import replace

from openpilot.starpilot.tests.test_ford_typed_gas_intent import setup
from openpilot.starpilot.tests.ford_alpha_test_support import NOW, PRODUCER, change, native_change
from openpilot.starpilot.car.ford.gas_override import retained_gas_lateral_native


def test_reported_interrupt_fault_cannot_be_classified_as_retained_gas_intent():
  cp, cs, feedback, sm, native = setup()
  assert retained_gas_lateral_native(feedback, sm, cp, cs, sm['aolAxisState'], NOW + 20_000_000,
                                   latched=True, producer=PRODUCER) is not None
  native_change(sm, native, pandaInventory=(replace(native.pandaInventory[0], faults=8),))
  change(sm, 'pandaStates', lambda states: setattr(states[0], 'faults', ['interruptRateCan2']))
  before = dict(vars(feedback))
  assert retained_gas_lateral_native(feedback, sm, cp, cs, sm['aolAxisState'], NOW + 20_000_000,
                                   latched=True, producer=PRODUCER) is None
  assert vars(feedback) == before
