"""Initial integrity epoch zero is valid; ownership/deadlines remain unchanged."""
import pytest

from openpilot.starpilot.aol.intent_companion import IntentCompanion
from openpilot.starpilot.tests import test_ford_reverse_intent as reverse
from openpilot.starpilot.tests import test_ford_alpha_reverse_intent as alpha_reverse
from openpilot.starpilot.tests import test_ford_physical_pause_intent as physical
from openpilot.starpilot.tests import test_ford_typed_gas_intent as gas
from openpilot.starpilot.tests import test_ford_command_pause as command
from openpilot.starpilot.tests import ford_alpha_test_support as alpha_support
from openpilot.starpilot.car.ford.reverse_intent import retain_reverse_intent
from openpilot.starpilot.car.ford.physical_pause import retain_physical_pause_intent
from openpilot.starpilot.car.ford.command_pause import retain_command_pause_intent
from openpilot.starpilot.car.ford.gas_override import retained_gas_lateral_native

CASES = ['stock-reverse','alpha-reverse','command','typed-gas',*physical.REASONS]


def initial_epoch_receipts(monkeypatch):
  epoch = IntentCompanion(None).integrity_epoch
  assert epoch == 0
  original = reverse.sample
  def zero(*args, **kwargs):
    peers, native = original(*args, **kwargs)
    reverse.change(peers, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch', epoch))
    return peers, native
  for module in (reverse, physical, alpha_support, command):
    monkeypatch.setattr(module, 'sample', zero)
  return zero


def setup_case(case, monkeypatch):
  zero = initial_epoch_receipts(monkeypatch)
  now = reverse.NOW + 20_000_000
  if case == 'stock-reverse':
    cp, cs, feedback = reverse.baseline()
    cs.gearShifter = 'reverse'
    peers, _ = zero(cp, reverse.NOW + 10_000_000, now, 2, allowed=False)
    function = retain_reverse_intent
  elif case == 'alpha-reverse':
    cp, cs, feedback, peers, _ = alpha_reverse.setup()
    function = retain_reverse_intent
  elif case == 'command':
    cp, cs, feedback, peers, _ = command.pause()
    now = reverse.NOW + 40_000_000
    function = retain_command_pause_intent
  elif case == 'typed-gas':
    cp, cs, feedback, peers, _ = gas.setup()
    function = retained_gas_lateral_native
  else:
    cp, cs, feedback, peers, _ = physical.setup(case)
    function = retain_physical_pause_intent
  assert feedback.availability_epoch == feedback.signature[15] == peers['aolAxisState'].intentIntegrityEpoch == 0
  return cp, cs, feedback, peers, now, function


@pytest.mark.parametrize('case', CASES)
def test_valid_initial_consumer_epoch_preserves_only_qualified_existing_intent(case, monkeypatch):
  cp, cs, feedback, peers, now, function = setup_case(case, monkeypatch)
  before = dict(vars(feedback))
  result = function(feedback, peers, cp, cs, peers['aolAxisState'], now, latched=True, producer=reverse.PRODUCER)
  assert result is not None and result is not False
  assert vars(feedback) == before


@pytest.mark.parametrize('case', CASES)
@pytest.mark.parametrize('fault',['no-baseline','different-current','different-previous','producer','expired'])
def test_initial_zero_cannot_replace_missing_or_changed_owner_or_extend_deadline(case, fault, monkeypatch):
  cp, cs, feedback, peers, now, function = setup_case(case, monkeypatch)
  producer = reverse.PRODUCER
  if fault == 'no-baseline':
    feedback.availability_epoch = None
  elif fault == 'different-current':
    reverse.change(peers, 'aolAxisState', lambda axis: setattr(axis, 'intentIntegrityEpoch', 1))
  elif fault == 'different-previous':
    previous = list(feedback.signature)
    previous[15] = 1
    feedback.signature = tuple(previous)
  elif fault == 'producer':
    producer = 'different-card'
  else:
    now += 200_000_001
  before = dict(vars(feedback))
  result = function(feedback, peers, cp, cs, peers['aolAxisState'], now, latched=True, producer=producer)
  assert result is None or result is False
  assert vars(feedback) == before
