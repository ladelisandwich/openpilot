from openpilot.selfdrive.ui.lib.prime_state import PrimeState, PrimeType


class _Params:
  def __init__(self):
    self.store = {}

  def get(self, key):
    return self.store.get(key)

  def put(self, key, value):
    self.store[key] = value


def _state(prime_type):
  state = PrimeState.__new__(PrimeState)
  import threading
  state._lock = threading.Lock()
  state._params = _Params()
  state.prime_type = prime_type
  return state


def test_only_comma_prime_data_plans_count_as_comma_sim():
  # the hotspot may share the cellular connection on every other state, including a device that is not
  # paired or whose prime status has not been fetched yet
  comma_sims = {PrimeType.MAGENTA, PrimeType.BLUE, PrimeType.MAGENTA_NEW, PrimeType.PURPLE}
  for prime_type in PrimeType:
    assert _state(prime_type).uses_comma_sim() is (prime_type in comma_sims), prime_type


def test_unpaired_and_unknown_are_not_prime_and_not_paired():
  assert _state(PrimeType.UNPAIRED).is_paired() is False
  assert _state(PrimeType.UNKNOWN).is_prime() is False
  assert _state(PrimeType.NONE).is_paired() is True
