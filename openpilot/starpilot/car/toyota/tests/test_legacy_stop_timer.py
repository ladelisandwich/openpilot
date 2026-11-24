import itertools

from opendbc.car import gen_empty_fingerprint
from opendbc.car.toyota import legacy_stop_timer
from opendbc.car.toyota.fingerprints import FW_VERSIONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware


def test_factory_owner_scope_and_full_cp_unchanged():
  assert not legacy_stop_timer.enabled(None)
  count = 0
  for car in CAR:
    for address, dlc in ((0x2FF, 8), (0x2FF, 4), (0x2AA, 8), (None, 0)):
      fp = gen_empty_fingerprint()
      if address is not None:
        fp[0][address] = dlc
      cp = CarInterface.get_params(car, fp, firmware(car) if car in FW_VERSIONS else [], True, False, False)
      original = cp.to_dict()
      active = legacy_stop_timer.enabled(cp)
      if car not in legacy_stop_timer.CARS or not cp.openpilotLongitudinalControl:
        assert not active
      if active:
        count += 1
        assert car in legacy_stop_timer.CARS and cp.minEnableSpeed < 0
      assert cp.to_dict() == original
  assert count == 16  # Seven SDSU, seven DLC4, Camry/C-HR 2AA.


def test_dom_latch_transition_oracle_and_inactive_neutral_veto():
  for previous_stop, previous_request, stopped, pcm, resume, active in itertools.product(
      (False, True), (False, True), (False, True), (0, 7, 8), (False, True), (False, True)):
    expected = previous_request
    if stopped and not previous_stop:
      expected = True
    if pcm != 8 or resume:
      expected = False
    if not active and stopped:
      expected = True
    assert legacy_stop_timer.request(previous_stop, previous_request, stopped, pcm, resume, active) == expected
