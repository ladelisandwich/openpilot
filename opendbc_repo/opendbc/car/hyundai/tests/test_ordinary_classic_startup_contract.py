import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.classic_long_aol import CLASSIC_LONG_IDS, qualified
from opendbc.car.hyundai.ecu_startup import stock_copy
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR


def factory_cases():
  cases = []
  for identity in sorted(CLASSIC_LONG_IDS):
    for lda in (False, True):
      fp = gen_empty_fingerprint()
      if lda:
        fp[0][0x391] = 8
      requested = CarInterface.get_params(identity, fp, [], True, False, False)
      if qualified(requested):
        stock = CarInterface.get_params(identity, fp, [], False, False, False)
        cases.append(pytest.param(requested, stock, id=f'{identity}-lda{int(lda)}'))
  assert cases
  return cases


@pytest.mark.parametrize('requested,stock', factory_cases())
def test_current_classic_factory_stock_fallback_changes_no_tunes_or_other_fields(requested, stock):
  before = requested.to_dict()
  restored = stock_copy(requested)
  assert restored.to_dict() == stock.to_dict()
  assert requested.to_dict() == before
  assert not restored.openpilotLongitudinalControl and restored.pcmCruise


@pytest.mark.parametrize('identity', [CAR.HYUNDAI_SONATA, CAR.HYUNDAI_SONATA_HYBRID])
def test_ordinary_classic_requested_takeover_requires_prepublication_owner(identity):
  fp = gen_empty_fingerprint()
  fp[0][0x391] = 8
  cp = CarInterface.get_params(identity, fp, [], True, False, False)
  assert qualified(cp)
  sent = []
  owner = CarInterface.startup_owner(cp, (list, sent.extend), requested=True)
  assert not sent
  assert owner is not None, 'ORDINARY_CLASSIC_TAKEOVER_HAS_NO_PREPUBLICATION_OWNER'


@pytest.mark.parametrize('requested,stock', factory_cases())
def test_actual_factory_selects_owner_before_any_transmission(requested, stock):
  sent = []
  owner = CarInterface.startup_owner(requested, (list, sent.extend), requested=True)
  assert owner is not None
  assert not sent and not owner.published
  assert owner.cp.to_dict() == requested.to_dict()
