"""New fixed and owner-slot discovery contexts; real matcher and public get_car."""
import pytest
from opendbc.car.volkswagen.tests import test_mqb_aol_discovery as discovery


@pytest.mark.parametrize('manual', (False, True))
@pytest.mark.parametrize('gateway,alpha', ((False, False), (True, False), (True, True)))
@pytest.mark.parametrize('offset,fixed', ((0, True), (4, False), (4, True)))
def test_topology_fixed_discovery(manual, gateway, alpha, offset, fixed):
  discovery.test_normal_golf_discovery_and_card_stock_aol_owner(manual, gateway, alpha, offset, fixed)
