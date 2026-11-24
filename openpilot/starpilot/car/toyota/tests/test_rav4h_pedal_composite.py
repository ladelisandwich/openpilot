"""Exact independent RAV4H smartDSU/pedal factory contract."""

import pytest
from opendbc.car.toyota import pedal, pedal_composite, rav4h_filter
from opendbc.car.toyota.interface import CarInterface, apply_toyota_auto_hold
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.aol import qualified
from openpilot.starpilot.car.toyota.tests.test_rav4h_caller_native import firmware


def factory(alpha=False, fp=None):
  return CarInterface.get_params(
    CAR.TOYOTA_RAV4H, {0: {0x2FF: 8, 0x201: 6}, 1: {}, 2: {}} if fp is None else fp, firmware(CAR.TOYOTA_RAV4H), alpha, False, False
  )


@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('stock', (False, True))
def test_parent_configuration_tunes_and_stock_pair_are_exact(alpha, stock):
  parent = factory(alpha, {0: {0x2FF: 8}, 1: {}, 2: {}})
  cp = factory(alpha)
  assert pedal_composite.configured(cp) and pedal.configured(cp) and rav4h_filter.configured(cp) and qualified(cp)
  assert cp.safetyConfigs[0].safetyParam == 0xB049
  expected = parent.to_dict()
  expected['longitudinalActuatorDelay'] = pytest.approx(.2)
  expected['stopAccel'] = -1.5
  assert pedal_composite.parent_cp(cp).to_dict() == expected
  if stock:
    assert rav4h_filter.prepare_stock(cp) and rav4h_filter.prepare_stock(parent)
    assert cp.safetyConfigs[0].safetyParam == 0xB249
    expected = parent.to_dict()
    expected['longitudinalActuatorDelay'] = pytest.approx(.2)
    expected['stopAccel'] = -1.5
    assert pedal_composite.parent_cp(cp).to_dict() == expected
    assert pedal_composite.configured(cp) and qualified(cp)


@pytest.mark.parametrize(
  'fp',
  (
    {0: {0x2FF: 8, 0x201: 5}},
    {0: {0x2FF: 8}, 2: {0x201: 6}},
    {0: {0x2FF: 8, 0x201: 6}, 1: {0x201: 6}},
    {0: {0x2FF: 7, 0x201: 6}},
    {0: {0x2FF: 8, 0x201: 6, 0x2AA: 8}},
    {0: {0x2FF: 8, 0x201: 6}, 2: {0x2FF: 4}},
    {0: {0x201: 6}, 2: {0x2FF: 8}},
  ),
)
def test_unqualified_topology_is_not_composite(fp):
  assert not pedal_composite.configured(factory(fp=fp))


@pytest.mark.parametrize(
  'field,value',
  (
    ('brand', 'ford'),
    ('notCar', True),
    ('passive', True),
    ('dashcamOnly', True),
    ('pcmCruise', False),
    ('secOcRequired', True),
    ('carFingerprint', CAR.TOYOTA_RAV4_TSS2),
    ('flags', 0),
  ),
)
def test_foreign_parameter_variants_are_rejected(field, value):
  cp = factory()
  setattr(cp, field, value)
  assert not pedal_composite.configured(cp)


@pytest.mark.parametrize('stock', (False, True))
def test_existing_rav4h_hold_admission_uses_parent_and_stock_withdrawal(stock):
  cp = factory()
  assert apply_toyota_auto_hold(cp, True)
  assert pedal_composite.configured(cp) and qualified(cp)
  if stock:
    assert pedal.prepare_stock(cp)
    assert not cp.flags & ToyotaFlags.AUTO_BRAKE_HOLD and not cp.alternativeExperience & 128
    assert not cp.openpilotLongitudinalControl and pedal_composite.configured(cp)
