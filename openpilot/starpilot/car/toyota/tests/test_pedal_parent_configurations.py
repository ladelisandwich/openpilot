"""Every scoped Dom pedal/parent combination retains the exact current parent tune."""
import pytest
from opendbc.car import structs
from opendbc.car.toyota import dsu_bypass, pedal_composite
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.starpilot.car.toyota.aol import qualified
from openpilot.starpilot.car.toyota.tests.test_dsu_bypass import acquired, fingerprint as bypass_fingerprint

ROWS = [
  ('LEXUS_NX', '2FF8', 'catalog'),
  ('LEXUS_RX', '2FF8', 'catalog'),
  ('TOYOTA_AVALON', '2FF8', 'catalog'),
  ('TOYOTA_HIGHLANDER', '2FF8', 'catalog'),
  ('TOYOTA_RAV4H', '2FF8', 'catalog'),
  ('TOYOTA_SIENNA', '2FF8', 'catalog'),
  ('LEXUS_ES_TSS2', '2FF8', 'catalog'),
  ('LEXUS_IS_TSS2', '2FF8', 'catalog'),
  ('LEXUS_LC_TSS2', '2FF8', 'catalog'),
  ('LEXUS_LS', '2FF8', 'catalog'),
  ('LEXUS_NX_TSS2', '2FF8', 'catalog'),
  ('LEXUS_RC_TSS2', '2FF8', 'catalog'),
  ('LEXUS_RX_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_ALPHARD_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_AVALON_2019', '2FF8', 'catalog'),
  ('TOYOTA_AVALON_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_CAMRY', '2FF8', 'catalog'),
  ('TOYOTA_CAMRY_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_CHR', '2FF8', 'catalog'),
  ('TOYOTA_CHR_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_COROLLA_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_HIGHLANDER_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_MIRAI', '2FF8', 'catalog'),
  ('TOYOTA_PRIUS_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_RAV4_TSS2', '2FF8', 'catalog'),
  ('TOYOTA_RAV4_TSS2_2022', '2FF8', 'catalog'),
  ('TOYOTA_RAV4_TSS2_2023', '2FF8', 'catalog'),
  ('LEXUS_CTH', '2FF8', 'catalog'),
  ('LEXUS_ES', '2FF8', 'catalog'),
  ('LEXUS_GS_F', '2FF8', 'catalog'),
  ('LEXUS_IS', '2FF8', 'catalog'),
  ('LEXUS_RC', '2FF8', 'catalog'),
  ('TOYOTA_COROLLA', '2FF8', 'catalog'),
  ('TOYOTA_MATRIX_RETROFIT', '2FF8', 'catalog'),
  ('TOYOTA_PRIUS_V', '2FF8', 'catalog'),
  ('TOYOTA_RAV4', '2FF8', 'catalog'),
  ('TOYOTA_PRIUS', '2FF8', 'catalog'),
  ('LEXUS_CTH', '2FF4', 'catalog'),
  ('LEXUS_ES', '2FF4', 'catalog'),
  ('LEXUS_GS_F', '2FF4', 'catalog'),
  ('LEXUS_IS', '2FF4', 'catalog'),
  ('LEXUS_NX', '2FF4', 'catalog'),
  ('LEXUS_RC', '2FF4', 'catalog'),
  ('LEXUS_RX', '2FF4', 'catalog'),
  ('TOYOTA_AVALON', '2FF4', 'catalog'),
  ('TOYOTA_COROLLA', '2FF4', 'catalog'),
  ('TOYOTA_HIGHLANDER', '2FF4', 'catalog'),
  ('TOYOTA_MATRIX_RETROFIT', '2FF4', 'catalog'),
  ('TOYOTA_PRIUS_V', '2FF4', 'catalog'),
  ('TOYOTA_RAV4', '2FF4', 'catalog'),
  ('TOYOTA_RAV4H', '2FF4', 'catalog'),
  ('TOYOTA_SIENNA', '2FF4', 'catalog'),
  ('LEXUS_ES_TSS2', '2FF4', 'catalog'),
  ('LEXUS_IS_TSS2', '2FF4', 'catalog'),
  ('LEXUS_LC_TSS2', '2FF4', 'catalog'),
  ('LEXUS_LS', '2FF4', 'catalog'),
  ('LEXUS_NX_TSS2', '2FF4', 'catalog'),
  ('LEXUS_RC_TSS2', '2FF4', 'catalog'),
  ('LEXUS_RX_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_ALPHARD_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_AVALON_2019', '2FF4', 'catalog'),
  ('TOYOTA_AVALON_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_CAMRY_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_COROLLA_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_HIGHLANDER_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_MIRAI', '2FF4', 'catalog'),
  ('TOYOTA_PRIUS_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_RAV4_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_CAMRY', '2FF4', 'catalog'),
  ('TOYOTA_CHR', '2FF4', 'catalog'),
  ('TOYOTA_CHR_TSS2', '2FF4', 'catalog'),
  ('TOYOTA_RAV4_TSS2_2022', '2FF4', 'catalog'),
  ('TOYOTA_RAV4_TSS2_2023', '2FF4', 'catalog'),
  ('TOYOTA_PRIUS', '2FF4', 'catalog'),
  ('LEXUS_ES_TSS2', '2AA8', 'catalog'),
  ('LEXUS_IS_TSS2', '2AA8', 'catalog'),
  ('LEXUS_LC_TSS2', '2AA8', 'catalog'),
  ('LEXUS_NX_TSS2', '2AA8', 'catalog'),
  ('LEXUS_RC_TSS2', '2AA8', 'catalog'),
  ('LEXUS_RX_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_ALPHARD_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_AVALON_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_CAMRY_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_COROLLA_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_HIGHLANDER_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_MIRAI', '2AA8', 'catalog'),
  ('TOYOTA_PRIUS_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_RAV4_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_CAMRY', '2AA8', 'catalog'),
  ('TOYOTA_CHR', '2AA8', 'catalog'),
  ('TOYOTA_CHR_TSS2', '2AA8', 'catalog'),
  ('TOYOTA_RAV4_TSS2_2022', '2AA8', 'catalog'),
  ('TOYOTA_RAV4_TSS2_2023', '2AA8', 'catalog'),
  ('TOYOTA_PRIUS', '2FF8', 'EPS73_8965B47070'),
  ('TOYOTA_PRIUS_RETROFIT', '2FF8', 'EPS73_8965B47070'),
  ('TOYOTA_PRIUS', '2FF4', 'EPS73_8965B47070'),
  ('TOYOTA_PRIUS_RETROFIT', '2FF4', 'EPS73_8965B47070'),
  ('LEXUS_CTH', 'camera343_or_4CB8', 'catalog'),
  ('LEXUS_ES', 'camera343_or_4CB8', 'catalog'),
  ('LEXUS_GS_F', 'camera343_or_4CB8', 'catalog'),
  ('LEXUS_IS', 'camera343_or_4CB8', 'catalog'),
  ('LEXUS_LS', 'camera343_or_4CB8', 'catalog'),
  ('LEXUS_NX', 'camera343_or_4CB8', 'catalog'),
  ('LEXUS_RC', 'camera343_or_4CB8', 'catalog'),
  ('LEXUS_RX', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_AVALON', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_AVALON_2019', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_CAMRY', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_CHR', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_COROLLA', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_HIGHLANDER', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_MATRIX_RETROFIT', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_PRIUS', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_PRIUS_RETROFIT', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_PRIUS_V', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_RAV4', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_RAV4H', 'camera343_or_4CB8', 'catalog'),
  ('TOYOTA_SIENNA', 'camera343_or_4CB8', 'catalog'),
]


def sources(identity, topology, rack):
  fp = bypass_fingerprint(identity) if topology == 'camera343_or_4CB8' else {0: {0x260: 8, 0x262: 8, 0x1D2: 8, 0x1D3: 8, 0x224: 8, 0x226: 8}, 1: {}, 2: {}}
  if topology in ('2FF8', '2FF4', '2AA8'):
    fp[0][0x2AA if topology == '2AA8' else 0x2FF] = 4 if topology == '2FF4' else 8
  if identity in dsu_bypass.PROFILES:
    fp[0][0x262] = 8 if identity in dsu_bypass.EPS8_CARS or identity == CAR.TOYOTA_PRIUS else 5
  if rack == 'EPS73_8965B47070':
    fp[0][0x262] = 8
  fw = acquired(identity)
  if rack == 'EPS73_8965B47070':
    for row in fw:
      if row.ecu == structs.CarParams.Ecu.eps:
        row.fwVersion = b'8965B47070' + bytes(6)
  return fp, fw


@pytest.mark.parametrize('name,topology,rack', ROWS)
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('alpha_long', (False, True))
def test_all_parent_tunes_native_projection_and_stock_restore(name, topology, rack, stock, alpha_long):
  identity = getattr(CAR, name)
  fp, fw = sources(identity, topology, rack)
  parent = CarInterface.get_params(identity, fp, fw, alpha_long, False, False)
  child_fp = {bus: dict(frames) for bus, frames in fp.items()}
  child_fp[0][0x201] = 6
  cp = CarInterface.get_params(identity, child_fp, fw, alpha_long, False, False)
  assert pedal_composite.configured(cp) and qualified(cp)
  assert cp.flags & ToyotaFlags.GAS_INTERCEPTOR
  assert cp.minEnableSpeed == -1.
  projected = pedal_composite.parent_cp(cp)
  assert projected.safetyConfigs[0].safetyParam == parent.safetyConfigs[0].safetyParam
  for field in (
    'lateralTuning', 'longitudinalTuning', 'steerActuatorDelay', 'steerLimitTimer', 'longitudinalActuatorDelay',
    'minSteerSpeed', 'wheelSpeedFactor', 'mass', 'wheelbase', 'steerRatio', 'centerToFront', 'autoResumeSng',
  ):
    if field == 'longitudinalActuatorDelay' and cp.openpilotLongitudinalControl:
      assert cp.longitudinalActuatorDelay == pytest.approx(max(parent.longitudinalActuatorDelay, .2))
    else:
      assert cp.to_dict()[field] == parent.to_dict()[field], field
  if stock:
    assert pedal_composite.prepare_stock(cp)
    assert pedal_composite.configured(cp) and qualified(cp) and not cp.openpilotLongitudinalControl
    assert cp.safetyConfigs[0].safetyParam == (parent.safetyConfigs[0].safetyParam | 0x2200)


@pytest.mark.parametrize('name,topology,rack', ROWS)
def test_dom_low_speed_entry_requires_qualified_pedal(name, topology, rack):
  identity = getattr(CAR, name)
  fp, fw = sources(identity, topology, rack)
  base = CarInterface.get_params(identity, fp, fw, True, False, False)
  fp[0][0x201] = 6
  cp = CarInterface.get_params(identity, fp, fw, True, False, False)
  assert pedal_composite.configured(cp) and cp.minEnableSpeed == -1.
  for field in ('lateralTuning', 'longitudinalTuning', 'longitudinalActuatorDelay', 'stopAccel'):
    if field == 'longitudinalActuatorDelay' and cp.openpilotLongitudinalControl:
      assert cp.longitudinalActuatorDelay == pytest.approx(max(base.longitudinalActuatorDelay, .2))
    elif field == 'stopAccel' and cp.openpilotLongitudinalControl:
      assert cp.stopAccel == -1.5
    else:
      assert cp.to_dict()[field] == base.to_dict()[field], field
  fp[0][0x201] = 7
  rejected = CarInterface.get_params(identity, fp, fw, True, False, False)
  assert not pedal_composite.configured(rejected)
  assert not rejected.flags & ToyotaFlags.GAS_INTERCEPTOR


def test_shared_hold_car_mappings():
  from opendbc.car.toyota.interface import apply_toyota_auto_hold
  from opendbc.car.toyota.values import TOYOTA_AUTO_HOLD_CARS, uses_toyota_auto_hold_aeb
  for name, topology, rack in ROWS:
    identity = getattr(CAR, name)
    if identity not in TOYOTA_AUTO_HOLD_CARS:
      fp, fw = sources(identity, topology, rack)
      fp[0][0x201] = 6
      rejected = CarInterface.get_params(identity, fp, fw, True, False, False)
      assert not apply_toyota_auto_hold(rejected, True)
      continue
    for has_pedal in (False, True):
      fp, fw = sources(identity, topology, rack)
      if has_pedal:
        fp[0][0x201] = 6
      cp = CarInterface.get_params(identity, fp, fw, True, False, False)
      original = cp.to_dict()
      assert apply_toyota_auto_hold(cp, True), (name, topology, has_pedal)
      permission = 256 if uses_toyota_auto_hold_aeb(cp) else 128
      assert cp.alternativeExperience & (128 | 256) == permission
      assert qualified(cp)
      for key in ('lateralTuning', 'longitudinalTuning', 'longitudinalActuatorDelay', 'stopAccel'):
        assert cp.to_dict()[key] == original[key]
      assert not apply_toyota_auto_hold(cp, False)
      assert not cp.flags & ToyotaFlags.AUTO_BRAKE_HOLD

  from opendbc.car.toyota.pedal import CAMERA_PEDAL_CARS
  for identity in CAMERA_PEDAL_CARS & TOYOTA_AUTO_HOLD_CARS:
    fp, fw = sources(identity, 'standalone2016', 'catalog')
    fp[0][0x201] = 6
    cp = CarInterface.get_params(identity, fp, fw, True, False, False)
    assert apply_toyota_auto_hold(cp, True), identity
    assert qualified(cp)
