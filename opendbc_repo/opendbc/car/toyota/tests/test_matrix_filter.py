"""Manual Matrix filter enrollment and unchanged factory values."""
import struct

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.toyota import legacy_advert_filter, legacy_sdsu, matrix_filter
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR


def factory(fingerprint, firmware=(), alpha=False):
  return CarInterface.get_params(CAR.TOYOTA_MATRIX_RETROFIT, fingerprint, list(firmware), alpha, False, False)


@pytest.mark.parametrize('length', (4, 8))
@pytest.mark.parametrize('alpha', (False, True))
@pytest.mark.parametrize('stock', (False, True))
def test_manual_matrix_filter_factory_preserves_tunes(length, alpha, stock):
  baseline = factory(gen_empty_fingerprint(), alpha=alpha)
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x2FF] = length
  cp = factory(fingerprint, alpha=alpha)
  assert not cp.dashcamOnly, 'MANUAL_MATRIX_FILTER_NOT_ADMITTED'
  assert cp.openpilotLongitudinalControl
  assert matrix_filter.configured(cp, length=length)
  if stock:
    owner = legacy_sdsu if length == 8 else legacy_advert_filter
    assert owner.prepare_stock(cp)
  assert cp.safetyConfigs[0].safetyParam == matrix_filter.PROTOCOLS[length][stock]
  assert cp.openpilotLongitudinalControl == (not stock)
  assert not cp.autoResumeSng and not cp.alphaLongitudinalAvailable
  assert cp.lateralTuning.torque.latAccelFactor == struct.unpack('<f', struct.pack('<f', 4.05))[0]
  assert cp.lateralTuning.torque.friction == struct.unpack('<f', struct.pack('<f', .1))[0]
  actual, before = cp.to_dict(), baseline.to_dict()
  for state in (actual, before):
    for field in ('flags', 'safetyConfigs', 'openpilotLongitudinalControl'):
      state.pop(field)
  assert actual == before, 'Filter enrollment must preserve all other existing Matrix configuration and tunes'


@pytest.mark.parametrize('length', (4, 8))
@pytest.mark.parametrize('mutation', ('wrong-primary-length', 'wrong-mirror-length', 'foreign-bus',
                                    'pedal-pt', 'pedal-cam', 'camera-filter-pt', 'camera-filter-cam',
                                    'wrong-eps-length', 'wrong-main-length', 'wrong-camera-acc-length'))
def test_manual_matrix_filter_rejects_other_topologies(length, mutation):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x2FF] = length
  if mutation == 'wrong-primary-length':
    fingerprint[0][0x2FF] = 7
  elif mutation == 'wrong-mirror-length':
    fingerprint[2][0x2FF] = 8 if length == 4 else 4
  elif mutation == 'foreign-bus':
    fingerprint[1][0x2FF] = length
  elif mutation.startswith('pedal-'):
    fingerprint[0 if mutation.endswith('pt') else 2][0x201] = 5
  elif mutation.startswith('camera-filter-'):
    fingerprint[0 if mutation.endswith('pt') else 2][0x2AA] = 8
  elif mutation == 'wrong-eps-length':
    fingerprint[0][0x262] = 8
  elif mutation == 'wrong-main-length':
    fingerprint[0][0x1D3] = 7
  else:
    fingerprint[2][0x343] = 7
  assert not matrix_filter.hardware_supported(CAR.TOYOTA_MATRIX_RETROFIT, fingerprint, [], length=length)
  cp = factory(fingerprint)
  assert cp.dashcamOnly and not cp.openpilotLongitudinalControl
  assert not matrix_filter.configured(cp, length=length)


@pytest.mark.parametrize('length', (4, 8))
@pytest.mark.parametrize('field,value', (('logging', True), ('bus', 1), ('brand', 'gm'), ('fwVersion', b'')))
def test_manual_matrix_filter_rejects_invalid_acquired_firmware_metadata(length, field, value):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x2FF] = length
  fw = structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.eps, address=0x7A1,
                              fwVersion=b'negative-metadata-fixture', brand='toyota', bus=0)
  setattr(fw, field, value)
  assert not matrix_filter.hardware_supported(CAR.TOYOTA_MATRIX_RETROFIT, fingerprint, [fw], length=length)
  cp = factory(fingerprint, [fw])
  assert cp.dashcamOnly and not cp.openpilotLongitudinalControl
