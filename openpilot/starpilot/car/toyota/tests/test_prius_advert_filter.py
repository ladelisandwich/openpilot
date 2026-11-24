"""Four-byte Prius admission is independent of eight-byte status and EPS73."""

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.toyota import prius_advert_filter as owner
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC
from openpilot.starpilot.car.toyota.tests.test_prius_sdsu_filter import original_firmware, ORIGINAL


@pytest.mark.parametrize(
  'bad',
  (
    'unknown_eps',
    'unknown_camera',
    'missing_eps',
    'missing_camera',
    'logging_eps',
    'logging_camera',
    'wrong_brand',
    'wrong_fw_bus',
    'pedal',
    'twoaa',
    'bus1',
    'mixed_mirror8',
    'bad_primary7',
    'bad_eps5',
    'bad_main7',
    'unknown_retrofit_firmware',
  ),
)
def test_original_prius_advert_refuses_unqualified_factory_sources(bad):
  fw = original_firmware(ORIGINAL[0])
  fp = gen_empty_fingerprint()
  fp[0][0x2FF] = 4
  if bad.startswith(('unknown_', 'missing_', 'logging_')):
    ecu = structs.CarParams.Ecu.eps if bad.endswith('eps') else structs.CarParams.Ecu.fwdCamera
    if bad.startswith('missing'):
      fw = [item for item in fw if item.ecu != ecu]
    else:
      for item in fw:
        if item.ecu == ecu:
          if bad.startswith('unknown'):
            item.fwVersion = b'unknown'
          else:
            item.logging = True
  elif bad in ('wrong_brand', 'wrong_fw_bus'):
    eps = next(item for item in fw if item.ecu == structs.CarParams.Ecu.eps)
    if bad == 'wrong_brand':
      eps.brand = 'other'
    else:
      eps.bus = 1
  elif bad == 'pedal':
    fp[0][0x201] = 5
  elif bad == 'twoaa':
    fp[0][0x2AA] = 8
  elif bad == 'bus1':
    fp[1][0x2FF] = 4
  elif bad == 'mixed_mirror8':
    fp[2][0x2FF] = 8
  elif bad == 'bad_primary7':
    fp[0][0x2FF] = 7
  elif bad == 'bad_eps5':
    fp[0][0x262] = 5
  elif bad == 'bad_main7':
    fp[0][0x1D3] = 7
  else:
    fw = original_firmware(b'8965B47071' + bytes(6))
  assert not owner.hardware_supported(CAR.TOYOTA_PRIUS, fp, fw)
  cp = CarInterface.get_params(CAR.TOYOTA_PRIUS, fp, fw, True, False, False)
  assert not owner.configured(cp) and cp.safetyConfigs[0].safetyParam not in owner.WORDS


@pytest.mark.parametrize('address,bus,length', ((0x201, 0, 6), (0x2AA, 0, 8), (0x2FF, 1, 4), (0x2FF, 0, 8), (0x2FF, 2, 8), (0x2FF, 0, 7)))
def test_original_prius_advert_parser_conflict_survives_source_recovery(address, bus, length):
  packer = CANPacker(DBC[CAR.TOYOTA_PRIUS][Bus.pt])
  parser = owner.PriusAdvertCANParser(DBC[CAR.TOYOTA_PRIUS][Bus.pt], [('PCM_CRUISE_2', 10), ('EPS_STATUS', 25)], 0)
  main = packer.make_can_msg('PCM_CRUISE_2', 0, {'MAIN_ON': 1, 'SET_SPEED': 90})
  eps = packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': 1})
  parser.update([(2_000_000_000, [main, eps, (0x2FF, bytes(4), 0), (0x2FF, bytes(4), 2)])])
  assert parser.main_valid and parser.eps_valid and not parser.hardware_ambiguous
  parser.update([(2_010_000_000, [(address, bytes(length), bus)])])
  assert parser.hardware_ambiguous
  invalid = (eps[0], eps[1][:-1] + bytes([eps[1][-1] ^ 1]), 0)
  parser.update([(2_020_000_000, [invalid])])
  assert parser.hardware_ambiguous and not parser.eps_valid
  parser.update([(2_030_000_000, [main, eps, (0x2FF, bytes(4), 0)])])
  assert parser.hardware_ambiguous and parser.main_valid and parser.eps_valid
  assert 0x2FF not in parser.message_states and 'SDSU' not in parser.vl
