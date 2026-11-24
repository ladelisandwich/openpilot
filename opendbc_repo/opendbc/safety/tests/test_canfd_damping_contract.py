"""Each Dom camera damping calibration has one exact native payload contract."""
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai.hyundaicanfd import create_steering_messages
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.safety.tests.libsafety import libsafety_py


@pytest.mark.parametrize('car,gas,damping', (
  (CAR.HYUNDAI_IONIQ_5, 1, 0), (CAR.HYUNDAI_KONA_EV_2ND_GEN, 1, 0),
  (CAR.GENESIS_GV60_EV_1ST_GEN, 1, 0), (CAR.HYUNDAI_TUCSON_4TH_GEN, 0, 0),
  (CAR.KIA_CARNIVAL_4TH_GEN, 0, 100), (CAR.KIA_CARNIVAL_2025, 0, 100),
  (CAR.KIA_CARNIVAL_HEV_4TH_GEN, 2, 100), (CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, 1, 100),
))
@pytest.mark.parametrize('alternate', (False, True))
def test_one_exact_damping_calibration_per_profile(car, gas, damping, alternate):
  safety = libsafety_py.libsafety
  if safety.set_safety_hooks(structs.CarParams.SafetyModel.allOutput, 0) != 0:
    pytest.skip('LONG actuation remains disabled in RELEASE')
  word = gas | 4 | 16 | (128 if alternate else 0) | (8192 if damping else 0)
  cp = SimpleNamespace(carFingerprint=car, flags=int(HyundaiFlags.CANFD_LKA_STEER_MSG |
                       (HyundaiFlags.CANFD_LKA_STEER_MSG_ALT if alternate else 0)), openpilotLongitudinalControl=True)
  bus = SimpleNamespace(ACAN=0, ECAN=1)
  packer = CANPacker('hyundai_canfd_generated')
  frames = create_steering_messages(packer, cp, bus, False, False, 0)
  def tx(frame):
    return bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])))
  try:
    safety.init_tests()
    safety.set_alternative_experience(0)
    assert safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, word) == 0
    safety.set_timer(1_000_000)
    assert frames[0][1][13] == 100 and frames[1][1][8] == damping
    assert tx(frames[0]) and tx(frames[1])
    assert not tx(frames[1])  # A-CAN never gains a second independent steering authority.
    for wrong in (100 if damping == 0 else 0, 1, 99, 255):
      assert tx(frames[0])
      forged = packer.make_can_msg('LKAS_ALT' if alternate else 'LKAS', 0,
                                  {'LKA_OptUsmSta': 2, 'LKA_SysIndReq': 1,
                                   'StrTqReqVal': 0, 'ActToiSta': 0, 'Damping_Gain': wrong})
      assert not tx(forged), (car, alternate, damping, wrong)
      assert not tx(frames[1])  # A rejected attempt consumes the one-use mirror.
  finally:
    safety.init_tests()
    safety.set_alternative_experience(0)
    safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)


@pytest.mark.parametrize('car', (CAR.KIA_CARNIVAL_4TH_GEN, CAR.KIA_CARNIVAL_2025,
                               CAR.KIA_CARNIVAL_HEV_4TH_GEN, CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN))
def test_interface_selects_exception_and_stock_fallback_restores_original_word(car):
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.ecu_startup import stock_copy
  from opendbc.car.hyundai.canfd_stock_aol import qualified_long
  fp = gen_empty_fingerprint()
  fp[2].update({0x50: 16, 0x2A4: 24})
  fp[1].update({0x1CF: 8, 0x35: 32})
  firmware = [structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.adas)]
  stock = CarInterface.get_params(car, fp, firmware, False, False, False)
  cp = CarInterface.get_params(car, fp, firmware, True, False, False)
  assert cp.openpilotLongitudinalControl
  assert cp.safetyConfigs[-1].safetyParam & 0x2000
  if car == CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN:
    assert qualified_long(cp)
    cp.safetyConfigs[-1].safetyParam |= 0x800
    cp.alternativeExperience = 32
    assert qualified_long(cp, marked_only=True)
    cp.safetyConfigs[-1].safetyParam &= ~0x800
    cp.alternativeExperience = 0
  fallback = stock_copy(cp)
  assert fallback.safetyConfigs[-1].safetyParam == stock.safetyConfigs[-1].safetyParam
  assert stock_copy(stock).safetyConfigs[-1].safetyParam == stock.safetyConfigs[-1].safetyParam
