"""Actual selected Card startup and guarded native pedal configuration helpers."""
import hashlib
import struct
from contextlib import contextmanager
from unittest.mock import patch

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.fingerprints import FINGERPRINTS
from opendbc.car.tesla.values import CAR
from opendbc.car.tesla.preap.pedal_contract import M1, M2, D
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.controlsd import Controls


def native_words(calibration, word, cp_bytes):
  prefix = struct.pack('<HHHHff', 1, calibration.bus, word, 0xD051, calibration.factor, calibration.zero)
  return struct.unpack('<24H', prefix + hashlib.sha256(cp_bytes).digest())


def configure_native(safety, calibration, word, cp_bytes):
  safety.init_tests()
  assert safety.set_safety_hooks(0, 0) == 0
  safety.set_aol_test_heartbeat(False)
  safety.set_alternative_experience(32)
  words = native_words(calibration, word, cp_bytes)
  for i, value in enumerate(words):
    assert safety.tesla_preap_test_config_word(i, value)
  assert safety.tesla_preap_test_config_word(0xFFFF, 0xC051)
  assert safety.set_safety_hooks(39, word) == 0
  assert not safety.tesla_preap_test_config_word(0, 1), 'car-mode calibration mutation must fail'


def feedback_frame(calibration, counter, *, di=0., state=0, wrong_bus=False, bad_channel=False):
  physical = calibration.physical(di)
  one = int((physical - D) / M1)
  two = int((physical - D) / M2) + (200 if bad_channel else 0)
  data = struct.pack('>HHB', one, two, ((state & 15) << 4) | (counter & 15)) + b'\x00'
  # Byte5 feedback checksum was unvalidated/unspecified in the working Dom path.
  # Do not invent the unrelated generic pedal's CRC in this actual fixture.
  return 0x552, data, 2 - calibration.bus if wrong_bus else calibration.bus


@contextmanager
def actual_pedal_startup(bus, radar_mode, release, monkeypatch):
  from opendbc.car import car_helpers
  from openpilot.selfdrive.car import card as card_module
  from openpilot.starpilot.car.tesla.pedal_startup import KEY, decode
  for name, value in (('SIMULATION', '1'), ('REPLAY', '1'), ('AOL_REPLAY_RUNTIME', '1')):
    monkeypatch.setenv(name, value)
  fingerprint = gen_empty_fingerprint()
  fingerprint[0] = dict(FINGERPRINTS[CAR.TESLA_MODEL_S_PREAP][0])
  with OpenpilotPrefix():
    params = Params()
    for key, value in (('OpenpilotEnabledToggle', True), ('AlwaysOnLateral', False),
                       ('NAPPedalEnabled', True), ('NAPPedalCalibDone', True), ('IsReleaseBranch', release)):
      params.put_bool(key, value, block=True)
    params.put('NAPPedalCanBus', bus, block=True)
    params.put('NAPPedalCalibFactor', 1.0, block=True)
    params.put('NAPPedalCalibZero', 0.0, block=True)
    params.put('TeslaPreAPRadarConfig', {'version': 1, 'mode': radar_mode, 'offset': .25}, block=True)
    observed = (CAR.TESLA_MODEL_S_PREAP, fingerprint, '0' * 17, [], structs.CarParams.FingerprintSource.can, True)
    with patch.object(car_helpers, 'fingerprint', return_value=observed), \
         patch.object(card_module, 'can_comm_callbacks', return_value=(lambda *args: [], lambda frames: None)), \
         patch.object(card_module.messaging, 'recv_one_retry', return_value=messaging.new_message('can', 1)):
      card = Car()
    word = {'off': 1, 'front': 3, 'nosecone': 7}[radar_mode]
    assert card.CP.openpilotLongitudinalControl and not card.CP.pcmCruise
    assert card.CP.safetyConfigs[0].safetyParam == word and card.CP.alternativeExperience == 32
    assert card.CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.teslaPreap
    assert card.CP.radarUnavailable == (radar_mode == 'off')
    cp_bytes = params.get('CarParams')
    calibration = decode(card.CP, cp_bytes, params.get(KEY))
    assert calibration is not None and calibration.bus == bus
    assert card.CI.CS.preap.pedal_feedback.calibration == calibration
    assert card.CI.CC.preap.pedal_control.codec.calibration == calibration
    controls = Controls()
    assert controls.CP.to_dict() == card.CP.to_dict()
    # Later saved edits cannot change the per-drive parser/controller contract.
    params.put('NAPPedalCalibFactor', 2.0, block=True)
    params.put('NAPPedalCanBus', 2 - bus, block=True)
    assert card.CI.CS.preap.pedal_feedback.calibration == calibration
    yield card, controls, calibration, cp_bytes
