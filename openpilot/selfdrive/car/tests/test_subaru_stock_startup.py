"""Automatic Subaru identity must reach the final Card/Controls stock owner."""
import gc
from unittest.mock import patch

import pytest

from opendbc.car import car_helpers, gen_empty_fingerprint, structs
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.values import CAR
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.controlsd import Controls


@pytest.mark.parametrize("identity,word", ((CAR.SUBARU_ASCENT_2023, 177), (CAR.SUBARU_LEGACY_2025, 145)))
def test_automatic_firmware_reaches_final_stock_owner(identity, word, monkeypatch):
  monkeypatch.setenv("SIMULATION", "1")
  monkeypatch.setenv("FINGERPRINT", "")
  monkeypatch.setenv("SKIP_FW_QUERY", "")
  monkeypatch.setenv("DISABLE_FW_CACHE", "1")
  fw = [structs.CarParams.CarFw(ecu=ecu, address=address, subAddress=sub or 0,
                              fwVersion=versions[0], brand="subaru")
        for (ecu, address, sub), versions in FW_VERSIONS[identity].items()]
  fingerprint = gen_empty_fingerprint()
  card = controls = None
  with OpenpilotPrefix():
    saved = Params()
    for key, value in (("OpenpilotEnabledToggle", True), ("AlwaysOnLateral", False),
                       ("AlphaLongitudinalEnabled", True), ("IsReleaseBranch", False)):
      saved.put_bool(key, value, block=True)
    try:
      # Only acquisition is substituted: real topology/discovery dispatch, exact
      # matching, get_car, pre-create policy, parser/controller and CP publication run.
      with patch.object(card_module.messaging, "recv_one_retry", return_value=messaging.new_message("can", 1)), \
           patch.object(card_module, "can_comm_callbacks", return_value=(lambda *a, **k: [], lambda frames: None)), \
           patch.object(card_module, "obd_callback", return_value=lambda enabled: None), \
           patch.object(car_helpers, "get_vin", return_value=(-1, -1, "0" * 17)), \
           patch.object(car_helpers, "get_present_ecus", return_value=set()), \
           patch.object(car_helpers, "get_fw_versions_ordered", return_value=fw) as queried, \
           patch.object(car_helpers, "can_fingerprint", return_value=(None, fingerprint)), \
           patch.object(car_helpers, "normal_bus_topology", wraps=car_helpers.normal_bus_topology) as topology:
        card = Car()
        assert topology.call_count == queried.call_count == 1
      cp = card.CP
      assert cp.carFingerprint == identity and not cp.fuzzyFingerprint
      assert cp.fingerprintSource == structs.CarParams.FingerprintSource.fw
      assert [x.to_dict() for x in cp.carFw] == [x.to_dict() for x in fw]
      assert cp.pcmCruise and not cp.openpilotLongitudinalControl and not cp.alphaLongitudinalAvailable
      assert not cp.dashcamOnly and not cp.passive
      assert len(cp.safetyConfigs) == 1 and cp.safetyConfigs[0].safetyParam == word
      assert cp.steerControlType == structs.CarParams.SteerControlType.angle
      assert card.CI.CP is cp and card.CI.CC is not None
      with structs.CarParams.from_bytes(saved.get("CarParams")) as published:
        assert published.to_dict() == cp.to_dict()
      controls = Controls()
      assert controls.CP.to_dict() == cp.to_dict()
      assert not card.aol_qualified and card.aol_card_intent is None
    finally:
      del controls, card
      gc.collect()
