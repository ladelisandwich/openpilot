"""Public MEB get_car: transport fixtures, genuine FW matcher/factory/Card finalization."""
import os
from unittest import mock
import pytest
from opendbc.can import CANPacker
from opendbc.car import car_helpers, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS
from opendbc.car.volkswagen.values import CAR, DBC
from opendbc.car import Bus
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car


@pytest.mark.parametrize('identity', (CAR.VOLKSWAGEN_ID4_MK1, CAR.VOLKSWAGEN_ID4_MK2))
@pytest.mark.parametrize('stock', (False, True))
@pytest.mark.parametrize('offset,fixed', ((0, False), (4, True)))
def test_normal_meb_discovery_and_final_card_owner(identity, stock, offset, fixed):
  # Full synthetic ECU reply inventory from the current source DB; not an observed physical capture.
  fw = [structs.CarParams.CarFw(ecu=ecu, address=address, subAddress=sub or 0, fwVersion=versions[0],
                              brand='volkswagen', bus=offset)
        for (ecu, address, sub), versions in FW_VERSIONS[identity].items()]
  vin = 'WVWZZZE2ZMP000000' if identity == CAR.VOLKSWAGEN_ID4_MK1 else 'WVWZZZE8ZRP000000'
  exact, matches = match_fw_to_car(fw, vin, log=False)
  assert exact == (identity == CAR.VOLKSWAGEN_ID4_MK2) and matches == {identity}
  packer = CANPacker(DBC[identity][Bus.pt])
  messages = [packer.make_can_msg('Getriebe_11', offset, {'GE_Fahrstufe': 8}),
              packer.make_can_msg('QFK_01', offset + 1, {'LatCon_HCA_Status': 2})]
  frames = [CanData(address=addr, dat=data, src=bus) for addr, data, bus in messages]

  def receive(wait_for_one=False):
    return [frames]
  with OpenpilotPrefix(), mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    store = Params()
    store.put_bool('AlwaysOnLateral', True, block=True)
    store.put_bool('OpenpilotEnabledToggle', True, block=True)
    store.put_bool('SafeMode', False, block=True)
    store.put_bool('DisableOpenpilotLongitudinal', stock, block=True)
    store.remove('AlphaLongitudinalEnabled')
    store.remove('VehicleSelection')
    environment = {k: v for k, v in os.environ.items() if k not in ('FINGERPRINT', 'SKIP_FW_QUERY', 'DISABLE_FW_CACHE')}
    with (mock.patch.dict(os.environ, environment, clear=True),
          mock.patch.object(car_helpers, 'get_vin', return_value=(0, offset, vin)),
          mock.patch.object(car_helpers, 'get_present_ecus', return_value=set()),
          mock.patch.object(car_helpers, 'get_fw_versions_ordered', return_value=fw)):
      # Actual startup owner is the same pre-create hook used by Card's public acquisition path.
      from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences
      snapshot = VehicleStartupPreferences.read(store, enabled=True)
      ci = car_helpers.get_car(receive, lambda *args: None, lambda _: None, False, False,
        forced_candidate=identity if fixed else None,
        pre_create_hook=lambda cp, candidate, fingerprints, car_fw: snapshot.prepare(cp, fingerprints=fingerprints))
    cp = ci.CP
    assert cp.carFingerprint == identity
    assert cp.fingerprintSource == (structs.CarParams.FingerprintSource.fixed if fixed else structs.CarParams.FingerprintSource.fw)
    assert cp.openpilotLongitudinalControl != stock and cp.pcmCruise == stock
    card = Car(CI=ci, RI=ci.RadarInterface(cp))
    try:
      assert card.aol_qualified and card.CP.alternativeExperience == 32
      assert card.CP.safetyConfigs[-1].safetyModel == structs.CarParams.SafetyModel.volkswagenMeb
      assert len(card.CP.safetyConfigs) == (2 if offset else 1)
      with structs.CarParams.from_bytes(store.get('CarParams')) as published:
        assert published.to_dict() == card.CP.to_dict()
    finally:
      card.vehicle_startup.close()
