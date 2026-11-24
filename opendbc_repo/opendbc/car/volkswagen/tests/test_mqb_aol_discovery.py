"""Synthetic raw discovery receipts through real matcher/get_car/Card finalization.

Transport query responses are explicit fixture input; matcher and finalCP are real.
This does not establish an observed vehicle/route or native command permission.
"""
import os
from unittest import mock

import pytest
from opendbc.can import CANPacker
from opendbc.car import car_helpers, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.volkswagen.values import CAR
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.vehicle import policy_for


@pytest.mark.parametrize('manual', (False, True))
@pytest.mark.parametrize('gateway,alpha', ((False, False), (True, False), (True, True)))
def test_normal_golf_discovery_and_card_stock_aol_owner(manual, gateway, alpha, offset=0, fixed=False):
  fixture = {'kind': 'EXPLICIT_SYNTHETIC_FULL_ECU_DISCOVERY_INPUT_FROM_REUSED_INVENTORY_NOT_ROUTE_OR_PHYSICAL',
   'identity': 'VOLKSWAGEN_GOLF_MK7',
   'vin': 'WVWZZZAUZGW000000',
   'firmware': [{'ecu': 'engine',
                 'address': 2016,
                 'subAddress': None,
                 'fwVersion': 'f1873034453930363031364120f18937363937'},
                {'ecu': 'transmission',
                 'address': 2017,
                 'subAddress': None,
                 'fwVersion': 'f1873039473932373734394150f18932393433'},
                {'ecu': 'srs',
                 'address': 1813,
                 'subAddress': None,
                 'fwVersion': 'f1873551303935393635354141f18930333836f1821131343133303031313133313230303433313134333137313231433131314339313133'},
                {'ecu': 'eps',
                 'address': 1810,
                 'subAddress': None,
                 'fwVersion': 'f1873351303930393134344620f18935303433f1820536314130313631324130'},
                {'ecu': 'fwdRadar',
                 'address': 1879,
                 'subAddress': None,
                 'fwVersion': 'f1873551303930373536374720f18930333930f182013031'}],
   'origin': 'Synthetic ECU bytes from configuration-matrix.json; actual matcher must verify uniqueness.'}
  fw = [structs.CarParams.CarFw(ecu=getattr(structs.CarParams.Ecu, str(r['ecu'])),
        address=r['address'], subAddress=r['subAddress'] or 0, fwVersion=bytes.fromhex(str(r['fwVersion'])),
        brand='volkswagen', bus=offset) for r in fixture['firmware']]
  exact, candidates = match_fw_to_car(fw, fixture['vin'], log=False)
  assert exact and candidates == {CAR.VOLKSWAGEN_GOLF_MK7}, (exact, candidates)
  packer = CANPacker('vw_mqb')
  incoming = [packer.make_can_msg('GRA_ACC_01', offset, {'GRA_Hauptschalter': 1})]
  if not manual:
    incoming.append(packer.make_can_msg('Getriebe_11', offset, {}))
  if gateway:
    incoming.append(packer.make_can_msg('ESP_21', offset + 1, {}))
  frames = [CanData(address=address, dat=data, src=bus) for address, data, bus in incoming]

  def receive(wait_for_one=False):
    return [frames]

  with OpenpilotPrefix(), mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    params = Params()
    params.put_bool('OpenpilotEnabledToggle', True, block=True)
    params.put_bool('AlwaysOnLateral', True, block=True)
    params.put_bool('AlphaLongitudinalEnabled', alpha, block=True)
    params.put_bool('SafeMode', False, block=True)
    params.put('MainCruiseButtonControl', 0, block=True)
    params.put('LKASButtonControl', 0, block=True)
    params.remove('VehicleSelection')
    environment = {key: value for key, value in os.environ.items()
                   if key not in ('FINGERPRINT', 'SKIP_FW_QUERY', 'DISABLE_FW_CACHE')}
    with (mock.patch.dict(os.environ, environment, clear=True),
          mock.patch.object(car_helpers, 'get_vin', return_value=(0, 0, fixture['vin'])),
          mock.patch.object(car_helpers, 'get_present_ecus', return_value=set()),
          mock.patch.object(car_helpers, 'get_fw_versions_ordered', return_value=fw)):
      # Normal public get_car: do NOT stub fingerprint, match_fw_to_car or candidate selection.
      ci = car_helpers.get_car(receive, lambda *args, **kwargs: None, lambda _: None, alpha, False,
                               forced_candidate=CAR.VOLKSWAGEN_GOLF_MK7 if fixed else None)
    cp = ci.CP
    assert cp.carFingerprint == CAR.VOLKSWAGEN_GOLF_MK7
    assert cp.fingerprintSource == (structs.CarParams.FingerprintSource.fixed if fixed else
                                    structs.CarParams.FingerprintSource.fw) and not cp.fuzzyFingerprint
    assert cp.carVin == fixture['vin'] and len(cp.carFw) == len(fw)
    assert cp.transmissionType == (structs.CarParams.TransmissionType.manual if manual else
                                  structs.CarParams.TransmissionType.automatic)
    assert cp.networkLocation == (structs.CarParams.NetworkLocation.gateway if gateway else
                                 structs.CarParams.NetworkLocation.fwdCamera)
    assert cp.pcmCruise == (not alpha) and cp.openpilotLongitudinalControl == alpha
    policy = policy_for(cp)
    assert policy.intent_supported and policy.runtime_supported
    card = Car(CI=ci, RI=ci.RadarInterface(cp))
    try:
      assert card.aol_qualified and card.aol_card_intent is not None
      assert card.CP.safetyConfigs[-1].safetyParam == ((24 if manual else 8) | int(alpha))
      assert len(card.CP.safetyConfigs) == (2 if offset else 1)
      if offset:
        assert card.CP.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.noOutput
        assert card.CP.safetyConfigs[0].safetyParam == 0
      assert card.CP.alternativeExperience == 32
      with structs.CarParams.from_bytes(params.get('CarParams')) as published:
        assert published.to_dict() == card.CP.to_dict()
    finally:
      card.vehicle_startup.close()
