"""Parsed-CAN regression coverage for Rivian BASE GEN1 control boundaries."""
# Torque-overlay handshake originally authored by TonyJOM in "Rivian Angle Support"
# (c0e9507313ed1e1b35a5ff83f0c22ee135e535f9).
import time

import pytest
from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.rivian.values import CAR, CarControllerParams


class ControllerRig:
  def __init__(self, alpha=False, gen2=False):
    fp = gen_empty_fingerprint()
    if not gen2:
      fp[0][0x321] = 7
    factory = interfaces[CAR.RIVIAN_R1]
    self.cp = factory.get_params(CAR.RIVIAN_R1, fp, [], alpha, False, False)
    self.ci = factory(self.cp)
    self.ci.update([])  # register actual lazy parser signals before first CAN batch
    self.packer = CANPacker('rivian_primary_actuator')
    self.tick = 0
    for _ in range(2):
      self.feed()

  def feed(self, *, ack=False, fault=False, unavailable=False, gear=4, gas=False, brake=False, feature=1, angle=0.):
    self.tick += 1
    counter = self.tick % 15
    messages = [
      ('ESP_Status', 0, {'ESP_Status_Counter': counter, 'ESP_Vehicle_Speed': 18. * 3.6, 'ESP_Vehicle_Speed_Q': 1}),
      ('VDM_PropStatus', 0, {'VDM_PropStatus_Counter': counter, 'VDM_VehicleSpeed': 18. * 3.6, 'VDM_VehicleSpeedQ': 1,
                            'VDM_Prndl_Status': gear, 'VDM_AcceleratorPedalPosition': 1 if gas else 0}),
      ('iBESP2', 0, {'iBESP2_AliveCounter': counter, 'iBESP2_BrakePedalApplied': brake, 'iBESP2_BrakePedalApplied_Q': 1}),
      ('EPAS_SystemStatus', 0, {'EPAS_SystemStatus_Counter': counter, 'EPAS_TorsionBarTorque': 0.,
                               'H_CAN_EPSS_ToiActive': ack, 'H_CAN_EPSS_ToiFlt': fault, 'H_CAN_EPS_ToiUnavailable': unavailable}),
      ('EPAS_AdasStatus', 0, {'EPAS_AdasStatus_Counter': counter, 'EPAS_InternalSas': angle}),
      ('VDM_AdasSts', 0, {'VDM_AdasStatus_Counter': counter}),
      ('RCM_Status', 0, {'RCM_Status_IND_WARN_BELT_DRIVER': 0}),
      ('ACM_Status', 2, {'ACM_Status_Counter': counter, 'ACM_FeatureStatus': feature}),
      ('ACM_lkaHbaCmd', 2, {'ACM_lkaHbaCmd_Counter': counter}),
      ('ACM_AebRequest', 2, {}),
      ('Cluster', 1, {'Cluster_Unit': 0, 'Cluster_VehicleSpeed': 18. * 3.6}),
      ('IndicatorLights', 1, dict.fromkeys(('RearDriverDoor', 'FrontPassengerDoor', 'DriverDoor', 'RearPassengerDoor'), 2)),
    ]
    if not self.cp.flags:
      messages.append(('SCCM_WheelTouch', 0, {'SCCM_WheelTouch_Counter': counter}))
    frames = [self.packer.make_can_msg(name, bus, values) for name, bus, values in messages]
    self.last_frames = frames
    stamp = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    state = self.ci.update([(stamp, frames)])
    assert state.canValid and not state.canTimeout
    return state

  def request(self, *, lat=True, long=None, enabled=True, torque=.2, accel=.5):
    cc = structs.CarControl(enabled=enabled, latActive=lat, longActive=bool(self.cp.openpilotLongitudinalControl) if long is None else long)
    cc.actuators.torque, cc.actuators.accel = torque, accel
    return cc.as_reader()

  def apply(self, cc):
    output, frames = self.ci.apply(cc, time.monotonic_ns())
    return output, self.decode(frames), frames

  def decode(self, frames):
    parser = CANParser('rivian_primary_actuator', [('ACM_lkaHbaCmd', 0), ('ACM_longitudinalRequest', 0)], 0)
    parser.update([(time.monotonic_ns(), frames)])
    return parser.vl


def test_parsed_ack_before_nonzero_torque():
  rig = ControllerRig()
  rig.feed(ack=False)
  assert rig.apply(rig.request())[0].torqueOutputCan == 0
  for _ in range(2):
    rig.feed(ack=True)
    assert rig.apply(rig.request())[0].torqueOutputCan == 0
  rig.feed(ack=True)
  assert rig.apply(rig.request())[0].torqueOutputCan == 3


@pytest.mark.parametrize('failure', ['fault', 'unavailable'])
def test_fault_release_and_parsed_reengagement(failure):
  rig = ControllerRig()
  for _ in range(4):
    rig.feed(ack=True)
    rig.apply(rig.request())
  rig.feed(ack=True, **{failure: True})
  output, values, _ = rig.apply(rig.request())
  assert output.torqueOutputCan == 0
  assert not values['ACM_lkaHbaCmd']['ACM_lkaActToi']
  for _ in range(2):
    rig.feed(ack=False)
    assert rig.apply(rig.request())[0].torqueOutputCan == 0
  for _ in range(2):
    rig.feed(ack=True)
    assert rig.apply(rig.request())[0].torqueOutputCan == 0
  rig.feed(ack=True)
  assert rig.apply(rig.request())[0].torqueOutputCan > 0


def test_missing_ack_timeout_releases_request():
  rig = ControllerRig()
  for _ in range(50):
    rig.feed(ack=False)
    output, values, _ = rig.apply(rig.request())
    assert output.torqueOutputCan == 0
  assert not values['ACM_lkaHbaCmd']['ACM_lkaActToi']


def test_reverse_withdraws_and_drive_needs_new_ack():
  rig = ControllerRig(alpha=True)
  for _ in range(4):
    rig.feed(ack=True)
    rig.apply(rig.request())
  rig.feed(gear=2, ack=True)
  output, values, _ = rig.apply(rig.request())
  assert output.torqueOutputCan == 0
  assert not values['ACM_lkaHbaCmd']['ACM_lkaActToi']
  assert not values['ACM_longitudinalRequest']['ACM_longInterfaceEnable']
  rig.feed(gear=4, ack=False)
  assert rig.apply(rig.request())[0].torqueOutputCan == 0


@pytest.mark.parametrize('gas,long_active,enabled,gear,expected_enabled', [
  (True, True, True, 4, True), (False, False, True, 4, False),
  (False, True, False, 4, False), (False, True, True, 2, False),
])
def test_long_axis_gas_and_gear_boundary(gas, long_active, enabled, gear, expected_enabled):
  rig = ControllerRig(alpha=True)
  rig.feed(gas=gas, gear=gear)
  _, values, _ = rig.apply(rig.request(lat=False, long=long_active, enabled=enabled))
  assert values['ACM_longitudinalRequest']['ACM_AccelerationRequest'] == 0
  assert bool(values['ACM_longitudinalRequest']['ACM_longInterfaceEnable']) == expected_enabled


def test_real_cancel_payload_preserves_current_five_frame_sequence():
  rig = ControllerRig()
  cc = rig.request(enabled=False, lat=False).as_builder()
  cc.cruiseControl.cancel = True
  cc = cc.as_reader()
  for tick in range(6):
    rig.feed(feature=1)
    _, _, frames = rig.apply(cc)
    decoder = CANParser('rivian_primary_actuator', [('VDM_AdasSts', 0)], 2)
    decoder.update([(time.monotonic_ns(), frames)])
    assert decoder.vl['VDM_AdasSts']['VDM_AdasInterfaceStatus'] == int(tick < 5)
    assert decoder.vl['VDM_AdasSts']['VDM_UserAdasRequest'] == 1


@pytest.mark.parametrize('alpha', [False, True])
def test_upstream_defaults_and_envelope_unchanged(alpha):
  rig = ControllerRig(alpha=alpha)
  assert rig.cp.flags == 0 and rig.cp.openpilotLongitudinalControl == alpha
  assert rig.cp.safetyConfigs[0].safetyParam == int(alpha)
  assert rig.cp.longitudinalActuatorDelay == pytest.approx(.35)
  assert rig.cp.stopAccel == 0
  assert CarControllerParams.STEER_MAX_LOOKUP == ([9, 17], [350, 250])
  assert (CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN) == (3, 5)


def test_gen2_keeps_upstream_initial_torque_behavior():
  rig = ControllerRig(gen2=True)
  rig.feed(ack=False)
  assert rig.cp.flags == 1 and not rig.cp.openpilotLongitudinalControl
  assert rig.apply(rig.request())[0].torqueOutputCan == 3


def test_existing_high_angle_toi_cycle_releases_without_limit_change():
  rig = ControllerRig()
  for _ in range(4):
    rig.feed(ack=True)
    rig.apply(rig.request())
  for _ in range(90):
    rig.feed(ack=True, angle=91.)
    output, values, _ = rig.apply(rig.request())
  assert output.torqueOutputCan == 0
  assert not values['ACM_lkaHbaCmd']['ACM_lkaActToi']


def test_disabled_lateral_outputs_zero():
  rig = ControllerRig()
  rig.feed(ack=True)
  output, values, _ = rig.apply(rig.request(enabled=False, lat=False))
  assert output.torqueOutputCan == 0
  assert not values['ACM_lkaHbaCmd']['ACM_lkaActToi']
