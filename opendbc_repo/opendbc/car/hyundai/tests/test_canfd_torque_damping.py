"""Dom 823a5d5 camera/ADAS damping contract, checked at the CAN wire boundary."""
from itertools import product

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus, create_steering_messages
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags


TORQUE_CARS = tuple(car for car in CAR if car.config.flags & HyundaiFlags.CANFD
                    and not car.config.flags & HyundaiFlags.CANFD_ANGLE_STEERING)
LKAS_DAMPING_100 = frozenset((CAR.KIA_CARNIVAL_4TH_GEN, CAR.KIA_CARNIVAL_2025,
                            CAR.KIA_CARNIVAL_HEV_4TH_GEN, CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN))


@pytest.mark.parametrize('car', TORQUE_CARS)
@pytest.mark.parametrize('topology', ('lfa', 'lkas', 'lkas_alt'))
def test_dom_torque_wire_is_independent_of_alpha_long(car, topology):
  cp = structs.CarParams()
  cp.carFingerprint = car
  cp.flags = int(car.config.flags & ~(HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT))
  if topology != 'lfa':
    cp.flags |= int(HyundaiFlags.CANFD_LKA_STEER_MSG)
  if topology == 'lkas_alt':
    cp.flags |= int(HyundaiFlags.CANFD_LKA_STEER_MSG_ALT)
  can = CanBus(cp)
  for enabled, active, torque in product((False, True), (False, True), (-137, 0, 139)):
    by_mode = []
    for longitudinal in (False, True):
      cp.openpilotLongitudinalControl = longitudinal
      packets = create_steering_messages(CANPacker(DBC[car][Bus.pt]), cp, can, enabled, active, torque)
      steering_address = {'lfa': 0x12A, 'lkas': 0x50, 'lkas_alt': 0x110}[topology]
      expected = [(steering_address, can.ECAN if topology == 'lfa' else can.ACAN)]
      if longitudinal and topology != 'lfa':
        expected.insert(0, (0x12A, can.ECAN))
      assert [(addr, bus) for addr, _, bus in packets] == expected
      for address, data, _ in packets:
        lfa = address == 0x12A
        damping = 100 if lfa or car in LKAS_DAMPING_100 else 0
        # Independent frozen DBC bit positions; no parser/helper-derived oracle.
        assert data[13 if lfa else 8] == damping
        bits = int.from_bytes(data, 'little')
        assert (bits >> 41 & 0x7FF) - 1024 == torque
        assert (bits >> 52 & 3) == int(active)
        assert (bits >> 38 & 7) == (2 if enabled else 1)
      by_mode.append(packets[-1])
    assert by_mode[0] == by_mode[1]


@pytest.mark.parametrize('car', tuple(car for car in CAR if car.config.flags & HyundaiFlags.CANFD_ANGLE_STEERING))
def test_torque_helper_keeps_existing_angle_fallback_damping(car):
  cp = structs.CarParams()
  cp.carFingerprint = car
  cp.flags = int(car.config.flags | HyundaiFlags.CANFD_LKA_STEER_MSG)
  for longitudinal in (False, True):
    cp.openpilotLongitudinalControl = longitudinal
    packets = create_steering_messages(CANPacker(DBC[car][Bus.pt]), cp, CanBus(cp), True, True, 0)
    assert packets[-1][1][8] == 100
