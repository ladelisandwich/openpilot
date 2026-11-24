import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC


def setup(identity=CAR.HYUNDAI_IONIQ_5_PE, *, enabled=True):
  fingerprint = gen_empty_fingerprint()
  fingerprint[2].update({0x110:32, 0x362:32})
  fingerprint[1].update({0x1CF:8, 0x35:32, 0x1A0:32})
  if enabled:
    fingerprint[1][0x1BA] = 24
  cp = CarInterface.get_params(identity, fingerprint, [], False, False, False)
  assert cp.deprecated.enableBsm == enabled
  assert not cp.openpilotLongitudinalControl
  ci = CarInterface(cp)
  return cp, ci, CANPacker(DBC[identity][Bus.pt])


def corner(packer, state, *, bus=1):
  return packer.make_can_msg('BLINDSPOTS_FRONT_CORNER_2', bus, {'SIDE_DETECT_STATE':state})


@pytest.mark.parametrize('state,left,right', [(0,False,False), (0x10,True,False), (0x08,False,True), (0x18,True,True)])
def test_actual_pe_factory_parser_and_serialized_state_restore_original_corner_bits(state, left, right):
  cp, ci, packer = setup()
  before = cp.to_dict()
  result = ci.update([(1_000_000_000, [corner(packer, state)])])
  assert (result.leftBlindspot, result.rightBlindspot) == (left, right)
  with structs.CarState.from_bytes(result.to_bytes()) as decoded:
    assert (decoded.leftBlindspot, decoded.rightBlindspot) == (left, right)
  assert cp.to_dict() == before


@pytest.mark.parametrize('failure', ['wrong_bus', 'checksum'])
def test_optional_corner_wrong_bus_or_corrupt_checksum_cannot_create_blindspot(failure):
  _, ci, packer = setup()
  address, data, bus = corner(packer, 0x18)
  if failure == 'wrong_bus':
    bus = 0
  else:
    corrupted = bytearray(data)
    corrupted[0] ^= 1
    data = bytes(corrupted)
  result = ci.update([(1_000_000_000, [CanData(address, data, bus)])])
  assert not result.leftBlindspot and not result.rightBlindspot


def test_pe_stale_corner_expires_at_existing_receive_freshness_boundary():
  _, ci, packer = setup()
  stamp = 1_000_000_000
  assert ci.update([(stamp, [corner(packer, 0x18)])]).leftBlindspot
  heartbeat = packer.make_can_msg('BLINKERS', 1, {})
  boundary = ci.update([(stamp + 200_000_000, [heartbeat])])
  assert boundary.leftBlindspot and boundary.rightBlindspot
  expired = ci.update([(stamp + 200_000_001, [heartbeat])])
  assert not expired.leftBlindspot and not expired.rightBlindspot


def test_absent_factory_bsm_capability_does_not_enroll_from_corner_traffic():
  cp, ci, packer = setup(enabled=False)
  before = cp.to_dict()
  result = ci.update([(1_000_000_000, [corner(packer, 0x18)])])
  assert not result.leftBlindspot and not result.rightBlindspot
  assert cp.to_dict() == before


def test_sibling_ioniq5_n_does_not_inherit_pe_corner_extension():
  _, ci, packer = setup(CAR.HYUNDAI_IONIQ_5_N)
  result = ci.update([(1_000_000_000, [corner(packer, 0x18)])])
  assert not result.leftBlindspot and not result.rightBlindspot


def test_existing_rear_bsm_is_preserved_independently_of_invalid_corner():
  _, ci, packer = setup()
  rear = packer.make_can_msg('ADAS_CMD_50_50ms', 1, {'BCW_LtIndSta':1, 'BCW_RtIndSta':0})
  address, data, bus = corner(packer, 0x08)
  corrupted = bytearray(data)
  corrupted[0] ^= 1
  result = ci.update([(1_000_000_000, [rear, CanData(address, bytes(corrupted), bus)])])
  assert result.leftBlindspot and not result.rightBlindspot
