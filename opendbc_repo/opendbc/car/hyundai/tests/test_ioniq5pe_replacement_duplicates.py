import pytest

from opendbc.car import structs
from opendbc.car.hyundai.tests import test_ioniq5pe_pending_startup as startup_fixture


@pytest.mark.parametrize('key', [(0x1A0, 1), (0x100, 0), (0xCB, 1), (0x730, 1)])
@pytest.mark.parametrize('corrupt', [False, True])
def test_duplicate_critical_first_replacement_cannot_hide_prior_frame(key, corrupt):
  case = startup_fixture.TestPEPendingStartup(methodName='runTest')
  case.setUp()
  owner = None
  try:
    owner, ci, packer = case.instance()
    case.warm(owner, ci, packer)
    case.commit(owner, ci)
    case.warm(owner, ci, packer)
    case.acknowledge(owner, ci)
    assert owner.can_emit(ci.CC)
    _, frames = ci.apply(structs.CarControl.new_message().as_reader(), case.boot)
    assert not owner.handed_off and not owner.thread.joined
    address, data, bus = next(frame for frame in frames if (frame[0], frame[2]) == key)
    if corrupt:
      raw = bytearray(data)
      raw[0] ^= 1
      data = bytes(raw)
    duplicate = [(address, data, bus), *frames]
    assert sum((frame[0], frame[2]) == key for frame in duplicate) == 2
    owner.sent(duplicate, valid=True)
    assert not owner.handed_off
    assert not owner.thread.joined
    assert not owner.can_emit(ci.CC)
    with pytest.raises(RuntimeError, match='duplicate critical frame'):
      owner.check()
  finally:
    if owner is not None:
      case.admitted = True
      owner.close()
    case.doCleanups()
