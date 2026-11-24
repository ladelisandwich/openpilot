"""Transit CAN angle quantization must stay within the native encoded envelope."""
import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.ford.fordcan import CanBus, create_lka_msg
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_ford_aol import TestFordAolDriverIntent as _Native


@pytest.mark.parametrize('word', (12, 13))
@pytest.mark.parametrize('sign,endpoint', ((1, 4072), (-1, 24)))
def test_saturated_angle_with_physical_cruise_stays_inside_native_envelope(sign, endpoint, word):
  native = _Native(methodName='runTest')
  native.setUp()
  try:
    native.reset(word)
    native.pump(40, main=3)
    native.pump(10, main=4, mask=1)
    assert native.safety.get_controls_allowed()
    for angle in (1., 2., 3., 4., 5., 5.8, 6.):
      native.pump(3, main=4, mask=1)
      address, data, bus = create_lka_msg(native.packer, CanBus(fingerprint=gen_empty_fingerprint()), True, sign * angle,
                                        2 if sign > 0 else 4, 0., transit=True)
      raw_angle = ((data[2] & 15) << 8) | data[3]
      assert 24 <= raw_angle <= 4072
      if angle >= 5.8:
        assert raw_angle == endpoint
      assert native.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
    # The adjacent encoded values remain prohibited by unchanged native safety.
    rejected_angle = 4073 if sign > 0 else 23
    rejected = bytearray(data)
    rejected[2] = (rejected[2] & 0xF0) | (rejected_angle >> 8)
    rejected[3] = rejected_angle & 0xFF
    assert not native.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, rejected))
    address, data, bus = create_lka_msg(native.packer, CanBus(fingerprint=gen_empty_fingerprint()), False, sign * 6., transit=True)
    assert data[0] >> 5 == 0
    assert native.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
  finally:
    native.tearDown()
