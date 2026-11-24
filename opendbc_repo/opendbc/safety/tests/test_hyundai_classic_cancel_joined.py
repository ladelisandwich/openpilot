import pytest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.aol.tests.classic_cancel_test_helpers import _exercise_classic_join


class TestClassicCancelDebug:
  @pytest.mark.parametrize('withdrawal', ['toggle-off', 'native-request-off'])
  def test_physical_cancel_keeps_lateral_until_real_withdrawal(self, withdrawal, monkeypatch):
    _exercise_classic_join(withdrawal, monkeypatch, expect_retention=True)


class TestClassicCancelReleaseDenied:
  @pytest.mark.parametrize('word', [0xC04, 0xC05, 0xC06, 0xC44, 0xC45, 0xC46])
  def test_release_does_not_admit_classic_longitudinal_or_independent_lateral(self, word):
    safety = libsafety_py.libsafety
    safety.init_tests()
    safety.set_alternative_experience(32)
    assert safety.set_safety_hooks(CarParams.SafetyModel.hyundai, word) == 0
    assert not safety.safety_config_valid()
    safety.set_aol_test_heartbeat(True)
    safety.aol_set_host_request(3)
    assert safety.aol_get_permission_mask() == 0
    tester = libsafety_py.make_CANPacket(0x7d0, 0, b"\x02\x3e\x80" + bytes(5))
    assert not safety.safety_tx_hook(tester)
