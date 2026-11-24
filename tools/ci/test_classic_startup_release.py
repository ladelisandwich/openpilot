"""Release must deny debug-only classic longitudinal startup configurations."""
import pytest

from opendbc.car import structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.hyundai.classic_long_aol import qualified as qualified_classic, aol_word
from opendbc.car.hyundai.hyundaican import create_acc_commands
from opendbc.car.hyundai.tests.test_classic_family_startup_caller import family_cases
from opendbc.can import CANPacker
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.aol.tests.test_classic_long_profiles import params as classic_params


class TestClassicStartupReleaseDenied:
  @pytest.mark.parametrize('identity', family_cases())
  @pytest.mark.parametrize('aol', [False, True])
  def test_release_keeps_experimental_startup_and_longitudinal_tx_closed(self, identity, aol):
    release_cp = classic_params(identity, release=True, lda=True)
    assert not release_cp.openpilotLongitudinalControl
    assert release_cp.pcmCruise
    assert not qualified_classic(release_cp)
    assert CarInterface.startup_owner(release_cp, (list, lambda frames: None), requested=True) is None

    # A debug host profile must not gain longitudinal authority on release
    # firmware, even with a host request and controls_allowed asserted.
    cp = classic_params(identity, lda=True)
    if aol:
      cp.safetyConfigs[0].safetyParam = aol_word(cp)
      cp.alternativeExperience = 32
    native = libsafety_py.libsafety
    packer = CANPacker('hyundai_can_generated')
    try:
      native.set_alternative_experience(cp.alternativeExperience)
      assert native.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
      native.init_tests()
      if aol:
        assert not native.safety_config_valid()
      for controls_allowed in (False, True):
        native.set_controls_allowed(controls_allowed)
        native.set_aol_test_heartbeat(True)
        native.aol_set_host_request(3)
        assert native.aol_get_permission_mask() == 0
        # Test neutral startup output as well as actual acceleration requests.
        for enabled, accel in ((False, 0.0), (True, 1.0), (True, -1.0)):
          frames = create_acc_commands(packer, enabled, accel, 1.0, 0, structs.CarControl.HUDControl(),
                                       0, False, False, bool(cp.flags & HyundaiFlags.USE_FCA), cp)
          frames.append((0x7D0, b'\x02\x3e\x80' + bytes(5), 0))
          assert {0x420, 0x421, 0x389, 0x7D0} <= {address for address, _, _ in frames}
          for address, data, bus in frames:
            assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data)), hex(address)
    finally:
      native.init_tests()
      native.set_alternative_experience(0)
      native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
