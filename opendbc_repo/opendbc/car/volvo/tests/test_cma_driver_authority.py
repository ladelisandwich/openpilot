"""Actual parsed driver input preserves the selected Dom authority envelope."""
import pytest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.volvo.tests.test_cma import CMA, interface, sources, packet
from opendbc.safety.tests.libsafety import libsafety_py


@pytest.mark.parametrize('identity', CMA)
def test_actual_parser_controller_native_override_release_and_reengage(identity):
  cp, ci = interface(identity)
  lib = libsafety_py.libsafety
  assert lib.set_safety_hooks(cp.safetyConfigs[0].safetyModel.raw, cp.safetyConfigs[0].safetyParam) == 0
  lib.init_tests()
  packer = CANPacker('volvo_mid_1')
  cc = structs.CarControl.new_message()
  cc.enabled = True
  frame = 0
  # Expected plateaus are the pinned Dom tune, not a new generic tune.
  for active, torque, count, expected in [
    (True, 0., 300, (614., -614.)),
    (True, 20., 200, (0., -130.)),
    (True, -20., 200, (130., 0.)),
    (True, 0., 300, (614., -614.)),
    (False, 0., 50, (0., 0.)),
    (True, 0., 300, (614., -614.)),
  ]:
    cc.latActive = active
    for _ in range(count):
      data = [packer.make_can_msg('DRIVER_INPUT', 2, {'STEERING_DRIVER_INPUT': -torque})
              if msg[0] == 0x15 else msg for msg in sources(identity, frame=frame)]
      lib.set_timer(10000 + frame * 10000)
      for msg in data:
        assert lib.safety_rx_hook(packet(msg)), (identity, frame, msg)
      lib.safety_tick()
      state = ci.update([(1_000_000_000 + frame * 10_000_000, data)])
      assert state.canValid and state.steeringTorque == torque
      assert state.steeringPressed == (abs(torque) > 2)
      _, output = ci.apply(cc.as_reader(), 1_000_000_000 + frame * 10_000_000)
      for msg in output:
        assert lib.safety_tx_hook(packet(msg)), (identity, frame, torque, msg)
      frame += 1
    assert (ci.CC.lca_auth_pos, ci.CC.lca_auth_neg) == expected
