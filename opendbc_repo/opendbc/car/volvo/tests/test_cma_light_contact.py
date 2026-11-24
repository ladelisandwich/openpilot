"""Intentional Dom contact pulse, strict override thresholds and cooldown.

Actual CAN parser/controller/native calls; numerical expectations pin Dom's
deliberate tune, without introducing a generic steering tune.
"""
import pytest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.volvo.tests.test_cma import CMA, interface, packet, sources
from opendbc.safety.tests.libsafety import libsafety_py


class DriverInput:
  def __init__(self, identity):
    self.identity = identity
    self.cp, self.ci = interface(identity)
    self.lib = libsafety_py.libsafety
    cfg = self.cp.safetyConfigs[0]
    assert self.lib.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam) == 0
    self.lib.init_tests()
    self.packer = CANPacker('volvo_mid_1')
    self.command = structs.CarControl(enabled=True, latActive=True)
    self.frame = 0

  def step(self, torque):
    data = [self.packer.make_can_msg('DRIVER_INPUT', 2, {'STEERING_DRIVER_INPUT': -torque})
            if msg[0] == 0x15 else msg for msg in sources(self.identity, frame=self.frame)]
    self.lib.set_timer(10000 + self.frame * 10000)
    for msg in data:
      assert self.lib.safety_rx_hook(packet(msg)), (self.identity, self.frame, msg)
    self.lib.safety_tick()
    now = 1_000_000_000 + self.frame * 10_000_000
    state = self.ci.update([(now, data)])
    assert state.canValid and state.steeringTorque == torque
    assert state.steeringPressed == (abs(torque) > 2)
    _, output = self.ci.apply(self.command.as_reader(), now)
    assert output
    for msg in output:
      assert self.lib.safety_tx_hook(packet(msg)), (self.identity, self.frame, torque, msg)
    self.frame += 1
    return self.ci.CC

  def saturated(self):
    for _ in range(300):
      controller = self.step(0)
    assert (controller.lca_auth_pos, controller.lca_auth_neg) == (614., -614.)


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('direction', [-1, 1])
def test_light_contact_has_one_bounded_pulse_then_rebuilds_while_contact_remains(identity, direction):
  driver = DriverInput(identity)
  driver.saturated()
  for index in range(15):
    controller = driver.step(direction * 4)
    assert not controller.lca_auth_override_active
    assert controller.lca_auth_light_frames == 15 - index
    assert controller.lca_auth_pos == 614. - (index + 1) * 25.
    assert controller.lca_auth_neg == -controller.lca_auth_pos
  controller = driver.step(direction * 4)
  assert controller.lca_auth_light_frames == 0
  assert controller.lca_auth_pos == pytest.approx(239. + 2.3)
  for _ in range(200):
    controller = driver.step(direction * 4)
    assert not controller.lca_auth_override_active
    assert controller.lca_auth_light_frames == 0
  assert (controller.lca_auth_pos, controller.lca_auth_neg) == (614., -614.)


@pytest.mark.parametrize('identity', CMA)
@pytest.mark.parametrize('direction', [-1, 1])
def test_override_hysteresis_and_contact_cooldown_do_not_flap_at_boundary(identity, direction):
  driver = DriverInput(identity)
  driver.saturated()
  controller = driver.step(direction * 5)
  assert not controller.lca_auth_override_active  # Entry is strictly greater than5.
  # Explicit disengagement resets any light pulse from that first contact.
  driver.command.latActive = False
  driver.step(0)
  driver.command.latActive = True
  for magnitude in (6, 5, 4, 3):
    controller = driver.step(direction * magnitude)
    assert controller.lca_auth_override_active
    assert controller.lca_auth_real_off_frames == 0
    assert controller.lca_auth_light_frames == 0
  controller = driver.step(direction * 2)
  assert not controller.lca_auth_override_active  # Exit is strictly below3.
  assert controller.lca_auth_real_off_frames == 1
  controller = driver.step(direction * 4)
  assert controller.lca_auth_light_frames == 0  # Rising contact suppressed during cooldown.
  driver.step(direction * 2)
  for _ in range(26):
    controller = driver.step(direction * 2)
  assert controller.lca_auth_real_off_frames == 29
  controller = driver.step(direction * 4)
  assert controller.lca_auth_real_off_frames == 30
  assert controller.lca_auth_light_frames == 0  # Strictly more than30 required.
  driver.step(direction * 2)
  controller = driver.step(direction * 4)
  assert controller.lca_auth_real_off_frames == 32
  assert controller.lca_auth_light_frames == 15
  assert not controller.lca_auth_override_active
