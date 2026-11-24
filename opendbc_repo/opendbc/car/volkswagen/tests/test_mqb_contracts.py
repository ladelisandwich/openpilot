"""MQB configuration and gap-button regressions."""
import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.values import CAR, DBC, CanBus, VolkswagenFlags

MQB = [car for car in CAR if not car.config.flags & (VolkswagenFlags.PQ | VolkswagenFlags.MLB | VolkswagenFlags.MEB)]

@pytest.mark.parametrize("candidate", MQB)
@pytest.mark.parametrize("gateway", [False, True])
@pytest.mark.parametrize("alpha_long", [False, True])
def test_final_mqb_delay_and_native_configuration(candidate, gateway, alpha_long):
  fingerprint = {bus: {} for bus in range(8)}
  fingerprint[0][0xAD] = 8
  if gateway:
    fingerprint[1][0xFD] = 8
  cp = CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)
  assert cp.longitudinalActuatorDelay == pytest.approx(0.25 if candidate == CAR.VOLKSWAGEN_TAOS_MK1 else 0.15)
  assert cp.safetyConfigs[-1].safetyModel == structs.CarParams.SafetyModel.volkswagen
  assert cp.safetyConfigs[-1].safetyParam == int(alpha_long)
  assert cp.openpilotLongitudinalControl == alpha_long
  assert cp.pcmCruise == (not alpha_long)
  assert DBC[candidate][Bus.pt] == "vw_mqb"

@pytest.mark.parametrize("candidate", MQB)
@pytest.mark.parametrize("value", [1, 3])
def test_gap_encoding_real_parser_to_carstate_button_edges(candidate, value):
  fingerprint = {bus: {} for bus in range(8)}
  fingerprint[0][0xAD] = 8
  cp = CarInterface.get_params(candidate, fingerprint, [], False, False, False)
  owner = CarState(cp)
  parsers = owner.get_can_parsers(cp)
  # Real CarState reads register lazy parser messages before the first CAN frame.
  initial = owner.update(parsers)
  assert not any(event.type == structs.CarState.ButtonEvent.Type.gapAdjustCruise for event in initial.buttonEvents)
  packer = CANPacker(DBC[candidate][Bus.pt])
  for tick, raw, pressed in ((1, value, True), (2, value, None), (3, 0, False)):
    msg = packer.make_can_msg("GRA_ACC_01", CanBus(cp).pt, {"COUNTER": tick, "GRA_Verstellung_Zeitluecke": raw})
    updated = parsers[Bus.pt].update([(tick * 20_000_000, [msg])])
    assert msg[0] in updated
    assert parsers[Bus.pt].vl["GRA_ACC_01"]["GRA_Verstellung_Zeitluecke"] == raw
    state = owner.update(parsers)
    events = [event for event in state.buttonEvents if event.type == structs.CarState.ButtonEvent.Type.gapAdjustCruise]
    assert len(events) == (0 if pressed is None else 1)
    if pressed is not None:
      assert events[0].pressed == pressed
