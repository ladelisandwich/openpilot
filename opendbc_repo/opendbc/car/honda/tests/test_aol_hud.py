"""Root-run real factory/controller/codec lane HUD regression; controlled CarState."""
import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, DBC, HondaFlags


@pytest.mark.parametrize("identity", (CAR.HONDA_CIVIC, CAR.HONDA_ACCORD, CAR.HONDA_CIVIC_2022, CAR.ACURA_RDX_3G_MMR))
@pytest.mark.parametrize("active", (False, True))
@pytest.mark.parametrize("visible", (False, True))
@pytest.mark.parametrize("available", (False, True))
def test_actual_controller_preserves_intentional_lane_hud(identity, active, visible, available):
  cp = CarInterface.get_params(identity, gen_empty_fingerprint(), [], True, False, False)
  assert cp.openpilotLongitudinalControl
  ci = CarInterface(cp)
  ci.update([])
  state = structs.CarState(vEgo=30.0)
  state.cruiseState.available = available
  ci.CS.out = state
  cc = structs.CarControl(enabled=active, latActive=active)
  cc.hudControl.lanesVisible = visible
  _, frames = ci.apply(cc.as_reader(), 10_000_000_000)
  decoder = CANParser(DBC[identity][Bus.pt], [("LKAS_HUD", 10)], ci.CC.CAN.lkas)
  assert decoder.update([(10_000_000_000, frames)])
  encoded = decoder.vl["LKAS_HUD"]
  if cp.flags & HondaFlags.BOSCH_ALT_RADAR:
    assert bool(encoded["SOLID_LANES"]) == active
    assert bool(encoded["DASHED_LANES"]) == available
  else:
    assert bool(encoded["SOLID_LANES"]) == (visible or active)
    if cp.flags & HondaFlags.BOSCH_RADARLESS:
      assert bool(encoded["DASHED_LANES"]) == (visible or active)


@pytest.mark.parametrize("identity", (CAR.ACURA_RDX_3G_MMR, CAR.HONDA_ODYSSEY_5G_MMR,
                                     CAR.ACURA_TLX_2G, CAR.ACURA_MDX_4G))
@pytest.mark.parametrize("available", (False, True))
def test_alt_radar_enabled_longitudinal_with_lateral_inactive_keeps_upstream_hud(identity, available):
  cp = CarInterface.get_params(identity, gen_empty_fingerprint(), [], True, False, False)
  assert cp.openpilotLongitudinalControl
  assert cp.flags & HondaFlags.BOSCH_ALT_RADAR
  ci = CarInterface(cp)
  ci.update([])
  state = structs.CarState(vEgo=30.0)
  state.cruiseState.available = available
  ci.CS.out = state
  cc = structs.CarControl(enabled=True, longActive=True, latActive=False)
  cc.hudControl.lanesVisible = True  # Actual Controls.publish uses enabled.
  _, frames = ci.apply(cc.as_reader(), 10_000_000_000)
  decoder = CANParser(DBC[identity][Bus.pt], [("LKAS_HUD", 10)], ci.CC.CAN.lkas)
  assert decoder.update([(10_000_000_000, frames)])
  encoded = decoder.vl["LKAS_HUD"]
  assert not encoded["SOLID_LANES"], "ALT_RADAR_LAT_INACTIVE_MUST_NOT_SHOW_SOLID_LANES"
  assert bool(encoded["DASHED_LANES"]) == available
