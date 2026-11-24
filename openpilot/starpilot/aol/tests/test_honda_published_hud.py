"""Existing actual family callers -> published AOL-only command -> real CAN codec.

Transport, parsed CarState and native status are synthetic inputs from the existing
stock Honda fixture; no native firmware/device/parser-RX health claim. Production
Card/SD/Controls construction, authority checks, publish, CI.apply and CAN codec run.
No command enabled/latActive/lanesVisible field is assigned by this test.
"""
from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.honda.values import CAR, DBC, HondaFlags
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.tests.stock_lateral_test_helpers import honda_family_case


def test_actual_published_aol_only_command_keeps_lane_hud_and_native_denial_clears_it():
  with OpenpilotPrefix():
    case = honda_family_case("honda", False)
    try:
      sd, controls, step, card, _ = case.family_callers(False)
      assert card.CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH
      assert not card.CP.openpilotLongitudinalControl
      ci = card.CI
      ci.update([])  # Initialize actual CS auxiliary dictionaries, not bus health.
      for first_tick, allowed in ((2, True), (12, False)):
        frames = []
        # Both callers and sender advance at100Hz; no stale command replay or
        # controller frame assignment is used to reach the real10Hz HUD tick.
        for tick in range(first_tick, first_tick + 10):
          axis, _, command = step(tick, allowed=allowed)
          event = controls.pm.messages["carControl"]
          published = event.carControl
          assert event.valid
          assert not sd.enabled and not sd.active
          assert not published.enabled and not published.longActive
          assert not published.hudControl.lanesVisible
          assert bool(axis.lateralActive) == bool(command.latActive) == bool(published.latActive) == allowed
          # Supplied parsed physical state, unmodified actual published command.
          ci.CS.out = sd.CS_prev
          _, sent = ci.apply(published, int(event.logMonoTime))
          frames.extend(sent)
        names = ("LKAS_HUD_A", "LKAS_HUD_B") if card.CP.flags & HondaFlags.BOSCH_EXT_HUD else ("LKAS_HUD",)
        decoder = CANParser(DBC[card.CP.carFingerprint][Bus.pt], [(name, 10) for name in names], ci.CC.CAN.lkas)
        assert decoder.update([(int(event.logMonoTime), frames)])
        for name in names:
          actual = bool(decoder.vl[name]["SOLID_LANES"])
          if allowed:
            assert actual, "AOL_ONLY_PUBLISHED_COMMAND_LOSES_LANE_HUD"
          else:
            assert not actual, "NATIVE_DENIAL_MUST_CLEAR_AOL_LANE_HUD"
    finally:
      case.doCleanups()
