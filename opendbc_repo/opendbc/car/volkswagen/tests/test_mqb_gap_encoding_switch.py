"""Real Golf MQB parser/CarState gap edges across pressed-value changes."""
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC, CanBus


def test_golf_gap_pressed_encoding_switch_has_no_false_edges():
  identity = CAR.VOLKSWAGEN_GOLF_MK7
  fingerprint = {bus: {} for bus in range(8)}
  fingerprint[0][0xAD] = 8
  cp = CarInterface.get_params(identity, fingerprint, [], False, False, False)
  owner = CarState(cp)
  parsers = owner.get_can_parsers(cp)
  gap = structs.CarState.ButtonEvent.Type.gapAdjustCruise
  initial = owner.update(parsers)
  assert not any(event.type == gap for event in initial.buttonEvents)
  packer = CANPacker(DBC[identity][Bus.pt])

  # Both1 and3 mean pressed;2 releases, and subsequent0 adds no second release.
  sequence = ((1, (True,)), (3, ()), (1, ()), (2, (False,)), (0, ()))
  for tick, (raw, expected_edges) in enumerate(sequence, start=1):
    msg = packer.make_can_msg("GRA_ACC_01", CanBus(cp).pt,
                               {"COUNTER": tick, "GRA_Verstellung_Zeitluecke": raw})
    updated = parsers[Bus.pt].update([(tick * 20_000_000, [msg])])
    assert msg[0] in updated, (tick, raw, "frame not accepted")
    assert parsers[Bus.pt].vl["GRA_ACC_01"]["GRA_Verstellung_Zeitluecke"] == raw
    state = owner.update(parsers)
    edges = tuple(event.pressed for event in state.buttonEvents if event.type == gap)
    assert edges == expected_edges, (tick, raw, edges, expected_edges)
