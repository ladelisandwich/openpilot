"""The native brake/paddle withdrawal may precede the host's pedal sample."""
from contextlib import ExitStack
from dataclasses import replace

import pytest

from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.tests.test_gm_held_native import peers
from openpilot.starpilot.aol.tests.test_gm_source_deadline import profiles
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm.values import DBC, GMFlags, is_bolt_pedal_profile
from openpilot.common.params import Params
from openpilot.starpilot.aol.tests import test_gm as gm_fixture
from openpilot.starpilot.aol.tests.test_gm_held_native import NOW, parse as bolt_parse
from openpilot.starpilot.aol.tests.test_gm_startup_main import parsed_packets


@pytest.mark.parametrize('cp', profiles(), ids=lambda cp: f'{cp.carFingerprint}-{cp.safetyConfigs[0].safetyParam:x}')
@pytest.mark.parametrize('health_allowed', [False, True])
def test_all_volt_and_bolt_profiles_keep_lateral_intent_when_native_long_withdraws_first(cp, health_allowed):
  with OpenpilotPrefix(), ExitStack() as cleanup:
    saved = Params()
    saved.put_bool('AlwaysOnLateral', True, block=True)
    card = gm_fixture.TestGmAol.card(cp.as_reader().as_builder(), saved)
    cleanup.callback(card.vehicle_startup.close)
    packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])
    def parse(tick):
      now = NOW + tick * 10_000_000
      if is_bolt_pedal_profile(card.CP):
        packets = bolt_parse(card, now, counter=tick % 4, packets_only=True)
        packets.append(packer.make_can_msg('ASCMActiveCruiseControlStatus', 2, {'ACCCruiseState': 0}))
      else:
        packets = parsed_packets(card.CP, packer, now, tick, 20., True)
        if card.CP.flags & GMFlags.VOLT_CAMERA_NO_ACCEL_POS:
          packets.append(packer.make_can_msg('EBCMBrakePedalPosition', 0, {'BrakePedalPosition': 0}))
      card.CI.update([(now - 1, packets)])
      cs = card.CI.update([(now, packets)])
      assert cs.canValid and not cs.canTimeout
      return cs
    feedback = TransportPauseFeedback(card.CP)
    now = NOW + 20_000_000
    if int(card.CP.safetyConfigs[0].safetyParam) == 0x1CD:
      parse(0)  # Receive the first 25 Hz factory clock before the later pedal sample.
    cs = parse(2)
    card.aol_card_intent.update(cs, fault_active=False, now_ns=now)
    assert card.aol_card_intent.allowed_latch
    incoming, prior = peers(card.CP, card.slc_producer_session, now)
    assert not feedback.observe(incoming, card.CP, cs, now, latched=True,
      intent_producer_session=card.slc_producer_session)
    now += 10_000_000
    cs = parse(3)
    native = replace(prior, observedMonoTime=now, validUntilMonoTime=now + 200_000_000,
      longitudinalAllowed=False, pandaInventory=(replace(prior.pandaInventory[0], controlsAllowed=health_allowed),))
    incoming, _ = peers(card.CP, card.slc_producer_session, now, controls=health_allowed, native=native, sequence=2)
    reset = feedback.observe(incoming, card.CP, cs, now, latched=True,
      intent_producer_session=card.slc_producer_session)
    card.aol_card_intent.update(cs, fault_active=reset, now_ns=now)
    assert not reset and not feedback.failed and card.aol_card_intent.allowed_latch
    assert not native.longitudinalAllowed
