"""Real Card publication, nonconflating IPC, SelfdriveD and Controls clock crossing."""
from contextlib import ExitStack
from unittest.mock import patch
import pytest
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car.tests.publication_fixture import initialize_publication_sources
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.car.gm.steering_companion import SteeringCompanion


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('full_op', [False, True])
def test_real_card_socket_clock_crossing_preserves_sdgm_axes(c9, full_op):
  from openpilot.selfdrive.car.card import Car
  actual_recv, actual_optional = messaging.recv_one, messaging.recv_one_or_none
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup, OpenpilotPrefix():
    cleanup.callback(case.doCleanups)
    messaging.reset_context()
    sd, controls, step = case._authority_callers(full_op, cp=sdgm_params(alpha=True, brake_c9=c9))
    from openpilot.starpilot.aol.tests.test_gm_source_deadline import bind_steering_receipts
    step = bind_steering_receipts(sd, step, cleanup)
    assert step(0)[0].lateralActive
    pm = messaging.PubMaster(['carState', 'carOutput', 'starpilotCarState'])
    sd.car_state_sock = messaging.sub_sock('carState', timeout=100)
    sd.steering_companion = SteeringCompanion(messaging.sub_sock('starpilotCarState', conflate=False))
    latest = messaging.sub_sock('starpilotCarState', conflate=True, timeout=100)
    card = Car.__new__(Car)
    card.CP, card.pm = sd.CP, pm
    initialize_publication_sources(card)
    authority = card.CI.CS.steering_authority
    card.sm = messaging.SubMaster(['carControl'], ignore_avg_freq=['carControl'])
    neutral = messaging.new_message('carControl', valid=True, logMonoTime=10_000_000_000)
    card.sm.update_msgs(10., [neutral.as_reader()])
    card.sm.frame = 1
    card.car_params_published = True
    card.last_actuators_output = car.CarControl.Actuators.new_message()
    card.slc_replay = card.curve_replay = card.aol_replay = False
    card.can_rcv_cum_timeout_counter = 0
    card.rk = Ratekeeper(100., print_delay_threshold=None)
    card.v_cruise_helper = VCruiseHelper(card.CP)
    source = 10_010_000_000
    later = source + 7_000_000
    clock = [source + 6_000_000]
    original_message = messaging.new_message

    def publish(stamp):
      authority.enabled, authority.latched = True, False
      cs = fixture.car_state()
      cs.cruiseState.available = True
      with patch('openpilot.selfdrive.car.card.messaging.new_message', side_effect=lambda *a, **kw:
                 original_message(*a, **dict(kw, logMonoTime=stamp))):
        card.state_publish(cs, None)

    def receive_first(sock):
      publish(source)
      first = actual_recv(sock)
      assert first.logMonoTime == source
      publish(later)
      return first

    def receive_latest(_timeout):
      message = actual_recv(latest)
      assert message.logMonoTime == later
      sd.sm.update_msgs(clock[0] / 1e9, [message])

    def receive_raw(sock):
      if sock is sd.car_state_sock:
        return None
      message = actual_optional(sock)
      if message is not None and message.logMonoTime > source:
        clock[0] = int(message.logMonoTime) + 1_000_000
      return message

    sd.sm.update.side_effect = receive_latest
    with patch('openpilot.selfdrive.selfdrived.selfdrived.messaging.recv_one', side_effect=receive_first), \
         patch('openpilot.starpilot.car.gm.steering_companion.messaging.recv_one_or_none', side_effect=receive_raw), \
         patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', side_effect=lambda: clock[0]):
      axis, _, cc = step(1)
    assert axis.sourceCarStateMonoTime == source
    assert axis.lateralActive and cc.latActive and bool(cc.longActive) == full_op
    assert log.OnroadEvent.EventName.controlsMismatch not in sd.events.names
    assert sd.steering_companion.samples[source][0] == source
    assert sd.steering_companion.latest is not None
    assert sd.steering_companion.latest[0] == later
    assert axis.validUntilMonoTime == axis.observedMonoTime + 30_000_000
