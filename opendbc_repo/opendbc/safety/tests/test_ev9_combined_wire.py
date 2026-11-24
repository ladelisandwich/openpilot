from openpilot.cereal import messaging
from openpilot.starpilot.aol.tests.test_classic_location_stock_intent import host_heartbeat_bridge as host_heartbeat_bridge
from openpilot.starpilot.aol.wire import IntentState, encode_intent
from opendbc.safety.tests import test_ev9_combined_aol as native_fixture


def axis_and_intent(now, *, session='ev9-combined-axis', long_requested=False):
  axis = messaging.new_message('aolAxisState', valid=True, logMonoTime=now)
  axis.aolAxisState = {'qualified': True, 'sessionId': session, 'observedMonoTime': now,
                      'validUntilMonoTime': now + 30_000_000, 'sourceCarStateMonoTime': now,
                      'desiredLateral': True, 'desiredLongitudinal': long_requested,
                      'intentProducerSessionId': 'ev9-combined-card'}
  intent = messaging.new_message('aolIntentWire', 0, valid=True, logMonoTime=now)
  intent.aolIntentWire = encode_intent(IntentState('ev9-combined-card', 1, now, now, now + 30_000_000,
                                                 True, False, False, True, True))
  return axis.to_bytes(), intent.to_bytes()


def test_exact_ev9_cpp_wire_negotiates_real_native_lateral_only_and_rejects_old_sessions(host_heartbeat_bridge):
  case = native_fixture.TestEV9CombinedDebug(methodName='runTest')
  try:
    case.setUp()
    case.warm()
    # Observe the physical gesture; the real C++ plan supplies every heartbeat/request.
    case.tick(lkas=1)
    case.tick()
    native = case.safety
    host_heartbeat_bridge.reset_host()
    plans = []
    for _ in range(4):
      case.tick()
      now = case.now * 1000
      axis, intent = axis_and_intent(now)
      plan = host_heartbeat_bridge.plan_host(28, 0x5c95, native.aol_get_request_mask(), native.aol_get_permission_mask(),
                                             now, axis, len(axis), intent, len(intent), False)
      plans.append(plan)
      native.aol_set_host_request(plan & 0xff)
      native.set_aol_test_heartbeat(bool(plan & 0x100))
      host_heartbeat_bridge.complete_host(native.aol_get_request_mask(), native.aol_get_permission_mask())
    assert any(plan & 1 for plan in plans)
    assert native.aol_get_permission_mask() == 1
    assert not native.get_controls_allowed()
    assert native.safety_tx_hook(case.angle(value=1.1))
    assert not native.safety_tx_hook(case.scc(1.))

    # A changed producer session withdraws before any new authority is considered.
    case.tick()
    now = case.now * 1000
    axis, intent = axis_and_intent(now, session='changed-ev9-axis')
    plan = host_heartbeat_bridge.plan_host(28, 0x5c95, native.aol_get_request_mask(), native.aol_get_permission_mask(),
                                           now, axis, len(axis), intent, len(intent), False)
    assert plan & 3 == 0
    native.aol_set_host_request(plan & 0xff)
    native.set_aol_test_heartbeat(bool(plan & 0x100))
    host_heartbeat_bridge.complete_host(native.aol_get_request_mask(), native.aol_get_permission_mask())
    assert native.aol_get_permission_mask() == 0
    assert not native.safety_tx_hook(case.angle())

    # Adjacent unregistered profile and expired axis envelopes cannot acquire a request.
    for word, old in ((0x5c97, False), (0x5c95, True)):
      host_heartbeat_bridge.reset_host()
      for _ in range(3):
        case.tick()
        now = case.now * 1000
        axis, intent = axis_and_intent(now - 300_000_000 if old else now)
        plan = host_heartbeat_bridge.plan_host(28, word, native.aol_get_request_mask(), native.aol_get_permission_mask(),
                                               now, axis, len(axis), intent, len(intent), False)
        assert plan & 3 == 0
        native.aol_set_host_request(plan & 0xff)
        native.set_aol_test_heartbeat(bool(plan & 0x100))
        host_heartbeat_bridge.complete_host(native.aol_get_request_mask(), native.aol_get_permission_mask())
        assert native.aol_get_permission_mask() == 0
  finally:
    case.doCleanups()
