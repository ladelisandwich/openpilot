"""Volt route c0508: source cutoff must not replace an unexpired axis proof."""
from collections import deque
from contextlib import ExitStack
from unittest.mock import patch

import pytest

from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from openpilot.cereal import messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.aol.intent import read_settings
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.aol.tests.test_input_deadline_continuity import BASE, intent


def bind_steering_receipts(sd, step, cleanup):
  """Deliver the same declared producer receipt through its actual socket path."""
  if sd.steering_companion is None:
    return step
  pending = deque()
  original = messaging.recv_one_or_none

  def receive(sock):
    if sock is sd.steering_companion.sock:
      return pending.popleft() if pending else None
    return original(sock)

  cleanup.enter_context(patch('openpilot.selfdrive.selfdrived.selfdrived.messaging.recv_one_or_none', side_effect=receive))

  def observed_step(tick, **kwargs):
    if not kwargs.get('timeout', False):
      stamp = kwargs.get('state_stamp', 10_000_000_000 + tick * 10_000_000 + kwargs.get('offset_ns', 0))
      receipt = messaging.new_message('starpilotCarState', valid=True, logMonoTime=stamp)
      receipt.starpilotCarState.sourceCarStateMonoTime = stamp
      receipt.starpilotCarState.lateralAuthorityUnavailable = kwargs.get('authority_unavailable', False)
      kwargs['authority_message'] = receipt
      pending.append(receipt.as_reader())
    return step(tick, **kwargs)

  return observed_step


def profiles():
  from opendbc.car.gm.tests.test_volt_camera_control import camera_params
  from opendbc.car.gm.tests.test_volt_camera_removed import removed_params
  from opendbc.car.gm.tests.test_bolt_volt_configurations import ordinary_params
  from opendbc.car.gm.tests.test_bolt_pedal import params as bolt_params
  from opendbc.car.gm.values import CAR
  result = [sdgm_params(alpha=True, brake_c9=c9) for c9 in (False, True)]
  result += [camera_params(alpha=True)]
  result += [removed_params(alpha=True, alternate=alternate) for alternate in (False, True)]
  result += [ordinary_params(CAR.CHEVROLET_VOLT_ASCM, alpha=True, sascm=True,
    accelerator=not c9, radar=radar) for c9 in (False, True) for radar in (False, True)]
  result += [bolt_params(candidate, pedal=True, camera=not removed, removed=removed) for candidate in
    (CAR.CHEVROLET_BOLT_CC_2017, CAR.CHEVROLET_BOLT_CC_2018_2021, CAR.CHEVROLET_BOLT_CC_2022_2023,
     CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL) for removed in (False, True)]
  return result


@pytest.mark.parametrize('cp', [cp for cp in profiles() if cp.carFingerprint != 'CHEVROLET_VOLT_ASCM'],
                         ids=lambda cp: f'{cp.carFingerprint}-{cp.safetyConfigs[0].safetyParam:x}')
@pytest.mark.parametrize('mode', ['ordinary', 'brake', 'regen'])
@pytest.mark.parametrize('successor', ['fresh', 'missing'])
def test_volt_cutoff_keeps_only_original_lease_then_consumes_next_source(mode, successor, cp):
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    full = mode == 'ordinary'
    sd, controls, step = case._authority_callers(full, cp=cp.as_reader().as_builder(), genuine_events=True)
    step = bind_steering_receipts(sd, step, cleanup)
    sd.aol_settings = read_settings(sd.params)
    cs = fixture.car_state(brake=mode == 'brake')
    cs.regenBraking = mode == 'regen'
    old, _, command = step(2, state_stamp=BASE, offset_ns=5_467_396,
      intent_message=intent(BASE, 3), observed_state=cs, native_request_long=full)
    assert old.lateralActive and command.latActive and bool(command.longActive) == full
    deadline = old.validUntilMonoTime
    clock = [BASE + 29_900_000]
    actual_controls_receive = controls.sm.update_msgs

    def empty_control_receive(timeout):
      assert timeout == 0
      actual_controls_receive(clock[0] / 1e9, [])

    cleanup.enter_context(patch.object(controls.sm, 'update', side_effect=empty_control_receive))
    original = sd.update_process_health

    def finish_preamble():
      value = original()
      clock[0] = BASE + 32_985_260
      return value

    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', side_effect=lambda: clock[0]), \
         patch.object(sd, 'update_process_health', side_effect=finish_preamble):
      held, _, _ = step(5, timeout=True, state_stamp=BASE, offset_ns=-100_000,
        intent_message=intent(BASE, 3), observed_state=cs, native_request_long=full)
    from opendbc.car.gm.steering_authority import monitored_profile
    if monitored_profile(cp):
      # This monitored profile also needs its original steering-health receipt.
      # At this point that source is older than30ms. Its missing successor cannot
      # inherit health from an unexpired axis envelope or a fresh Panda grant.
      assert not sd.aol_axis_publication_deferred
      assert not held.lateralActive and not held.longitudinalActive
      assert sd.steering_authority_unavailable and sd.aol_authority_lost
      return
    assert sd.aol_axis_publication_deferred
    assert held.to_dict() == old.to_dict() and held.validUntilMonoTime == deadline
    assert not sd.aol_authority_lost
    # Consumers continue to enforce the OLD deadline independently.
    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', return_value=deadline + 1):
      expired, _ = controls.state_control()
    assert not expired.latActive and not expired.longActive
    fresh = BASE + 36_000_000
    axis, state, command = step(6, state_stamp=fresh, timeout=successor == 'missing',
      intent_message=intent(fresh, 4) if successor == 'fresh' else intent(BASE, 3),
      observed_state=cs, native_request_long=full)
    assert not sd.aol_axis_publication_deferred
    if successor == 'fresh':
      assert axis.sourceCarStateMonoTime == fresh and axis.lateralActive and command.latActive
      assert bool(command.longActive) == full and bool(state.enabled) == full
      assert not sd.aol_authority_lost
    else:
      assert not axis.lateralActive and not axis.longitudinalActive
      assert not command.latActive and not command.longActive


@pytest.mark.parametrize('bad', ['cancel', 'new_brake', 'new_paddle', 'door', 'belt', 'temporary_eps',
  'permanent_eps', 'invalid_can', 'main_off', 'native_deny', 'native_rx', 'producer', 'pause', 'deadline'])
def test_volt_deferral_cannot_hide_driver_edges_or_failed_proof(bad):
  from dataclasses import replace
  from opendbc.car.structs import car
  from openpilot.cereal import messaging
  from openpilot.starpilot.aol.wire import decode_intent, encode_intent, decode_safety, encode_safety

  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(False, cp=sdgm_params(alpha=True), genuine_events=True)
    step = bind_steering_receipts(sd, step, cleanup)
    sd.aol_settings = read_settings(sd.params)
    old, _, command = step(2, state_stamp=BASE, offset_ns=4_530_833,
      intent_message=intent(BASE, 3), native_request_long=False)
    assert old.lateralActive and command.latActive
    now = BASE + 30_721_946
    cs = sd.CS_prev.as_builder()
    if bad == 'cancel':
      cs.buttonEvents = [car.CarState.ButtonEvent(type='cancel', pressed=True)]
    elif bad in ('new_brake', 'new_paddle', 'door', 'belt', 'temporary_eps', 'permanent_eps', 'invalid_can', 'main_off'):
      if bad == 'main_off':
        cs.cruiseState.available = False
      else:
        setattr(cs, {'new_brake': 'brakePressed', 'new_paddle': 'regenBraking', 'door': 'doorOpen',
          'belt': 'seatbeltUnlatched', 'temporary_eps': 'steerFaultTemporary', 'permanent_eps': 'steerFaultPermanent',
          'invalid_can': 'canValid'}[bad], bad != 'invalid_can')
    elif bad.startswith('native_'):
      native = decode_safety(sd.sm['aolSafetyWire'])
      native = replace(native, lateralAllowed=False) if bad == 'native_deny' else replace(native,
        pandaInventory=tuple(replace(p, safetyRxChecksInvalid=True) for p in native.pandaInventory))
      message = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=native.observedMonoTime)
      message.aolSafetyWire = encode_safety(native)
      sd.sm.update_msgs(now / 1e9, [message.as_reader()])
    elif bad in ('producer', 'pause'):
      message = intent(BASE + 29_000_000, 4)
      value = decode_intent(message.aolIntentWire)
      message.aolIntentWire = encode_intent(replace(value, **(
        {'producerSessionId': 'different-card'} if bad == 'producer' else {'pauseLateral': True})))
      sd.aol_intent_companion._observe(message.logMonoTime, message.valid, message.aolIntentWire)
    else:
      now = old.validUntilMonoTime
    initial = (sd.state_machine.state, sd.state_machine.soft_disable_timer, sd.enabled, sd.active)
    sd.car_state_samples = ((cs.as_reader(), True),)
    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', return_value=now):
      assert not sd.defer_expired_axis_publication(cs, initial, now_ns=now)
    assert old.validUntilMonoTime == BASE + 34_530_833


@pytest.mark.parametrize('cp', [cp for cp in profiles() if cp.carFingerprint == 'CHEVROLET_VOLT_ASCM'],
                         ids=lambda cp: f'{cp.safetyConfigs[0].safetyParam:x}')
def test_ascm_separate_steering_companion_expiry_still_withdraws_outputs(cp):
  from openpilot.cereal import messaging
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with OpenpilotPrefix(), ExitStack() as cleanup:
    cleanup.callback(case.doCleanups)
    sd, _, step = case._authority_callers(True, cp=cp.as_reader().as_builder(), genuine_events=True)
    step = bind_steering_receipts(sd, step, cleanup)
    sd.aol_settings = read_settings(sd.params)
    from collections import deque
    queue = deque()
    original_update = sd.sm.update_msgs
    original_receive = messaging.recv_one_or_none
    def update(timestamp, frames):
      queue.extend(frame for frame in frames if frame.which() == 'starpilotCarState')
      return original_update(timestamp, frames)
    def receive(sock):
      if sock is sd.steering_companion.sock:
        return queue.popleft() if queue else None
      return original_receive(sock)
    cleanup.enter_context(patch.object(sd.sm, 'update_msgs', side_effect=update))
    cleanup.enter_context(patch.object(messaging, 'recv_one_or_none', side_effect=receive))
    authority = messaging.new_message('starpilotCarState', valid=True, logMonoTime=BASE)
    authority.starpilotCarState.sourceCarStateMonoTime = BASE
    # This family has an ADDITIONAL source-bound steering companion. Its
    # original lifetime is independent of the axis publication's later expiry.
    old, _, command = step(2, state_stamp=BASE, offset_ns=5_467_396,
      intent_message=intent(BASE, 3), authority_message=authority)
    assert old.lateralActive and command.latActive and command.longActive
    clock = [BASE + 29_900_000]
    original = sd.update_process_health
    def finish_preamble():
      value = original()
      clock[0] = BASE + 32_985_260
      return value
    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', side_effect=lambda: clock[0]), \
         patch.object(sd, 'update_process_health', side_effect=finish_preamble):
      axis, _, command = step(5, timeout=True, offset_ns=-100_000,
        intent_message=intent(BASE, 3), authority_message=authority)
    assert not sd.aol_axis_publication_deferred
    assert not axis.lateralActive and not command.latActive and not command.longActive
    assert old.validUntilMonoTime == BASE + 35_467_396
