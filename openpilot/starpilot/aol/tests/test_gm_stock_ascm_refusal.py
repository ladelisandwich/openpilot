"""Stock ASCM refusal must withdraw actual callers and publish a takeover alert."""
from contextlib import ExitStack
import os
from unittest import mock

import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.gm import gmcan
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.car.gm.tests.test_ascm_intercept import params
from opendbc.car.gm.values import CAR, GMFlags, DBC
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.starpilot.aol.tests.test_gm_sdgm_source_wait import BASE, joined_case
from openpilot.starpilot.aol.tests.gm_gap_cpp_bridge import gm_host_heartbeat_bridge as gm_host_heartbeat_bridge


@pytest.mark.parametrize('c9', [False, True])
def test_stock_ascm_refusal_withdraws_output_and_publishes_takeover(gm_host_heartbeat_bridge, c9):
  cp = params(CAR.CHEVROLET_VOLT_ASCM, accelerator=not c9, sascm=False, alpha=False)
  cp.flags |= int(GMFlags.HAS_BSM)
  assert cp.pcmCruise and not cp.openpilotLongitudinalControl
  assert cp.safetyConfigs[0].safetyParam == (0x605 if c9 else 0x205)
  with OpenpilotPrefix(), ExitStack() as cleanup, mock.patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    card, sd, controls, native, current, physical, sample_native, card_receive, drive, apply_command = joined_case(
      cleanup, gm_host_heartbeat_bridge, c9=c9, cp=cp)
    assert not sd.enabled and not native.get_controls_allowed()
    original_update = controls.sm.update_msgs

    def curved_model(timestamp, messages):
      # Supply a nonzero model input; Controls, packing and native checks stay real.
      changed = []
      for message in messages:
        if message.which() == 'modelV2':
          message = message.as_builder()
          message.modelV2.action.desiredCurvature = .002
          message = message.as_reader()
        changed.append(message)
      return original_update(timestamp, changed)

    cleanup.enter_context(mock.patch.object(controls.sm, 'update_msgs', side_effect=curved_model))
    positive_torque = False
    for tick in range(26, 42):
      now = BASE + tick * 10_000_000
      rows = physical(now, tick, eps=1)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, tick)
      apply_command(now + 2, command)
      assert axis.lateralActive and command.latActive and sample_native(now + 3) == 1
      positive_torque |= card.last_actuators_output.torqueOutputCan != 0
    assert positive_torque
    warning = None
    neutral_sent = False
    for tick in range(42, 56):
      now = BASE + tick * 10_000_000
      rows = physical(now, tick, eps=0)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, tick)
      apply_command(now + 2, command)
      mask = sample_native(now + 3)
      assert card.CI.CS.steering_authority.latched, {
        'monitorEnabled': card.CI.CS.steering_authority.enabled,
        'axisActive': axis.lateralActive, 'commandActive': command.latActive,
        'requestedTorque': command.actuators.torque, 'nativeMask': mask, 'alert': state.alertType,
      }
      assert current['last_cs'].steerFaultTemporary and not current['last_cs'].steerFaultPermanent
      assert not axis.lateralActive and not command.latActive and not command.longActive
      assert mask == 0
      sent = card.pm.messages.get('sendcan')
      for frame in sent.sendcan if sent is not None else ():
        if frame.address == 0x180 and frame.src == 0:
          payload = bytes(frame.dat)
          assert not payload[0] & 8 and ((payload[0] & 7) << 8 | payload[1]) == 0
          neutral_sent = True
      assert EventName.controlsMismatch in sd.events.names
      if state.alertText1:
        warning = state
    assert neutral_sent and not card.last_actuators_output.torqueOutputCan
    assert warning is not None
    assert warning.alertText1 == 'TAKE CONTROL IMMEDIATELY'
    assert str(warning.alertSound) == 'warningImmediate'
    delivered = controls.sm['selfdriveState']
    assert delivered.alertText1 == warning.alertText1
    assert str(delivered.alertSound) == 'warningImmediate'
    # A valid small active command is rejected by the native side after OFF.
    packer = CANPacker(DBC[card.CP.carFingerprint][Bus.pt])
    address, data, bus = gmcan.create_steering_control(packer, 0, 1, 0, True)
    assert not native.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
    # Healthy feedback alone must not clear a refusal or restore actuation.
    for tick in range(56, 66):
      now = BASE + tick * 10_000_000
      rows = physical(now, tick, eps=1)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, tick)
      apply_command(now + 2, command)
      assert card.CI.CS.steering_authority.latched
      assert not axis.lateralActive and not command.latActive and sample_native(now + 3) == 0
    for tick in range(66, 71):
      now = BASE + tick * 10_000_000
      rows = physical(now, tick, eps=1, main=False)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, tick)
      apply_command(now + 2, command)
      assert not command.latActive and sample_native(now + 3) == 0
    recovered = False
    for tick in range(71, 87):
      now = BASE + tick * 10_000_000
      rows = physical(now, tick, eps=1, main=True)
      card_receive(now, rows)
      axis, state, command = drive(now + 1, tick)
      apply_command(now + 2, command)
      mask = sample_native(now + 3)
      recovered |= bool(axis.lateralActive and command.latActive and mask == 1)
      assert not command.longActive and not state.enabled
    assert recovered and not card.CI.CS.steering_authority.latched
