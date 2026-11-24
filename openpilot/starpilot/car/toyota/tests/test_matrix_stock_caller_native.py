"""Manual Matrix stock PCM through real Card, SelfdriveD, Controls and native TX.

Synthetic boundaries are manual vehicle selection, CAN/perception producers,
input time and USB transport. Native permission, process health and actuator
commands are never forced. This does not establish hardware timing acceptance.
"""
from dataclasses import replace
import time

import pytest
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.aol.wire import decode_safety, encode_safety
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native = helpers.source_matched_native
daemon_gc_scope = helpers.daemon_gc_scope


def run(scenario, bridge):
  tick = 0
  actual_vetoes = 0
  fault_ns = 0

  def physical(packer, **fields):
    nonlocal tick, fault_ns
    tick += 1
    fields['sdsu'] = None
    if scenario in ('startup-main', 'standstill'):
      fields['cruise'] = False
      fields['stopped'] = scenario == 'standstill'
    if scenario == 'pcm-cancellation' and tick >= 260:
      fields.update(cruise=False, main=True)
    if scenario == 'settings-off' and 260 <= tick < 470:
      fields['cruise'] = False
    if scenario in ('eps-temporary', 'eps-permanent') and 260 <= tick < 315:
      fields['eps'] = 9 if scenario == 'eps-temporary' else 3
    if scenario == 'main-off' and tick >= 260:
      fields.update(main=False, cruise=False)
    frames = helpers.physical(packer, **fields)
    assert len(next(data for address, data, bus in frames if address == 0x262 and bus == 0)) == 5
    if scenario in ('main-expiry', 'eps-expiry') and 260 <= tick < 315:
      missing = 0x1D3 if scenario == 'main-expiry' else 0x262
      frames = [frame for frame in frames if not (frame[0] == missing and frame[2] == 0)]
    if scenario == 'required-rx' and 260 <= tick < 315:
      if tick == 260:
        fault_ns = time.monotonic_ns()
      frames = [(address, data[:-1] + bytes([data[-1] ^ 1]) if address == 0x260 and bus == 0 else data, bus)
                for address, data, bus in frames]
    return frames

  def on_state(current, host, cs):
    assert host.CP.carFingerprint == CAR.TOYOTA_MATRIX_RETROFIT
    assert host.CP.safetyConfigs[0].safetyParam == 856 and host.CP.flags == 0
    assert host.CP.pcmCruise and not host.CP.openpilotLongitudinalControl
    assert abs(host.CP.lateralTuning.torque.latAccelFactor - 4.05) < 1e-6
    assert abs(host.CP.lateralTuning.torque.friction - .10) < 1e-6
    if scenario == 'eps-temporary' and 270 <= current < 310:
      assert cs.steerFaultTemporary
      assert host.aol_card_intent.allowed_latch
    if scenario == 'eps-permanent' and 270 <= current < 310:
      assert cs.steerFaultPermanent
      # Toyota derives the request from physical MAIN. Faults veto commands;
      # this is not another family's explicit button latch.
      assert not host.aol_card_intent.explicit_latch
      assert host.aol_card_intent.allowed_latch == cs.cruiseState.available

  def observe(current, host, cs, cc, safety, native):
    assert not cc.longActive and not safety.aol_get_permission_mask() & 2
    if scenario == 'standstill' and 220 <= current < 250:
      assert host.CP.minSteerSpeed <= 0 and host.CP.steerAtStandstill
      assert cs.vEgo < .01 and cc.latActive and not cc.longActive
    if scenario == 'startup-main' and 220 <= current < 250:
      assert cs.cruiseState.available and not cs.cruiseState.enabled
      assert host.aol_card_intent is not None and host.aol_card_intent.allowed_latch
      assert cc.latActive and safety.aol_get_permission_mask() == 1
    if scenario == 'pcm-cancellation' and current >= 300:
      assert cs.cruiseState.available and not cs.cruiseState.enabled
      assert host.aol_card_intent.allowed_latch and cc.latActive
    if scenario in ('main-expiry', 'eps-expiry', 'required-rx', 'native-identity', 'eps-temporary', 'eps-permanent') and 306 <= current < 310:
      assert not cc.latActive and not cc.longActive
    if scenario == 'eps-permanent' and 270 <= current < 310:
      assert cs.steerFaultPermanent and not cc.latActive and not cc.longActive
      assert safety.aol_get_permission_mask() == 0
    if scenario == 'required-rx' and 270 <= current < 315:
      assert not cc.latActive and not cc.longActive
      assert safety.aol_get_permission_mask() == 0 and actual_vetoes > 0
      if current == 270:
        wire = next(event for event in native if event.which() == 'aolSafetyWire')
        observed = decode_safety(bytes(wire.aolSafetyWire))
        assert observed is not None and not observed.lateralAllowed and not observed.longitudinalAllowed
        assert fault_ns <= observed.observedMonoTime <= fault_ns + 100_000_000
        assert observed.validUntilMonoTime - observed.observedMonoTime == 200_000_000
    if scenario == 'main-off' and current >= 290:
      assert not cc.latActive and not cc.longActive
    if scenario == 'settings-off' and current == 470:
      assert not cc.latActive and not cc.longActive
    if scenario == 'settings-off' and current >= 480:
      assert cc.enabled and cc.latActive and not cc.longActive
      assert not host.aol_card_intent.settings.enabled
    if scenario == 'settings-off' and 300 <= current < 315:
      assert not host.aol_card_intent.settings.enabled
      assert not host.aol_card_intent.allowed_latch and not cc.latActive and not cc.longActive

  def rx_expected(address, data, bus, default, cp):
    del data, cp
    if scenario == 'required-rx' and address == 0x260 and bus == 0 and 260 <= tick < 315:
      return False
    return default

  def tx_expected(current, frame, cp, safety):
    nonlocal actual_vetoes
    if scenario == 'required-rx' and 260 <= current < 315 and frame.address == 0x2E4:
      assert not safety.safety_config_valid()
      assert safety.aol_get_permission_mask() == safety.aol_get_request_mask() == 0
      decoded = helpers.decode(cp, (frame.address, bytes(frame.dat), frame.src), 'STEERING_LKA')
      active = bool(decoded['STEER_REQUEST'] or decoded['STEER_TORQUE_CMD'])
      # A fresh 10Hz Panda report has not yet reached the host on the first
      # corrupt RX. The independent native hook must veto every active packet.
      actual_vetoes += int(active)
      return not active
    return True

  original_native = helpers.native_events

  def native_events(*args, **kwargs):
    messages = original_native(*args, **kwargs)
    if scenario == 'native-identity' and 260 <= tick < 315:
      result = []
      for event in messages:
        if event.which() == 'aolSafetyWire':
          wire = event.as_reader().as_builder()
          state = decode_safety(bytes(wire.aolSafetyWire))
          assert state is not None
          wire.aolSafetyWire = encode_safety(replace(state, safetyParam=state.safetyParam ^ 1))
          event = wire.as_reader()
        result.append(event)
      return result
    return messages

  permanent = scenario == 'main-off'
  actual_scenario = ('identity_refusal' if permanent else 'physical' if scenario == 'physical' else
                     'cold_low' if scenario == 'standstill' else 'nominal' if scenario == 'startup-main' else 'main_lease')
  with pytest.MonkeyPatch.context() as monkeypatch:
    if scenario == 'native-identity':
      monkeypatch.setattr(helpers, 'native_events', native_events)
    helpers.run_case(True, actual_scenario, candidate=CAR.TOYOTA_MATRIX_RETROFIT,
                     bridge=bridge, input_clock=True, aol=scenario != 'settings-off', sdsu_words=(856, 856),
                     firmware_override=[], fingerprint_frames=[], manual_selection=True,
                     physical_frames=physical, on_state=on_state, on_sample=observe,
                     native_rx_expected=rx_expected, native_tx_expected=tx_expected)


@pytest.mark.parametrize('scenario', ('startup-main', 'standstill', 'physical', 'pcm-cancellation', 'main-expiry', 'eps-expiry', 'required-rx',
                                      'native-identity', 'eps-temporary', 'eps-permanent', 'main-off', 'settings-off'))
def test_actual_matrix_stock_lifecycle(scenario, host_heartbeat_bridge):
  run(scenario, host_heartbeat_bridge)
