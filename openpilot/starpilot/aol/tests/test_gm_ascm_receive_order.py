"""Card's exact raw companion can precede the cached SubMaster projection."""
from contextlib import ExitStack
from unittest.mock import patch
import pytest
from opendbc.car.gm.tests.test_volt_sdgm_control import sdgm_params
from opendbc.car.gm.tests.test_ascm_intercept import params as ascm_params
from opendbc.car.gm.values import CAR, apply_gm_auto_hold
from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car.tests.publication_fixture import initialize_publication_sources
from openpilot.starpilot.aol.tests import test_ioniq_availability as fixture
from openpilot.starpilot.car.gm.steering_companion import SteeringCompanion


@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('radar', [False, True])
@pytest.mark.parametrize('hold', [False, True])
@pytest.mark.parametrize('full_op', [False, True])
@pytest.mark.parametrize('condition', ['healthy', 'sm_fault', 'sm_invalid', 'sm_wrong_source',
                                     'sm_expired', 'sm_expired_fault', 'sm_expired_invalid', 'sm_future', 'missing_exact', 'raw_fault',
                                     'raw_invalid', 'raw_wrong_source', 'raw_future', 'raw_expired', 'source_invalid'])
def test_ascm_exact_card_raw_companion_survives_older_cached_projection(c9, radar, hold, full_op, condition, prepared=False):
  from openpilot.selfdrive.car.card import Car
  from openpilot.starpilot.aol.tests.test_gm_source_deadline import bind_steering_receipts
  actual_recv, actual_optional = messaging.recv_one, messaging.recv_one_or_none
  case = fixture.IoniqAvailabilityTests(methodName='runTest')
  with ExitStack() as cleanup, OpenpilotPrefix():
    cleanup.callback(case.doCleanups)
    messaging.reset_context()
    cp = ascm_params(CAR.CHEVROLET_VOLT_ASCM, sascm=True, accelerator=not c9, radar=radar, alpha=True)
    apply_gm_auto_hold(cp, hold)
    if prepared:
      from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences
      VehicleStartupPreferences(volt_one_pedal=True, gm_auto_hold=True, volt_sng=True).prepare(cp)
      assert cp.safetyConfigs[0].safetyParam == 0xD114 and cp.flags == 16
    sd, _, step = case._authority_callers(full_op, cp=cp)
    from opendbc.car.gm.steering_authority import monitored_profile
    assert monitored_profile(sd.CP) and sd.CP.openpilotLongitudinalControl and not sd.CP.pcmCruise
    cp_before = sd.CP.to_dict()
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
    now = source + (31_000_000 if condition == 'raw_expired' else 4_000_000)
    older = now - (33_000_000 if condition.startswith('sm_expired') else 10_000_000)
    original_message = messaging.new_message

    def publish(stamp, *, fault=False, valid=True, enabled=True):
      authority.enabled, authority.latched = enabled, fault
      cs = fixture.car_state()
      cs.canValid = valid
      cs.cruiseState.available = True
      with patch('openpilot.selfdrive.car.card.messaging.new_message', side_effect=lambda *a, **kw:
                 original_message(*a, **dict(kw, logMonoTime=stamp))):
        card.state_publish(cs, None)

    publish(older, fault=condition in ('sm_fault','sm_expired_fault'), valid=condition not in ('sm_invalid','sm_expired_invalid'))
    assert actual_recv(sd.car_state_sock).logMonoTime == older
    cached = actual_recv(latest).as_builder()
    assert cached.logMonoTime == older
    if condition == 'sm_wrong_source':
      cached.starpilotCarState.sourceCarStateMonoTime -= 1
    if condition == 'sm_future':
      cached.logMonoTime = now + 1
      cached.starpilotCarState.sourceCarStateMonoTime = now + 1

    def receive_first(sock):
      publish(source, fault=condition == 'raw_fault', enabled=condition != 'missing_exact', valid=condition != 'source_invalid')
      first = actual_recv(sock)
      assert first.logMonoTime == source
      return first

    def receive_cached(_timeout):
      sd.sm.update_msgs(now / 1e9, [cached.as_reader()])

    def receive_raw(sock):
      if sock is sd.car_state_sock:
        return None
      message = actual_optional(sock)
      if message is not None and message.logMonoTime == source and condition in ('raw_invalid', 'raw_wrong_source', 'raw_future'):
        message = message.as_builder()
        if condition == 'raw_invalid':
          message.valid = False
        elif condition == 'raw_wrong_source':
          message.starpilotCarState.sourceCarStateMonoTime -= 1
        else:
          message.logMonoTime = now + 1
          message.starpilotCarState.sourceCarStateMonoTime = now + 1
        return message.as_reader()
      return message

    sd.sm.update.side_effect = receive_cached
    with patch('openpilot.selfdrive.selfdrived.selfdrived.messaging.recv_one', side_effect=receive_first), \
         patch('openpilot.starpilot.car.gm.steering_companion.messaging.recv_one_or_none', side_effect=receive_raw), \
         patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', return_value=now):
      axis, _, cc = step(1)
    assert sd.CP.to_dict() == cp_before
    healthy = condition in ('healthy', 'sm_expired')
    assert axis.sourceCarStateMonoTime == (0 if condition in ('raw_expired', 'source_invalid') else source)
    assert bool(axis.lateralActive) == healthy and bool(cc.latActive) == healthy
    assert bool(cc.longActive) == (healthy and full_op)
    assert (log.OnroadEvent.EventName.controlsMismatch in sd.events.names) == (not healthy)
    if healthy:
      assert sd.sm.logMonoTime['starpilotCarState'] == older
      assert sd.steering_companion.samples[source][0] == source
      assert axis.validUntilMonoTime == axis.observedMonoTime + 30_000_000



@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('radar', [False, True])
@pytest.mark.parametrize('hold', [False, True])
@pytest.mark.parametrize('alternative_experience', [32, 96])
def test_raw_projection_owner_keeps_exact_nonpedal_scope(c9, radar, hold, alternative_experience):
  from openpilot.starpilot.car.gm.aol import steering_raw_projection_owner
  cp = ascm_params(CAR.CHEVROLET_VOLT_ASCM, sascm=True, accelerator=not c9, radar=radar, alpha=True)
  cp.alternativeExperience = alternative_experience
  apply_gm_auto_hold(cp, hold)
  original = cp.to_dict()
  assert steering_raw_projection_owner(cp) == (alternative_experience == 32)
  for fault in ('unknown_ae', 'passive', 'dashcam', 'not_car', 'wrong_word', 'wrong_model', 'wrong_brand', 'wrong_identity', 'extra_panda'):
    invalid = cp.as_reader().as_builder()
    if fault == 'unknown_ae':
      invalid.alternativeExperience = 0xFF
    elif fault == 'passive':
      invalid.passive = True
    elif fault == 'dashcam':
      invalid.dashcamOnly = True
    elif fault == 'not_car':
      invalid.notCar = True
    elif fault == 'wrong_word':
      invalid.safetyConfigs[0].safetyParam = 0xFFFF
    elif fault == 'wrong_model':
      invalid.safetyConfigs[0].safetyModel = 'silent'
    elif fault == 'wrong_brand':
      invalid.brand = 'toyota'
    elif fault == 'wrong_identity':
      invalid.carFingerprint = 'UNKNOWN_VOLT'
    else:
      invalid.safetyConfigs = [invalid.safetyConfigs[0].to_dict()] * 2
    assert not steering_raw_projection_owner(invalid), fault
  ordinary = cp.as_reader().as_builder()
  ordinary.alternativeExperience = 0
  assert steering_raw_projection_owner(ordinary)
  assert cp.to_dict() == original


@pytest.mark.parametrize('topology', ['camera', 'gateway', 'ascm', 'sdgm'])
@pytest.mark.parametrize('alpha', [False, True])
@pytest.mark.parametrize('be', [False, True])
@pytest.mark.parametrize('release', [False, True])
def test_raw_projection_owner_excludes_separate_pedal_profiles(topology, alpha, be, release):
  from opendbc.car.gm.tests import test_volt_transitions as factory
  from openpilot.starpilot.car.gm.aol import steering_raw_projection_owner
  options = {'alpha': alpha, 'be': be, 'release': release}
  if topology in ('ascm', 'sdgm'):
    options['sascm'] = True
  cp = getattr(factory, 'volt_' + topology + '_pedal_params')(**options)
  cp.alternativeExperience = 32
  assert not steering_raw_projection_owner(cp)


@pytest.mark.parametrize('sdgm', [False, True])
@pytest.mark.parametrize('alpha,sascm', [(False, True), (True, False)])
def test_raw_projection_owner_includes_stock_ascm_without_changing_stock_sdgm(sdgm, alpha, sascm):
  from openpilot.starpilot.car.gm.aol import steering_raw_projection_owner
  cp = sdgm_params(alpha=alpha, sascm=sascm) if sdgm else ascm_params(CAR.CHEVROLET_VOLT_ASCM, alpha=alpha, sascm=sascm)
  cp.alternativeExperience = 32
  assert steering_raw_projection_owner(cp) == (not sdgm)


@pytest.mark.parametrize('full_op', [False, True])
@pytest.mark.parametrize('condition', ['healthy', 'sm_expired', 'sm_expired_fault', 'raw_fault'])
def test_recorded_sascm_one_pedal_hold_configuration(full_op, condition):
  test_ascm_exact_card_raw_companion_survives_older_cached_projection(False, True, True, full_op, condition, prepared=True)


@pytest.mark.parametrize('sdgm', [False, True])
@pytest.mark.parametrize('c9', [False, True])
@pytest.mark.parametrize('initial_active', [False, True])
@pytest.mark.parametrize('condition', ['sm_expired', 'sm_expired_fault', 'raw_fault', 'source_invalid', 'raw_expired'])
def test_normal_control_receipts_with_aol_disabled(sdgm, c9, initial_active, condition):
  from openpilot.common.params import Params
  from openpilot.selfdrive.car.card import Car
  from openpilot.selfdrive.controls.controlsd import Controls
  from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.selfdrive.selfdrived.state import StateMachine
  from openpilot.starpilot.aol.runtime import AxisDecision
  from openpilot.starpilot.lateral.tests.test_lane_runtime import feed

  with OpenpilotPrefix():
    cp = sdgm_params(alpha=True, brake_c9=c9) if sdgm else ascm_params(
      CAR.CHEVROLET_VOLT_ASCM, sascm=True, accelerator=not c9, radar=True, alpha=True)
    assert cp.alternativeExperience == 0
    saved = Params()
    saved.put_bool('OpenpilotEnabledToggle', True, block=True)
    saved.put_bool('AlwaysOnLateral', False, block=True)
    saved.put('CarParams', cp.to_bytes(), block=True)
    controls = Controls()
    assert not controls.aol_replay
    pm = messaging.PubMaster(['carState', 'carOutput', 'starpilotCarState'])
    cs_sock = messaging.sub_sock('carState', timeout=100)
    raw_sock = messaging.sub_sock('starpilotCarState', conflate=False)
    cached_sock = messaging.sub_sock('starpilotCarState', conflate=True, timeout=100)
    card = Car.__new__(Car)
    card.CP, card.pm = cp, pm
    initialize_publication_sources(card)
    card.sm = messaging.SubMaster(['carControl'], ignore_avg_freq=['carControl'])
    card.sm.update_msgs(10.0, [messaging.new_message('carControl', valid=True, logMonoTime=10_000_000_000).as_reader()])
    card.sm.frame = 1
    card.car_params_published = True
    card.last_actuators_output = car.CarControl.Actuators.new_message()
    card.slc_replay = card.curve_replay = card.aol_replay = False
    card.can_rcv_cum_timeout_counter = 0
    card.rk = Ratekeeper(100.0, print_delay_threshold=None)
    card.v_cruise_helper = VCruiseHelper(cp)
    now = 10_100_000_000
    source = now - (31_000_000 if condition == 'raw_expired' else 4_000_000)
    old = now - 33_000_000
    actual_message = messaging.new_message

    def publish(stamp, *, fault=False, valid=True):
      cs = fixture.car_state()
      cs.canValid = valid
      card.CI.CS.steering_authority.latched = fault
      with patch('openpilot.selfdrive.car.card.messaging.new_message', side_effect=lambda *a, **kw: actual_message(*a, **dict(kw, logMonoTime=stamp))):
        card.state_publish(cs, None)
      received = messaging.recv_one(cs_sock)
      assert received.logMonoTime == stamp
      return received

    publish(old, fault=condition == 'sm_expired_fault')
    cached = messaging.recv_one(cached_sock)
    sm = messaging.SubMaster(['starpilotCarState'], ignore_avg_freq=['starpilotCarState'])
    sm.update_msgs(now / 1e9, [cached])
    latest = publish(source, fault=condition == 'raw_fault', valid=condition != 'source_invalid')
    sd = SelfdriveD.__new__(SelfdriveD)
    sd.CP, sd.sm = cp.as_reader(), sm
    sd.active = initial_active
    sd.aol_axis_decision = AxisDecision()
    sd.aol_car_state_log_ns = source
    sd.conditional_car_state_valid = True
    sd.steering_authority_unavailable = False
    sd.steering_companion = SteeringCompanion(raw_sock)
    sd.events = Events()
    with patch('openpilot.selfdrive.selfdrived.selfdrived.REPLAY', False), \
         patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic_ns', return_value=now):
      sd._update_steering_authority(latest.carState)
    healthy = condition == 'sm_expired'
    machine = StateMachine()
    machine.state = log.SelfdriveState.OpenpilotState.enabled if initial_active else log.SelfdriveState.OpenpilotState.disabled
    enabled, active = machine.update(sd.events)
    for tick in range(10):
      feed(controls, now - (10 - tick) * 10_000_000, tick, enabled=False, active=False)
    feed(controls, now, 10, enabled=enabled, active=active)
    controls.sm.update_msgs(now / 1e9, [latest])
    with patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=now):
      command, _ = controls.state_control()
    assert command.latActive == command.longActive == (healthy and initial_active)
    assert active == enabled == (healthy and initial_active)
    assert not controls.CP.alternativeExperience and not saved.get_bool('AlwaysOnLateral')
    if initial_active:
      assert (log.OnroadEvent.EventName.controlsMismatch in sd.events.names) == (not healthy)
