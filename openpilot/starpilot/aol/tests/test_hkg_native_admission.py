"""EV6/EV9 structural reuse must equal full validation after every live change."""
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.aol import runtime, vehicle, transport_pause
from openpilot.starpilot.aol.tests.test_native_observation_admission import receive
from openpilot.starpilot.aol.tests.test_panda_inventory import NOW, bound_state, sm_for
from openpilot.starpilot.aol.tests.test_runtime import car_state


@pytest.fixture(params=('ev6', 'ev6_two_pandas', 'ev9'))
def cp(request):
  if request.param == 'ev9':
    from openpilot.starpilot.tests.test_ev9_lead_inputs import long_params
    result = long_params()
  else:
    from openpilot.starpilot.tests.test_ev6_startup import params
    result = params()
    result.openpilotLongitudinalControl, result.pcmCruise = True, False
    result.safetyConfigs = ([{'safetyModel': 'noOutput', 'safetyParam': 0}] if request.param == 'ev6_two_pandas' else []) + [
      {'safetyModel': 'hyundaiCanfd', 'safetyParam': 0x815}]
  result.alternativeExperience = 32
  return result


def compare(sm, cp, *, now=NOW, session=None):
  cached = runtime.current_native(sm, cp, now_ns=now, axis_session_id=session)
  with patch.object(runtime, 'native_admission_key', return_value=None):
    expected = runtime.current_native(sm, cp, now_ns=now, axis_session_id=session)
  assert cached == expected
  return cached


def test_repeat_checks_reuse_one_packet_without_changing_cp_or_deadline(cp):
  before = cp.to_dict()
  state = bound_state(cp)
  sm = sm_for(cp, state)
  with patch.object(runtime, 'native_matches_cp', wraps=runtime.native_matches_cp) as profile:
    for tick in range(21):
      assert runtime.current_native(sm, cp, now_ns=NOW + tick * 10_000_000) == state
    assert profile.call_count == 1
    assert runtime.current_native(sm, cp, now_ns=state.validUntilMonoTime + 1) is None
    receive(sm, bound_state(cp, now=NOW + 300_000_000))
    assert runtime.current_native(sm, cp, now_ns=NOW + 300_000_000) is not None
    assert profile.call_count == 2
  assert cp.to_dict() == before


@pytest.mark.parametrize('field', ('flags', 'radarUnavailable', 'alphaLongitudinalAvailable', 'steerControlType',
                                  'passive', 'notCar', 'dashcamOnly', 'openpilotLongitudinalControl', 'pcmCruise',
                                  'alternativeExperience', 'safetyParam', 'safetyModel', 'safetyConfigs', 'carVin'))
def test_every_cp_mutation_revalidates_against_full_oracle(cp, field):
  state = bound_state(cp)
  sm = sm_for(cp, state)
  assert runtime.current_native(sm, cp, now_ns=NOW) == state
  key = vehicle.native_admission_key(cp)
  if field == 'safetyParam':
    cp.safetyConfigs[-1].safetyParam ^= 1
  elif field == 'safetyModel':
    cp.safetyConfigs[-1].safetyModel = 'noOutput'
  elif field == 'safetyConfigs':
    cp.safetyConfigs = []
  elif field == 'steerControlType':
    cp.steerControlType = 'torque' if str(cp.steerControlType) == 'angle' else 'angle'
  elif field in ('radarUnavailable', 'alphaLongitudinalAvailable', 'passive', 'notCar', 'dashcamOnly',
                 'openpilotLongitudinalControl', 'pcmCruise'):
    setattr(cp, field, not getattr(cp, field))
  elif field == 'carVin':
    cp.carVin = 'MUTATED'
  else:
    setattr(cp, field, int(getattr(cp, field)) ^ 1)
  assert vehicle.native_admission_key(cp) != key
  with patch.object(runtime, 'native_matches_cp', wraps=runtime.native_matches_cp) as profile:
    compare(sm, cp)
    assert profile.call_count == 2


@pytest.mark.parametrize('change', ('rx', 'heartbeat', 'fault', 'serial', 'inventory', 'lateral', 'longitudinal',
                                   'session', 'expired', 'future', 'invalid', 'not_alive', 'not_seen'))
def test_live_changes_match_full_oracle(cp, change):
  state = bound_state(cp)
  sm = sm_for(cp, state)
  assert runtime.current_native(sm, cp, now_ns=NOW) == state
  now, session = NOW, None
  if change in ('rx', 'heartbeat', 'fault', 'inventory'):
    field, value = {'rx': ('safetyRxChecksInvalid', True), 'heartbeat': ('heartbeatLost', True),
                    'fault': ('faults', 1), 'inventory': ('safetyParam', 0)}[change]
    slots = (*state.pandaInventory[:-1], replace(state.pandaInventory[-1], **{field: value}))
    receive(sm, replace(state, pandaInventory=slots))
  elif change in ('serial', 'lateral', 'longitudinal'):
    field, value = {'serial': ('pandaSerial', 'other'), 'lateral': ('lateralAllowed', False),
                    'longitudinal': ('longitudinalAllowed', False)}[change]
    receive(sm, replace(state, **{field: value}))
  elif change == 'session':
    session = 'other'
  elif change == 'expired':
    now = state.validUntilMonoTime + 1
  elif change == 'future':
    now -= 1
  else:
    field = {'invalid': 'valid', 'not_alive': 'alive', 'not_seen': 'seen'}[change]
    getattr(sm, field)['aolSafetyWire'] = False
  result = compare(sm, cp, now=now, session=session)
  if change in ('lateral', 'longitudinal'):
    assert result is not None and not getattr(result, 'lateralAllowed' if change == 'lateral' else 'longitudinalAllowed')
  else:
    assert result is None


def test_latest_panda_fault_veto_stays_live(cp):
  state = bound_state(cp)
  sm = sm_for(cp, state)
  assert transport_pause.healthy_transport(sm, cp, car_state(), state.axisSessionId, NOW)
  message = messaging.new_message('pandaStates', len(cp.safetyConfigs), valid=True, logMonoTime=state.sourcePandaStatesMonoTime)
  for i, panda in enumerate(sm['pandaStates']):
    message.pandaStates[i] = panda
  message.pandaStates[-1].heartbeatLost = True
  sm.update_msgs(NOW / 1e9, [message.as_reader()])
  assert runtime.current_native(sm, cp, now_ns=NOW) == state
  assert not transport_pause.healthy_transport(sm, cp, car_state(), state.axisSessionId, NOW)


def test_unknown_ducktype_never_serializes_or_caches():
  def unexpected_copy():
    raise AssertionError('unknown CP must not be serialized')

  cp = SimpleNamespace(brand='hyundai', carFingerprint='KIA_EV9', schema=None,
                       copy=unexpected_copy)
  assert vehicle.native_admission_key(cp) is None


def test_mutation_during_validation_cannot_store_old_snapshot(cp):
  state = bound_state(cp)
  sm = sm_for(cp, state)
  original = runtime.native_inventory_matches_cp

  def mutate(native, params):
    result = original(native, params)
    params.carVin = 'CHANGED-DURING-MISS'
    return result

  with patch.object(runtime, 'native_inventory_matches_cp', side_effect=mutate):
    assert runtime.current_native(sm, cp, now_ns=NOW) == state
  assert sm._aol_native_admission is None
  with patch.object(runtime, 'native_matches_cp', wraps=runtime.native_matches_cp) as profile:
    assert runtime.current_native(sm, cp, now_ns=NOW) == state
    assert profile.call_count == 1
