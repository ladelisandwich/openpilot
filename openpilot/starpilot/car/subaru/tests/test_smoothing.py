"""Frozen source/CarParams association and exact Dom speed schedule."""
import json
import math
from pathlib import Path
import pytest
from opendbc.car import structs
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR
from openpilot.cereal import log
from openpilot.starpilot.car.subaru import smoothing as tune


class Saved:
  def __init__(self, root):
    self.root = root
    root.mkdir(parents=True, exist_ok=True)

  def get_param_path(self, key):
    return str(self.root / key)

  def put(self, key, raw, block=True):
    Path(self.get_param_path(key)).write_bytes(raw)


def cp_for(candidate=CAR.SUBARU_OUTBACK_2023):
  return CarInterface.get_params(candidate, {0: {}, 1: {}, 2: {}}, [], False, False, False)


def installed(tmp_path, *, candidate=CAR.SUBARU_OUTBACK_2023, manual=None):
  params = Saved(tmp_path)
  if manual is not None:
    params.put('DeveloperUI', b'1')
    params.put(tune.KEY, manual)
  snapshot = tune.read_snapshot(params, enabled=True)
  cp = tune.prepare(cp_for(candidate), snapshot)
  raw = cp.to_bytes()
  tune.install(cp, raw, snapshot, params)
  params.put('CarParams', raw)
  return cp, params, snapshot


@pytest.mark.parametrize('raw', [b'nan', b'inf', b'-inf', b'-0.005', b'2.005', b'.001', b'', b'false', b'1e9999'])
def test_malformed_choice_is_not_a_tune(raw):
  with pytest.raises(ValueError):
    tune.selected(raw)


@pytest.mark.parametrize('candidate,expected', [(CAR.SUBARU_OUTBACK_2023, .4), (CAR.SUBARU_FORESTER_2022, 0.),
                                               (CAR.SUBARU_ASCENT_2023, 0.), (CAR.SUBARU_FORESTER, 0.)])
def test_platform_defaults_and_no_native_authority(tmp_path, candidate, expected):
  original = cp_for(candidate)
  cp, params, _ = installed(tmp_path, candidate=candidate)
  assert tune.read_frozen(cp, params) == expected
  assert cp.flags == original.flags | tune.MARKER
  before, after = original.to_dict(), cp.to_dict()
  before['flags'] = after['flags']
  assert before == after


@pytest.mark.parametrize('manual', [b'0', b'.005', b'.4', b'2'])
def test_one_epoch_and_same_model_control_choice(tmp_path, manual):
  cp, params, _ = installed(tmp_path, manual=manual)
  model_choice = tune.read_frozen(cp, params)
  params.put(tune.KEY, b'1.5')
  params.put('DeveloperUI', b'0')
  controls_choice = tune.read_frozen(cp, params)
  assert model_choice == controls_choice == float(manual)
  for speed in (-1., 0., 2., 5., 8., 30.):
    expected = float(manual) * max(0., min(1., (8. - speed) / 6.))
    assert tune.seconds(speed, model_choice) == tune.seconds(speed, controls_choice) == expected


@pytest.mark.parametrize('mutation', ['maximum', 'version', 'hash', 'source', 'duplicate', 'nan', 'oversize', 'cp', 'missing'])
def test_corrupt_or_stale_context_rejects(tmp_path, mutation):
  cp, params, _ = installed(tmp_path)
  source = Path(params.get_param_path(tune.CONTEXT_KEY))
  raw = source.read_bytes()
  doc = json.loads(raw)
  if mutation == 'maximum':
    doc['maximum'] = '1'
  elif mutation == 'version':
    doc['version'] = True
  elif mutation == 'hash':
    doc['carParamsSha256'] = '0' * 64
  elif mutation == 'source':
    doc['sources'][0][1] = 'zz'
  elif mutation == 'duplicate':
    raw = raw[:-1] + b',"version":1}'
  elif mutation == 'nan':
    raw = raw.replace(b'"version":1', b'"version":NaN')
  elif mutation == 'oversize':
    raw = b' ' * 513
  elif mutation == 'cp':
    cp.steerRatio += 1.
  elif mutation == 'missing':
    raw = b''
  source.write_bytes(json.dumps(doc).encode() if mutation in ('maximum', 'version', 'hash', 'source') else raw)
  with pytest.raises(RuntimeError, match='startup context'):
    tune.read_frozen(cp, params)


def test_saved_change_during_discovery_stops_publication(tmp_path):
  params = Saved(tmp_path)
  snapshot = tune.read_snapshot(params, enabled=True)
  cp = tune.prepare(cp_for(), snapshot)
  params.put(tune.KEY, b'.4')
  with pytest.raises(RuntimeError, match='changed during startup'):
    tune.install(cp, cp.to_bytes(), snapshot, params)


@pytest.mark.parametrize('developer,safe,raw,expected', [(b'0', b'0', b'2', .4), (b'1', b'1', b'2', .4),
                                                        (b'1', b'0', b'nan', .4), (b'1', b'0', b'0', 0.)])
def test_manual_authority_and_zero_off(tmp_path, developer, safe, raw, expected):
  params = Saved(tmp_path)
  for key, value in [('DeveloperUI', developer), ('SafeMode', safe), (tune.KEY, raw)]:
    params.put(key, value)
  snapshot = tune.read_snapshot(params, enabled=True)
  assert tune.maximum_for(cp_for(), snapshot) == expected


def test_common_action_filter_retains_longitudinal_and_stop(tmp_path):
  previous = log.ModelDataV2.Action(desiredCurvature=.01, desiredAcceleration=-1., shouldStop=True)
  for tau in (0., .005, .4, 2.):
    action = log.ModelDataV2.Action(desiredCurvature=.02, desiredAcceleration=.7, shouldStop=False)
    tune.filter_action(action, previous, 2., tau)
    assert action.desiredAcceleration == pytest.approx(.7) and not action.shouldStop
    expected = .02 if tau == 0. else .01 + (.02 - .01) * (1. - math.exp(-.05 / tau))
    assert action.desiredCurvature == pytest.approx(expected)
  assert tune.seconds(math.nan, .4) == 0.
  assert tune.seconds(2., math.inf) == 0.
  assert tune.prepare(cp_for(), None).flags & tune.MARKER == 0
  foreign = structs.CarParams(brand='mock')
  assert tune.read_frozen(foreign, Saved(tmp_path / 'unused')) == 0.



def test_component_compatibility_cannot_skip_a_marked_startup():
  from types import SimpleNamespace
  cp = cp_for()
  partial = SimpleNamespace(CP=cp)
  assert tune.control_maximum(partial) == 0.
  cp.flags |= tune.MARKER
  with pytest.raises(RuntimeError, match='startup was not initialized'):
    tune.control_maximum(partial)
  partial.subaru_smoothing = .4
  assert tune.control_maximum(partial) == .4
  foreign = SimpleNamespace(CP=SimpleNamespace(brand='toyota', flags=tune.MARKER))
  assert tune.control_maximum(foreign) == 0.
