import math
import random
import struct
import warnings

import numpy as np
import pytest

from openpilot.common import pid
from openpilot.common.scalar_clip import clip_float64


def outcome(function, *values):
  with warnings.catch_warnings(record=True) as seen, np.errstate(all='warn'):
    warnings.simplefilter('always')
    try:
      result = function(*values)
      array = np.asarray(result)
      value = ('value', type(result), array.dtype.str, array.shape, array.tobytes())
    except Exception as error:
      value = ('error', type(error), str(error))
  return value, [(item.category, str(item.message)) for item in seen]


def test_float64_clip_exact_bits_types_errors_and_warnings():
  bits = (0, 1, 1 << 63, (1 << 63) + 1, 0x7ff0000000000001, 0xfff0000000000001,
          0x7ff8000000004321, 0xfff8000000004321, 0x7ff0000000000000, 0xfff0000000000000)
  values = [struct.unpack('>d', value.to_bytes(8, 'big'))[0] for value in bits]
  randomizer = random.Random(20261024)
  values += [struct.unpack('>d', randomizer.getrandbits(64).to_bytes(8, 'big'))[0] for _ in range(1500)]
  bounds = ((-1.0, 1.0), (-0.0, 1.0), (-1.0, 0.0), (-1.0, -0.0),
            (-0.0, 0.0), (0.0, -0.0), (1.0, 1.0), (1.0, -1.0), (-math.inf, math.inf), (math.nan, 1.0))
  for value in values:
    for lower, upper in bounds:
      for typed in (value, np.float64(value)):
        assert outcome(clip_float64, typed, lower, upper) == outcome(np.clip, typed, lower, upper)


@pytest.mark.parametrize('value', [0, True, np.int64(1), np.float32(0.125), np.array(0.125),
                                  np.array([0.125, 2.0]), np.array([1, 2]), None, 'bad', 1 + 2j])
def test_other_domains_keep_original_numpy(value):
  for lower, upper in ((0, 1), (-1.0, 1.0), (None, 1.0), (1, 0), ('x', 1)):
    assert outcome(clip_float64, value, lower, upper) == outcome(np.clip, value, lower, upper)


def state(controller):
  return tuple((type(value), np.asarray(value).dtype.str, np.asarray(value).tobytes())
               for value in (controller.p, controller.i, controller.d, controller.f, controller.control))


@pytest.mark.parametrize('gains', [(0.3, 0.05, 0.0), (([0.0, 30.0], [0.2, 0.4]), ([0.0, 30.0], [0.05, 0.1]), 0.01)])
@pytest.mark.parametrize('limits', [(-1.0, 1.0), (-0.0, 0.0), (0.0, -0.0), (-1e308, 1e308)])
def test_actual_pid_trace_preserves_outputs_and_integrator_bits(monkeypatch, gains, limits):
  baseline = pid.PIDController(*gains, neg_limit=limits[0], pos_limit=limits[1])
  candidate = pid.PIDController(*gains, neg_limit=limits[0], pos_limit=limits[1])
  randomizer = random.Random(20261024)
  for index in range(600):
    args = {'error': randomizer.uniform(-5.0, 5.0), 'error_rate': randomizer.uniform(-2.0, 2.0),
            'speed': randomizer.uniform(0.0, 40.0), 'feedforward': randomizer.uniform(-1.0, 1.0),
            'freeze_integrator': index % 7 == 0}
    monkeypatch.setattr(pid, 'clip_float64', np.clip)
    # Both run the actual PID update; only the original NumPy operation differs.
    with warnings.catch_warnings(record=True) as original_warnings:
      warnings.simplefilter('always')
      baseline.update(**args)
    monkeypatch.setattr(pid, 'clip_float64', clip_float64)
    with warnings.catch_warnings(record=True) as candidate_warnings:
      warnings.simplefilter('always')
      candidate.update(**args)
    assert state(candidate) == state(baseline)
    assert [(w.category, str(w.message)) for w in candidate_warnings] == [(w.category, str(w.message)) for w in original_warnings]
