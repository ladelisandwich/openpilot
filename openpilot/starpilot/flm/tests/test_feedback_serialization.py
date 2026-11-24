from dataclasses import asdict, replace
from types import SimpleNamespace

import pytest

from opendbc.car.steering_projection import CODECS, TorqueProjection, projection_to_dict
from openpilot.cereal import messaging
from openpilot.starpilot.flm import feedback_observation as f
from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence
from openpilot.starpilot.flm.physical_input_observation import body_digest


def projection(codec):
  family = codec.split('.')[0]
  limiter = 'normalized_rate' if family == 'honda' else 'measured' if family in ('toyota', 'chrysler') else 'driver'
  names = ('STEER_DELTA_UP', 'STEER_DELTA_DOWN')
  if limiter == 'driver':
    names += ('STEER_DRIVER_ALLOWANCE', 'STEER_DRIVER_MULTIPLIER', 'STEER_DRIVER_FACTOR')
  elif limiter == 'measured':
    names += ('STEER_ERROR_MAX',)
  return TorqueProjection(1, family, 'recorded-family-fixture', 0, codec, 'honda_lookup' if family == 'honda' else 'round',
                          1, 0.125, 0.125, None, 10, 100.0,
                          (-1.0, 1.0) if family == 'honda' else (), (-100.0, 100.0) if family == 'honda' else (),
                          limiter, -0.0, 0.0, tuple((name, 1.0) for name in names), (), True, 0x123, 0, b'\x00\xff', 0.125)


def applied(witness):
  return AppliedEvidence(50, 60, 70, 'normalized_torque', 0.125, True, True, False, False, False, True, witness, 1, 80)


def old_digest(value):
  witness = projection_to_dict(value.projection)
  if witness is None:
    raise ValueError('bounded actual projection body')
  return body_digest({**asdict(value), 'projection': witness})


@pytest.mark.parametrize('codec', sorted(CODECS))
@pytest.mark.parametrize('scalar', [0.0, -0.0, 0.125, -1.0])
def test_applied_digest_exact_across_all_supported_projection_codecs(codec, scalar):
  value = replace(applied(projection(codec)), applied=scalar)
  assert f.applied_digest(value) == old_digest(value)
  changed = replace(value, carrier_frame=2)
  assert f.applied_digest(changed) == old_digest(changed) != old_digest(value)


@pytest.mark.parametrize('field,value', [('final_raw', True), ('frame', -1), ('can_data', bytearray(b'x')),
                                         ('shape', (1.0,)), ('limits', ()), ('reported_torque', float('nan'))])
def test_malformed_projection_remains_rejected(field, value):
  item = applied(projection('hyundai.LKAS11')._replace(**{field: value}))
  with pytest.raises(ValueError, match='bounded actual projection body'):
    f.applied_digest(item)


def test_nonprimitive_field_keeps_original_deepcopy_fallback():
  class Number:
    def __deepcopy__(self, memo):
      return 0.25
  item = replace(applied(projection('hyundai.LKAS11')), applied=Number())
  assert f.applied_digest(item) == old_digest(item)


def test_real_caroutput_publication_keeps_exact_binding_and_detects_held_mutation():
  held = messaging.new_message('carOutput')
  held.carOutput.actuatorsOutput.torque = 0.125
  capture = f.CardFeedbackCapture('a' * 64, 'b' * 64)
  capture.begin_assignment()
  value = applied(projection('hyundai.LKAS11'))
  capture.committed(held.carOutput.actuatorsOutput, value)
  event = messaging.new_message('carOutput')
  event.logMonoTime = 100
  event.valid = True
  event.carOutput.actuatorsOutput = held.carOutput.actuatorsOutput
  row = capture.publication(event)
  assert row.output_known and row.applied_sha256 == old_digest(value)
  assert row.co_body_sha256 == f.co_body(100, True, event.carOutput.actuatorsOutput.to_dict())
  assert (row.output_control_ns, row.output_state_ns, row.output_apply_ns, row.carrier_observed_ns) == (50, 60, 70, 80)
  held.carOutput.actuatorsOutput.torque = 0.25
  event.logMonoTime = 101
  with pytest.raises(ValueError, match='held owner output changed'):
    capture.publication(event)
  assert capture.output is None and capture.applied is None


def test_unknown_publication_never_invents_apply_evidence():
  capture = f.CardFeedbackCapture('a' * 64, 'b' * 64)
  event = messaging.new_message('carOutput')
  event.logMonoTime = 100
  row = capture.publication(event)
  assert not row.output_known and row.applied_sha256 == ''
  assert (row.output_control_ns, row.output_state_ns, row.output_apply_ns, row.carrier_observed_ns) == (0, 0, 0, 0)


@pytest.mark.parametrize('item', [None, SimpleNamespace(projection=projection('hyundai.LKAS11'))])
def test_exact_applied_type_stays_required(item):
  with pytest.raises(ValueError, match='exact typed held applied evidence'):
    f.applied_digest(item)


def test_custom_projection_limit_name_preserves_discarded_deepcopy_callbacks():
  calls = []
  class Name(str):
    def __deepcopy__(self, memo):
      calls.append(str(self))
      return str(self)
  witness = projection('hyundai.LKAS11')
  witness = witness._replace(limits=((Name(witness.limits[0][0]), witness.limits[0][1]), *witness.limits[1:]))
  item = applied(witness)
  for digest in (old_digest, f.applied_digest):
    calls.clear()
    with pytest.raises(ValueError, match='physical exact original body primitive'):
      digest(item)
    assert calls == ['STEER_DELTA_UP']


def test_custom_operand_mutation_preserves_original_field_copy_order():
  def make():
    item = applied(projection('hyundai.LKAS11'))
    class Number:
      def __deepcopy__(self, memo):
        object.__setattr__(item, 'carrier_frame', 9)
        return 0.25
    object.__setattr__(item, 'applied', Number())
    return item
  assert f.applied_digest(make()) == old_digest(make())
