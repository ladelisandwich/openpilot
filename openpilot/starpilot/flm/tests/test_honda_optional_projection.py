"""Actual Honda limiter/CAN emission retains optional unlimited integer on wire."""
import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence
from openpilot.starpilot.flm.learning_wire import read_applied, write_applied
from openpilot.starpilot.flm.projection_wire import read_projection, write_projection
from openpilot.starpilot.flm.tests.test_steering_projection import HONDA, apply, control, install_observer, make_interface, settle


def original_projection(*, clipped):
  ci = make_interface(HONDA.HONDA_CIVIC)
  assert install_observer(ci)
  settle(ci, control(0.))
  if clipped:
    actuators, packets, projection = apply(ci, control(.8))
    assert projection.unlimited_raw is None
  else:
    actuators, packets, projection = settle(ci, control(.002))[-1]
    assert type(projection.unlimited_raw) is int
  assert (projection.can_address, projection.can_data, projection.can_bus) in packets
  return projection, float(actuators.torque)


@pytest.mark.parametrize('clipped', [False, True])
def test_actual_honda_card_applied_wire_roundtrip(clipped):
  projection, applied = original_projection(clipped=clipped)
  evidence = AppliedEvidence(10_000_000_000, 10_000_000_000, 10_001_000_000,
    'normalized_torque', applied, True, True, False, False, clipped, True, projection)
  message = messaging.new_message('starpilotCarState', valid=True, logMonoTime=evidence.apply_ns)
  write_applied(message.starpilotCarState, evidence)
  decoded = read_applied(messaging.log_from_bytes(message.to_bytes()))
  assert decoded == evidence
  assert decoded.projection.unlimited_raw == projection.unlimited_raw
  assert decoded.limited is clipped
  assert decoded.projection.can_data == projection.can_data


def test_legacy_integer_projection_still_decodes_exactly():
  projection, _ = original_projection(clipped=False)
  wire = write_projection(projection)
  wire['version'] = 1
  wire.pop('unlimitedRawPresent')
  message = messaging.new_message('starpilotCarState')
  message.starpilotCarState.appliedEvidence.projection = wire
  decoded = read_projection(messaging.log_from_bytes(message.to_bytes()).starpilotCarState.appliedEvidence.projection)
  assert decoded == projection


@pytest.mark.parametrize('mutation', ['nonzero_absent', 'future_version'])
def test_optional_integer_does_not_hide_malformed_projection(mutation):
  projection, _ = original_projection(clipped=True)
  wire = write_projection(projection)
  if mutation == 'nonzero_absent':
    wire['unlimitedRaw'] = 1
  else:
    wire['version'] = 3
  message = messaging.new_message('starpilotCarState')
  message.starpilotCarState.appliedEvidence.projection = wire
  parsed = messaging.log_from_bytes(message.to_bytes()).starpilotCarState.appliedEvidence.projection
  with pytest.raises(ValueError):
    read_projection(parsed)
