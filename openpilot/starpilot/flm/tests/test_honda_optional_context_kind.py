"""Only the observer's actual optional-normalized-Honda case is wire absence."""
import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm.projection_wire import read_projection, write_projection
from openpilot.starpilot.flm.tests.test_steering_projection import HONDA, apply, control, install_observer, make_interface, settle


@pytest.mark.parametrize('field,value', [('family', 'toyota'), ('mappingKind', 'round'), ('limiterKind', 'driver')])
def test_absence_cannot_claim_another_native_context(field, value):
  ci = make_interface(HONDA.HONDA_CIVIC)
  assert install_observer(ci)
  settle(ci, control(0.))
  _, _, projection = apply(ci, control(.8))
  assert projection.unlimited_raw is None
  wire = write_projection(projection)
  wire[field] = value
  message = messaging.new_message('starpilotCarState')
  message.starpilotCarState.appliedEvidence.projection = wire
  parsed = messaging.log_from_bytes(message.to_bytes()).starpilotCarState.appliedEvidence.projection
  with pytest.raises(ValueError, match='invalid absent normalized Honda command'):
    read_projection(parsed)
