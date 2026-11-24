"""Compiled old/new occupied-field wire checks for optional preAP radar context."""
import capnp
import pytest

from openpilot.cereal import custom
from openpilot.starpilot.car.tesla.tests.schema_fixture import layout

HISTORICAL_SCHEMA = '''@0xb526ba661d550a59;
struct StarPilotCarControl @0x81c2f05a394cf4af {
  hudControl @0 :HUDControl;
  steeringLimitInfo @1 :SteeringLimitInfo;

  struct HUDControl {
    audibleAlert @0 :AudibleAlert;

    enum AudibleAlert {
      none @0;

      engage @1;
      disengage @2;
      refuse @3;

      warningSoft @4;
      warningImmediate @5;

      prompt @6;
      promptRepeat @7;
      promptDistracted @8;

      # Random Events
      angry @9;
      continued @10;
      dejaVu @11;
      doc @12;
      fart @13;
      firefox @14;
      goat @15;
      hal9000 @16;
      mail @17;
      nessie @18;
      noice @19;
      startup @20;
      thisIsFine @21;
      uwu @22;
    }
  }

  struct SteeringLimitInfo {
    valid @0 :Bool;
    modelLimitErrorDeg @1 :Float32;
    resumeLimitErrorDeg @2 :Float32;
    cooperativeLimitErrorDeg @3 :Float32;
    cooperativeOffsetDeg @4 :Float32;
    monoTime @5 :UInt64;
    combinedLimitErrorDeg @6 :Float32;
  }

  navigation @2 :Navigation;

  struct Navigation {
  sessionId @0 :Text;
    frameMonoTime @1 :UInt64;
    startedMonoTime @2 :UInt64;
    revision @3 :Text;
    enabled @4 :Bool;
    status @5 :Text;
    destinationName @6 :Text;
    instruction @7 :Instruction;
    route @8 :List(Coordinate);
    nextManeuver @9 :Instruction;
    locationMonoTime @10 :UInt64;
    controlValid @11 :Bool;

    struct Coordinate {
      latitude @0 :Float64;
      longitude @1 :Float64;
    }
    struct Instruction {
      text @0 :Text;
      maneuverType @1 :Text;
      maneuverModifier @2 :Text;
      distanceMeters @3 :Float32;
      remainingDistanceMeters @4 :Float32;
      remainingDurationSeconds @5 :Float32;
    }
    version @12 :UInt16;
  }
}
'''


@pytest.fixture
def historical(tmp_path):
  path = tmp_path / 'tesla-occupied-fields.capnp'
  path.write_text(HISTORICAL_SCHEMA)
  parser = capnp.SchemaParser()
  yield parser.load(str(path)).StarPilotCarControl


def populated(schema):
  return schema.new_message(
    hudControl={'audibleAlert': 'warningImmediate'},
    steeringLimitInfo={'valid': True, 'modelLimitErrorDeg': 1.25, 'resumeLimitErrorDeg': 2.5,
                       'cooperativeLimitErrorDeg': 3.75, 'cooperativeOffsetDeg': -0.5,
                       'monoTime': 123456789, 'combinedLimitErrorDeg': 4.0},
    navigation={'sessionId': 'tesla-old-context', 'frameMonoTime': 1234, 'startedMonoTime': 1000,
                'revision': 'v1', 'enabled': True, 'status': 'active', 'destinationName': 'Destination',
                'instruction': {'text': 'Continue', 'maneuverType': 'turn', 'maneuverModifier': 'left',
                                'distanceMeters': 12.5, 'remainingDistanceMeters': 30.0,
                                'remainingDurationSeconds': 45.0},
                'route': [{'latitude': 41.25, 'longitude': -87.5}],
                'nextManeuver': {'text': 'Then right', 'distanceMeters': 25.0},
                'locationMonoTime': 1100, 'controlValid': True, 'version': 1})


def assert_occupied_equal(old, new):
  for field in ('hudControl', 'steeringLimitInfo', 'navigation'):
    assert getattr(old, field).to_dict() == getattr(new, field).to_dict()


def test_historical_occupied_fields_decode_under_current_schema(historical):
  old = populated(historical)
  with custom.StarPilotCarControl.from_bytes(old.to_bytes()) as current:
    assert_occupied_equal(old, current)
    assert current.phoneGps.sourceMonoTime == 0


def test_current_extension_preserves_old_values_through_old_reader_and_relay(historical):
  current = populated(custom.StarPilotCarControl)
  current.phoneGps = {'sourceMonoTime': 123456, 'latitude': 41.25, 'longitude': -87.5, 'hasFix': True}
  original = current.to_bytes()
  startup = custom.StarPilotCarControl.PreapRadarStartup.new_message(
    version=1, kind=4, carParamsSha256=bytes(range(32)), safetyModel=39, safetyParam=6, lateralOffset=0.25)
  with custom.StarPilotCarControl.PreapRadarStartup.from_bytes(startup.to_bytes()) as decoded:
    assert decoded.to_dict() == startup.to_dict()
  current.clear_write_flag()
  assert current.to_bytes() == original
  with historical.from_bytes(original) as old:
    assert_occupied_equal(old, current)
    relayed = old.as_builder().to_bytes()
  with custom.StarPilotCarControl.from_bytes(relayed) as recovered:
    assert_occupied_equal(current, recovered)
    assert recovered.phoneGps.to_dict() == current.phoneGps.to_dict()


def test_occupied_field_slots_and_types_remain_identical(historical):
  current = custom.StarPilotCarControl
  for name in ('hudControl', 'steeringLimitInfo', 'navigation'):
    assert historical.schema.fields[name].proto.as_builder().to_bytes() == current.schema.fields[name].proto.as_builder().to_bytes()
  for nested in ('HUDControl', 'SteeringLimitInfo', 'Navigation'):
    old, new = getattr(historical, nested), getattr(current, nested)
    assert layout(old.schema) == layout(new.schema)
