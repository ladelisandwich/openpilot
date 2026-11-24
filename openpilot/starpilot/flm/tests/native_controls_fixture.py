"""Private real SubMaster -> Controls fixture extension, not a mock parameter setter."""
from unittest import mock

from openpilot.cereal import messaging
from openpilot.selfdrive.locationd.torqued import VERSION
from openpilot.starpilot.flm.tests.test_registered_controls_global import feed, source_packet


def native_packet(controls, stamp, *, factor=None, offset=0., friction=None,
                  valid=True, use_params=True, outer_valid=True, version=VERSION, source_offset_ns=0):
  cp_tune = controls.CP.lateralTuning.torque
  event = messaging.new_message('lateralTorqueParameters', valid=outer_valid, logMonoTime=stamp + source_offset_ns)
  row = event.lateralTorqueParameters
  row.version, row.valid, row.useParams = version, valid, use_params
  row.latAccelFactorFiltered = cp_tune.latAccelFactor if factor is None else factor
  row.latAccelOffsetFiltered = offset
  row.frictionCoefficientFiltered = cp_tune.friction if friction is None else friction
  row.totalBucketPoints, row.calPerc, row.decay = 4000, 100., 50.
  return messaging.log_from_bytes(event.to_bytes())


def native_frame(controls, now, tick, *, native=True, native_overrides=None, **health):
  """Publish every4Hz and use production state_control, actual selection and update."""
  feed(controls, now, tick, speed=30., calibration=(0., 0., 0.), **health)
  if now - controls._global_fixture_source_ns >= 250_000_000:
    controls.sm.update_msgs(now / 1e9, [source_packet(controls, now, tick, (0., 0., 0.))])
    controls._global_fixture_source_ns = now
  last = getattr(controls, '_native_fixture_last', None)
  if native and last is None:
    for stamp in (now - 500_000_000, now - 250_000_000):
      controls.sm.update_msgs(stamp / 1e9, [native_packet(controls, stamp, **(native_overrides or {}))])
    last = now - 250_000_000
  if native and now - last >= 250_000_000:
    controls.sm.update_msgs(now / 1e9, [native_packet(controls, now, **(native_overrides or {}))])
    controls._native_fixture_last = now
  with mock.patch('openpilot.selfdrive.controls.controlsd.time.monotonic_ns', return_value=now):
    return controls.state_control()
