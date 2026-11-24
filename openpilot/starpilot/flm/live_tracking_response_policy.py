"""Measured, bounded numerical response job; never a current-control permission."""

PURPOSE = 'live-response-bootstrap-v1'
WORKER_MODULE = 'openpilot.starpilot.flm.live_tracking_response_worker'
MAX_CPU = 180
MAX_WALL_NS = 240_000_000_000
# The passing device run measured RSS, not VmPeak. Retain its proven AS bound.
MAX_AS = 2*1024*1024*1024
MAX_INPUT = 8*1024*1024
MAX_OUTPUT = 65536


def production_result(result):
  from openpilot.starpilot.flm.live_tracking_response import certificate_from_dict
  if type(result) is not dict or result.get('live_job_version') != PURPOSE:
    raise ValueError('exact response bootstrap protocol required')
  if result.get('profile') is not None or result.get('current_permission') is not False:
    raise ValueError('response calibration cannot grant profile or current permission')
  if 'diagnostic_purpose' in result or 'worker_purpose' in result or 'production_admissible' in result:
    raise ValueError('bootstrap math cannot supply worker admission metadata')
  identified = result.get('status') == 'response_identified'
  if identified:
    certificate_from_dict(result['certificate'])
  elif result.get('certificate') is not None:
    raise ValueError('unidentified response cannot carry a certificate')
  return dict(result, worker_purpose=PURPOSE, production_admissible=identified)


def validate_production_result(result):
  if (type(result) is not dict or result.get('worker_purpose') != PURPOSE or
      result.get('production_admissible') is not True or 'diagnostic_purpose' in result):
    raise ValueError('exact production response worker report required')
  body = {key: value for key, value in result.items() if key not in ('worker_purpose', 'production_admissible')}
  if production_result(body) != result:
    raise ValueError('qualified response report required')
  return result
