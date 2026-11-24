"""Bounded telemetry validation; no application or model admission authority."""
import math
from typing import TypedDict, cast


from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, coordinate_bounds


class CoordinateRow(TypedDict):
  name: str
  value: int | float


def _hex(value):
  return type(value) is str and len(value) == 64 and all(char in '0123456789abcdef' for char in value)


def _number(value, maximum):
  return type(value) in (int, float) and math.isfinite(value) and 0 <= value <= maximum


def _coordinates(rows):
  if type(rows) is not list or len(rows) != 30:
    raise ValueError('complete tracking coordinates')
  for name, raw_row in zip(NAMES, rows, strict=True):
    row = cast(CoordinateRow, raw_row)
    low, high = coordinate_bounds(name)
    if (type(row) is not dict or set(row) != {'name', 'value'} or row['name'] != name or
        type(row['value']) not in (int, float) or not math.isfinite(row['value']) or not low <= row['value'] <= high):
      raise ValueError('actual canonical tracking coordinate')
  return rows


def tracking_objects(current, *, fresh):
  result = {}
  tracking = current.get('tracking')
  if tracking is not None:
    keys = {'version', 'status', 'reason', 'group', 'names', 'trialId', 'phase', 'requiredEpisodes',
      'beforeEpisodes', 'futureEpisodes', 'modelStatus', 'lagMs', 'baselineErrorRms', 'futureErrorRms',
      'baselineReversalHz', 'futureReversalHz', 'applicationAcknowledged', 'applicationSettledMonoTime',
      'lastCorrectionMonoTime', 'bindingCurrent'}
    if (type(tracking) is not dict or set(tracking) != keys or tracking['version'] != 1 or
        any(type(tracking[name]) is not str or len(tracking[name]) > 128 for name in ('status', 'reason', 'modelStatus')) or
        any(tracking[name] is not None and (type(tracking[name]) is not str or len(tracking[name]) > 96)
            for name in ('group', 'phase')) or
        tracking['trialId'] is not None and not _hex(tracking['trialId']) or
        type(tracking['names']) is not list or len(tracking['names']) > 4 or
        any(type(name) is not str or name not in NAMES for name in tracking['names']) or
        len(set(tracking['names'])) != len(tracking['names']) or
        type(tracking['applicationAcknowledged']) is not bool or type(tracking['bindingCurrent']) is not bool or
        any(type(tracking[key]) is not int or not 0 <= tracking[key] <= 24 for key in ('beforeEpisodes', 'futureEpisodes')) or
        tracking['requiredEpisodes'] is not None and
          (type(tracking['requiredEpisodes']) is not int or not 3 <= tracking['requiredEpisodes'] <= 24) or
        any(tracking[key] is not None and (type(tracking[key]) is not int or not 0 < tracking[key] < 2**63)
            for key in ('applicationSettledMonoTime', 'lastCorrectionMonoTime')) or
        any(tracking[key] is not None and not _number(tracking[key], maximum) for key, maximum in
          (('lagMs', 500), ('baselineErrorRms', 100), ('futureErrorRms', 100), ('baselineReversalHz', 100), ('futureReversalHz', 100)))):
      raise ValueError('bounded tracking state telemetry')
    result['tracking'] = dict(tracking,
      bindingCurrent=tracking['bindingCurrent'] and fresh,
      applicationAcknowledged=tracking['applicationAcknowledged'] and fresh)
  values = current.get('trackingValues')
  if values is not None:
    if (type(values) is not dict or set(values) != {'version', 'accepted', 'pending', 'acceptedSha256', 'pendingSha256'} or
        values['version'] != 1 or not _hex(values['acceptedSha256']) or
        (values['pending'] is None) != (values['pendingSha256'] is None) or
        values['pendingSha256'] is not None and not _hex(values['pendingSha256'])):
      raise ValueError('bounded learned tracking values')
    _coordinates(values['accepted'])
    if values['pending'] is not None:
      _coordinates(values['pending'])
    result['trackingValues'] = values
  plot = current.get('trackingPlot')
  if plot is not None:
    if (type(plot) is not dict or set(plot) != {'version', 'episodeId', 'profileSha256', 'lagNs', 'units',
        'sampled', 'totalEndpoints', 'points', 'bindingCurrent', 'ageSeconds'} or
        plot['version'] != 'live-tracking-paired-plot-v1' or not _hex(plot['episodeId']) or not _hex(plot['profileSha256']) or
        type(plot['lagNs']) is not int or not 0 <= plot['lagNs'] <= 500_000_000 or plot['units'] != 'm/s²' or
        type(plot['sampled']) is not bool or type(plot['bindingCurrent']) is not bool or
        type(plot['totalEndpoints']) is not int or not 4 <= plot['totalEndpoints'] <= 240 or
        not _number(plot['ageSeconds'], 1e9) or type(plot['points']) is not list or not 4 <= len(plot['points']) <= 64 or
        len(plot['points']) > plot['totalEndpoints']):
      raise ValueError('bounded genuine tracking plot')
    previous = 0
    for point in plot['points']:
      if (type(point) is not dict or set(point) != {'controlMonoTime', 'applyMonoTime', 'poseMonoTime',
          'desiredForceAccel', 'measuredForceAccel', 'errorForceAccel'} or
          any(type(point[key]) is not int for key in ('controlMonoTime', 'applyMonoTime', 'poseMonoTime')) or
          not 0 < point['controlMonoTime'] <= point['applyMonoTime'] <= point['poseMonoTime'] < 2**63 or
          not point['applyMonoTime']+plot['lagNs'] <= point['poseMonoTime'] <= point['applyMonoTime']+plot['lagNs']+30_000_000 or
          point['poseMonoTime'] <= previous or
          any(type(point[key]) not in (int, float) or not math.isfinite(point[key]) or abs(point[key]) > 100
              for key in ('desiredForceAccel', 'measuredForceAccel', 'errorForceAccel')) or
          point['errorForceAccel'] != point['desiredForceAccel']-point['measuredForceAccel']):
        raise ValueError('original tracking endpoint timestamps/force values')
      previous = point['poseMonoTime']
    result['trackingPlot'] = dict(plot, bindingCurrent=plot['bindingCurrent'] and fresh)
  return result
