"""Summarize actual captured phase timing without granting a health verdict."""

import hashlib
import json
from pathlib import Path
import struct


def quantile(values, fraction):
  return sorted(values)[round((len(values) - 1) * fraction)] if values else None


def phase(rows, start, end):
  selected = sorted((row for row in rows if start <= row[3] < end), key=lambda row: row[0])
  wall = [(row[4] - row[3]) / 1e6 for row in selected]
  cpu = [(row[6] - row[5]) / 1e6 for row in selected]
  gaps = [(b[3] - a[3]) / 1e6 for a, b in zip(selected, selected[1:], strict=False)]
  return {
    'calls': len(selected),
    'observedCallsPerSecond': len(selected) / ((end - start) / 1e9),
    'wallMsP50': quantile(wall, 0.5),
    'wallMsP95': quantile(wall, 0.95),
    'wallMsP99': quantile(wall, 0.99),
    'wallMsMax': max(wall, default=None),
    'threadCpuMsP95': quantile(cpu, 0.95),
    'startGapMsMax': max(gaps, default=None),
    'raised': sum(row[7] for row in selected),
  }


def summarize(output, skip_seconds=10):
  output = Path(output)
  result = json.loads((output / 'result.json').read_text())
  (start,) = struct.unpack('<Q', (output / 'release.bin').read_bytes())
  # The feeder exports the same source duration used by the driver.
  duration = result['sourceDurationNs']
  end = start + duration
  start += round(skip_seconds * 1e9)
  if not 0 <= skip_seconds or start >= end:
    raise ValueError('Skip must leave a nonempty source window')
  report = {
    'sourceHead': result['sourcePins']['sourceHead'],
    'failure': result['failure'],
    'everAuthorityReady': result['everAuthorityReady'],
    'activeObserverSamples': result['activeObserverSamples'],
    'qualifiedActiveLatencyBench': False,
    'placement': result['placement'],
    'skipSeconds': skip_seconds,
    'syntheticReserve': result['syntheticReserve'],
    'actors': {},
    'limits': result['limits'] + ['No actual inference/audio/Android Auto workload is launched. Timing alone does not qualify steering.'],
  }
  for name in ('card', 'controlsd', 'selfdrived'):
    metadata = json.loads((output / f'{name}-timing.json').read_text())
    if metadata.get('observerErrors'):
      raise ValueError('Timing observer failed; inspect actor metadata')
    report['actors'][name] = {}
    for label, entry in metadata['timing'].items():
      raw = (output / Path(entry['path']).name).read_bytes()
      if hashlib.sha256(raw).hexdigest() != entry['sha256']:
        raise ValueError('Timing capture hash mismatch')
      rows = list(struct.iter_unpack('<' + 'q' * 20, raw))
      if entry['rowsDropped'] or entry['rowsRetained'] > len(rows):
        raise ValueError('Timing ring lost rows; shorten source window')
      values = phase(rows[: entry['rowsRetained']], start, end)
      if 'averagePeriodNs' in metadata['sampleColumns'].get(label, []):
        values['rollingLagFlags'] = sum(row[9] for row in rows[: entry['rowsRetained']] if start <= row[3] < end)
      report['actors'][name][label] = values
  reserve = output / 'reserve.json'
  if result['syntheticReserve'] is not None:
    proof = json.loads(reserve.read_text())
    report['reserveCompleted'] = proof['completed']
    report['reserveMissedPulses'] = proof['missedPulses']
  native = output / 'native-receipt.json'
  if native.exists():
    report['nativeReceipt'] = json.loads(native.read_text())
  return report
