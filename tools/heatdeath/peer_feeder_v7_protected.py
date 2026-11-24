"""Owned independent peer publisher; no observer, disk I/O or authority checks in timed loop."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import sys
import time

from peer_timing_probe_v2 import Probe, point


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--repo', required=True)
  parser.add_argument('--inputs', type=Path, required=True)
  parser.add_argument('--release', type=Path, required=True)
  parser.add_argument('--clocks', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  assert os.geteuid() == 0 and sys.platform == 'linux' and sys.dont_write_bytecode
  assert os.environ.get('SP_PRIVATE_NATIVE_BENCH') == '1'
  assert os.environ['OPENPILOT_PREFIX'].startswith('replay-bench-')
  assert all(name not in os.environ for name in ('REPLAY', 'SIMULATION', 'CEREAL_FAKE', 'FAKESEND', 'STARTED'))
  output = args.output.resolve()
  assert output.parent / 'params' == Path(os.environ['PARAMS_ROOT']).resolve()
  assert output.parent == args.release.resolve().parent == args.clocks.resolve().parent
  assert not output.exists()
  from driver_aa_v47_fixture import configure_imports, read_peers, reanchor, OWNED

  configure_imports(args.repo)
  from openpilot.cereal import messaging

  meta = json.loads((args.inputs / 'inputs.json').read_text())
  for name, expected in meta['files'].items():
    assert hashlib.sha256((args.inputs / name).read_bytes()).hexdigest() == expected
  suppressed = {}
  peers = read_peers(args.inputs / 'peers.trace', messaging, suppressed)
  services = sorted({event.which() for _, event in peers})
  assert not set(services) & (OWNED | {'managerState'})
  os.environ['MSGQ_PREALLOC'] = '1'  # Current msgq prefaults this private stimulus producer's queues before readiness.
  publisher = messaging.PubMaster(services)
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  # Only the fixture moves: real UI uses0, virtual pandad2, physical pandad3.
  fixture_cpus = {1} if os.environ.get('SP_PRIVATE_PEER_CORE1') == '1' else {3} if os.environ.get('SP_PRIVATE_PEER_CORE3') == '1' else {0, 1, 2, 3}
  os.sched_setaffinity(0, fixture_cpus)
  from placement import apply

  fixture_cpus = set(apply('peer'))
  # Materialize real source payloads before readiness, without sending any
  # warm-up stimulus. Exercise the exact field/serializer path at zero clock
  # delta, prove byte equality, then retain each single-use mutable builder.
  prewarm_start = time.monotonic_ns()
  prepared, prepared_bytes = [], 0
  assert len(peers) <= 100000, 'Bounded prepared peer event count exceeded'
  for offset, event in peers:
    service, builder = event.which(), event.as_builder()
    original = builder.to_bytes()
    builder.clear_write_flag()
    reanchor(builder, offset, meta, meta['sourceStartMonoNs'], meta['sourceStartMonoNs'] + meta['sourceBootMinusMonoNs'])
    assert builder.to_bytes() == original, 'Prewarm changed actual source bytes'
    builder.clear_write_flag()
    prepared_bytes += len(original)
    assert prepared_bytes <= 128 * 1024 * 1024, 'Bounded prepared peer payload bytes exceeded'
    prepared.append((offset, service, builder))
  del peers
  peers = prepared
  prewarm = {
    'events': len(peers),
    'payloadBytes': prepared_bytes,
    'durationNs': time.monotonic_ns() - prewarm_start,
    'zeroDeltaByteEquality': True,
    'publishedStimulus': 0,
    'transportWarmup': 'PubMaster queue construction only; no warm-up send or IPC memory modification',
  }
  os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(1))
  assert os.sched_getscheduler(0) == os.SCHED_FIFO and os.sched_getparam(0).sched_priority == 1
  fixture_policy = {
    'scope': 'Private test stimulus main thread only; after CFS prewarm, before readiness/release',
    'policy': os.sched_getscheduler(0),
    'priority': os.sched_getparam(0).sched_priority,
    'affinity': sorted(os.sched_getaffinity(0)),
    'productionPolicyChange': False,
  }
  probe = Probe(services)
  counts = dict.fromkeys(services, 0)
  index = maximum_late = maximum_dispatch = maximum_send = budget_yields = 0
  failure = None
  started = finished = None
  pre_release = True

  def stop(signum, _frame):
    raise SystemExit('peer feeder signal ' + str(signum))

  for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
    signal.signal(signum, stop)
  try:
    ready = output.with_name('peer-ready.json')
    assert not ready.exists()
    ready_temporary = ready.with_suffix('.tmp')
    assert not ready_temporary.exists()
    ready_temporary.write_text(
      json.dumps(
        {
          'pid': os.getpid(),
          'services': services,
          'events': len(peers),
          'prewarm': prewarm,
          'fixturePolicy': fixture_policy,
          'suppressedRecordedOwners': suppressed,
          'readyMonoNs': time.monotonic_ns(),
        }
      )
      + '\n'
    )
    ready_temporary.replace(ready)
    wait_deadline = time.monotonic_ns() + 30_000_000_000
    while not args.release.exists():
      assert time.monotonic_ns() < wait_deadline, 'No native release within30s of peer readiness'
      probe.sleep(0.005, 0)
    clocks = json.loads(args.clocks.read_text())
    mono, boot = clocks['monoStartNs'], clocks['bootStartNs']
    assert type(mono) is type(boot) is int and mono > 0 and boot > 0
    assert args.release.read_bytes() == struct.pack('<Q', mono)
    assert clocks['prefix'] == os.environ['OPENPILOT_PREFIX']
    deadline = mono + meta['durationNs']
    started = time.monotonic_ns()
    pre_release = False
    while time.monotonic_ns() < deadline:
      now = time.monotonic_ns()
      next_offset = peers[index][0] if index < len(peers) else meta['durationNs']
      probe.loop(index, next_offset, now - mono - next_offset)
      if now < mono:
        probe.sleep(min(0.001, (mono - now) / 1e9), 1)
        continue
      dispatch_start, dispatched = time.monotonic_ns(), 0
      dispatch_point = point()
      while index < len(peers) and peers[index][0] <= now - mono:
        if dispatched >= 64 or time.monotonic_ns() - dispatch_start >= 2_000_000:
          budget_yields += 1
          break  # Fairness yield; actual source lateness remains a hard failure.
        offset, service, builder = peers[index]
        late = time.monotonic_ns() - mono - offset
        maximum_late = max(maximum_late, late)
        if late > 20_000_000:
          probe.dispatch(index, dispatched, dispatch_point, point())
          probe.send(service, index, offset, late, -1, 0, -1, [point()])
        assert late <= 20_000_000, 'Peer source feed >20ms late; harness invalid'
        send_start = time.monotonic_ns()
        points, stage, completed = [point()], 0, 0
        try:
          publisher.send(service, reanchor(builder, offset, meta, mono, boot))
          completed = time.monotonic_ns()
          stage = 3
        finally:
          points.append(point())
          probe.send(service, index, offset, completed - mono - offset if completed else late, stage, completed, -1, points)
        counts[service] += 1
        index += 1
        dispatched += 1
        maximum_send = max(maximum_send, completed - send_start)
        maximum_late = max(maximum_late, completed - mono - offset)
        assert completed - mono - offset <= 20_000_000, 'Peer source publication completed >20ms late; harness invalid'
      probe.dispatch(index, dispatched, dispatch_point, point())
      maximum_dispatch = max(maximum_dispatch, time.monotonic_ns() - dispatch_start)
      probe.sleep(0.001, 2)
    assert index == len(peers), 'Unpublished peer source events at source deadline'
  except BaseException as error:
    failure = type(error).__name__ + ': ' + str(error)
  finally:
    finished = time.monotonic_ns()
    probe_path = probe.export(output)
    output.write_text(
      json.dumps(
        {
          'kind': 'private-independent-peer-feeder-v7-protected-source',
          'timingProbePath': probe_path,
          'timingProbeErrors': probe.errors,
          'msgqPreallocEnv': os.environ.get('MSGQ_PREALLOC'),
          'failure': failure,
          'events': len(peers),
          'published': index,
          'counts': counts,
          'maximumLatenessNs': maximum_late,
          'maximumDispatchNs': maximum_dispatch,
          'startedMonoNs': started,
          'finishedMonoNs': finished,
          'maximumSendNs': maximum_send,
          'dispatchBudgetYields': budget_yields,
          'dispatchBudgetNs': 2000000,
          'dispatchMessagesPerPassMaximum': 64,
          'sourceLatenessLimitNs': 20000000,
          'latenessCheckedBeforeAndAfterPublication': True,
          'prewarm': prewarm,
          'fixturePolicy': fixture_policy,
          'failedBeforeRelease': pre_release,
          'suppressedRecordedOwners': suppressed,
          'policy': os.sched_getscheduler(0),
          'priority': os.sched_getparam(0).sched_priority,
          'affinity': sorted(os.sched_getaffinity(0)),
          'timedDiskWrites': False,
        },
        indent=2,
      )
      + '\n'
    )
  assert failure is None, failure


if __name__ == '__main__':
  main()
