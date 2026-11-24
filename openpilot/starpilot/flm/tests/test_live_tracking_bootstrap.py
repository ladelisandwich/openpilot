"""Bootstrap parent selection/isolated purpose; no fabricated fit/admission."""
from types import SimpleNamespace

import pytest

from openpilot.starpilot.flm.source_plant_fit import bootstrap_slot_rows


def row(stamp, lower=None, upper=None):
  return {'mono_ns': stamp, 'source_lower_ns': stamp if lower is None else lower,
    'source_upper_ns': stamp if upper is None else upper}


def test_clock_interval_selection_is_predeclared_not_response_selected():
  rows = [row(100, 99, 101), row(101, 100, 102), row(150), row(199, 198, 200), row(200, 199, 201)]
  rows[1]['measured_error'] = 999.
  rows[2]['measured_error'] = -999.
  assert bootstrap_slot_rows(rows, 100, 200) == tuple(rows[1:4])
  rows[1]['measured_error'], rows[2]['measured_error'] = -1., 1.
  assert bootstrap_slot_rows(rows, 100, 200) == tuple(rows[1:4])


@pytest.mark.parametrize('low,high', [(None, 101), (True, 101), (102, 101), (100, float('nan'))])
def test_unknown_or_invalid_clock_interval_rejects_not_silently_dropped(low, high):
  value = row(101)
  value['source_lower_ns'], value['source_upper_ns'] = low, high
  with pytest.raises(ValueError, match='known original'):
    bootstrap_slot_rows([value], 100, 200)


def test_minimum_must_be_counted_after_clock_only_filter():
  rows = [row(100, 99, 100), *(row(stamp) for stamp in range(101, 140))]
  assert len(rows) == 40 and len(bootstrap_slot_rows(rows, 100, 200)) == 39


@pytest.mark.parametrize('age,export', [(38_819_999_999, False), (38_820_000_000, True)])
def test_actual_parent_export_available_after_eight_slots_without_later_fit_geometry(age, export):
  from collections import OrderedDict
  from dataclasses import replace
  from openpilot.starpilot.flm.tests.test_live_tracking_bootstrap_cohorts import source_data
  from openpilot.starpilot.flm.tests.test_live_tracking_requalification import synthetic_pair
  from openpilot.starpilot.flm.live_tracking_bootstrap_producer import response_snapshot
  binding,q,frames,poses,events,controls,checkpoints=source_data(bad_until=0,seconds=40)
  now=q+age
  frames=tuple(row for row in frames if row.control_ns<=now)
  poses=tuple(row for row in poses if row.completed_ns<=now)
  events=tuple(row for row in events if row.apply_ns<=now)
  controls=tuple(row for row in controls if row.control_ns<=now)
  checkpoints=tuple(row for row in checkpoints if row[0]<=now)
  exported=[]
  def original_source_packet(*args):
    exported.append(args)
    # Parent availability only: NEVER fabricate a qualified child packet.
    raise ValueError('parent-only export boundary reached')
  seed=SimpleNamespace(_binding=binding,original=SimpleNamespace(lag_ns=170_000_000))
  c,o,_=synthetic_pair(seed,0)
  c=replace(c,control_ns=now-1_000_000,state_ns=now-2_000_000,source_receipt_ns=now-2_000_000,
    native_required=False,native_allowed=False)
  o=replace(o,control_ns=c.control_ns,state_ns=c.state_ns,apply_ns=now-500_000)
  # These are explicit scheduling-only metadata. Any attempted child/native
  # admission fails at the original_source_packet boundary above.
  feedback=SimpleNamespace(failed=False,poison_through_ns=0,failures={},
    consumptions={frame.control_ns:object()for frame in frames})
  adapter=SimpleNamespace(envelope=object(),key=('availability-only',),identity=binding.identity,
    baseline_revision=binding.baseline_revision,original_feedback_archive=feedback,
    original_context_receipts=SimpleNamespace(controls=OrderedDict((row.control_ns,row)for row in controls)),
    replay=SimpleNamespace(records=frames,checkpoints=checkpoints,
      emissions=SimpleNamespace(events=OrderedDict((row.apply_ns,row)for row in events))),
    original_source_packet=original_source_packet)
  capture=SimpleNamespace(epoch=1,epoch_start_ns=q,epoch_cutoff_ns=q+100_000_000_000,rows=poses)
  collector=SimpleNamespace(adapter=adapter,original_source_capture=capture,ready=[(c,o)],last_fit=0,snapshot_attempts=0)
  # The actual fixture's20Hz physical clock is170ms after its control clock;
  # the first real pose crossing38.8s is38.82s, not an invented38.8s pose.
  if export:
    with pytest.raises(ValueError,match='parent-only export boundary'):
      response_snapshot(collector,binding,now)
  else:
    assert response_snapshot(collector,binding,now) is None
  assert bool(exported) is export
  assert collector.last_fit==collector.snapshot_attempts==0


def test_legacy_process_policy_is_unchanged_and_bootstrap_is_explicit_diagnostic(monkeypatch):
  from openpilot.starpilot.flm.fit_process import FitProcess
  from openpilot.starpilot.flm.fit_worker import MAX_CPU
  from openpilot.starpilot.flm.live_tracking_bootstrap_process import LiveBootstrapProcess
  from openpilot.starpilot.flm.live_tracking_bootstrap_worker import MAX_CPU as BOOTSTRAP_CPU, PURPOSE
  assert MAX_CPU == 20 and FitProcess.wall_ns == 30_000_000_000
  assert FitProcess.interval_ns == 60_000_000_000
  assert FitProcess.worker_module == 'openpilot.starpilot.flm.fit_worker'
  assert BOOTSTRAP_CPU == 180 and LiveBootstrapProcess.wall_ns == 300_000_000_000
  assert PURPOSE == 'live-response-bootstrap-diagnostic-v1'
  monkeypatch.delenv('STARPILOT_LIVE_BOOTSTRAP_PURPOSE', raising=False)
  process = LiveBootstrapProcess()
  assert not process.submit({}, token=None, now_ns=1)
  assert process.child is None and process.reason == 'production_bootstrap_budget_not_yet_qualified'
