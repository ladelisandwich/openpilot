"""Representation vectors only; no fitted profile, native or physical proof."""
from copy import deepcopy
from dataclasses import asdict, dataclass, replace
import gc
import hashlib
import pickle
from types import SimpleNamespace
from collections.abc import Callable
from typing import cast, TypedDict
import weakref

import pytest

from openpilot.starpilot.flm import normalized_trace_archive as n
from openpilot.starpilot.flm.canonical_surface_adapter import NAMES, full_profile_sha256
from openpilot.starpilot.flm.effect_trace import FullEffectTrace, ordered, step_full, trace_dict
from openpilot.starpilot.flm.full_surface_profile import FullSurfaceEnvelope, neutral_coordinates
from openpilot.starpilot.flm.primitive_source_memo import independent_primitive_graph, MAX_LOGICAL_BYTES
from openpilot.starpilot.flm.tests.test_fullrate_pipeline import snapshots
from openpilot.starpilot.flm.torque_raw_fit import control_from_dict


def structural_trace(stamp=10_000_000_000):
  envelope = FullSurfaceEnvelope.initial(cp_sha256='1' * 64, controller_revision='source-only-structural-vector',
    source_epoch='2' * 64, reset_generation='0', selected_baseline_revision='3' * 64,
    trial_names=('ff_gain_left',), persistent_source_epoch='4' * 64)
  before = tuple((name, -0.0 if index == 0 else value) for index, (name, value) in enumerate(neutral_coordinates()))
  applied = ordered(step_full(dict(before), dict(envelope.coordinates), now_ns=stamp, previous_ns=0, qualified=True))
  return FullEffectTrace(1,envelope,stamp,0,True,before,envelope.coordinates,applied,
    full_profile_sha256(envelope.selected_baseline_revision,applied)).validate()


def fixture():
  trace = structural_trace()
  control, _, _ = snapshots(trace.control_ns)
  control = replace(control,effect_trace=trace,effect_prepare_ns=trace.control_ns)
  builder = n.ArchiveBuilder()
  wire = builder.record(control)
  archive = n.TraceArchive(builder.literal())
  owner = SimpleNamespace(trace_archive=archive)
  decoded = control_from_dict(wire,trace_archive=archive)
  return trace,control,wire,archive,owner,decoded


def _shape_frame(evidence):
  # Representation-only batch/dispatch vector. No selected-stage/native proof.
  from openpilot.starpilot.flm.controller_replay import ReplayFrame
  return ReplayFrame(evidence.control_ns,evidence.replay_inputs,evidence.requested,
    evidence.applied_coordinates,effect_trace=evidence.effect_trace,
    effect_prepare_ns=evidence.effect_prepare_ns)


def test_exact_all_fields_types_bits_and_scalar_refs():
  trace,control,wire,archive,owner,decoded = fixture()
  assert type(decoded.effect_trace) is n.TraceRef
  assert set(vars(decoded.effect_trace)) == {'index','prepare_ns'}
  assert not any(type(value) is n.TraceArchive for value in vars(decoded.effect_trace).values())
  restored = n.trace_for(decoded,owner)
  assert pickle.dumps(trace_dict(restored),protocol=5) == pickle.dumps(trace_dict(trace),protocol=5)
  assert restored.before[0][1].hex() == '-0x0.0p+0'
  assert n.stored_record(decoded) == wire
  assert n.current_record(control,archive.literal) == wire
  assert n.record_body(decoded,owner)['effect_trace']['declaration'] == archive.literal['envelopes'][0]
  assert n.record_body(decoded,owner)['effect_trace']['row'] == archive.literal['rows'][0]
  assert len(restored.before) == len(restored.target) == len(restored.applied) == len(NAMES) == 30


def test_value_deduplication_not_object_identity_or_hash_only():
  trace = structural_trace()
  builder = n.ArchiveBuilder()
  first = builder.encode(trace)
  same = builder.encode(deepcopy(trace))
  assert first == same and len(builder.rows) == len(builder.envelopes) == 1
  second_trace = structural_trace(trace.control_ns + 10_000_000)
  second = builder.encode(second_trace)
  assert second['index'] != first['index']
  assert len(builder.rows) == 2 and len(builder.envelopes) == 1
  assert set(builder.literal()) == {'version','manifest','envelopes','rows'}


@pytest.mark.parametrize('mutation', ['index','clock','manifest','missing_metadata','applied','hash','nonfinite','row_type'])
def test_changed_or_missing_literal_fields_fail(mutation):
  _,_,wire,archive,_,_ = fixture()
  literal = deepcopy(archive.literal)
  ref = deepcopy(wire['effect_trace'])
  row = list(literal['rows'][0])
  if mutation == 'index':
    ref['index'] = 10
  elif mutation == 'clock':
    ref['prepare_ns'] += 1
  elif mutation == 'manifest':
    literal['manifest'] = '0' * 64
  elif mutation == 'missing_metadata':
    literal['envelopes'][0].pop('source_epoch')
  elif mutation == 'applied':
    row[7] = (0.1, *row[7][1:])
  elif mutation == 'hash':
    row[8] = '0' * 64
  elif mutation == 'nonfinite':
    row[5] = (float('nan'), *row[5][1:])
  elif mutation == 'row_type':
    row[4] = 1
  literal['rows'] = (tuple(row),)
  with pytest.raises((ValueError,TypeError)):
    changed = n.TraceArchive(literal)
    changed.resolve(n.read_ref(ref,changed)).validate()


def test_legacy_expanded_control_path_remains_exact_and_missing_owner_declines():
  trace,control,_,_,_,decoded = fixture()
  assert control_from_dict(asdict(control)) == control
  with pytest.raises(ValueError,match='literal source archive'):
    n.trace_for(decoded,SimpleNamespace())
  with pytest.raises(ValueError):
    control_from_dict(n.stored_record(decoded))


def test_entire_literal_owner_key_changes_for_unconsumed_rows():
  _,_,_,archive,owner,_ = fixture()
  before = hashlib.sha256(pickle.dumps(n.archive_key(owner),protocol=5)).digest()
  declaration = archive.literal['envelopes'][0]
  declaration['controller_revision'] += '-changed'
  after = hashlib.sha256(pickle.dumps(n.archive_key(owner),protocol=5)).digest()
  assert before != after


def test_archive_parse_and_reference_parse_do_not_expand_all_traces(monkeypatch):
  trace = structural_trace()
  builder = n.ArchiveBuilder()
  refs = [builder.encode(replace(trace,control_ns=trace.control_ns+index*10_000_000)) for index in range(64)]
  # Same neutral .01s first-step trace remains valid for these distinct clocks.
  import openpilot.starpilot.flm.effect_trace as module
  original = module.FullEffectTrace
  reserved_bound = n.reservation_upper_bound()
  monkeypatch.setattr(n,'reservation_upper_bound',lambda: reserved_bound)
  live, peaks = [], []
  def construct(*args, **kwargs):
    value = original(*args, **kwargs)
    live.append(weakref.ref(value))
    peaks.append(sum(ref() is not None for ref in live))
    return value
  monkeypatch.setattr(module,'FullEffectTrace',construct)
  archive = n.TraceArchive(builder.literal())
  parsed = [n.read_ref(ref,archive) for ref in refs]
  assert not live
  for ref in parsed:
    expanded = archive.resolve(ref)
    assert len(expanded.applied) == 30
    del expanded
  gc.collect()
  assert max(peaks) == 1 and not any(ref() is not None for ref in live)


def test_reserved_allocation_uses_same_combined_logical_cap_before_clone(monkeypatch):
  import openpilot.starpilot.flm.primitive_source_memo as module
  from openpilot.starpilot.flm.native_primitives import FALLBACK
  monkeypatch.setattr(module,'_try_native',lambda *args: FALLBACK)
  # Bytes cost9, tuple9; outer reservation wrapper adds18 plus reserved bytes.
  amount = MAX_LOGICAL_BYTES - n.RESERVED_BYTES - 27
  exact = b'x' * amount
  copied = independent_primitive_graph(exact,reserved_bytes=n.RESERVED_BYTES)
  assert copied == exact
  with pytest.raises(ValueError,match='logical expansion bound'):
    independent_primitive_graph(exact+b'x',reserved_bytes=n.RESERVED_BYTES)
  with pytest.raises(ValueError,match='reservation'):
    independent_primitive_graph((),reserved_bytes=n.RESERVED_BYTES+1)


def test_complete_packet_membership_and_unused_rows():
  _,_,wire,archive,_,_ = fixture()
  packet = {'version':n.SOURCE_VERSION,'trace_archive':archive.literal,
            'context_receipts':{'controls':(wire,)},'actual_frames':(deepcopy(wire),)}
  assert n.reserve_for_packet(packet) == n.RESERVED_BYTES
  changed = deepcopy(packet)
  class PreparedFrame(TypedDict):
    effect_prepare_ns: int
  cast(tuple[PreparedFrame, ...], changed['actual_frames'])[0]['effect_prepare_ns'] += 1
  with pytest.raises(ValueError,match='membership'):
    n.validate_packet(changed)
  changed = deepcopy(packet)
  class ArchiveRows(TypedDict):
    rows: tuple[object, ...]
  changed_archive = cast(ArchiveRows, changed['trace_archive'])
  changed_archive['rows'] += (changed_archive['rows'][0],)
  with pytest.raises(ValueError,match='unused or omitted'):
    n.validate_packet(changed)
  with pytest.raises(ValueError,match='legacy'):
    n.reserve_for_packet({**packet,'version':'unpaired-original-source-v8'})


def test_fresh_complete_owner_key_rejects_unknown_objects_without_reduce_callbacks():
  _,_,_,archive,owner,_ = fixture()
  calls = []
  class Executable:
    def __reduce__(self):
      calls.append('executed')
      raise AssertionError('must reject before pickle')
  archive.literal['envelopes'] += ({**archive.literal['envelopes'][0],'controller_revision':Executable()},)
  archive.literal['rows'] += (archive.literal['rows'][0],)  # Header remains valid; ALL declarations must be checked.
  with pytest.raises(ValueError):
    n.archive_key(owner)
  assert not calls


def test_mutable_work_guard_rechecks_whole_table_without_permission_cache():
  _,_,_,archive,owner,_ = fixture()
  @n.guarded_archive()
  def work(engine):
    declaration = engine.trace_archive.literal['envelopes'][0]
    declaration['controller_revision'] += '-changed'
    return 'not-admission'
  with pytest.raises(ValueError,match='changed during mutable work'):
    work(owner)
  @n.guarded_archive()
  def ordinary(engine):
    return 'legacy'
  assert ordinary(SimpleNamespace()) == 'legacy'


def test_transient_and_retained_cache_reservation_proven_below_combined_limit():
  assert n.reservation_upper_bound() <= n.RESERVED_BYTES == 512 * 1024
  assert n.RESERVED_BYTES < MAX_LOGICAL_BYTES


def test_unknown_physical_body_keeps_legacy_asdict_callback_and_error_order():
  from openpilot.starpilot.flm.physical_input_observation import _physical_asdict
  calls = []
  @dataclass
  class Unknown:
    value: int = 1
    def __getattr__(self,name):
      calls.append(name)
      raise AttributeError(name)
  body = Unknown()
  expected = asdict(body)
  oracle_calls = tuple(calls)
  calls.clear()
  assert _physical_asdict(body) == expected
  assert tuple(calls) == oracle_calls
  class NonDataclass:
    def __getattr__(self,name):
      calls.append(name)
      raise AttributeError(name)
  body = NonDataclass()
  calls.clear()
  with pytest.raises(TypeError) as original_error:
    cast(Callable[[object], object], asdict)(body)
  oracle_calls = tuple(calls)
  calls.clear()
  with pytest.raises(TypeError) as new_error:
    _physical_asdict(body)
  assert str(new_error.value) == str(original_error.value)
  assert tuple(calls) == oracle_calls


def test_snapshot_frame_batch_has_one_full_guard_with_fresh_row_resolution(monkeypatch):
  # Guard-placement unit only; fake normalize never confers native admission.
  from openpilot.starpilot.flm.controller_replay import ControllerReplay
  _,_,_,archive,_,decoded = fixture()
  frame = _shape_frame(decoded)
  wire = n.stored_record(frame)
  keys, parsed = [], []
  original_key = n.archive_key
  def counted(owner):
    keys.append('fresh-whole-table')
    return original_key(owner)
  monkeypatch.setattr(n,'archive_key',counted)
  owner = ControllerReplay.__new__(ControllerReplay)
  owner.trace_archive, owner.failed = archive, False
  monkeypatch.setattr(owner, 'checkpoint', lambda *args: None, raising=False)
  def normalize(frame,**kwargs):
    parsed.append(n.trace_for(frame,owner).control_ns)
    return 'shape-only'
  monkeypatch.setattr(owner, '_normalize_frame', normalize)
  ControllerReplay._load_snapshot_frames(owner,(wire,),{frame.control_ns:None},normalized=True)
  assert keys == ['fresh-whole-table','fresh-whole-table']
  assert parsed == [frame.control_ns]
  def mutate(frame,**kwargs):
    owner.trace_archive.literal['envelopes'][0]['controller_revision'] += '-changed'
    return 'not-admitted'
  monkeypatch.setattr(owner, '_normalize_frame', mutate)
  with pytest.raises(ValueError,match='changed during mutable work'):
    ControllerReplay._load_snapshot_frames(owner,(wire,),{frame.control_ns:None},normalized=True)


@pytest.mark.parametrize('normalized',[False,True])
def test_snapshot_subclass_retains_public_normalize_dispatch(normalized, monkeypatch):
  from openpilot.starpilot.flm.controller_replay import ControllerReplay
  _,control,_,archive,_,decoded = fixture()
  calls = []
  class Override(ControllerReplay):
    def normalize(self,frame,**kwargs):
      calls.append(frame.control_ns)
      return 'subclass-shape-only'
    def _normalize_frame(self,*args,**kwargs):
      raise AssertionError('unknown subclass must retain original public dispatch')
  owner = Override.__new__(Override)
  owner.failed = False
  monkeypatch.setattr(owner, 'checkpoint', lambda *args: None, raising=False)
  if normalized:
    owner.trace_archive = archive
    frame = _shape_frame(decoded)
    body = n.stored_record(frame)
  else:
    frame = _shape_frame(replace(control,effect_trace=None))
    body = asdict(frame)
  owner._load_snapshot_frames((body,),{frame.control_ns:None},normalized=normalized)
  assert calls == [frame.control_ns]
