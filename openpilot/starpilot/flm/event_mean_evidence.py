"""Explicit event-only paired-mean evidence model; never native authority.

Coverage is conditional on independent, approximately normal ORIGINAL whole-
block gains. Source non-overlap is checked; it does not establish independence.
"""
from dataclasses import asdict, dataclass
import hashlib
import math
from collections.abc import Sequence
from typing import TYPE_CHECKING, TypedDict, cast

if TYPE_CHECKING:
  from openpilot.starpilot.flm.adaptive_profile import Block


class _EventPoint(TypedDict):
  control_ns: int
  apply_ns: int
  mono_ns: int


class _EventPacket(TypedDict, total=False):
  event_windows: tuple[tuple[int, int, str], ...]
  samples: Sequence[_EventPoint]
  grouping_revision: str
  identity: str
  block_id: str
  cohort: str

VERSION='event-paired-whole-block-t95-v1'
ASSUMPTION='independent-approximately-normal-paired-whole-block-gains'
_SEAL=object()


def _digest(value):
  from openpilot.starpilot.flm.physical_input_observation import body_digest
  return body_digest(value)


def arithmetic_lower(values, ids):
  """Numerical primitive ONLY; this function cannot qualify an event or model."""
  from openpilot.starpilot.flm.automatic import T95
  if (type(values) is not tuple or type(ids) is not tuple or len(ids)!=len(values) or
      not 2<=len(values)<=len(T95)+1 or
      any(type(identity) is not str or not identity for identity in ids) or len(set(ids))!=len(ids) or
      any(type(value) not in (int,float) or not math.isfinite(value) for value in values)):
    raise ValueError('event paired-mean finite unique bounded units')
  mean=sum(values)/len(values)
  variance=sum((value-mean)**2 for value in values)/(len(values)-1)
  result=mean-T95[len(values)-2]*math.sqrt(variance/len(values))
  if not math.isfinite(result):
    raise ValueError('event paired-mean finite bound')
  return result


@dataclass(frozen=True)
class WholeBlockUnit:
  block_id: str
  cohort: str
  first_ns: int
  last_ns: int
  windows: tuple[tuple[int, int, str], ...]
  block_sha256: str


@dataclass(frozen=True)
class EventMeanEvidence:
  version: str
  assumption: str
  identity: str
  source_geometry_sha256: str
  units: tuple[WholeBlockUnit,...]
  split: int
  digest: str
  _seal: object


def _model_body(model):
  return {'version':model.version,'assumption':model.assumption,'identity':model.identity,
    'source_geometry_sha256':model.source_geometry_sha256,'units':tuple(asdict(unit) for unit in model.units),
    'split':model.split}


def build_model(engine, events, blocks):
  """Bind child-reconstructed whole-source lineage and exact numeric blocks.

  Caller still must run every controller/native/physical/heldout/current guard.
  No parent-declared block count, renamed boundary, or arbitrary packet is proof.
  """
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  from openpilot.starpilot.flm.adaptive_profile import Block
  from openpilot.starpilot.flm.event_windows import whole_event_windows,GROUPING_REVISION
  if (type(engine) is not EventControllerReplay or type(events) is not tuple or type(blocks) is not tuple or
      not 8<=len(blocks)<=24 or len(events)!=len(blocks)):
    raise ValueError('actual whole-block event mean model source')
  from openpilot.starpilot.flm.feedback_replay import require_feedback_engine
  require_feedback_engine(engine)
  actual=tuple(whole_event_windows(engine))
  allowed={(first,last,kind) for _,first,last,kind in actual}
  units=[]
  used=set()
  previous=0
  for event,block in zip(events,blocks,strict=True):
    if type(block) is not Block or type(event) is not dict:
      raise ValueError('typed original event mean model blocks')
    event = cast(_EventPacket, event)
    windows=event.get('event_windows')
    if type(windows) is not tuple or not windows or any(type(row) is not tuple or len(row)!=3 or row not in allowed for row in windows):
      raise ValueError('event mean model complete original geometry')
    windows = cast(tuple[tuple[int, int, str], ...], windows)
    if any(first<=previous_end for (_,previous_end,_),(first,_,_) in zip(windows,windows[1:], strict=False)):
      raise ValueError('event mean model complete original geometry')
    expected='event-v3:'+hashlib.sha256(repr((GROUPING_REVISION,engine.identity,engine.baseline_revision,
      engine.objective.context_key,windows)).encode()).hexdigest()
    if (event.get('grouping_revision')!=GROUPING_REVISION or event.get('identity')!=engine.identity or
        event.get('block_id')!=expected or block.block_id!=expected or block.identity!=engine.identity or
        block.block_id in used or block.cohort!=event.get('cohort') or windows[0][0]<=previous or
        len(block.samples)!=len(event.get('samples',()))):
      raise ValueError('event mean model disjoint original block identity')
    for sample,point in zip(block.samples,event['samples'],strict=True):
      if (sample.control_ns,sample.applied_ns,sample.mono_ns)!=(point['control_ns'],point['apply_ns'],point['mono_ns']):
        raise ValueError('event mean model exact original endpoints')
      if not any(first<=sample.control_ns<=last for first,last,_ in windows):
        raise ValueError('event mean model endpoint outside complete source event')
    used.add(block.block_id)
    previous=windows[-1][1]
    units.append(WholeBlockUnit(block.block_id,block.cohort,windows[0][0],previous,windows,_digest(asdict(block))))
  geometry=_digest({'actual_windows':actual,'identity':engine.identity,'baseline':engine.baseline_revision,
    'context':engine.objective.context_key,'cp_bytes':engine.factory.cp.as_builder().to_bytes()})
  model=EventMeanEvidence(VERSION,ASSUMPTION,engine.identity,geometry,tuple(units),len(units)//2,'',_SEAL)
  from dataclasses import replace
  return replace(model,digest=_digest(_model_body(model)))


def validate_model(model, blocks):
  from openpilot.starpilot.flm.adaptive_profile import Block
  if (type(model) is not EventMeanEvidence or model._seal is not _SEAL or model.version!=VERSION or
      model.assumption!=ASSUMPTION or model.digest!=_digest(_model_body(model)) or
      type(blocks) is not tuple or len(blocks)!=len(model.units)):
    raise ValueError('owned event paired-mean model')
  if any(type(block) is not Block or (block.identity,block.block_id,block.cohort)!=(model.identity,unit.block_id,unit.cohort) or
      _digest(asdict(block))!=unit.block_sha256 for block,unit in zip(blocks,model.units,strict=True)):
    raise ValueError('event paired-mean model changed numeric evidence')
  return model


def lower(model, blocks, values, *, cohort):
  """Only ALL exact heldout units in one original cohort determine n and df."""
  if (type(model) is not EventMeanEvidence or model._seal is not _SEAL or model.version!=VERSION or
      model.assumption!=ASSUMPTION or model.digest!=_digest(_model_body(model))):
    raise ValueError('owned event paired-mean model')
  expected=tuple(unit for unit in model.units[model.split:] if unit.cohort==cohort)
  if type(blocks) is not tuple or len(blocks)!=len(expected):
    raise ValueError('all original heldout cohort units required')
  for block,unit in zip(blocks,expected,strict=True):
    if (block.identity,block.block_id,block.cohort)!=(model.identity,unit.block_id,cohort) or _digest(asdict(cast('Block', block)))!=unit.block_sha256:
      raise ValueError('event paired-mean heldout identity/body changed')
  return arithmetic_lower(values,tuple(unit.block_id for unit in expected))


def report_basis(model):
  if type(model) is not EventMeanEvidence or model._seal is not _SEAL or model.digest!=_digest(_model_body(model)):
    raise ValueError('owned event mean report model')
  return {**_model_body(model),'digest':model.digest,'upper_cdf':.975,'two_sided_confidence':.95,
    'disjoint_source_does_not_prove_independence':True,'table':'automatic.T95-conservative-upper-rounded-df1..23',
    'heldout_units_by_cohort':{cohort:sum(unit.cohort==cohort for unit in model.units[model.split:])
      for cohort in sorted({unit.cohort for unit in model.units})}}
