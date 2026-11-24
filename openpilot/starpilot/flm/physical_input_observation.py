"""Physical-only observed inputs. No profile, candidate or current authority.

Records are immutable child-derived evidence/tamper bindings, NOT certificates
that a caller executed native verification. Wire callers cannot supply these as
proof: the actual child recomputes them from original replay/context receipts.
Performance tags never replace original bodies or hard source/native guards.
"""
from dataclasses import asdict, dataclass, replace, fields
import hashlib
import json
import math
import re
from openpilot.starpilot.flm.native_primitives import try_native as _try_native, FALLBACK as _NATIVE_FALLBACK

PROTOCOL = 'verified-delivered-physical-observation-v1'
_LOWER_HEX = re.compile('[0-9a-f]*')


def _hash(value):
  return type(value) is str and len(value) == 64 and _LOWER_HEX.fullmatch(value) is not None


def _typed_body(item):
  """Original exact primitive transform; no metadata or permission shortcuts."""
  kind = type(item)
  if item is None or kind in (bool, int, str):
    return [kind.__name__, item]
  if kind is float and math.isfinite(item):
    return ['float64', item.hex()]
  if kind is bytes:
    return ['bytes', item.hex()]
  if kind in (tuple, list):
    return [kind.__name__, [_typed_body(child) for child in item]]
  if kind is dict and all(type(key) is str for key in item):
    return ['dict', [[key, _typed_body(item[key])] for key in sorted(item)]]
  raise ValueError('physical exact original body primitive')


def _body_digest_original(value):
  """Bit/type exact original primitives, including CAN bytes and signed zero."""
  return hashlib.sha256(json.dumps(_typed_body(value), separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _exact_primitive_body(value):
  """Validate before serialization, matching original rejection traversal."""
  kind=type(value)
  if value is None or kind in (bool,int,str,bytes):
    return True
  if kind is float:
    return math.isfinite(value)
  if kind in (tuple,list):
    return all(_exact_primitive_body(child) for child in value)
  if kind is dict and all(type(key) is str for key in value):
    return all(_exact_primitive_body(value[key]) for key in sorted(value))
  return False


def _large_sequence(value, depth=0):
  """Only size selection. Never an integrity, source or type-validation claim."""
  if type(value) in (tuple,list):
    return len(value)>256
  if type(value) is dict and depth<3:
    for child in value.values():
      child_depth = depth + 1
      if type(child) in (tuple,list):
        if len(child)>256:
          return True
      elif type(child) is dict and _large_sequence(child,child_depth):
        return True
  return False


def _typed_json_chunks(value):
  """Byte-identical typed JSON, batching whole rows through the C encoder."""
  kind=type(value)
  if kind in (tuple,list) and len(value)>256:
    yield '["tuple",[' if kind is tuple else '["list",['
    for start in range(0,len(value),64):
      if start:
        yield ','
      batch=[_typed_body(child) for child in value[start:start+64]]
      yield json.dumps(batch,separators=(',', ':'),allow_nan=False)[1:-1]
    yield ']]'
  elif kind is dict and _large_sequence(value):
    yield '["dict",['
    for index,key in enumerate(sorted(value)):
      if index:
        yield ','
      yield '['+json.dumps(key,separators=(',', ':'),allow_nan=False)+','
      yield from _typed_json_chunks(value[key])
      yield ']'
    yield ']]'
  else:
    yield json.dumps(_typed_body(value),separators=(',', ':'),allow_nan=False)


def _body_digest_python(value):
  """Same exact digest; ordinary row bodies use one original C JSON encoding."""
  if not _large_sequence(value):
    return _body_digest_original(value)
  try:
    supported=_exact_primitive_body(value)
  except RecursionError:
    return _body_digest_original(value)
  if not supported:
    return _body_digest_original(value)
  digest=hashlib.sha256()
  for chunk in _typed_json_chunks(value):
    digest.update(chunk.encode())
  return digest.hexdigest()


class _UnsupportedPhysicalBody(Exception):
  pass


def _physical_asdict(value, *, _trace_owner=None, **overrides):
  from openpilot.starpilot.flm.normalized_trace_archive import TraceRef,record_body
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
  if type(value) is ControlEvidence and type(value.effect_trace) is TraceRef:
    if _trace_owner is None:
      raise ValueError('normalized physical record requires literal owner')
    return {**record_body(value,_trace_owner),**overrides}
  from openpilot.starpilot.flm.torque_emission_replay import Emission
  from opendbc.car.steering_projection import TorqueProjection
  registered = (ControlEvidence, AppliedEvidence, Emission, OriginalPerformanceTags, PhysicalDeliveredInput)
  if type(value) not in registered or type(value) is ControlEvidence and value.effect_trace is not None:
    return {**asdict(value), **overrides}

  def convert(item):
    kind = type(item)
    if item is None or kind in (bool, int, float, str, bytes):
      return item
    if kind is TorqueProjection:
      return TorqueProjection(*(convert(child) for child in item))
    if kind is tuple:
      return tuple(convert(child) for child in item)
    if kind is list:
      return [convert(child) for child in item]
    if kind is dict:
      return {convert(key): convert(child) for key, child in item.items()}
    if kind in registered:
      return {field.name: convert(getattr(item, field.name)) for field in fields(item)}
    raise _UnsupportedPhysicalBody

  try:
    result = {field.name: convert(getattr(value, field.name)) for field in fields(value)}
    result.update(overrides)
    return result
  except _UnsupportedPhysicalBody:
    return {**asdict(value), **overrides}


def _copy_callback_free(value):
  from opendbc.car.steering_projection import TorqueProjection
  kind = type(value)
  if value is None or kind in (bool, int, float, str, bytes):
    return True
  if kind in (tuple, list, TorqueProjection):
    return all(_copy_callback_free(child) for child in value)
  if kind is dict:
    return all(_copy_callback_free(key) and _copy_callback_free(child) for key, child in value.items())
  return False


def _applied_body_hashes(output, original_output, witness) -> tuple[str, str]:
  from openpilot.starpilot.flm.fullrate_collector import AppliedEvidence
  from openpilot.starpilot.flm.torque_emission_replay import projection
  from opendbc.car.steering_projection import projection_to_dict, TorqueProjection
  original_body = _physical_asdict(original_output)
  original_body['projection'] = projection_to_dict(projection(original_output.projection))
  original = body_digest(original_body)
  if (type(output) is AppliedEvidence and output is original_output and output.projection is witness and
      type(witness) is TorqueProjection and all(_copy_callback_free(getattr(output, field.name)) for field in fields(output))):
    return original, original
  effective_body = _physical_asdict(output)
  effective_body['projection'] = projection_to_dict(witness)
  return original, body_digest(effective_body)


def _emission_body_hash(event):
  from opendbc.car.steering_projection import projection_to_dict
  body = _physical_asdict(event)
  body['witness'] = projection_to_dict(event.witness)
  return body_digest(body)


def body_digest(value) -> str:
  result = _try_native('whole_body_digest', value)
  return _body_digest_python(value) if result is _NATIVE_FALLBACK else result


@dataclass(frozen=True)
class OriginalPerformanceTags:
  control_limited: bool
  output_limited: bool
  controller_saturated: bool
  model_goal_wobble: bool
  controller_chatter: bool


@dataclass(frozen=True)
class PhysicalDeliveredInput:
  """Distinct from strict DeliveredCommand; has no permissive health defaults."""
  protocol: str
  control_ns: int
  state_ns: int
  output_state_ns: int
  apply_ns: int
  command: float
  source_receipt_ns: int
  native_observed_ns: int
  native_expires_ns: int
  carrier_frame: int
  carrier_observed_ns: int
  encoded_witness_sha256: str
  control_sha256: str
  original_applied_sha256: str
  effective_applied_sha256: str
  actual_emission_sha256: str
  identity: str
  generation: int
  source_epoch: str
  baseline_revision: str
  controller_revision_sha256: str
  native_identity: str
  native_required: bool
  limiter_kind: str
  tags: OriginalPerformanceTags
  disposition: str
  binding_digest: str
  observation_digest: str


@dataclass(frozen=True)
class VerifiedPhysicalTape:
  protocol: str
  commands: tuple[PhysicalDeliveredInput, ...]
  binding_digest: str
  verified_tape_digest: str
  verifier_source_sha256: str


def observation_digest(row):
  value = _physical_asdict(row)
  value.pop('observation_digest')
  return body_digest(value)


def physical_tape_digest(binding, commands):
  return body_digest({'protocol': PROTOCOL, 'binding': asdict(binding),
                      'commands': [_physical_asdict(row) for row in commands]})


def validate_input(row, binding, binding_digest):
  """Pure structural/custody guard; not a native/source proof executor."""
  if type(row) is not PhysicalDeliveredInput or type(row.tags) is not OriginalPerformanceTags:
    raise ValueError('explicit physical-only input type')
  if row.protocol != PROTOCOL or any(type(value) is not bool for value in asdict(row.tags).values()):
    raise ValueError('physical-only protocol and exact original boolean tags')
  expected = 'observed_input_performance_tagged' if any(asdict(row.tags).values()) else 'observed_input_clean'
  integer_fields = (row.control_ns, row.state_ns, row.output_state_ns, row.apply_ns,
                    row.source_receipt_ns, row.carrier_frame, row.carrier_observed_ns)
  if (any(type(value) is not int or value <= 0 for value in integer_fields) or
      not row.state_ns <= row.control_ns <= row.state_ns + 30_000_000 or
      not row.state_ns <= row.output_state_ns <= row.apply_ns or
      not row.control_ns <= row.apply_ns <= row.control_ns + 30_000_000 or
      row.apply_ns - row.output_state_ns > 30_000_000 or
      not row.source_receipt_ns <= row.control_ns <= row.source_receipt_ns + 500_000_000 or
      not row.apply_ns <= row.carrier_observed_ns <= row.apply_ns + 100_000_000 or
      type(row.command) not in (int, float) or not math.isfinite(row.command) or abs(row.command) > 1 or
      type(row.generation) is not int or row.generation < 0 or
      type(row.native_required) is not bool or row.native_required is not binding.native_required or
      any(type(value) is not int for value in (row.native_observed_ns, row.native_expires_ns)) or
      (not 0 < row.native_observed_ns <= row.control_ns <= row.apply_ns <= row.native_expires_ns or
       row.native_expires_ns - row.native_observed_ns > 200_000_000 if row.native_required else
       (row.native_observed_ns, row.native_expires_ns) != (0, 0)) or
      (row.identity, row.source_epoch, row.baseline_revision, row.controller_revision_sha256, row.native_identity) !=
      (binding.identity, binding.source_epoch, binding.baseline_revision, binding.controller_revision, binding.native_identity) or
      row.binding_digest != binding_digest or row.disposition != expected or
      row.limiter_kind not in ('driver', 'measured', 'normalized_rate') or
      any(not _hash(value) for value in (row.encoded_witness_sha256, row.control_sha256,
        row.original_applied_sha256, row.effective_applied_sha256, row.actual_emission_sha256,
        row.binding_digest, row.observation_digest)) or row.observation_digest != observation_digest(row)):
    raise ValueError('physical original clocks/leases/identity/CAN/tag custody')


def from_original(control, output, engine, binding, *, original_output):
  """Actual child boundary. Performance labels preserved; hard guards remain.

  Must be called after original_context_receipts.verify. Never accepts caller
  callbacks, saved qualified flags, strict-record conversions or fake engines.
  """
  from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
  from openpilot.starpilot.flm.controller_replay import ControllerReplay
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  from openpilot.starpilot.flm.torque_eligibility import consumed_sources_current
  from openpilot.starpilot.flm.torque_emission_replay import delivered, projection
  from openpilot.starpilot.flm.runtime_adapter_registry import digest
  from openpilot.starpilot.flm.plant_lag import SourceBinding, binding_digest
  from opendbc.car.steering_projection import projection_to_dict

  if (type(control) is not ControlEvidence or type(output) is not AppliedEvidence or
      type(original_output) is not AppliedEvidence or type(binding) is not SourceBinding or
      type(engine) not in (ControllerReplay, EventControllerReplay) or
      type(engine) is EventControllerReplay and type(engine.engine) is not ControllerReplay):
    raise ValueError('original physical source/actual replay types required')
  flags = (control.enabled, control.manual, control.active, control.valid, control.override,
    control.fault, control.saturated, control.limited, control.model_wobble, control.controller_chatter,
    control.native_required, control.native_allowed, output.valid, output.active, output.override,
    output.fault, output.limited, output.native_allowed, original_output.native_allowed)
  if (any(type(value) is not bool for value in flags) or engine.failed or not engine.raw_mode or
      not control.enabled or control.manual or not control.active or not control.valid or
      control.override or control.fault or any(control.applied_coordinates) or
      not output.valid or not output.active or output.override or output.fault or not output.native_allowed or
      output.domain != 'normalized_torque' or control.domain != output.domain or
      output.control_ns != control.control_ns or not consumed_sources_current(control, output)):
    raise ValueError('physical hard validity/override/fault/neutral/source-consumption guard')
  stamps = (control.source_receipt_ns, control.native_observed_ns, control.native_expires_ns,
            output.carrier_frame, output.carrier_observed_ns)
  if (any(type(value) is not int for value in stamps) or
      type(control.generation) is not int or control.generation < 0 or
      not control.native_required and control.native_allowed or
      not 0 < control.source_receipt_ns <= control.control_ns <= control.source_receipt_ns + 500_000_000 or
      output.carrier_frame <= 0 or not output.apply_ns <= output.carrier_observed_ns <= output.apply_ns + 100_000_000 or
      (not control.native_allowed or not 0 < control.native_observed_ns <= control.control_ns <= output.apply_ns <= control.native_expires_ns or
       control.native_expires_ns - control.native_observed_ns > 200_000_000 if control.native_required else
       (control.native_observed_ns, control.native_expires_ns) != (0, 0))):
    raise ValueError('physical original source/native lease/carrier guard')
  if output != original_output and (not control.native_required or
      output != replace(original_output, native_allowed=True)):
    raise ValueError('physical original applied body changed')
  # verify_original_replay installs this map only after the actual original
  # controller/native/context replay. A real class plus a truthy method is not
  # by itself proof that this particular ControlEvidence was in that replay.
  originals = getattr(engine, 'original_context_controls', None)
  original_control = originals.get(control.control_ns) if type(originals) is dict else None
  control_body_sha256 = body_digest(_physical_asdict(control,_trace_owner=engine))
  if (type(original_control) is not ControlEvidence or
      original_control is not control and body_digest(_physical_asdict(original_control,_trace_owner=engine)) != control_body_sha256):
    raise ValueError('physical verified original context owner body required')
  if (control.native_required and not control.native_allowed or
      engine.identity != binding.identity or engine.baseline_revision != binding.baseline_revision or
      engine._recorded_cp_digest() != binding.cp_sha256 or
      digest(control.controller_revision) != binding.controller_revision or
      (control.identity, control.source_epoch, control.baseline_revision, control.native_identity) !=
      (binding.identity, binding.source_epoch, binding.baseline_revision, binding.native_identity)):
    raise ValueError('physical original selected/native identity binding')
  witness = projection(output.projection)
  if (projection_to_dict(witness) is None or not witness.steer_requested or
      witness.requested_torque != control.requested or witness.reported_torque != output.applied or
      witness.family != str(engine.factory.cp.brand) or
      witness.fingerprint != str(engine.factory.cp.carFingerprint) or witness.flags != int(engine.factory.cp.flags) or
      not engine.applied_matches(control, output)):
    raise ValueError('physical actual native/CAN/limiter-state proof')
  # applied_matches just ran the actual decoder/encoder proof. Bind that exact
  # archived event without executing a second limiter replay for every input.
  event = engine.emissions.events.get(output.apply_ns)
  if (event is None or (event.control_ns, event.state_ns, event.apply_ns) !=
      (control.control_ns, output.state_ns, output.apply_ns) or event.witness != witness or
      event.limited is not output.limited or
      event.native_required is not control.native_required or
      (event.native_identity, event.native_observed_ns, event.native_expires_ns) !=
      (control.native_identity, control.native_observed_ns, control.native_expires_ns) or
      not event.valid or not event.active or event.override or event.fault or not event.native_allowed):
    raise ValueError('physical exact actual emission native owner')
  tags = OriginalPerformanceTags(control.limited, original_output.limited, control.saturated,
                                 control.model_wobble, control.controller_chatter)
  row = PhysicalDeliveredInput(PROTOCOL, control.control_ns, control.state_ns, output.state_ns,
    output.apply_ns, -delivered(output), control.source_receipt_ns, control.native_observed_ns,
    control.native_expires_ns, output.carrier_frame, output.carrier_observed_ns,
    body_digest(projection_to_dict(witness)), control_body_sha256,
    *_applied_body_hashes(output, original_output, witness),
    _emission_body_hash(event), control.identity, control.generation, control.source_epoch,
    control.baseline_revision, digest(control.controller_revision), control.native_identity, control.native_required,
    witness.limiter_kind, tags, 'observed_input_performance_tagged' if any(asdict(tags).values()) else
    'observed_input_clean', binding_digest(binding), '')
  row = replace(row, observation_digest=observation_digest(row))
  validate_input(row, binding, binding_digest(binding))
  return row
