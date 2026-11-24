"""Historical numerical response reuse; never current command permission.

The original eight identification blocks remain provenance. A new source epoch
requires new independent physical residuals under its own actual native guards,
not eight invented/relabelled identification blocks. Failed checks restart fresh
observation; an unqualified historical model is never returned to a consumer.
"""

from collections import deque
from dataclasses import asdict, dataclass, replace
import hashlib
import math
import struct

VERSION = 'retained-current-residual-v1'


def _primitive_key(value, depth=0):
  if depth > 8:
    raise ValueError('bounded stable physical context')
  kind = type(value)
  if kind is tuple:
    if len(value) > 32:
      raise ValueError('bounded stable context tuple')
    return ('tuple', tuple(_primitive_key(item, depth + 1) for item in value))
  if kind is str and value.isascii() and 0 < len(value) <= 192:
    return ('str', value)
  if kind is bool:
    return ('bool', value)
  if kind is int and value.bit_length() <= 64:
    return ('int', value)
  if kind is float and math.isfinite(value):
    return ('float64', struct.pack('!d', value))
  raise ValueError('exact finite stable context primitives')


@dataclass(frozen=True)
class StableResponseBasis:
  identity: str
  cp_sha256: str
  controller_revision: str
  baseline_revision: str
  calibration_rpy: tuple
  vehicle_model: tuple
  hardware: tuple
  native_required: bool

  def key(self):
    if (
      type(self) is not StableResponseBasis
      or type(self.native_required) is not bool
      or type(self.calibration_rpy) is not tuple
      or len(self.calibration_rpy) != 3
      or any(type(v) is not float or not math.isfinite(v) for v in self.calibration_rpy)
      or type(self.vehicle_model) is not tuple
      or len(self.vehicle_model) != 7
      or any(type(v) is not float or not math.isfinite(v) or v == 0 for v in self.vehicle_model)
      or type(self.hardware) is not tuple
      or not self.hardware
    ):
      raise ValueError('complete measured stable response basis')
    return _primitive_key(
      (
        self.identity,
        self.cp_sha256,
        self.controller_revision,
        self.baseline_revision,
        self.calibration_rpy,
        self.vehicle_model,
        self.hardware,
        self.native_required,
      )
    )


def basis_from_pose(binding, pose, *, now_ns, native_key=(), hardware_inventory=()):
  """Use measured pose/calibration/VM and the actual current owner inventory.

  Required-axis identity is checked in full BEFORE omitting its ephemeral axis
  session from the retained numerical basis. Ordinary hardware inventory must be
  supplied by the real current observer; no serial or receiver contract is made
  up here. Missing inventory prevents reuse and leaves bootstrap available.
  """
  from openpilot.starpilot.flm.original_source_capture import validate_pose
  from openpilot.starpilot.flm.live_tracking_response import physical_calibration_generation
  from openpilot.starpilot.flm.runtime_adapter_registry import native_identity_digest

  binding.validate()
  validate_pose(pose, now_ns)
  if (
    len(pose.context) != 15
    or (pose.identity, pose.source_epoch, pose.baseline_revision, pose.controller_revision, pose.calibration, pose.native_identity)
    != (
      binding.identity,
      binding.source_epoch,
      binding.baseline_revision,
      binding.controller_revision,
      physical_calibration_generation(binding),
      binding.native_identity,
    )
    or pose.native_required is not binding.native_required
    or pose.context[5] != binding.controller_revision
    or type(pose.context[10]) is not tuple
    or len(pose.context[10]) != 3
  ):
    raise ValueError('current original physical context differs')
  if binding.native_required:
    if type(native_key) is not tuple or len(native_key) != 5 or native_identity_digest(native_key) != binding.native_identity:
      raise ValueError('actual current complete native inventory required')
    hardware = ('axis-hardware-v1', *native_key[1:])
  else:
    if native_key != () or type(hardware_inventory) is not tuple or not hardware_inventory:
      raise ValueError('actual ordinary hardware inventory required')
    config = pose.ordinary_safety[4]
    if type(config) is not tuple or len(config) != 3:
      raise ValueError('ordinary current configuration inventory')
    # Preserve config and inventory. controlsAllowed/RX/heartbeat/fault values
    # are CURRENT guards in validate_pose/current observer, not historical keys.
    hardware = ('ordinary-hardware-v1', hardware_inventory, config[0], config[1], tuple(tuple(row[:4]) for row in config[2]))
  value = StableResponseBasis(
    binding.identity,
    binding.cp_sha256,
    binding.controller_revision,
    binding.baseline_revision,
    pose.calibration_rpy,
    (*pose.context[10], *pose.context[11:]),
    hardware,
    binding.native_required,
  )
  value.key()
  return value


def compatible_basis(historical, current):
  from openpilot.starpilot.flm.runtime_adapter_registry import vm_regime

  historical.key()
  current.key()
  normalized = vm_regime(dict(zip(("vm.sR", "vm.cF", "vm.cR"), current.vehicle_model[:3], strict=True)), historical.vehicle_model[:3])
  if normalized is None:
    return False
  # selected_identity includes source_epoch. Compare stable physical context,
  # while each pose still must match its OWN current canonical binding above.
  return historical.key() == replace(current, identity=historical.identity, vehicle_model=(*normalized, *current.vehicle_model[3:])).key()


def _rebound(certificate, binding, evidence_digest, *, command_range=None):
  from openpilot.starpilot.flm.live_tracking_response import _revision, validate_certificate, TrackingSourceContext

  value = replace(
    certificate,
    binding=replace(binding, physical_model_revision='0' * 64),
    model_revision='0' * 64,
    provenance=('retained-current-source-v3' if type(certificate) is TrackingSourceContext else VERSION),
    parent_model_revision=certificate.model_revision,
    requalification_sha256=evidence_digest,
  )
  if command_range is not None:
    low, high = command_range
    if not certificate.command_low <= low < high <= certificate.command_high:
      raise ValueError('local response range cannot extrapolate original measurements')
    value = replace(value, command_low=low, command_high=high)
  revision = _revision(value)
  return validate_certificate(replace(value, model_revision=revision, binding=replace(value.binding, physical_model_revision=revision)))


class RetainedResponseRequalifier:
  """Fresh independent residual/range witness before exposing a rebound model.

  This object contains no Profile, native lease or actuation admission. It may
  run across a new UUID/epoch only when measured stable basis remains identical.
  It never reuses the previous monitor's rows, permission or pending trial.
  """

  def __init__(self, certificate, historical_basis, current_binding):
    from openpilot.starpilot.flm.live_tracking_response import validate_certificate, ResponseMonitor

    self.original = validate_certificate(certificate)
    if type(historical_basis) is not StableResponseBasis:
      raise ValueError('exact retained measured basis')
    self.basis_key = historical_basis.key()
    self._historical_basis = historical_basis
    if (
      certificate.binding.identity,
      certificate.binding.cp_sha256,
      certificate.binding.controller_revision,
      certificate.binding.baseline_revision,
      certificate.binding.native_required,
    ) != (
      historical_basis.identity,
      historical_basis.cp_sha256,
      historical_basis.controller_revision,
      historical_basis.baseline_revision,
      historical_basis.native_required,
    ):
      raise ValueError('retained certificate/basis mismatch')
    current_binding.validate()
    if (current_binding.cp_sha256, current_binding.controller_revision, current_binding.baseline_revision, current_binding.native_required) != (
      historical_basis.cp_sha256,
      historical_basis.controller_revision,
      historical_basis.baseline_revision,
      historical_basis.native_required,
    ):
      raise ValueError('car/controller/tune/native domain changed')
    # Private provisional certificate is NEVER a returned/reusable result.
    self._candidate = _rebound(self.original, current_binding, '0' * 64)
    self._monitor = ResponseMonitor(self._candidate)
    self._binding = self._candidate.binding
    self._rows = deque(maxlen=80)
    self._frozen_certificate = None
    self.reason = 'fresh_independent_requalification_required'

  def observe(self, control, output, pose, *, now_ns, current_native, native_key=(), hardware_inventory=()):
    from openpilot.starpilot.flm.torque_emission_replay import delivered

    try:
      if current_native is not True:
        raise ValueError('current native permission unavailable')
      if self._historical_basis.key() != self.basis_key:
        raise ValueError("retained stable basis mutated")
      current = basis_from_pose(self._binding, pose, now_ns=now_ns, native_key=native_key, hardware_inventory=hardware_inventory)
      if not compatible_basis(self._historical_basis, current):
        raise ValueError('measured_calibration_vm_or_hardware_changed')
      if control.controller_revision != self._binding.controller_revision or control.generation != pose.context[2]:
        raise ValueError("current controller/generation differs from measured pose")
      accepted = self._monitor.observe(control, output, pose, now_ns=now_ns, binding=self._binding, current_native=current_native)
      if not accepted:
        if self._monitor.reason != 'qualified':
          self._rows.clear()
        self.reason = self._monitor.reason
        return False
      self._rows.append((pose.mono_ns, control.control_ns, output.apply_ns, float(-delivered(output)), float(pose.speed_mps)))
      valid_stamps = {row[0] for row in self._monitor.rows}
      self._rows = deque((row for row in self._rows if row[0] in valid_stamps), maxlen=80)
      self.reason = 'fresh_independent_requalification_required'
      return True
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as error:
      self._monitor.reset(str(error))
      self._rows.clear()
      self.reason = str(error)
      return False

  def interrupt(self, reason):
    """Withdraw fresh samples only; preserve the historical numerical record."""
    self._monitor.reset(reason)
    self._rows.clear()
    self.reason = reason

  def persistent_residual_failure(self, now_ns):
    """Fresh qualified numerical failure, never a current permission verdict."""
    from openpilot.starpilot.flm.live_tracking_response import validate_certificate

    validate_certificate(self.original)
    if self._historical_basis.key() != self.basis_key:
      raise ValueError("retained stable basis mutated")
    return self._monitor.persistent_residual_failure(now_ns)

  def qualified(self, now_ns):
    """Return numerical certificate + freshly validated physical witness only."""
    from openpilot.starpilot.flm.live_tracking_response import ResponseWitness, validate_response_witness, validate_certificate, TrackingSourceContext

    validate_certificate(self.original)
    if self._historical_basis.key() != self.basis_key:
      raise ValueError("retained stable basis mutated")
    witness = self._monitor.witness(now_ns)
    if witness is None or len(self._rows) < 8:
      return None
    speeds = [row[4] for row in self._rows]
    if min(speeds) <= 0 or max(speeds) > 1.15 * min(speeds):
      self.reason = 'fresh_comparable_speed_range_required'
      return None
    if self._frozen_certificate is not None:
      # Ordinary poses cannot silently extend a published numerical scope.
      return self._frozen_certificate, witness
    commands = [row[3] for row in self._rows]
    # Retention is not slope identification. Restrict support to the exact
    # distinct delivered codec coordinates seen in this fresh residual window.
    # The independent original identification and its thresholds are unchanged.
    source_only = type(self.original) is TrackingSourceContext
    if not source_only and min(commands) == max(commands):
      self.reason = 'fresh_local_range_required'
      return None
    # Do not freeze support on the leading edge of a startup transient. A
    # monotone run's next genuine sample can already lie outside its min/max,
    # starving the separate fresh ingress before it can publish any restore.
    # Require the ORIGINAL delivered input to revisit its measured range from
    # both directions through a complete excursion (two reversals). A single
    # startup peak is not a revisited local operating range. This is input-only readiness, not a residual-selected
    # window, and neither expands the original support nor renews permission.
    changes = [right - left for left, right in zip(commands, commands[1:], strict=False) if right != left]
    signs = [value > 0 for value in changes]
    reversals = sum(left != right for left, right in zip(signs, signs[1:], strict=False))
    if not source_only and reversals < 2:
      self.reason = 'fresh_local_range_revisit_required'
      return None
    digest = hashlib.sha256(repr((VERSION, self.original.model_revision, self.basis_key, tuple(self._rows), witness.residual_rmse)).encode('ascii')).hexdigest()
    cert = _rebound(self.original, self._binding, digest, command_range=None if source_only else (min(commands), max(commands)))
    fresh = ResponseWitness(
      cert,
      cert.binding,
      witness.last_control_ns,
      witness.last_apply_ns,
      witness.last_pose_ns,
      witness.window_start_ns,
      witness.independent_poses,
      witness.residual_rmse,
      witness.valid_until_ns,
    )
    validate_response_witness(fresh, cert.binding, now_ns)
    self._frozen_certificate = cert
    self._binding = cert.binding
    # Existing rows are real observations inside this exact min/max; model
    # coefficients and lag did not change. No new pose or permission is made.
    self._monitor.certificate = cert
    self.reason = 'retained_response_freshly_requalified'
    return cert, fresh


def retained_to_dict(certificate, basis):
  from openpilot.starpilot.flm.live_tracking_response import certificate_to_dict

  # Constructor validates matching historical context, without claiming freshness.
  RetainedResponseRequalifier(certificate, basis, certificate.binding)
  return {"version": VERSION, "certificate": certificate_to_dict(certificate), "basis": asdict(basis), "current_permission": False, "pending_trial": None}


def retained_from_dict(value):
  from openpilot.starpilot.flm.live_tracking_response import certificate_from_dict

  if (
    type(value) is not dict
    or set(value) != {"version", "certificate", "basis", "current_permission", "pending_trial"}
    or value["version"] != VERSION
    or value["current_permission"] is not False
    or value["pending_trial"] is not None
    or type(value["basis"]) is not dict
    or set(value["basis"]) != set(StableResponseBasis.__dataclass_fields__)
  ):
    raise ValueError("historical numerical model only; no permission or pending trial")

  def freeze(item, depth=0):
    if depth > 8:
      raise ValueError("bounded historical numerical basis")
    if type(item) in (list, tuple):
      if len(item) > 32:
        raise ValueError("bounded historical numerical inventory")
      return tuple(freeze(row, depth + 1) for row in item)
    _primitive_key(item)
    return item

  raw = dict(value["basis"])
  for key in ("calibration_rpy", "vehicle_model", "hardware"):
    raw[key] = freeze(raw[key])
  basis = StableResponseBasis(**raw)
  certificate = certificate_from_dict(value["certificate"])
  RetainedResponseRequalifier(certificate, basis, certificate.binding)
  return certificate, basis
