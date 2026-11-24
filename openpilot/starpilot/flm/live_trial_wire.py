"""Additive custom live-trial messages; never parse them as replay profiles."""

from openpilot.starpilot.flm.live_trial_contract import VERSION, LiveTrialBinding, LiveTrialOffer, validate_offer
from openpilot.starpilot.flm.live_trial_owner import LiveTrialApplication

_BINDING = (
  'identity',
  'cp_sha256',
  'controller_revision',
  'baseline_revision',
  'source_epoch',
  'reset_generation',
  'calibration_generation',
  'vehicle_model_generation',
  'native_identity',
  'physical_model_revision',
  'persistent_source_epoch',
)
_WIRE = (
  'identity',
  'cpSha256',
  'controllerRevision',
  'baselineRevision',
  'sourceEpoch',
  'resetGeneration',
  'calibrationGeneration',
  'vehicleModelGeneration',
  'nativeIdentity',
  'physicalModelRevision',
  'persistentSourceEpoch',
)


def _binding(value):
  return {**{wire: getattr(value, name) for name, wire in zip(_BINDING, _WIRE, strict=True)}, 'nativeRequired': value.native_required}


def _read_binding(value):
  return LiveTrialBinding(*(str(getattr(value, name)) for name in _WIRE), native_required=bool(value.nativeRequired)).validate()


def write_trial(event, offer):
  validate_offer(offer)
  event.starpilotSteeringTracking.version = 1
  event.starpilotSteeringTracking.trial = {
    'version': 1,
    'protocol': VERSION,
    'binding': _binding(offer.binding),
    'trialId': offer.trial_id,
    'sequence': offer.sequence,
    'phase': offer.phase,
    'group': offer.group,
    'names': offer.names,
    'baseline': [{'name': n, 'value': v} for n, v in offer.baseline],
    'target': [{'name': n, 'value': v} for n, v in offer.target],
    'baselineSha256': offer.baseline_sha256,
    'targetSha256': offer.target_sha256,
    'startedMonoTime': offer.started_ns,
    'observedMonoTime': offer.observed_ns,
    'validUntilMonoTime': offer.expires_ns,
    'responseWitness': _write_witness(offer.response_witness, offer.binding, offer.observed_ns),
    'targetSequence': offer.target_sequence,
    'deadlineMonoTime': offer.deadline_ns,
  }


def read_trial(envelope):
  row = envelope.trial
  if int(row.version) == 0:
    return None
  if int(row.version) != 1 or str(row.protocol) != VERSION or len(row.names) > 4 or len(row.baseline) != 30 or len(row.target) != 30:
    raise ValueError('live trial wire version/bounds')
  return validate_offer(
    LiveTrialOffer(
      VERSION,
      _read_binding(row.binding),
      str(row.trialId),
      int(row.sequence),
      str(row.phase),
      str(row.group),
      tuple(str(v) for v in row.names),
      tuple((str(v.name), float(v.value)) for v in row.baseline),
      tuple((str(v.name), float(v.value)) for v in row.target),
      str(row.baselineSha256),
      str(row.targetSha256),
      int(row.startedMonoTime),
      int(row.observedMonoTime),
      int(row.validUntilMonoTime),
      _read_witness(row.responseWitness, _read_binding(row.binding), int(row.observedMonoTime)),
      int(row.targetSequence),
      int(row.deadlineMonoTime),
    )
  )


def write_application(parent, application):
  if type(application) is not LiveTrialApplication:
    raise ValueError('typed live application')
  parent.trackingApplication = {
    'version': 1,
    'binding': _binding(application.binding),
    'trialId': application.trial_id,
    'sequence': application.sequence,
    'controlMonoTime': application.control_ns,
    'status': application.status,
    'applied': [{'name': n, 'value': v} for n, v in application.applied],
    'appliedSha256': application.applied_sha256,
    'confirmedTargetSha256': application.confirmed_target_sha256,
    'reason': application.reason,
    'targetSequence': application.target_sequence,
  }


def read_application(parent, *, now_ns, offer):
  from openpilot.starpilot.flm.live_trial_contract import full_coordinates

  row = parent.trackingApplication
  if (
    int(row.version) != 1
    or len(row.applied) != 30
    or _read_binding(row.binding) != offer.binding
    or str(row.trialId) != offer.trial_id
    or not 0 < int(row.sequence) <= offer.sequence
    or int(row.targetSequence) != offer.target_sequence
    or not 0 < int(row.controlMonoTime) <= now_ns <= int(row.controlMonoTime) + 100_000_000
    or str(row.status) not in ('applied', 'neutral', 'rejected')
    or len(str(row.reason)) > 96
  ):
    return None
  values = tuple((str(v.name), float(v.value)) for v in row.applied)
  if full_coordinates(values, offer.binding.baseline_revision) != str(row.appliedSha256):
    return None
  confirmed = str(row.confirmedTargetSha256)
  if confirmed and (offer.phase != 'confirmed' or confirmed != offer.target_sha256 or values != offer.target or str(row.appliedSha256) != offer.target_sha256):
    return None
  return LiveTrialApplication(
    1,
    offer.binding,
    offer.trial_id,
    int(row.sequence),
    int(row.controlMonoTime),
    str(row.status),
    values,
    str(row.appliedSha256),
    confirmed,
    str(row.reason),
    int(row.targetSequence),
  )


def write_restore(event, offer):
  offer.validate()
  event.starpilotSteeringTracking.version = 1
  event.starpilotSteeringTracking.restore = {
    'version': 1,
    'protocol': offer.version,
    'binding': _binding(offer.binding),
    'historyScope': offer.history_scope,
    'historyTrialId': offer.history_trial_id,
    'target': [{'name': n, 'value': v} for n, v in offer.target],
    'targetSha256': offer.target_sha256,
    'sequence': offer.sequence,
    'observedMonoTime': offer.observed_ns,
    'validUntilMonoTime': offer.expires_ns,
    'responseWitness': _write_witness(offer.response_witness, offer.binding, offer.observed_ns),
  }


def read_restore(envelope):
  from openpilot.starpilot.flm.live_trial_contract import RESTORE_VERSION, LiveRestoreOffer

  row = envelope.restore
  if int(row.version) == 0:
    return None
  if int(row.version) != 1 or str(row.protocol) != RESTORE_VERSION or len(row.target) != 30:
    raise ValueError('confirmed restore wire version/bounds')
  return LiveRestoreOffer(
    RESTORE_VERSION,
    _read_binding(row.binding),
    str(row.historyScope),
    str(row.historyTrialId),
    tuple((str(v.name), float(v.value)) for v in row.target),
    str(row.targetSha256),
    int(row.sequence),
    int(row.observedMonoTime),
    int(row.validUntilMonoTime),
    _read_witness(row.responseWitness, _read_binding(row.binding), int(row.observedMonoTime)),
  ).validate()


def read_restore_application(parent, *, now_ns, offer):
  from openpilot.starpilot.flm.live_trial_contract import full_coordinates

  row = parent.trackingApplication
  if (
    int(row.version) != 1
    or len(row.applied) != 30
    or _read_binding(row.binding) != offer.binding
    or str(row.trialId) != offer.history_trial_id
    or not 0 < int(row.sequence) <= offer.sequence
    or not 0 < int(row.controlMonoTime) <= now_ns <= int(row.controlMonoTime) + 100_000_000
    or str(row.status) not in ('applied', 'restored', 'neutral', 'rejected')
    or len(str(row.reason)) > 96
  ):
    return None
  values = tuple((str(v.name), float(v.value)) for v in row.applied)
  if full_coordinates(values, offer.binding.baseline_revision) != str(row.appliedSha256):
    return None
  confirmed = str(row.confirmedTargetSha256)
  if str(row.status) == 'restored' and (values != offer.target or confirmed != offer.target_sha256 or str(row.appliedSha256) != offer.target_sha256):
    return None
  if str(row.status) != 'restored' and confirmed:
    return None
  return LiveTrialApplication(
    1,
    offer.binding,
    offer.history_trial_id,
    int(row.sequence),
    int(row.controlMonoTime),
    str(row.status),
    values,
    str(row.appliedSha256),
    confirmed,
    str(row.reason),
  )


def _write_witness(witness, binding, now_ns):
  from openpilot.starpilot.flm.live_tracking_response import validate_response_witness

  witness = validate_response_witness(witness, binding, now_ns)
  c = witness.certificate
  from openpilot.starpilot.flm.live_tracking_response import TrackingSourceContext
  if type(c) is TrackingSourceContext:
    return {'certificate':{'version':c.version,'modelRevision':c.model_revision,'lagNs':c.lag_ns,
      'sourceInventorySha256':c.source_inventory_sha256,'identifiedMonoTime':c.observed_ns,
      'provenance':c.provenance,'parentModelRevision':c.parent_model_revision,
      'requalificationSha256':c.requalification_sha256},
      'lastControlMonoTime':witness.last_control_ns,'lastApplyMonoTime':witness.last_apply_ns,
      'lastPoseMonoTime':witness.last_pose_ns,'windowStartMonoTime':witness.window_start_ns,
      'independentPoses':witness.independent_poses,'validUntilMonoTime':witness.valid_until_ns}
  return {
    'certificate': {
      'version': c.version,
      'modelRevision': c.model_revision,
      'lagNs': c.lag_ns,
      'slope': c.slope,
      'intercept': c.intercept,
      'commandLow': c.command_low,
      'commandHigh': c.command_high,
      'heldoutRmse': c.heldout_rmse,
      'trainBlockIds': c.train_block_ids,
      'heldoutBlockIds': c.heldout_block_ids,
      'sourceInventorySha256': c.source_inventory_sha256,
      'identificationSha256': c.identification_sha256,
      'identifiedMonoTime': c.identified_ns,
      'provenance': c.provenance,
      'parentModelRevision': c.parent_model_revision,
      'requalificationSha256': c.requalification_sha256,
    },
    'lastControlMonoTime': witness.last_control_ns,
    'lastApplyMonoTime': witness.last_apply_ns,
    'lastPoseMonoTime': witness.last_pose_ns,
    'windowStartMonoTime': witness.window_start_ns,
    'independentPoses': witness.independent_poses,
    'residualRmse': witness.residual_rmse,
    'validUntilMonoTime': witness.valid_until_ns,
  }


def _read_witness(row, binding, now_ns):
  from openpilot.starpilot.flm.live_tracking_response import ResponseCertificate, ResponseWitness, prepare_response_witness

  c = row.certificate
  from openpilot.starpilot.flm.live_tracking_response import SOURCE_VERSION,TrackingSourceContext
  if str(c.version)==SOURCE_VERSION:
    if (len(c.trainBlockIds) or len(c.heldoutBlockIds) or str(c.identificationSha256) or
        any(float(value)!=0. for value in(c.slope,c.intercept,c.commandLow,c.commandHigh,c.heldoutRmse,row.residualRmse))):
      raise ValueError('source context cannot claim fitted model fields')
    context=TrackingSourceContext(str(c.version),binding,str(c.modelRevision),int(c.lagNs),
      str(c.sourceInventorySha256),int(c.identifiedMonoTime),str(c.provenance),
      str(c.parentModelRevision),str(c.requalificationSha256))
    return prepare_response_witness(ResponseWitness(context,binding,int(row.lastControlMonoTime),
      int(row.lastApplyMonoTime),int(row.lastPoseMonoTime),int(row.windowStartMonoTime),
      int(row.independentPoses),None,int(row.validUntilMonoTime)),binding,now_ns)
  if len(c.trainBlockIds) != 4 or len(c.heldoutBlockIds) != 4:
    raise ValueError('bounded response certificate blocks')
  if any(len(str(v)) > 128 for v in (*c.trainBlockIds, *c.heldoutBlockIds)):
    raise ValueError('bounded response certificate block ids')
  certificate = ResponseCertificate(
    str(c.version),
    binding,
    str(c.modelRevision),
    int(c.lagNs),
    float(c.slope),
    float(c.intercept),
    float(c.commandLow),
    float(c.commandHigh),
    float(c.heldoutRmse),
    tuple(str(v) for v in c.trainBlockIds),
    tuple(str(v) for v in c.heldoutBlockIds),
    str(c.sourceInventorySha256),
    str(c.identificationSha256),
    int(c.identifiedMonoTime),
    str(c.provenance),
    str(c.parentModelRevision),
    str(c.requalificationSha256),
  )
  return prepare_response_witness(
    ResponseWitness(
      certificate,
      binding,
      int(row.lastControlMonoTime),
      int(row.lastApplyMonoTime),
      int(row.lastPoseMonoTime),
      int(row.windowStartMonoTime),
      int(row.independentPoses),
      float(row.residualRmse),
      int(row.validUntilMonoTime),
    ),
    binding,
    now_ns,
  )


def write_tracking_source(event, source):
  # Optional producer only: reuse exact existing source validation, not a fake publisher.
  from openpilot.cereal import messaging
  from openpilot.starpilot.flm.learning_wire import write_source_receipt

  temporary = messaging.new_message('autoSteeringLearning')
  write_source_receipt(temporary, source)
  event.starpilotSteeringTracking.version = 1
  event.starpilotSteeringTracking.sourceReceipt = temporary.autoSteeringLearning.learning.sourceReceipt


def read_tracking_source(envelope, *, now_ns, cp_sha256):
  from openpilot.starpilot.flm.learning_wire import _read_native_source_receipt
  from openpilot.starpilot.flm.runtime_adapter_registry import SourceState

  if int(envelope.version) != 1:
    return None
  return _read_native_source_receipt(envelope.sourceReceipt, now_ns, cp_sha256, SourceState)
