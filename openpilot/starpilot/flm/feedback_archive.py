"""Bounded ORIGINAL companion observations; no native/model authority."""
from collections import OrderedDict, deque
from dataclasses import astuple
import pickle

VERSION='original-feedback-archive-v1'
MAX_ROWS=12000


def archive_binding(packet):
  """No feedback field in its own binding; original context remains bit exact."""
  from openpilot.starpilot.flm.physical_input_observation import body_digest
  from opendbc.car.steering_projection import TorqueProjection,projection_to_dict
  context=packet['context_receipts']
  if type(context) is not dict or type(context.get('outputs')) is not tuple:
    raise ValueError('original feedback context output tuple required')
  outputs=[]
  for row in context['outputs']:
    projection=row.get('projection')
    if isinstance(projection,TorqueProjection):
      projection=projection_to_dict(projection)
      if projection is None:
        raise ValueError('original feedback projection binding schema')
      row={**row,'projection':projection}
    outputs.append(row)
  # export_packet precedes its existing classless projection normalization.
  # Bind exactly that known codec on BOTH sides; no full-frame copy/repack.
  context={**context,'outputs':tuple(outputs)}
  from openpilot.starpilot.flm.normalized_trace_archive import normalized_packet,reserve_for_packet
  normalized = normalized_packet(packet)
  reserve_for_packet(packet)
  body = {'version':VERSION + '-normalized-v2' if normalized else VERSION,'context_receipts':context,
    'replay_prefix':packet['replay_prefix'],'original_replay_source':packet['original_replay_source'],
    'capture_epoch':packet['capture_epoch'],'capture_epoch_start_ns':packet['capture_epoch_start_ns'],
    'capture_epoch_cutoff_ns':packet['capture_epoch_cutoff_ns']}
  if normalized:
    body['trace_archive'] = packet['trace_archive']
  return body_digest(body)


class OriginalFeedbackArchive:
  def __init__(self):
    self.publications=OrderedDict()
    self.consumptions=OrderedDict()
    self.failed=False
    self.failures=deque()
    self.poison_through_ns=0

  def reject(self, received_ns):
    if type(received_ns) is not int or received_ns<=0:
      self.failed=True
    else:
      if len(self.failures)>=MAX_ROWS:
        self.poison_through_ns=max(self.poison_through_ns,self.failures.popleft())
      self.failures.append(received_ns)
    return False

  def _record(self, destination, stamp, row, received_ns):
    if type(received_ns) is not int or not 0<stamp<=received_ns:
      return self.reject(received_ns)
    prior=destination.get(stamp)
    if prior is not None and pickle.dumps(astuple(prior),protocol=5)!=pickle.dumps(astuple(row),protocol=5):
      return self.reject(stamp)
    if prior is None and destination and stamp<next(reversed(destination)):
      return self.reject(stamp)
    destination[stamp]=row
    while len(destination)>MAX_ROWS:
      destination.popitem(last=False)
    return True

  def publication(self, row, received_ns):
    from openpilot.starpilot.flm.feedback_observation import validate_publication
    validate_publication(row)
    return self._record(self.publications,row.publication_ns,row,received_ns)

  def consumption(self, row, control, received_ns):
    from openpilot.starpilot.flm.feedback_observation import validate_consumption,f32
    from openpilot.starpilot.flm.fullrate_collector import ControlEvidence
    validate_consumption(row)
    if (type(control) is not ControlEvidence or row.control_ns!=control.control_ns or
        row.source_epoch!=control.source_epoch or row.requested_torque!=f32(control.requested) or
        row.lat_active is not control.active or row.published_limited is not control.limited or
        dict(control.replay_inputs).get('safety_limited')!=float(row.input_limited)):
      return self.reject(row.control_ns)
    return self._record(self.consumptions,row.control_ns,row,received_ns)

  def export(self, packet):
    from openpilot.starpilot.flm.feedback_archive_codec import compact_observations
    if self.failed:
      raise ValueError('original feedback archive failed')
    stamps=tuple(row['control_ns'] for row in packet['context_receipts']['controls'])
    if (not stamps or any(stamp not in self.consumptions for stamp in stamps) or
        self.poison_through_ns and stamps[0]<=self.poison_through_ns+100_000_000 or
        any(stamps[0]-100_000_000<=stamp<=stamps[-1]+100_000_000 for stamp in self.failures)):
      raise ValueError('complete original feedback consumption unavailable')
    consumptions=tuple(self.consumptions[stamp] for stamp in stamps)
    # Original metadata interval, independent of quality, response or fit outcome.
    # Keep the preceding consumed CO if retained, plus all publications through
    # the last original controller frame; no heldout/flag-based row filtering.
    first=min(row.publication_ns for row in consumptions)
    publications=tuple(row for stamp,row in self.publications.items() if first<=stamp<=stamps[-1])
    return compact_observations(publications,consumptions,binding_digest=archive_binding(packet))


def record_companion(collector,event,received_ns,*,control=None):
  """New-version tap only. Missing/malformed siblings poison this prefix."""
  archive=getattr(collector.adapter,'original_feedback_archive',None)
  if archive is None:
    return
  from openpilot.starpilot.flm.feedback_wire import read_publication,read_consumption
  try:
    if control is not None:
      if event.which() != 'starpilotLateralState':
        raise ValueError('original feedback consumption carrier')
      row=read_consumption(event.starpilotLateralState)
      if row is None or not archive.consumption(row,control,received_ns):
        raise ValueError('complete original feedback consumption required')
    else:
      if event.which() != 'starpilotCarState':
        raise ValueError('original feedback publication carrier')
      row=read_publication(event.starpilotCarState)
      if row is None or not archive.publication(row,received_ns):
        raise ValueError('complete original feedback publication required')
  except (AttributeError,KeyError,TypeError,ValueError,OverflowError):
    archive.reject(received_ns)
