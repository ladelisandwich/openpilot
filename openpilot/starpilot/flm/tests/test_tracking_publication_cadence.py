"""Actual publisher, typed tracking wire and100Hz owner; declared unit grants.

No native/hardware authority is inferred. Unit wire generation supplies a
proposal through the official codec; production permission is never replaced.
"""
from dataclasses import replace
from types import SimpleNamespace
import time

import pytest

from openpilot.cereal import messaging
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.live_tracking_response import ResponseWitness, prepare_response_witness
from openpilot.starpilot.flm.live_trial_owner import TrackingTrialOwner
from openpilot.starpilot.flm.live_trial_wire import read_trial, read_tracking_source, write_trial, write_tracking_source
from openpilot.starpilot.flm.tests.test_live_tracking_daemon import BASE, LoopComplete, RecordingActor, source
from openpilot.starpilot.flm.tests.test_live_trial_owner import offer, binding, response

TOPIC = 'starpilotSteeringTracking'
PERIOD = 50_000_000


def delivery_run(monkeypatch, producer, phase, *, stop=False, withdraw=False):
  now = [BASE-10_000_000]
  unit_binding = replace(binding(), cp_sha256='c'*64, source_epoch='b'*64, reset_generation='a'*64,
    calibration_generation='d'*32, persistent_source_epoch='e'*64)
  unit_binding, witness = response(unit_binding, BASE-1_000_000_000)
  seed = replace(offer(BASE-1_000_000_000), binding=unit_binding, response_witness=witness, deadline_ns=BASE+30_000_000_000)
  seed.validate()
  sequence = [0]
  calls, sends, interruptions = [], [], []

  class SM:
    updated = {'deviceMotion': False}
    logMonoTime = {'deviceMotion': 0}
    def update(self, timeout=None):
      self.updated['deviceMotion'] = False
      if timeout == 0:
        return
      now[0] += 10_000_000
      if now[0] > BASE+3_000_000_000:
        raise LoopComplete
      if (now[0]-BASE) % PERIOD == 0:
        self.updated['deviceMotion'] = True
        self.logMonoTime['deviceMotion'] = now[0]

  class Actor(RecordingActor):
    def interrupt(self, reason, **kwargs):
      interruptions.append(reason)
    def tick(self, *args, **kwargs):
      offset = now[0]-BASE
      if offset % PERIOD != phase or stop and offset > 1_000_000_000 or withdraw and offset >= 1_500_000_000:
        return None
      sequence[0] += 1
      pose = now[0]-40_000_000  # Declared original physical pose, never restamped.
      witness = prepare_response_witness(ResponseWitness(seed.response_witness.certificate, seed.binding,
        pose, pose, pose, pose-500_000_000, 8, .001, pose+100_000_000), seed.binding, now[0])
      return replace(seed, sequence=sequence[0], observed_ns=now[0], expires_ns=pose+100_000_000, response_witness=witness)

  original = producer.tracking_message
  def writer(current_source, **kwargs):
    result = original(current_source, **kwargs)
    calls.append((kwargs['now_ns'], current_source, kwargs['offer']))
    return result
  def send(topic, message):
    if topic == TOPIC:
      sends.append((now[0], message.as_reader()))
  def read(stamp):
    value = source(stamp)
    return replace(value, enabled=False) if withdraw and stamp >= BASE+1_500_000_000 else value
  monkeypatch.setattr(producer, 'tracking_message', writer)
  monkeypatch.setattr(producer, 'drain_pair', lambda *args: None)
  started = time.monotonic_ns()
  with pytest.raises(LoopComplete):
    producer.run(SimpleNamespace(), SM(), object(), object(), SimpleNamespace(send=send), None, None,
      read, None, None, None, None, None, None, clock=lambda: now[0], live_actor=Actor())
  finished = time.monotonic_ns()
  assert all(started <= int(message.logMonoTime) <= finished for _, message in sends)
  assert all(message.starpilotSteeringTracking.trial.version == 0 for _, message in sends)
  monkeypatch.delenv('SIMULATION', raising=False)
  receiver = messaging.SubMaster(['selfdriveState', TOPIC], poll='selfdriveState')
  assert receiver.ignore_alive == receiver.ignore_valid == receiver.ignore_average_freq == [] and not receiver.simulation
  for stamp, message in sends:
    receiver.update_msgs(stamp/1e9, [message])
  assert receiver.all_checks([TOPIC])
  assert 16 <= 1/receiver.freq_tracker[TOPIC].avg_dt.get_average() <= 24

  # These are declared transport/unit-authority inputs, not a modification of
  # the producer's actual permission. The original offer/witness stay exact.
  packets = []
  for stamp, current_source, pending in calls:
    message = messaging.new_message(TOPIC, valid=True, logMonoTime=stamp)
    typed_source = replace(current_source, cp_sha256=seed.binding.cp_sha256, source_epoch=seed.binding.source_epoch,
      reset=seed.binding.reset_generation, calibration_generation=seed.binding.calibration_generation,
      persistent_source_epoch=seed.binding.persistent_source_epoch)
    write_tracking_source(message, typed_source)
    if pending is not None and current_source.enabled and pending.observed_ns <= stamp <= pending.expires_ns:
      write_trial(message, pending)
    packets.append((stamp, message.to_bytes(), pending))
  owner = TrackingTrialOwner(seed.binding)
  received = messaging.SubMaster(['selfdriveState', TOPIC], poll='selfdriveState')
  index = 0
  outcomes = []
  for stamp in range(BASE+500_000_000, BASE+2_900_000_001, 10_000_000):
    while index < len(packets) and packets[index][0]+2_000_000 <= stamp:
      sent, raw, pending = packets[index]
      event = messaging.log_from_bytes(raw)
      received.update_msgs((sent+2_000_000)/1e9, [event])  # Declared2ms unit transport.
      if event.starpilotSteeringTracking.trial.version == 1:
        decoded = read_trial(event.starpilotSteeringTracking)
        assert decoded is not None and pending is not None
        assert (decoded.observed_ns, decoded.expires_ns) == (pending.observed_ns, pending.expires_ns)
        assert decoded.response_witness.value == pending.response_witness.value
      index += 1
    pending = read_trial(received[TOPIC])
    try:
      if pending is None:
        owner.neutral('unit_tracking_proposal_unavailable')
        accepted = False
        reason = 'no_proposal'
      else:
        typed_source = read_tracking_source(received[TOPIC], now_ns=stamp, cp_sha256=seed.binding.cp_sha256)
        permission = bool(received.all_checks([TOPIC]) and typed_source is not None and typed_source.enabled and not typed_source.manual)
        accepted = owner.prepare(pending, now_ns=stamp, permission=permission, binding=seed.binding)
        reason = ''
    except ValueError as error:
      owner.neutral(str(error))
      accepted, reason = False, str(error)
    outcomes.append((stamp, accepted, reason, tuple(owner.applied_full.items())))
  if withdraw:
    assert 'learning_disabled' in interruptions
    assert next(stamp for stamp, message in sends if not message.starpilotSteeringTracking.sourceReceipt.enabled) == BASE+1_500_000_000
  return outcomes, seed, sends


@pytest.mark.parametrize('phase', (10_000_000, 20_000_000))
def test_immediate_qualified_wire_retains_freshness_at_every_hundred_hz_owner_tick(monkeypatch, phase):
  rows, seed, sends = delivery_run(monkeypatch, steerlearnerd, phase)
  assert all(accepted for _, accepted, _, _ in rows)
  assert rows[-1][3] == seed.target
  assert len(sends) == 61


def test_original_witness_expiry_withdraws_and_neutral_source_health_recovers(monkeypatch):
  rows, _seed, _sends = delivery_run(monkeypatch, steerlearnerd, 20_000_000, stop=True)
  assert all(accepted for stamp, accepted, _, _ in rows if stamp <= BASE+1_030_000_000)
  assert not any(accepted for stamp, accepted, _, _ in rows if stamp >= BASE+1_040_000_000)
  assert any('witness stale' in reason for _, _, reason, _ in rows)


def test_source_withdrawal_remains_prompt_on_original_pose_boundary(monkeypatch):
  rows, _seed, _sends = delivery_run(monkeypatch, steerlearnerd, 20_000_000, withdraw=True)
  assert not any(accepted for stamp, accepted, _, _ in rows if stamp >= BASE+1_520_000_000)
