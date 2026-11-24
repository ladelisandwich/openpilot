"""Real SubMaster/cereal rate and health contracts; no frequency flag injection.

update_msgs supplies deterministic transport receipt times to the production
SubMaster implementation. These tests do not claim physical source evidence.
"""

import pytest

from openpilot.cereal import messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.prefix import OpenpilotPrefix


LEGACY = 'autoSteeringLearning'
LIVE = 'starpilotSteeringTracking'
BASE = 700_000_000_000
DT = 10_000_000


def packet(service, now, *, valid=True):
  event = messaging.new_message(service, valid=valid, logMonoTime=now)
  return messaging.log_from_bytes(event.to_bytes())


def deliver(sm, tick, *, legacy=True, live=True, live_period=5, legacy_period=25, live_valid=True):
  now = BASE + tick * DT
  events = [packet('carState', now)]
  if legacy and tick % legacy_period == 0:
    events.append(packet(LEGACY, now))
  if live and tick % live_period == 0:
    events.append(packet(LIVE, now, valid=live_valid))
  sm.update_msgs(now / 1e9, events)
  return now


def test_separate_topics_keep_declared4_and20hz_and_ordinary_health(monkeypatch):
  monkeypatch.setenv('SIMULATION', '0')
  assert SERVICE_LIST[LEGACY].frequency == 4.0
  assert SERVICE_LIST[LIVE].frequency == 20.0
  with OpenpilotPrefix():
    sm = messaging.SubMaster(['carState', LEGACY, LIVE], frequency=100)
    assert not sm.simulation
    for tick in range(201):
      deliver(sm, tick)
      if tick >= 25:
        assert sm.all_checks(), (tick, sm.alive, sm.valid, sm.freq_ok)
    # The optional publisher can stop without changing the legacy publisher's
    # rate, validity or health. The live raw health fails at its real threshold.
    last_live = sm.recv_time[LIVE]
    for tick in range(201, 261):
      deliver(sm, tick, live=False)
    assert sm.recv_time[LIVE] == last_live
    assert not sm.alive[LIVE]
    assert not sm.all_checks()
    assert sm.all_checks(['carState', LEGACY])
    for tick in range(261, 301):
      deliver(sm, tick)
    assert sm.all_checks()


@pytest.mark.parametrize('wrong_service', [LEGACY, LIVE])
def test_standard_submaster_rejects_swapped_rates_without_forged_freq_flags(monkeypatch, wrong_service):
  monkeypatch.setenv('SIMULATION', '0')
  with OpenpilotPrefix():
    sm = messaging.SubMaster(['carState', LEGACY, LIVE], frequency=100)
    for tick in range(301):
      deliver(sm, tick, legacy_period=5 if wrong_service == LEGACY else 25,
              live_period=25 if wrong_service == LIVE else 5)
    assert sm.seen[wrong_service] and sm.valid[wrong_service] and sm.alive[wrong_service]
    assert not sm.freq_ok[wrong_service]
    assert not sm.all_freq_ok() and not sm.all_checks()
    other = LIVE if wrong_service == LEGACY else LEGACY
    assert sm.freq_ok[other]


def test_optional_live_failure_does_not_create_ordinary_comm_issue(monkeypatch):
  monkeypatch.setenv('SIMULATION', '0')
  with OpenpilotPrefix():
    sm = messaging.SubMaster(['carState', LEGACY, LIVE], frequency=100,
                             ignore_alive=[LIVE], ignore_valid=[LIVE], ignore_avg_freq=[LIVE])
    for tick in range(101):
      deliver(sm, tick)
    assert sm.freq_ok[LEGACY] and sm.freq_ok[LIVE]
    deliver(sm, 101, live_period=1, live_valid=False)
    assert not sm.valid[LIVE]
    assert sm.all_checks()
    for tick in range(102, 162):
      deliver(sm, tick, live=False)
    assert not sm.alive[LIVE]
    assert sm.all_checks()
    # A genuine ordinary input failure must still reach ordinary aggregation.
    now = BASE + 162 * DT
    sm.update_msgs(now / 1e9, [packet('carState', now, valid=False)])
    assert not sm.all_valid() and not sm.all_checks()
