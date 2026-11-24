"""An assigned button alone is not an accepted Traffic Mode request."""
from types import SimpleNamespace as NS
import pytest
from openpilot.starpilot.conditional_mode.traffic_status import TrafficDisplayProjector

NOW=10_000_000_000


def packet(stamp,sequence,reason,accepted=False,effective=False):
  return NS(trafficMode=NS(version=1,sessionId='a'*32,sequence=sequence,
    observedMonoTime=stamp,validUntilMonoTime=stamp+100_000_000,driveStartMonoTime=NOW-1_000_000_000,
    sourceEpoch=1,sourceBootTime=0 if 'unavailable' in reason else stamp+2_000_000_000-5_000_000,
    settingsFingerprint='b'*64,buttonMapFingerprint='c'*64,controllerSource=False,
    accepted=accepted,effective=effective,reason=reason,profileTargetReady=True,profileReason='qualified'))


def project(reader,wire,stamp):
  return reader.project(wire,now_mono_ns=stamp,now_boot_ns=stamp+2_000_000_000,
    drive_id=NOW-1_000_000_000,settings_fingerprint='b'*64,map_fingerprint='c'*64,
    map_assigned=True,profile_valid=True,long_active=True,selfdrive_enabled=True,car_valid=True,system_long=True)


@pytest.mark.parametrize('reason',['media_unavailable','can_unavailable','settings_unavailable','map_unavailable','clock_unavailable'])
@pytest.mark.parametrize('phase',['never_requested','was_active','off_after_active'])
def test_unavailable_status_requires_an_unfinished_accepted_request(reason,phase):
  reader=TrafficDisplayProjector()
  if phase!='never_requested':
    assert project(reader,packet(NOW,1,'active',True,True),NOW).state=='active'
  if phase=='off_after_active':
    assert project(reader,packet(NOW+10_000_000,2,'off'),NOW+10_000_000).state=='off'
  shown=project(reader,packet(NOW+20_000_000,3,reason),NOW+20_000_000)
  if phase=='was_active':
    assert shown is not None and shown.state=='unavailable_source'
  else:
    assert shown is None
