"""Actual daemon scheduling only; synthetic collector is not fit/native proof."""
from openpilot.starpilot.flm.tests._drain_fixture import separate_drain_fixture
from types import SimpleNamespace as S
import pytest
from openpilot.starpilot.flm import steerlearnerd
from openpilot.starpilot.flm.tests.test_live_tracking_daemon import BASE, source


@pytest.mark.parametrize('fresh_allowed', [True, False])
def test_pose_observer_consumes_delivery_that_arrived_during_decoding(monkeypatch, fresh_allowed):
  now = [BASE]
  class Stop(Exception):
    pass
  class SM:
    updated = {'deviceMotion': False}
    logMonoTime = {'deviceMotion': BASE}
    marker = 'old'
    pending = False
    def update(self, timeout=None):
      if timeout == 0 and self.pending:
        self.marker = 'new'
        self.updated['deviceMotion'] = True
        self.logMonoTime['deviceMotion'] = now[0]
        self.pending = False
  sm = SM()
  evidence = S(source_epoch='b'*64, source_receipt_ns=BASE, control_ns=BASE,
    identity='a'*64, generation=1, domain='normalized_torque', stage_revision='stage',
    response_revision='response', controller_mode='torque', policy='policy',
    baseline_revision='d'*64, controller_revision='controller', turn_assist=False)
  adapter = S(event_collection=True, source_plant_protocol='fixture',
    bind_source=lambda value: None, select_objective=lambda key: None, prepare_control=lambda value: value)
  collector = S(adapter=adapter, ready=[(evidence, object())], control=lambda *args: None)
  monkeypatch.setattr('openpilot.starpilot.flm.event_collector.EventFullrateCollector', lambda value: collector)
  monkeypatch.setattr('openpilot.starpilot.flm.source_plant_runtime.start_original_capture', lambda value: True)
  monkeypatch.setattr('openpilot.starpilot.flm.native_health.current_native_health',
    lambda *args: sm.marker == 'new' and fresh_allowed)
  monkeypatch.setattr(steerlearnerd, 'read_control', lambda event: evidence)
  seen = []
  def observe(_sm, cp, current, stamp, native):
    seen.append((_sm.marker, stamp, native, _sm.logMonoTime['deviceMotion']))
    raise Stop
  monkeypatch.setattr('openpilot.starpilot.flm.source_plant_runtime.observe', observe)
  control_socket, applied_socket = object(), object()
  def drain(sock, callback, *args):
    if sock is control_socket:
      callback(S(which=lambda:'starpilotLateralState', starpilotLateralState=S(learningEvidence=S(version=1))), None, BASE)
    else:
      now[0] += 80_000_000
      sm.pending = True
  monkeypatch.setattr(steerlearnerd, 'drain_pair', separate_drain_fixture(drain))
  actor=S(ingress=None, interrupt=lambda *a,**k: None, close=lambda *a: True)
  fitter=S(cancel=lambda *a: None, close=lambda *a: True)
  with pytest.raises(Stop):
    steerlearnerd.run(S(),sm,control_socket,applied_socket,S(),lambda *a:adapter,None,
      lambda stamp:source(stamp),lambda value:('objective',),lambda value:'objective',
      None,None,fitter,None,clock=lambda:now[0],live_actor=actor)
  assert seen == [('new', BASE+80_000_000, fresh_allowed, BASE+80_000_000)]
  assert sm.logMonoTime['deviceMotion'] == BASE+80_000_000
