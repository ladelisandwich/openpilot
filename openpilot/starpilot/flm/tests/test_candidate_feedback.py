"""Feedback chronology units plus actual registered controller/native paths.

Mocked original-tape seals in caller units grant NO original/current admission.
Fresh actual owner capture remains required for complete qualification.
"""
from types import SimpleNamespace
import pickle

import pytest

from openpilot.starpilot.flm.candidate_feedback import CandidateFeedbackLoop,CLOSED_POLICY,LEGACY_POLICY,policy
from openpilot.starpilot.flm.feedback_observation import f32
from openpilot.starpilot.flm.torque_emission_replay import EmissionHistory


def arithmetic_fixture(monkeypatch, *, context=False, two_windows=False):
  from openpilot.starpilot.flm import feedback_replay
  stamps = (100,110,120,130) if not two_windows else (100,110,200,210)
  rows = []
  for index,stamp in enumerate(stamps):
    previous = stamps[index-1] if index else 0
    row = SimpleNamespace(control_ns=stamp,prior_publish_ns=previous,publication_ns=stamp-1,
      recv_frame=index+1,receipt_ns=stamp,co_body_sha256='1'*64,
      reported_torque=f32(.18199999630451202),requested_torque=f32(.17206627130508423),
      lat_active=True,input_limited=False,published_limited=False)
    pub = SimpleNamespace(output_control_ns=stamp-10,output_apply_ns=stamp-9,
      publication_ns=stamp-1,output_carrier_sequence=index+1,native_identity='2'*64,
      applied_sha256='3'*64,reported_torque=row.reported_torque)
    rows.append(SimpleNamespace(consumption=row,publication=pub))
  context_rows = (rows[0].consumption,) if context else ()
  if two_windows:
    windows = (((100,110),SimpleNamespace(rows=tuple(rows[:2]),digest='4'*64),100),
               ((200,210),SimpleNamespace(rows=tuple(rows[2:]),digest='5'*64),200))
  else:
    windows = (((100,130),SimpleNamespace(rows=tuple(rows[1:] if context else rows),digest='4'*64),110 if context else 100),)
  evidence = SimpleNamespace(candidate_policy=CLOSED_POLICY,engine_input_digest='a'*64,
    original=SimpleNamespace(context_only=context_rows,digest='b'*64),windows=windows)
  engine = SimpleNamespace(emissions=EmissionHistory.__new__(EmissionHistory),verified_feedback_tape=evidence)
  monkeypatch.setattr(feedback_replay,'require_feedback_engine',lambda owner:evidence)
  frames = tuple(SimpleNamespace(control_ns=stamp,inputs=(('active',1.),('safety_limited',0.),('untouched',-0.))) for stamp in stamps)
  return engine,rows,frames


@pytest.mark.parametrize('candidate_request,reported',[(.17249134182929993,.18266665935516357),(.17164121568202972,.18199999630451202)])
def test_demonstrated_threshold_crossing_feeds_next_update_not_current(monkeypatch,candidate_request,reported):
  engine,rows,frames = arithmetic_fixture(monkeypatch)
  reports = {}
  loop = CandidateFeedbackLoop(engine,reports,(.01,),native_policy='strict')
  requests = {}
  previous = False
  observed_inputs = []
  for _index,(joined,frame) in enumerate(zip(rows,frames,strict=True)):
    reports[joined.publication.output_apply_ns] = f32(reported)
    before = loop.before(frame)
    assert dict(before)['safety_limited'] == float(previous)
    observed_inputs.append(previous)
    assert pickle.dumps(tuple(pair for pair in before if pair[0]!='safety_limited')) == pickle.dumps(
      tuple(pair for pair in frame.inputs if pair[0]!='safety_limited'))
    requests[frame.control_ns] = f32(candidate_request)
    loop.after(frame.control_ns,candidate_request)
    consumed = joined.publication.reported_torque if joined.publication.output_control_ns < 100 else f32(reported)
    previous = abs(f32(candidate_request)-consumed) > .01
  assert any(observed_inputs)
  proof = loop.finish(requests,reports)
  assert proof.row_count == 4 and proof.native_policy == 'strict'
  assert proof.original_digest == engine.verified_feedback_tape.original.digest


def test_missing_consumed_native_report_never_uses_nearest_or_physical_value(monkeypatch):
  engine,rows,frames = arithmetic_fixture(monkeypatch)
  reports = {}
  loop = CandidateFeedbackLoop(engine,reports,(.01,),native_policy='strict')
  loop.before(frames[0])
  loop.after(frames[0].control_ns,.17249134182929993)
  reports[rows[2].publication.output_apply_ns] = 0.  # Wrong apply coordinate is not a fallback.
  loop.before(frames[1])
  with pytest.raises(ValueError,match='not yet produced'):
    loop.after(frames[1].control_ns,.17249134182929993)


def test_complete_finish_rejects_changed_native_result_or_omitted_frame(monkeypatch):
  engine,rows,frames = arithmetic_fixture(monkeypatch)
  reports,requests = {},{}
  loop = CandidateFeedbackLoop(engine,reports,(.01,),native_policy='native-shadow-identification')
  for joined,frame in zip(rows,frames,strict=True):
    reports[joined.publication.output_apply_ns] = f32(.18266665935516357)
    loop.before(frame)
    requests[frame.control_ns]=f32(.17249134182929993)
    loop.after(frame.control_ns,requests[frame.control_ns])
  reports[rows[-1].publication.output_apply_ns] = f32(.17)
  with pytest.raises(ValueError,match='changed'):
    loop.finish(requests,reports)
  fresh = CandidateFeedbackLoop(engine,{},(.01,),native_policy='strict')
  with pytest.raises(ValueError,match='omitted'):
    fresh.finish({},fresh.reported)


def test_context_prefix_and_window_checkpoints_use_real_original_prior(monkeypatch):
  engine,rows,frames = arithmetic_fixture(monkeypatch,context=True)
  reports,requests = {},{}
  loop = CandidateFeedbackLoop(engine,reports,(.01,),native_policy='strict')
  for index,(joined,frame) in enumerate(zip(rows,frames,strict=True)):
    reports[joined.publication.output_apply_ns] = f32(.18266665935516357)
    inputs=loop.before(frame)
    request=joined.consumption.requested_torque if index==0 else f32(.17249134182929993)
    requests[frame.control_ns]=request
    loop.after(frame.control_ns,request)
    assert dict(inputs)['safety_limited'] == (0. if index<3 else 1.)
  assert loop.finish(requests,reports).row_count == 4
  engine,rows,frames = arithmetic_fixture(monkeypatch,two_windows=True)
  reports,requests = {},{}
  loop = CandidateFeedbackLoop(engine,reports,(.01,),native_policy='strict')
  for index,(joined,frame) in enumerate(zip(rows,frames,strict=True)):
    reports[joined.publication.output_apply_ns] = f32(.18266665935516357)
    assert dict(loop.before(frame))['safety_limited'] == (0. if index in (0,2) else 1.)
    requests[frame.control_ns]=f32(.17164121568202972)
    loop.after(frame.control_ns,requests[frame.control_ns])
  assert loop.finish(requests,reports).row_count == 4


def test_original_baseline_and_pre_activation_remain_exact(monkeypatch):
  engine,rows,frames = arithmetic_fixture(monkeypatch,context=True)
  reports={}
  loop=CandidateFeedbackLoop(engine,reports,(0.,),native_policy='strict',original_baseline=True)
  loop.before(frames[0])
  with pytest.raises(ValueError,match='baseline or preactivation'):
    loop.after(frames[0].control_ns,.17249134182929993)
  for unknown in (None,'made-up',1):
    with pytest.raises(ValueError,match='policy'):
      policy(unknown)
  assert policy(LEGACY_POLICY)==LEGACY_POLICY


def actual_caller_fixture(domain):
  """Actual controller/CAN numeric unit; original source seal remains mocked."""
  from openpilot.starpilot.flm.controller_replay import ControllerReplay,ReplayFrame,SelectedControllerFactory,_run,capture_snapshot,final_command,frame_input
  from openpilot.starpilot.flm.pid_controller_replay import PidControllerFactory
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  from openpilot.starpilot.flm.event_activation import EventActivationPlan,ZERO_REFERENCE_ACTIVATION_VERSION
  from openpilot.starpilot.flm.event_objective import EffectObjective
  from openpilot.starpilot.flm.torque_adapter import Objective
  from openpilot.starpilot.flm.torque_emission_replay import Emission
  from openpilot.starpilot.flm.tests.test_full_surface_profile import base
  from openpilot.starpilot.flm.effect_trace import FullEffectTrace
  from openpilot.starpilot.flm.selected_effects import EffectReplayRecordedSource
  from openpilot.starpilot.flm.tests.test_event_lazy_native import TOYOTA,HYUNDAI,HONDA,make_interface,install_observer,settle,control,apply,BASE_NS
  platform={'torque':TOYOTA.TOYOTA_COROLLA_TSS2,'starpilot':HYUNDAI.HYUNDAI_IONIQ_6,'pid':HONDA.HONDA_CIVIC}[domain]
  ci=make_interface(platform)
  assert install_observer(ci)
  settled=settle(ci,control(0.))
  factory=PidControllerFactory(ci.CP,'native') if domain=='pid' else SelectedControllerFactory(ci.CP,'starpilot' if domain=='starpilot' else 'standard')
  objective=Objective('feedforward','high','steady')
  original=ControllerReplay(factory,objective,identity='f'*64,baseline_revision='a'*64)
  actual,vm=factory.create()
  original.raw_mode=True
  first=BASE_NS+ci.CC.frame*10_000_000
  original.checkpoint(first,capture_snapshot(actual,vm))
  previous_flag=False
  previous_report=f32(settled[-1][0].torque)
  prior_stamp=BASE_NS+settled[-1][2].frame*10_000_000
  previous_event=SimpleNamespace(control_ns=prior_stamp,apply_ns=prior_stamp+1_000_000)
  frames,joins,contexts=[],[],[]
  envelope=base()
  for tick in range(60):
    stamp=first+tick*10_000_000
    inputs=frame_input(ci.CS.out,vm,SimpleNamespace(angleOffsetDeg=0.,roll=0.),active=True,
      safety_limited=previous_flag,desired_curvature=.00003,curvature_limited=False,lat_delay=.2)
    inputs+=(('control_frame',float(tick+1)),('raw_model_curvature',.00003))
    tuning=() if domain=='pid' else (('effective_factor',float(actual.torque_params.latAccelFactor)),
      ('offset',float(actual.torque_params.latAccelOffset)),('friction',float(actual.torque_params.friction)),
      ('pid_pos_limit',float(actual.pid.pos_limit)),('pid_neg_limit',float(actual.pid.neg_limit)))
    trace=FullEffectTrace(1,envelope,stamp,stamp-10_000_000,True,
      envelope.coordinates,envelope.coordinates,envelope.coordinates,envelope.full_profile_sha256).validate()
    value,_=_run(actual,vm,inputs,objective,(0.,0.),tuning,actual_stage_residual=(0.,0.),
      effect_trial=EffectReplayRecordedSource(trace))
    request=f32(final_command(value))
    flag=bool(abs(request-previous_report)>.01)
    row=SimpleNamespace(control_ns=stamp,prior_publish_ns=stamp-10_000_000 if tick else 0,
      publication_ns=stamp-1,recv_frame=tick+1,receipt_ns=stamp,co_body_sha256='1'*64,
      reported_torque=previous_report,requested_torque=request,lat_active=True,
      input_limited=previous_flag,published_limited=flag)
    if previous_event is None:
      contexts.append(row)
    else:
      pub=SimpleNamespace(output_control_ns=previous_event.control_ns,output_apply_ns=previous_event.apply_ns,
        publication_ns=stamp-1,output_carrier_sequence=tick,native_identity='2'*64,applied_sha256='3'*64,
        reported_torque=previous_report)
      joins.append(SimpleNamespace(consumption=row,publication=pub))
    frame=ReplayFrame(stamp,inputs,request,(0.,0.),frame_sequence=tick+1,actual_lac_command=value,
      limited=flag,torque_values=tuning,actual_stage_residual=(0.,0.),
      effect_trace=trace,effect_prepare_ns=trace.control_ns,
      controller_domain='torque_pid' if domain=='pid' else 'torque')
    frames.append(frame)
    output,packets,witness=apply(ci,control(value))
    if witness is not None:
      assert (witness.can_address,witness.can_data,witness.can_bus) in packets
      clipped=witness.unlimited_raw is None or witness.unlimited_raw!=witness.final_raw
      event=Emission(stamp,stamp,stamp+1_000_000,witness,'f'*64,'e'*64,'a'*64,
        True,True,False,False,clipped,False,False,0,0,'')
      original.emissions.events[event.apply_ns]=event
      original.emissions.decode_queue.append(event.apply_ns)
      original.emissions.by_control[event.control_ns]=event
      previous_event=event
    previous_report=f32(output.torque)
    previous_flag=flag
  assert not contexts
  original.records.extend(frames)
  bounds=(frames[0].control_ns,frames[-1].control_ns)
  original.replay_windows=(bounds,)
  selected=EffectObjective('ff.left',envelope.trial_names,envelope.trial_anchor,envelope.context_key)
  activation=frames[30].control_ns
  prepare=frames[30].effect_prepare_ns
  assert prepare==frames[30].effect_trace.control_ns==activation
  plan=EventActivationPlan(ZERO_REFERENCE_ACTIVATION_VERSION,original.identity,original.baseline_revision,
    selected.context_key,'b'*64,tuple('train-'+str(i) for i in range(4)),tuple('held-'+str(i) for i in range(4)),
    activation-10_000_000,tuple('stage-'+str(i) for i in range(9)),((bounds[0],bounds[1],activation,prepare),)).validate()
  engine=EventControllerReplay(original,selected,envelope.replay_anchor(),activation_plan=plan)
  evidence=SimpleNamespace(candidate_policy=CLOSED_POLICY,engine_input_digest='a'*64,
    binding='c'*64,original=SimpleNamespace(context_only=tuple(contexts),digest='b'*64),
    windows=((bounds,SimpleNamespace(rows=tuple(joins),digest='d'*64),activation),))
  engine.verified_feedback_tape=evidence
  return engine,evidence,tuple(joins),tuple(contexts)


@pytest.mark.parametrize('domain',['torque','starpilot','pid'])
@pytest.mark.parametrize('coordinates',[(0.,),(.00001,)])
@pytest.mark.parametrize('shadow',[False,True])
def test_actual_selected_controller_native_interleave_and_next_input(domain,coordinates,shadow,monkeypatch):
  from openpilot.starpilot.flm import feedback_replay,event_controller_replay
  engine,evidence,joins,contexts=actual_caller_fixture(domain)
  monkeypatch.setattr(feedback_replay,'require_feedback_engine',lambda owner:evidence)
  updates=[]
  native_sink=[]
  original_run=event_controller_replay._run
  original_native=engine.emissions.rollout
  def update(*args,**kwargs):
    result=original_run(*args,**kwargs)
    updates.append((args[2],f32(result[0])))
    return result
  def native(*args,**kwargs):
    native_sink.append(kwargs['feedback_reported'])
    return original_native(*args,**kwargs)
  monkeypatch.setattr(event_controller_replay,'_run',update)
  monkeypatch.setattr(engine.emissions,'rollout',native)
  kwargs={'_identification_token':engine._EventControllerReplay__identification_token} if shadow else {}
  requests,encoded,_=engine._rollout_uncached(coordinates,**kwargs)
  assert len(updates)==len(engine.records)==60
  reports=native_sink[0]
  associations={joined.consumption.control_ns:joined for joined in joins}
  context={row.control_ns:row for row in contexts}
  activation=evidence.windows[0][2]
  previous=False
  for frame,(inputs,request) in zip(engine.records,updates,strict=True):
    assert dict(inputs)['safety_limited']==float(previous)
    assert pickle.dumps(tuple(pair for pair in inputs if pair[0]!='safety_limited'))==pickle.dumps(
      tuple(pair for pair in frame.inputs if pair[0]!='safety_limited'))
    if frame.control_ns in context:
      row=context[frame.control_ns]
      report=row.reported_torque
    else:
      joined=associations[frame.control_ns]
      row=joined.consumption
      pub=joined.publication
      report=pub.reported_torque if pub.output_control_ns<activation else f32(reports[pub.output_apply_ns])
    previous=bool(row.lat_active and abs(request-report)>.01)
    if frame.control_ns<activation or coordinates==(0.,):
      assert previous is row.published_limited
  assert encoded and set(requests)=={row.control_ns for row in engine.records}
  proof=engine._EventControllerReplay__candidate_feedback_proof
  assert proof.row_count==60 and proof.native_policy==('native-shadow-identification' if shadow else 'strict')


def test_parent_submission_and_trial_restore_summaries_cannot_downgrade_policy():
  from openpilot.starpilot.flm.candidate_feedback import PROOF_VERSION,report_policy_matches
  from openpilot.starpilot.flm.source_plant_pipeline import _feedback_summary
  original=SimpleNamespace(digest='a'*64,rows=(1,),context_only=())
  evidence=SimpleNamespace(candidate_policy=CLOSED_POLICY,binding='b'*64,original=original)
  packet={'feedback_observations':{'digest':b'c'*32}}
  offered=_feedback_summary(evidence,packet,'actual-feedback-source-protocol')
  assert report_policy_matches(('b'*64,(b'c'*32).hex(),'trial-marker',CLOSED_POLICY),offered)
  assert report_policy_matches(('b'*64,(b'c'*32).hex(),'restore-marker',CLOSED_POLICY),offered)
  assert not report_policy_matches(('b'*64,(b'c'*32).hex(),'trial-marker'),offered)
  assert not report_policy_matches(('b'*64,(b'c'*32).hex(),'trial-marker',CLOSED_POLICY),
    {**offered,'candidate_feedback_proof_version':'old'})
  evidence.candidate_policy=LEGACY_POLICY
  old=_feedback_summary(evidence,packet,'actual-feedback-source-protocol')
  assert report_policy_matches(('b'*64,(b'c'*32).hex(),'legacy-marker'),old)
  assert not report_policy_matches(('b'*64,(b'c'*32).hex(),'trial-marker',CLOSED_POLICY),old)
  assert offered['candidate_feedback_proof_version']==PROOF_VERSION


def test_closed_policy_probe_never_calls_unknown_or_unowned_proxy_attributes():
  from openpilot.starpilot.flm.candidate_feedback import uses_closed_loop
  from openpilot.starpilot.flm.event_controller_replay import EventControllerReplay
  from openpilot.starpilot.flm.feedback_replay import FeedbackReplayEvidence,_SEAL
  calls=[]
  class Unknown:
    def __getattribute__(self,name):
      calls.append(name)
      raise AssertionError('unknown attribute must not be touched')
  class Subclass(EventControllerReplay):
    def __getattr__(self,name):
      calls.append(name)
      raise AssertionError('subclass attribute must not be touched')
  assert uses_closed_loop(Unknown()) is False
  assert uses_closed_loop(EventControllerReplay.__new__(EventControllerReplay)) is False
  assert uses_closed_loop(Subclass.__new__(Subclass)) is False
  assert not calls
  shell=EventControllerReplay.__new__(EventControllerReplay)
  shell.verified_feedback_tape=SimpleNamespace(candidate_policy=CLOSED_POLICY)
  assert uses_closed_loop(shell) is False
  shell.verified_feedback_tape=FeedbackReplayEvidence('a'*64,None,(),'b'*64,object(),CLOSED_POLICY)
  assert uses_closed_loop(shell) is False
  shell.verified_feedback_tape=FeedbackReplayEvidence('a'*64,None,(),'b'*64,_SEAL,CLOSED_POLICY)
  assert uses_closed_loop(shell) is True  # Optimization opt-out ONLY, never readiness.
