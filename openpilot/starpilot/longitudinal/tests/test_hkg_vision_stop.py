"""Pinned Dom stop-buffer comparison and actual native planner publication."""
import hashlib
import itertools
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.longitudinal.hkg_vision_stop import VisionStopBuffer, owner_for
from openpilot.starpilot.longitudinal.tests.test_lead_approach_planner import Frame, DRIVE_NS

DATA = json.loads((Path(__file__).parent / 'testdata/hkg_dom_stop_buffer.json').read_text())


def lead(speed=.2, distance=2., probability=.999, radar=False):
  return NS(status=True, present=True, radar=radar, modelProb=probability, vLead=speed, dRel=distance)


def dom_reference(clock):
  source = DATA['sourceFunction']
  assert hashlib.sha256(source.encode()).hexdigest() == DATA['sourceSha256']
  namespace = dict(DATA['constants'], time=NS(monotonic=lambda: clock[0] / 1e9),
                   get_vision_low_speed_stop_buffer_lead_speed_limits=lambda cp, a, b: (a, b))
  exec(compile(source, '<pinned Dom stop buffer>', 'exec'), namespace)
  return namespace['get_vision_low_speed_stop_buffer_cap']


def test_exact_source_boundaries_without_a_new_calibration():
  clock = [10_000_000_000]
  reference = dom_reference(clock)
  for speed, lead_speed, distance, probability, radar in itertools.product(
      (0., .349, .35, .8, 3., 6.5, 6.50001), (0., .2, 3.25, 3.25001, 4.),
      (1., 3.2, 5., 8., 10.), (.89999, .9, .9999), (False, True)):
    original = NS(CP=NS(), vision_low_speed_stop_hold_until=0.)
    current = VisionStopBuffer()
    obj = lead(lead_speed, distance, probability, radar)
    assert current.cap(obj, speed, -3.5, clock[0]) == reference(original, obj, speed, -3.5)


def test_recorded_niro_frames_keep_dom_buffer_and_original_braking_bounds():
  clock = [0]
  reference = dom_reference(clock)
  original = NS(CP=NS(), vision_low_speed_stop_hold_until=0.)
  current = VisionStopBuffer()
  changed = 0
  for frame in DATA['frames']:
    clock[0] = frame['ns']
    for row in frame['leads']:
      obj = lead(row['speed'], row['distance'], row['prob'])
      cap, stopping = current.cap(obj, frame['speed'], -3.5, clock[0])
      expected, expected_stop = reference(original, obj, frame['speed'], -3.5)
      assert stopping == expected_stop
      assert cap == expected
      if cap is not None:
        assert -3.5 <= min(frame['recordedTarget'], cap) <= frame['recordedTarget']
        changed += cap < frame['recordedTarget']
  assert changed > 0


def params(identity=CAR.KIA_NIRO_EV, alpha=True):
  return CarInterface.get_params(identity, gen_empty_fingerprint(), [], alpha, False, False)


def frame(index=0):
  sm = Frame(index)
  sm['carState'].vEgo = .8
  sm['carState'].aEgo = -.08
  sm['carState'].standstill = False
  sm['selfdriveState'].experimentalMode = True
  sm['modelV2'].action.shouldStop = False
  sm['modelV2'].action.desiredAcceleration = 0.
  sm['radarState'].leadTwo.present = False
  obj = sm['radarState'].leadOne
  obj.present, obj.radar, obj.modelProb = True, False, .99994
  obj.dRel, obj.vLead, obj.vLeadK, obj.vRel, obj.aLeadK = 2., .2, .2, -.6, 0.
  return sm


def test_native_planner_publishes_original_stop_buffer_with_unchanged_limits():
  cp = params()
  before = cp.to_dict()
  original, current = (LongitudinalPlanner(cp, init_v=.8) for _ in range(2))
  original.vision_stop_buffer = None
  changed = 0
  for index in range(10):
    sm = frame(index)
    original.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
    current.update(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS)
    assert original.mpc.solution_status == current.mpc.solution_status == 0
    assert -3.5 <= current.output_a_target <= original.output_a_target
    assert current.output_should_stop
    changed += current.output_a_target < original.output_a_target or not original.output_should_stop
  assert changed > 0 and cp.to_dict() == before
  class Published:
    messages = {}
    def send(self, service, message):
      self.messages[service] = message.as_reader()
  published = Published()
  sm.all_checks = lambda services: True
  create_message = messaging.new_message
  with patch.object(messaging, 'new_message', side_effect=lambda service: create_message(service, logMonoTime=sm.now_ns)):
    current.publish(sm, published)
  plan = published.messages['longitudinalPlan']
  assert plan.valid and plan.longitudinalPlan.shouldStop
  assert plan.longitudinalPlan.aTarget == pytest.approx(current.output_a_target)


@pytest.mark.parametrize('defect', ['gas', 'brake', 'stale', 'future', 'invalid', 'long_off', 'disabled', 'force', 'inactive'])
def test_invalid_or_inactive_sources_do_not_create_a_braking_override(defect):
  owner = VisionStopBuffer()
  sm = frame()
  if defect in ('gas', 'brake'):
    setattr(sm['carState'], defect + 'Pressed', True)
  elif defect == 'stale':
    sm.logMonoTime['radarState'] = sm.now_ns - 250_000_001
  elif defect == 'future':
    sm.logMonoTime['modelV2'] = sm.now_ns + 1
  elif defect == 'invalid':
    sm.valid['radarState'] = False
  elif defect == 'long_off':
    sm['carControl'].longActive = False
  elif defect == 'disabled':
    sm['selfdriveState'].enabled = False
  elif defect == 'force':
    sm['controlsState'].forceDecel = True
  assert owner.sample(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS, target=-.3, stopping=False,
                      accel_min=-3.5, active=defect != 'inactive') == (-.3, False)
  assert owner.hold_until_ns == 0


@pytest.mark.parametrize('defect', ['radar', 'low_prob', 'fast_lead', 'high_speed'])
def test_ordinary_moving_or_radar_following_keeps_its_existing_target(defect):
  owner = VisionStopBuffer()
  sm = frame()
  obj = sm['radarState'].leadOne
  if defect == 'radar':
    obj.radar = True
  elif defect == 'low_prob':
    obj.modelProb = .89
  elif defect == 'fast_lead':
    obj.vLead = 4.1
  else:
    sm['carState'].vEgo = 6.50001
  assert owner.sample(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS, target=-.3, stopping=False,
                      accel_min=-3.5, active=True) == (-.3, False)


def test_stronger_braking_and_existing_stop_intent_are_never_relaxed():
  sm = frame()
  owner = VisionStopBuffer()
  assert owner.sample(sm, now_ns=sm.now_ns, drive_id=DRIVE_NS, target=-3.5, stopping=True,
                      accel_min=-3.5, active=True) == (-3.5, True)
  assert owner.sample(frame(1), now_ns=sm.now_ns, drive_id=DRIVE_NS, target=-3.5, stopping=True,
                      accel_min=-3.5, active=False) == (-3.5, True)


def test_stock_long_unknown_profiles_and_foreign_brands_do_not_gain_an_owner():
  assert owner_for(params(alpha=False)) is None
  cp = params()
  assert isinstance(owner_for(cp), VisionStopBuffer)
  for field, value in (('passive', True), ('dashcamOnly', True), ('notCar', True), ('brand', 'toyota'),
                       ('carFingerprint', 'UNKNOWN_HYUNDAI')):
    changed = cp.as_reader().as_builder()
    setattr(changed, field, value)
    assert owner_for(changed) is None
