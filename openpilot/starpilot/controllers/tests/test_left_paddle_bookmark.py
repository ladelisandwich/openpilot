"""Physical Ioniq paddle edges use the existing authenticated wheel Bookmark path."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.starpilot.controllers.tests.test_wheel_actions import DRIVE, NOW, Publisher, candidate, services, state, TYPE
from openpilot.starpilot.controllers.tests.test_native_wheel_callbacks import native_session  # noqa: F401
from openpilot.starpilot.controllers.wheel_actions import WheelConsumer, WheelPublisher
from openpilot.starpilot.tests.test_nostalgia import _ioniq5_params
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
from openpilot.starpilot.ui.wheel_feature import WheelFeature

KEY = 'LeftPaddleButtonControl'


def save(params, value=b'8'):
  # Also permits the unchanged baseline to reproduce a behavioral failure.
  Path(params.get_param_path(KEY)).write_bytes(value)


def tick(owner, params, cp, pressed=None, frame=0):
  cs = state() if pressed is None else state(pressed, button=TYPE.altButton2)
  before = cs.to_bytes()
  commands = owner.observe(params, cp, cs, now_ns=NOW + frame * 10_000_000, drive_id=DRIVE)
  assert cs.to_bytes() == before  # The physical event still reaches longitudinal/AOL owners.
  return commands


@pytest.mark.parametrize('fingerprint', (CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_IONIQ_5_N,
                                         CAR.HYUNDAI_IONIQ_5_PE, CAR.HYUNDAI_IONIQ_6))
@pytest.mark.parametrize('longitudinal', (False, True))
def test_supported_paddle_bookmarks_on_release_once(tmp_path, fingerprint, longitudinal):
  params = Params(str(tmp_path))
  cp = _ioniq5_params(long=longitudinal) if fingerprint != CAR.HYUNDAI_IONIQ_6 else candidate(False)[1]
  cp.carFingerprint = fingerprint
  cp.openpilotLongitudinalControl = longitudinal
  save(params)
  owner = WheelPublisher(params)
  tick(owner, params, cp)
  assert not any(action for _, action in tick(owner, params, cp, True, 1))
  assert not any(action for _, action in tick(owner, params, cp, True, 2))
  assert tick(owner, params, cp, False, 3) == ((KEY, 8),)
  assert not owner.suppress_distance_release
  assert not any(action for _, action in tick(owner, params, cp, False, 4))


@pytest.mark.parametrize('initial', (None, b'0', b'1', b'8garbage'))
def test_default_off_and_invalid_assignment_have_no_action(tmp_path, initial):
  params = Params(str(tmp_path))
  if initial is not None:
    save(params, initial)
  cp = _ioniq5_params()
  owner = WheelPublisher(params)
  tick(owner, params, cp, True, 1)
  assert not any(action for _, action in tick(owner, params, cp, False, 2))


@pytest.mark.parametrize('reason', ('pause', 'invalid_pause', 'alternate', 'other_vehicle', 'invalid_can', 'new_drive', 'mapping'))
def test_pause_and_source_changes_revoke_held_paddle(tmp_path, reason):
  params = Params(str(tmp_path))
  cp = _ioniq5_params()
  save(params)
  owner = WheelPublisher(params)
  tick(owner, params, cp, True, 1)
  if reason in ('pause', 'invalid_pause'):
    Path(params.get_param_path('NostalgiaMode')).write_bytes(b'1' if reason == 'pause' else b'bad')
    tick(owner, params, cp, frame=2)
    params.put_bool('NostalgiaMode', False, block=True)
  elif reason == 'alternate':
    cp.flags = int(cp.flags | HyundaiFlags.CANFD_ALT_BUTTONS)
  elif reason == 'other_vehicle':
    cp.carFingerprint = CAR.KIA_EV6
  elif reason == 'invalid_can':
    invalid = state()
    invalid.canValid = False
    owner.observe(params, cp, invalid, now_ns=NOW + 20_000_000, drive_id=DRIVE)
  elif reason == 'new_drive':
    owner.observe(params, cp, state(), now_ns=NOW + 20_000_000, drive_id=DRIVE + 1)
  else:
    save(params, b'0')
    tick(owner, params, cp, frame=2)
    save(params)
  assert not any(action for _, action in tick(owner, params, cp, False, 3))


def test_pause_on_never_emits_and_consumer_rechecks_priority(tmp_path):
  params = Params(str(tmp_path))
  cp = _ioniq5_params()
  save(params)
  owner = WheelPublisher(params)
  sender = Publisher()
  owner.publish(((KEY, 8),), cp, sender, now_ns=NOW, drive_id=DRIVE, source_car_ns=NOW, source_control_ns=NOW)
  event = messaging.log_from_bytes(sender.events[0][1])
  assert WheelConsumer().accept(event, params, cp, services(cp), now_ns=NOW).action == 8
  params.put_bool('NostalgiaMode', True, block=True)
  assert WheelConsumer().accept(event, params, cp, services(cp), now_ns=NOW) is None
  tick(owner, params, cp, True, 1)
  assert not any(action for _, action in tick(owner, params, cp, False, 2))


def test_actual_can_parser_paddle_to_authenticated_bookmark(tmp_path):
  from opendbc.can import CANPacker
  from opendbc.car import Bus
  from opendbc.car.hyundai.hyundaicanfd import CanBus
  from opendbc.car.hyundai.tests.test_ioniq6_stock_parser import StockParserTests, params as i6_params
  from opendbc.car.hyundai.values import DBC
  params = Params(str(tmp_path))
  save(params)
  for cp in (_ioniq5_params(), i6_params(False)[0]):
    case = StockParserTests(methodName='runTest')
    ci = case.host(cp)
    cs = case.feed(ci, count=5)
    owner = WheelPublisher(params)
    owner.observe(params, cp, cs, now_ns=NOW, drive_id=DRIVE)
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    for index, pressed in enumerate((True, False), 6):
      ci.test_time_ns += 10_000_000
      frame = packer.make_can_msg('CRUISE_BUTTONS', CanBus(cp).ECAN,
                                 {'COUNTER': index, 'LEFT_PADDLE': int(pressed)})
      cs = ci.update([(ci.test_time_ns, [frame])])
      assert cs.canValid
      before = [(e.type, e.pressed) for e in cs.buttonEvents]
      commands = owner.observe(params, cp, cs, now_ns=NOW + index * 10_000_000, drive_id=DRIVE)
      assert [(e.type, e.pressed) for e in cs.buttonEvents] == before
      assert tuple((key, action) for key, action in commands if action) == (() if pressed else ((KEY, 8),))


def test_ui_exposes_off_bookmark_only_when_pause_off_and_rejects_stale_request(tmp_path):
  params = Params(str(tmp_path))
  cp = _ioniq5_params()
  owner = NS(params=params, vehicle_params=lambda: cp, configuration_vehicle=lambda: False,
             configuration_longitudinal=lambda: False, longitudinal_available=lambda: True, authority=lambda _: True)
  feature = WheelFeature(owner)
  row = next(row for row in feature.rows() if row.key == 'wheel:' + KEY)
  assert row.choices == ('Off', 'Bookmark') and row.value == 'Off'
  request = FeatureSettingsRequest(row.key, row.source, 'Bookmark', capability=row.capability)
  params.put_bool('NostalgiaMode', True, block=True)
  assert not feature.apply(request)
  assert feature.choices(KEY) == ()
  params.put_bool('NostalgiaMode', False, block=True)
  assert feature.apply(request)
  assert Path(params.get_param_path(KEY)).read_bytes() == b'8'
  cp.flags = int(cp.flags | HyundaiFlags.CANFD_ALT_BUTTONS)
  assert feature.choices(KEY) == ()


def test_paddle_receipt_reaches_native_bookmark_once(native_session, monkeypatch):  # noqa: F811
  runtime, session, ui, _, calls = native_session
  save(ui.params)
  owner = WheelPublisher(ui.params)
  tick(owner, ui.params, ui.CP, True, 1)
  commands = tick(owner, ui.params, ui.CP, False, 2)
  assert commands == ((KEY, 8),)
  sender = Publisher()
  owner.publish(commands, ui.CP, sender, now_ns=NOW, drive_id=DRIVE, source_car_ns=NOW, source_control_ns=NOW)
  event = messaging.log_from_bytes(sender.events[0][1])
  queue = deque((event, event))
  monkeypatch.setattr(runtime.messaging, 'recv_one_or_none', lambda _: queue.popleft() if queue else None)
  session._poll_wheel(NOW)
  assert calls == ['bookmark']


def test_neutral_and_unsupported_paddles_do_not_read_pause_setting(tmp_path, monkeypatch):
  from openpilot.starpilot.controllers import wheel_actions
  params = Params(str(tmp_path))
  cp = _ioniq5_params()
  save(params)
  owner = WheelPublisher(params)
  tick(owner, params, cp)
  reads = []
  read_saved = wheel_actions.read_saved
  monkeypatch.setattr(wheel_actions, 'read_saved', lambda params, key, size: (reads.append(key), read_saved(params, key, size))[1])
  for frame in range(1, 50):
    tick(owner, params, cp, frame=frame)
  assert reads == []
  cp.flags = int(cp.flags | HyundaiFlags.CANFD_ALT_BUTTONS)
  tick(owner, params, cp, True, 50)
  tick(owner, params, cp, False, 51)
  assert 'NostalgiaMode' not in reads
