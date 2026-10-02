"""Unit tests for the Mazda hybrid longitudinal state machines (opendbc/car/mazda/hybrid.py)."""
from opendbc.car import uds
from opendbc.car.mazda.hybrid import (CRZ_CTRL_ADDR, CRZ_INFO_ADDR, HEARTBEAT_ADDRS, RADAR_BUS, RADAR_COUNTER_ADDR, RADAR_UDS_ADDR,
                                      HybridArbiter, HybridRadarManager, MrccResync, RadarMaster, RadarWitness,
                                      ce_status_is_experimental, radar_session_msg)
from opendbc.car.mazda.longitudinal import accel_cmd_to_accel, accel_to_accel_cmd, build_crz_ctrl, build_crz_info

STOCK = 0      # src of the stock radar's frames
ECHO = 128     # src of our own transmitted frames coming back from the panda


def crz_info(accel=0.5, ctr=0, v=20.0, standby=False) -> bytes:
  if standby:
    return build_crz_info(0.0, ctr, False, False, v)
  return build_crz_info(accel, ctr, True, False, v)


def crz_ctrl(active: bool) -> bytes:
  return build_crz_ctrl(active, False, False, False)


def packet(*frames):
  return [(0, list(frames))]


class TestCeStatus:
  def test_standard_statuses(self):
    assert not ce_status_is_experimental(0)   # OFF
    assert not ce_status_is_experimental(1)   # USER_DISABLED
    for status in range(2, 9):
      assert ce_status_is_experimental(status)


class TestUdsFrames:
  def test_session_requests_match_panda_allowlist(self):
    for session in (uds.SESSION_TYPE.DEFAULT, uds.SESSION_TYPE.PROGRAMMING):
      msg = radar_session_msg(session)
      assert msg.address == RADAR_UDS_ADDR and msg.src == RADAR_BUS and len(msg.dat) == 8
      assert msg.dat[:3] == bytes([0x02, 0x10, session]) and session in (0x01, 0x02)


class TestAccelCmdMap:
  def test_accel_cmd_to_accel_inverts_accel_to_accel_cmd(self):
    for v in (0.0, 3.0, 8.0, 15.0, 25.0, 35.0):
      for accel in (-1.5, -0.7, -0.2, 0.0, 0.1, 0.6, 1.4):
        cmd = accel_to_accel_cmd(accel, v)
        assert abs(accel_cmd_to_accel(cmd, v) - accel) < 0.002


class TestRadarWitness:
  def test_alive_and_silent_edges(self):
    w = RadarWitness()
    assert not w.alive and w.silent and not w.seen
    w.update(packet((CRZ_INFO_ADDR, crz_info(), STOCK)))
    assert w.alive and not w.silent and w.seen and w.run_frames == 1
    for _ in range(RadarWitness.SILENT_PACKETS - 1):
      w.update(packet())
      assert w.alive
    w.update(packet())
    assert w.silent and not w.alive and w.run_frames == 0
    w.update(packet((CRZ_INFO_ADDR, crz_info(), STOCK)))
    assert w.alive and w.run_frames == 1
    w.update(packet((CRZ_INFO_ADDR, crz_info(), STOCK), (CRZ_INFO_ADDR, crz_info(), STOCK)))
    assert w.run_frames == 3

  def test_ignores_our_echo_and_other_buses(self):
    w = RadarWitness()
    w.update(packet((CRZ_INFO_ADDR, crz_info(), ECHO), (CRZ_INFO_ADDR, crz_info(), 2)))
    assert not w.seen and not w.alive and w.stock_accel_cmd is None

  def test_time_is_counted_in_packets_not_frames(self):
    # a late control loop draining several packets at once must see the silence in them
    w = RadarWitness()
    w.update(packet((CRZ_INFO_ADDR, crz_info(), STOCK)))
    w.update([(0, [])] * RadarWitness.SILENT_PACKETS)
    assert w.silent

  def test_accel_cmd_and_driving(self):
    w = RadarWitness()
    w.update(packet((CRZ_INFO_ADDR, crz_info(standby=True), STOCK)))
    assert w.stock_accel_cmd is None and not w.mrcc_driving
    w.update(packet((CRZ_INFO_ADDR, crz_info(accel=0.5, v=20.0), STOCK)))
    assert w.stock_accel_cmd == accel_to_accel_cmd(0.5, 20.0)
    assert w.mrcc_driving          # CRZ_CTRL never seen: the command alone is trusted
    w.update(packet((CRZ_CTRL_ADDR, crz_ctrl(False), STOCK)))
    assert not w.stock_cruise_active and not w.mrcc_driving
    w.update(packet((CRZ_CTRL_ADDR, crz_ctrl(True), STOCK)))
    assert w.stock_cruise_active and w.mrcc_driving
    w.update(packet((CRZ_INFO_ADDR, crz_info(standby=True), STOCK)))
    assert not w.mrcc_driving      # standby again: no command, whatever CRZ_CTRL says

  def test_counters_and_heartbeat_frames(self):
    w = RadarWitness()
    hb = bytes([0xff, 0xf7, 0xfe, 0xfe, 0x1f, 0xc0, 0x00, 0x89])
    w.update(packet((CRZ_INFO_ADDR, crz_info(ctr=7), STOCK), (RADAR_COUNTER_ADDR, hb, STOCK), (0x499, bytes(8), STOCK)))
    assert w.crz_info_counter == 7 and w.heartbeat_counter == 9
    assert set(w.heartbeat_frames()) == {RADAR_COUNTER_ADDR, 0x499}
    assert all(a in HEARTBEAT_ADDRS for a in w.heartbeat_frames())


def arb_update(a: HybridArbiter, n=1, emulating=False, engaged=True, long_active=True, experimental=False,
               standstill=False, brake=False, gas=False, v=20.0, accel=0.0) -> bool:
  want = a.want_emulation
  for _ in range(n):
    want = a.update(emulating, engaged, long_active, experimental, standstill, brake, gas, v, accel)
  return want


class TestHybridArbiter:
  def test_takeover_needs_two_seconds_of_experimental(self):
    a = HybridArbiter()
    assert not arb_update(a, HybridArbiter.TAKEOVER_DEBOUNCE_FRAMES - 1, experimental=True)
    assert arb_update(a, 1, experimental=True)

  def test_flickering_experimental_never_takes_over(self):
    a = HybridArbiter()
    for _ in range(20):
      assert not arb_update(a, HybridArbiter.TAKEOVER_DEBOUNCE_FRAMES - 10, experimental=True)
      assert not arb_update(a, 5, experimental=False)

  def test_takeover_gates(self):
    for kw in ({"long_active": False}, {"gas": True}, {"brake": True}, {"v": 1.5}, {"engaged": False}, {"experimental": False}):
      a = HybridArbiter()
      assert not arb_update(a, 400, **({"experimental": True} | kw))
    a = HybridArbiter()
    a.takeover_rejected = True
    assert not arb_update(a, 400, experimental=True)

  def test_mrcc_unavailable_takes_over_without_experimental(self):
    a = HybridArbiter()
    a.mrcc_unavailable = True
    assert not arb_update(a, HybridArbiter.DEBOUNCE_FRAMES - 1)   # a latched fault: the short debounce, not the 2 s one
    assert arb_update(a, 1)
    # and keeps the radar in standard mode
    assert arb_update(a, 400, emulating=True)

  def test_dwell_between_switches(self):
    a = HybridArbiter()
    assert arb_update(a, HybridArbiter.TAKEOVER_DEBOUNCE_FRAMES, experimental=True)
    # a disengage right after the takeover: the hand-back waits for the dwell, not just its own debounce
    assert arb_update(a, HybridArbiter.DWELL_FRAMES - 1, emulating=True, engaged=False)
    assert not arb_update(a, 1, emulating=True, engaged=False)

  def test_no_handback_while_engaged(self):
    # standard mode, above the speed MRCC could be set at, openpilot not braking: still no hand-back
    a = HybridArbiter()
    a.want_emulation = True
    assert arb_update(a, 6000, emulating=True, experimental=False, v=30.0, accel=0.0)
    # only a disengage hands the radar back
    assert not arb_update(a, HybridArbiter.DISENGAGED_DEBOUNCE_FRAMES, emulating=True, engaged=False)

  def test_handback_gates_when_engaged_handback_is_enabled(self, monkeypatch):
    monkeypatch.setattr(HybridArbiter, "HANDBACK_WHILE_ENGAGED", True)
    a = HybridArbiter()
    a.want_emulation = True
    assert arb_update(a, 400, emulating=True, v=8.0)               # below the speed MRCC can be set at
    assert arb_update(a, 400, emulating=True, accel=-0.6)          # openpilot is braking
    assert arb_update(a, HybridArbiter.DEBOUNCE_FRAMES - 1, emulating=True)
    assert not arb_update(a, 1, emulating=True)

  def test_disengaged_debounce_is_longer(self):
    a = HybridArbiter()
    a.want_emulation = True
    assert arb_update(a, HybridArbiter.DISENGAGED_DEBOUNCE_FRAMES - 1, emulating=True, engaged=False, experimental=True)
    assert not arb_update(a, 1, emulating=True, engaged=False, experimental=True)

  def test_standstill(self):
    a = HybridArbiter()
    a.want_emulation = True
    # a stop is never a reason to hand back on its own, even in standard mode
    assert arb_update(a, 400, emulating=True, standstill=True, v=0.0)
    # ... only while the driver holds the brake
    assert not arb_update(a, HybridArbiter.DEBOUNCE_FRAMES, emulating=True, standstill=True, v=0.0, brake=True)
    # and never a reason to take over
    b = HybridArbiter()
    assert not arb_update(b, 400, standstill=True, v=0.0, experimental=True)

  def test_takeover_rejected_when_cruise_drops_without_brake(self):
    a = HybridArbiter()
    a.note_takeover()
    arb_update(a, 1, emulating=True, experimental=True)
    arb_update(a, 1, emulating=True, experimental=True, engaged=False)
    assert a.takeover_rejected
    b = HybridArbiter()
    b.note_takeover()
    arb_update(b, 1, emulating=True, experimental=True)
    arb_update(b, 1, emulating=True, experimental=True, engaged=False, brake=True)
    assert not b.takeover_rejected
    c = HybridArbiter()
    c.note_takeover()
    arb_update(c, HybridArbiter.TAKEOVER_WATCH_FRAMES + 1, emulating=True, experimental=True)
    arb_update(c, 1, emulating=True, experimental=True, engaged=False)
    assert not c.takeover_rejected


class FakeWitness:
  def __init__(self, alive=True, run_frames=100, driving=False):
    self.alive = alive
    self.run_frames = run_frames
    self.mrcc_driving = driving
    self.stock_cruise_active = driving
    self.stock_accel_cmd = 100 if driving else None

  @property
  def silent(self):
    return not self.alive


def resync_run(r: MrccResync, w: FakeWitness, n: int, mrcc=True, engaged=True, brake=False, gas=False, v=20.0) -> int:
  presses = 0
  for _ in range(n):
    r.update(mrcc, engaged, w, brake, gas, v)
    presses += int(r.press_resume)
  return presses


class TestMrccResync:
  def test_standby_after_handback_is_nudged_then_given_up(self):
    r, w = MrccResync(), FakeWitness()
    r.note_handback()
    assert resync_run(r, w, MrccResync.CONFIRM_FRAMES) == 0          # standby has to hold for a second first
    assert resync_run(r, w, MrccResync.PULSE_FRAMES) == MrccResync.PULSE_FRAMES
    assert resync_run(r, w, MrccResync.SETTLE_FRAMES) == 0 and r.attempts == 1 and not r.failed
    assert resync_run(r, w, MrccResync.RETRY_FRAMES) == 0
    assert resync_run(r, w, MrccResync.PULSE_FRAMES) == MrccResync.PULSE_FRAMES
    assert resync_run(r, w, MrccResync.SETTLE_FRAMES) == 0 and r.attempts == 2 and not r.failed
    assert resync_run(r, w, MrccResync.RETRY_FRAMES) == 0
    assert r.failed and not r.press_resume
    assert resync_run(r, w, 1000) == 0                                  # latched: no more presses

  def test_radar_driving_ends_it(self):
    r, w = MrccResync(), FakeWitness()
    r.note_handback()
    resync_run(r, w, MrccResync.CONFIRM_FRAMES + 5)
    w.mrcc_driving = True
    resync_run(r, w, 1)
    assert not r.armed and r.attempts == 0 and not r.failed
    assert resync_run(r, w, 1000) == 0

  def test_never_over_pedals_or_at_low_speed_or_disengaged(self):
    for kw in ({"brake": True}, {"gas": True}, {"v": 1.0}, {"engaged": False}, {"mrcc": False}):
      r, w = MrccResync(), FakeWitness()
      r.note_handback()
      assert resync_run(r, w, 1000, **kw) == 0 and not r.failed

  def test_driver_engagement_clears_the_latch(self):
    r = MrccResync()
    r.failed = True
    r.note_engaged_by_driver()
    assert not r.failed and not r.armed and r.attempts == 0

  def test_radar_restart_on_its_own_arms_it(self):
    r, w = MrccResync(), FakeWitness(driving=True)
    resync_run(r, w, 10)
    w.alive = False
    resync_run(r, w, 10)
    w.alive, w.mrcc_driving, w.stock_accel_cmd = True, False, None
    assert r.armed is False
    resync_run(r, w, 1)
    assert r.armed
    assert resync_run(r, w, MrccResync.CONFIRM_FRAMES + MrccResync.PULSE_FRAMES) == MrccResync.PULSE_FRAMES

  def test_first_frames_of_a_drive_are_not_a_restart(self):
    r, w = MrccResync(), FakeWitness()
    assert resync_run(r, w, 1000) == 0 and not r.armed


def is_programming(msg):
  return msg is not None and msg.address == RADAR_UDS_ADDR and msg.dat[:3] == bytes([2, 0x10, uds.SESSION_TYPE.PROGRAMMING])


def is_default(msg):
  return msg is not None and msg.address == RADAR_UDS_ADDR and msg.dat[:3] == bytes([2, 0x10, uds.SESSION_TYPE.DEFAULT])


def is_tester_present(msg):
  return msg is not None and msg.address == RADAR_UDS_ADDR and msg.dat[:3] == bytes([2, 0x3E, 0x80])


class TestHybridRadarManager:
  def test_full_cycle(self):
    m, w = HybridRadarManager(), FakeWitness()
    assert m.update(False, w, True, True) == RadarMaster.MRCC and m.diagnostic is None and not m.transmitting
    assert m.update(True, w, True, True) == RadarMaster.SILENCING and is_programming(m.diagnostic)
    assert m.update(True, w, True, True) == RadarMaster.SILENCING and m.diagnostic is None and not m.transmitting
    w.alive, w.run_frames = False, 0
    assert m.update(True, w, True, True) == RadarMaster.EMULATING and m.just_took_over and m.transmitting and m.emulating
    presents = 0
    for _ in range(200):
      m.update(True, w, True, True)
      assert m.transmitting and m.state == RadarMaster.EMULATING and not m.just_took_over
      presents += int(is_tester_present(m.diagnostic))
    assert presents == 4
    # hand back: default session requested at once, frames held until the radar speaks
    assert m.update(False, w, True, True) == RadarMaster.RESTORING and is_default(m.diagnostic) and m.transmitting and m.emulating
    for _ in range(60):
      m.update(False, w, True, True)
      assert m.transmitting
    w.alive, w.run_frames = True, 1
    assert m.update(False, w, True, True) == RadarMaster.RESTORING and not m.transmitting and not m.just_handed_back
    w.run_frames = HybridRadarManager.RESTORE_CONFIRM_FRAMES
    assert m.update(False, w, True, True) == RadarMaster.MRCC and m.just_handed_back and not m.emulating
    assert not m.silence_failed and not m.restore_failed

  def test_no_takeover_without_prerequisites(self):
    m = HybridRadarManager()
    assert m.update(True, FakeWitness(alive=False), True, True) == RadarMaster.MRCC
    assert m.update(True, FakeWitness(), False, True) == RadarMaster.MRCC
    assert m.update(True, FakeWitness(), True, False) == RadarMaster.MRCC
    m.silence_failed = True
    assert m.update(True, FakeWitness(), True, True) == RadarMaster.MRCC

  def test_radar_returning_while_emulating_stops_our_frames_at_once(self):
    m, w = HybridRadarManager(), FakeWitness()
    m.update(True, w, True, True)
    w.alive = False
    m.update(True, w, True, True)
    assert m.state == RadarMaster.EMULATING and m.transmitting
    w.alive, w.run_frames = True, 1
    assert m.update(True, w, True, True) == RadarMaster.RESTORING and not m.transmitting and m.silence_failed
    w.run_frames = 5
    assert m.update(True, w, True, True) == RadarMaster.MRCC and m.just_handed_back
    for _ in range(100):
      assert m.update(True, w, True, True) == RadarMaster.MRCC    # no more takeovers this drive

  def test_silence_timeout(self):
    m, w = HybridRadarManager(), FakeWitness()
    for _ in range(HybridRadarManager.SILENCE_TIMEOUT_FRAMES):
      assert m.update(True, w, True, True) == RadarMaster.SILENCING
    assert m.update(True, w, True, True) == RadarMaster.RESTORING and m.silence_failed and not m.transmitting
    for _ in range(HybridRadarManager.UNDO_SETTLE_FRAMES):
      m.update(True, w, True, True)
    assert m.state == RadarMaster.MRCC and not m.just_handed_back

  def test_takeover_withdrawn_after_request_is_undone(self):
    m, w = HybridRadarManager(), FakeWitness()
    m.update(True, w, True, True)
    assert m.update(False, w, True, True) == RadarMaster.RESTORING and not m.transmitting and not m.emulating
    # the request landed late: the radar goes quiet after we stopped wanting it
    w.alive, w.run_frames = False, 0
    for _ in range(10):
      m.update(False, w, True, True)
      assert m.state == RadarMaster.RESTORING and not m.transmitting
    w.alive, w.run_frames = True, 10
    for _ in range(HybridRadarManager.UNDO_SETTLE_FRAMES + 1):
      m.update(False, w, True, True)
    assert m.state == RadarMaster.MRCC

  def test_restore_timeout_keeps_emulating(self):
    m, w = HybridRadarManager(), FakeWitness()
    m.update(True, w, True, True)
    w.alive = False
    m.update(True, w, True, True)
    assert m.update(False, w, True, True) == RadarMaster.RESTORING
    for _ in range(HybridRadarManager.RESTORE_TIMEOUT_FRAMES - 1):
      m.update(False, w, True, True)
      assert m.transmitting and m.state == RadarMaster.RESTORING
    m.update(False, w, True, True)
    assert m.state == RadarMaster.EMULATING and m.restore_failed and m.transmitting
    for _ in range(100):
      assert m.update(False, w, True, True) == RadarMaster.EMULATING and m.transmitting

  def test_bus_unhealthy_never_takes_over(self):
    m, w = HybridRadarManager(), FakeWitness()
    m.update(True, w, True, True)
    w.alive = False
    assert m.update(True, w, False, True) == RadarMaster.RESTORING and not m.transmitting
