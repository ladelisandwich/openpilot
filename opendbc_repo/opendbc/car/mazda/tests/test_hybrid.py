"""Unit tests for the Mazda hybrid longitudinal state machines (opendbc/car/mazda/hybrid.py)."""
from opendbc.car import uds
from opendbc.car.mazda.hybrid import (CRZ_CTRL_ADDR, CRZ_INFO_ADDR, HEARTBEAT_ADDRS, RADAR_BUS, RADAR_COUNTER_ADDR, RADAR_UDS_ADDR,
                                      HybridArbiter, HybridRadarManager, MrccResync, RadarMaster, RadarWitness,
                                      MrccSwitch, ce_status_is_experimental, ce_status_is_forced_experimental, radar_session_msg,
                                      HYBRID_MASTER_ACTIVE, HYBRID_MASTER_EMULATING, HYBRID_MASTER_PENDING, hybrid_master_status)
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

  def test_only_the_drivers_forced_experimental_switches_the_radar(self):
    assert ce_status_is_forced_experimental(2)                # USER_OVERRIDDEN
    for status in (0, 1, 3, 4, 5, 6, 7, 8):                   # OFF, USER_DISABLED and every automatic trigger
      assert not ce_status_is_forced_experimental(status)


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
  def test_release_for_mrcc_and_forget_takeover(self):
    a = HybridArbiter()
    a.want_emulation = True
    a.note_takeover()
    a.forget_takeover()
    arb_update(a, 1, emulating=True, experimental=False)
    arb_update(a, 1, emulating=True, experimental=False, engaged=False)   # openpilot's own drop
    assert not a.takeover_rejected
    a.release_for_mrcc()
    assert not a.want_emulation
    assert not arb_update(a, 1, emulating=True, engaged=False)           # no disengaged debounce to wait out

  def test_takeover_needs_half_a_second_of_experimental(self):
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


def sw_run(q: MrccSwitch, n: int, emulating=True, restoring=False, engaged=True, want_mrcc=True, cruise=True, v=25.0,
           standstill=False, accel=0.0):
  drops = 0  # cycles pressing CANCEL
  started = done = False
  for _ in range(n):
    q.update(emulating, restoring, engaged, want_mrcc, cruise, v, standstill, accel)
    # the replacement frames only go inactive in the hand-back window, never during the drop itself
    assert q.drop_cruise == (q.press_cancel and q.handing_back)
    drops += q.press_cancel
    started |= q.started
    done |= q.done
  return drops, started, done


class TestMrccSwitch:
  def test_drop_then_done(self):
    q = MrccSwitch()
    drops, started, _ = sw_run(q, MrccSwitch.DEBOUNCE_FRAMES - 1)
    assert drops == 0 and not started                                  # a choice must hold 0.3 s
    drops, started, _ = sw_run(q, 10)
    assert started and drops == 10 and q.dropping
    drops, _, done = sw_run(q, 1, cruise=False)                       # the car turned its cruise off
    assert done and drops == 0 and not q.dropping
    drops, started, done = sw_run(q, 100, engaged=False, cruise=False)
    assert drops == 0 and not started and not done                    # nothing more once disengaged

  def test_nothing_unless_emulating_and_mrcc_wanted(self):
    for kw in ({"emulating": False}, {"want_mrcc": False}, {"engaged": False}):
      q = MrccSwitch()
      drops, started, _ = sw_run(q, 200, **kw)
      assert drops == 0 and not started, kw

  def test_deferred_below_19_mph_at_a_stop_or_while_braking(self):
    for kw in ({"v": 8.0}, {"standstill": True, "v": 0.0}, {"accel": -1.5}):
      q = MrccSwitch()
      drops, started, _ = sw_run(q, 200, **kw)
      assert drops == 0 and not started, kw
      drops, started, _ = sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 1)    # conditions met: it starts
      assert started, kw

  def test_abandoned_if_openpilot_brakes_before_the_drop(self):
    q = MrccSwitch()
    sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 5)
    assert q.dropping
    drops, _, done = sw_run(q, 1, accel=-1.5)
    assert drops == 0 and not done and not q.dropping and not q.failed
    _, started, _ = sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 1)
    assert started                                                     # retried once the braking is over

  def test_a_drop_the_car_ignores_is_not_repeated_until_the_driver_chooses_again(self):
    q = MrccSwitch()
    drops, _, _ = sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + MrccSwitch.DROP_TIMEOUT_FRAMES + 50)
    assert q.failed and drops == MrccSwitch.DROP_TIMEOUT_FRAMES + 1
    drops, started, _ = sw_run(q, 500)
    assert drops == 0 and not started
    sw_run(q, 1, want_mrcc=False)                                      # the driver went back to experimental
    _, started, _ = sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 1)
    assert started

  def test_a_cruise_engaged_before_the_radar_is_back_is_dropped_again(self):
    q = MrccSwitch()
    sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 5)
    _, _, done = sw_run(q, 1, cruise=False)
    assert done and q.handing_back
    drops, _, _ = sw_run(q, 50, restoring=True, engaged=False, cruise=False)
    assert drops == 0                                                  # waiting for the radar
    drops, started, _ = sw_run(q, 3, restoring=True, engaged=True, cruise=True, v=5.0)
    assert drops == 3 and not started                                  # a RES on openpilot's frames: dropped at once, any speed
    drops, _, _ = sw_run(q, 20, restoring=True, engaged=False, cruise=False)
    assert drops == 0 and q.handing_back
    drops, _, _ = sw_run(q, 10, emulating=False, engaged=True, cruise=True)
    assert drops == 0 and not q.handing_back                           # the radar is master again: the driver's RES is MRCC's

  def test_hand_back_window_gives_up_if_the_restore_fails_or_the_car_keeps_the_cruise(self):
    q = MrccSwitch()
    sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 5)
    sw_run(q, 1, cruise=False)
    drops, started, _ = sw_run(q, 200, emulating=False, engaged=True, cruise=True)  # restore failed: nothing to hand back to
    assert drops == 0 and not started and not q.handing_back

    q = MrccSwitch()
    sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 5)
    sw_run(q, 1, cruise=False)
    drops, started, _ = sw_run(q, MrccSwitch.DEBOUNCE_FRAMES - 1, restoring=False, engaged=True, cruise=True)
    assert drops == 0 and not started and not q.handing_back           # back to normal service: a new drop debounces again

    q = MrccSwitch()
    sw_run(q, MrccSwitch.DEBOUNCE_FRAMES + 5)
    sw_run(q, 1, cruise=False)
    drops, _, _ = sw_run(q, MrccSwitch.DROP_TIMEOUT_FRAMES + 50, restoring=True, engaged=True, cruise=True)
    assert drops == MrccSwitch.DROP_TIMEOUT_FRAMES + 1 and not q.handing_back and q.failed


class TestHybridMasterStatus:
  def test_who_drives_and_whether_a_switch_is_coming(self):
    def st(transmitting, wants_emulation, engaged=True, takeover_possible=True, handback_possible=True):
      return hybrid_master_status(transmitting, wants_emulation, engaged, takeover_possible, handback_possible)

    E, P = HYBRID_MASTER_EMULATING, HYBRID_MASTER_PENDING
    assert st(False, False) == 0                              # MRCC, as chosen
    assert st(True, True) == E                                # emulation, as chosen
    assert st(False, True) == P                               # experimental chosen, takeover coming
    assert st(True, False) == E | P                           # standard chosen, switch deferred or in progress
    assert st(False, True, engaged=False) == 0                # no takeover while disengaged: nothing pending
    assert st(False, True, takeover_possible=False) == 0      # takeovers off for this drive
    assert st(True, False, handback_possible=False) == E      # restore failed / switch failed: openpilot keeps it


def test_witness_reads_acc_set_allowed():
  w = RadarWitness()
  for allowed in (False, True):
    frame = build_crz_info(0.0, 3, False, False, 20.0, acc_set_allowed=allowed)
    w.update([(0, [(CRZ_INFO_ADDR, frame, RADAR_BUS)])])
    assert w.set_allowed == allowed


def test_warming_up_radar_is_pending_until_ready():
  assert hybrid_master_status(False, False, False, True, True, mrcc_warming_up=True) == HYBRID_MASTER_PENDING
  assert hybrid_master_status(False, True, False, True, True, mrcc_warming_up=True) == HYBRID_MASTER_PENDING
  assert hybrid_master_status(False, False, False, True, True, mrcc_warming_up=False) == 0


def test_master_active_bit_rides_on_any_state():
  A, E, P = HYBRID_MASTER_ACTIVE, HYBRID_MASTER_EMULATING, HYBRID_MASTER_PENDING
  assert hybrid_master_status(False, False, True, True, True, master_active=True) == A          # MRCC driving
  assert hybrid_master_status(True, True, True, True, True, master_active=True) == E | A        # openpilot driving
  assert hybrid_master_status(True, False, True, True, True, master_active=True) == E | P | A   # still driving, switch coming
  assert hybrid_master_status(False, False, False, True, True, mrcc_warming_up=True) == P       # warming up, nobody driving
