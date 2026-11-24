"""Unit tests for the Torque Interceptor ceiling: the car controller's clamp at TI_CEILING and the guard under it
(opendbc/car/mazda/torque_interceptor.py)."""
from types import SimpleNamespace

from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.mazda.torque_interceptor import TiCeilingGuard
from opendbc.car.mazda.values import CarControllerParams, MazdaSafetyFlags

CCP = CarControllerParams(SimpleNamespace(flags=MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR))
TI_TRIP_FRAMES = 184   # the shortest unbroken 800 that faulted the TI (1.84 s)


def guard():
  return TiCeilingGuard(CCP.TI_CEILING_DIP, CCP.TI_CEILING_HOLD_FRAMES, CCP.TI_CEILING_DIP_FRAMES, CCP.TI_CEILING - CCP.TI_CEILING_DIP)


def drive(requests, driver_torque=0.0):
  """The car controller's lateral path, frame by frame: the request through the driver/rate limits,
  then the guard. Returns the commands sent."""
  g, last, sent = guard(), 0, []
  for request in requests:
    wanted = int(round(request * CCP.STEER_MAX))
    wanted = max(-CCP.TI_CEILING, min(CCP.TI_CEILING, wanted))        # the car controller's clamp
    torque = apply_driver_steer_torque_limits(wanted, last, driver_torque, CCP)
    cap = g.update(torque)
    if cap is not None:
      torque = max(-cap, min(cap, torque))
    sent.append(torque)
    last = torque
  return sent


def longest_run(sent, pred):
  best = run = 0
  for x in sent:
    run = run + 1 if pred(x) else 0
    best = max(best, run)
  return best


def test_gen1_ti_constants():
  assert CCP.STEER_MAX == 800
  assert CCP.TI_CEILING == 700                                          # the EPS torque sensor reads no more above ~650
  assert CCP.TI_CEILING < 790                                           # 790 restarted the TI's count on the car
  assert CCP.TI_CEILING - CCP.TI_CEILING_DIP <= CCP.STEER_DELTA_DOWN   # the dip is one frame's ramp down
  assert CCP.TI_CEILING_DIP < CCP.TI_CEILING
  assert CCP.TI_CEILING_HOLD_FRAMES + CCP.TI_CEILING_DIP_FRAMES < TI_TRIP_FRAMES


def test_held_ceiling_never_reaches_the_ti_fault():
  for sign in (1, -1):
    sent = drive([sign * 1.0] * 1500)   # 15 s of a full request, e.g. a long ramp
    assert max(abs(x) for x in sent) == CCP.TI_CEILING            # never past the ceiling...
    assert longest_run(sent, lambda x: abs(x) > CCP.TI_CEILING_DIP) < CCP.TI_CEILING_HOLD_FRAMES
    assert longest_run(sent, lambda x: abs(x) == CCP.TI_CEILING) < TI_TRIP_FRAMES   # ...and never held there long enough to trip
    held = sent[200:]
    assert sum(abs(x) for x in held) / len(held) > 0.98 * CCP.TI_CEILING   # the ceiling is still used nearly all the time
    assert all(sign * x >= 0 for x in sent)


def test_dips_respect_the_rate_limits():
  sent = drive([1.0] * 800 + [-1.0] * 800)
  # away from zero at most STEER_DELTA_UP a frame, toward zero at most STEER_DELTA_DOWN
  for prev, cur in zip([0] + sent[:-1], sent, strict=True):
    step = cur - prev
    if prev == 0 or (prev > 0) == (step > 0):
      assert abs(step) <= CCP.STEER_DELTA_UP, (prev, step)
    else:
      assert abs(step) <= CCP.STEER_DELTA_DOWN, (prev, step)


def test_a_held_request_below_the_ceiling_is_lowered_before_the_ti_calls_it_stuck():
  # the TI's VIOL_LKAS_STUK: any nonzero request that does not change; 0.85 is the dip level itself
  for level in (0.3, 0.8, 0.85, -0.85, -0.3):
    want = int(round(level * CCP.STEER_MAX))
    sent = drive([level] * 1500)
    assert longest_run(sent[1:], lambda x, w=want: x == w) < TI_TRIP_FRAMES
    assert all(abs(x) <= abs(want) and level * x >= 0 for x in sent)           # only ever lowered, never flipped
    held = sent[200:]
    assert sum(abs(x) for x in held) / len(held) > 0.98 * abs(want)          # and only for a moment
  assert set(drive([0.0] * 1000)) == {0}                                       # zero is never touched


def test_a_tiny_request_held_at_a_creep_goes_to_zero_for_a_moment():
  # route 00000097: -1 count held for 2.0 s at 0.5 mph tripped VIOL 0x11
  g, sent = guard(), []
  for _ in range(600):
    cap = g.update(-1)
    sent.append(-1 if cap is None else max(-cap, min(cap, -1)))
  assert 0 in sent
  assert longest_run(sent, lambda x: x == -1) < TI_TRIP_FRAMES
  assert longest_run(sent, lambda x: x == -1) <= CCP.TI_CEILING_HOLD_FRAMES + 1
  assert set(sent) == {-1, 0}


def test_a_changing_request_is_untouched():
  g = guard()
  for i in range(2000):
    torque = 200 + (i % 7)          # a curve's command that moves every frame
    assert g.update(torque) is None


def test_a_turn_that_lets_go_of_the_ceiling_starts_the_count_again():
  g = guard()
  for _ in range(CCP.TI_CEILING_HOLD_FRAMES - 1):
    assert g.update(CCP.TI_CEILING) is None
  assert g.update(600) is None        # the turn eased off
  for _ in range(CCP.TI_CEILING_HOLD_FRAMES - 1):
    assert g.update(-CCP.TI_CEILING) is None
  assert g.update(-CCP.TI_CEILING) == CCP.TI_CEILING_DIP
  for _ in range(CCP.TI_CEILING_DIP_FRAMES - 1):
    assert g.update(CCP.TI_CEILING) == CCP.TI_CEILING_DIP
  assert g.update(CCP.TI_CEILING) is None


def test_ti_not_steering_resets_the_count():
  g = guard()
  for _ in range(CCP.TI_CEILING_HOLD_FRAMES - 1):
    g.update(CCP.TI_CEILING)
  assert g.update(0) is None          # TI out of RUN or lateral off: nothing sent to it
  for _ in range(CCP.TI_CEILING_HOLD_FRAMES - 1):
    assert g.update(CCP.TI_CEILING) is None


def test_violation_names_follow_the_ti_led_codes():
  from opendbc.car.mazda.torque_interceptor import VIOLATIONS, violation_name
  # the codes seen on the CX-9: 0x11 at the held ceiling, 0x31 at power-down, 0x33 from the output stage
  assert violation_name(0x11) == "0x11 LKAS_STUK (class 2)"
  assert violation_name(0x31) == "0x31 HIN_ERR (class 2)"
  assert violation_name(0x33) == "0x33 TK_WLOAD (class 2)"
  assert violation_name(0) == ""
  assert violation_name(0x5A) == "0x5a (unknown)"
  # every code is two decimal digits read as hex, as the TI's green-LED codes are
  assert all(code >> 4 in range(1, 10) and code & 0xF in range(10) for code in VIOLATIONS)
  assert {c for _, c, _ in VIOLATIONS.values()} == {1, 2, 3}
