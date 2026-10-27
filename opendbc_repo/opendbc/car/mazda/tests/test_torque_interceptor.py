"""Unit tests for the Torque Interceptor ceiling: the car controller's clamp at TI_CEILING and the guard under it
(opendbc/car/mazda/torque_interceptor.py)."""
from types import SimpleNamespace

from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.mazda.torque_interceptor import TiCeilingGuard
from opendbc.car.mazda.values import CarControllerParams, MazdaSafetyFlags

CCP = CarControllerParams(SimpleNamespace(flags=MazdaSafetyFlags.GEN1 | MazdaSafetyFlags.TORQUE_INTERCEPTOR))
TI_TRIP_FRAMES = 184   # the shortest unbroken 800 that faulted the TI (1.84 s)


def guard():
  return TiCeilingGuard(CCP.TI_CEILING_DIP, CCP.TI_CEILING_HOLD_FRAMES, CCP.TI_CEILING_DIP_FRAMES)


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


def test_requests_below_the_ceiling_are_untouched():
  for level in (0.0, 0.3, 0.8, 0.85, -0.85):   # 0.85 is the dip level itself: at it, not above it
    sent = drive([level] * 1000)
    assert sent[-1] == int(round(level * CCP.STEER_MAX))
    assert len(set(sent[150:])) == 1


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
