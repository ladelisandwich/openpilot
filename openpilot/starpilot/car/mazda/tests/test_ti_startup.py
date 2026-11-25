import unittest
from types import SimpleNamespace

from opendbc.car import Bus
from opendbc.can.parser import MAX_BAD_COUNTER
from opendbc.car.mazda.values import MazdaSafetyFlags

from openpilot.starpilot.car.mazda.ti_startup import parser_healthy, ti_startup_pending


class State:
  def __init__(self, ok=True, counter_fail=0):
    self.ok, self.counter_fail = ok, counter_fail

  def valid(self, current_nanos, bus_timeout):
    return self.ok


def parser(bus_timeout=False, states=()):
  return SimpleNamespace(bus_timeout=bus_timeout, _last_update_nanos=1_000, message_states={i: s for i, s in enumerate(states)})


def parsers(pt=None, body=None, cam=None):
  return {Bus.pt: pt or parser(states=[State()]), Bus.body: body or parser(states=[State()]), Bus.cam: cam or parser()}


def cp(brand="mazda", ti=True):
  return SimpleNamespace(brand=brand, flags=MazdaSafetyFlags.TORQUE_INTERCEPTOR.value if ti else 0)


def cs(valid=False, timeout=True):
  return SimpleNamespace(canValid=valid, canTimeout=timeout)


class TestParserHealthy(unittest.TestCase):
  def test_healthy_requires_live_bus_valid_messages_and_good_counters(self):
    self.assertTrue(parser_healthy(parser(states=[State(), State()])))
    self.assertTrue(parser_healthy(parser()))  # no messages registered (the camera bus parser)
    self.assertFalse(parser_healthy(parser(bus_timeout=True)))
    self.assertFalse(parser_healthy(parser(states=[State(), State(ok=False)])))
    self.assertFalse(parser_healthy(parser(states=[State(counter_fail=MAX_BAD_COUNTER)])))


class TestTiStartup(unittest.TestCase):
  def test_pending_when_only_the_interceptor_bus_is_silent(self):
    self.assertTrue(ti_startup_pending(cp(), parsers(body=parser(bus_timeout=True)), cs()))

  def test_not_pending_once_carstate_is_valid(self):
    self.assertFalse(ti_startup_pending(cp(), parsers(), cs(valid=True, timeout=False)))

  def test_other_brands_and_mazda_without_interceptor_follow_the_generic_rule(self):
    self.assertFalse(ti_startup_pending(cp(brand="hyundai"), parsers(body=parser(bus_timeout=True)), cs()))
    self.assertFalse(ti_startup_pending(cp(ti=False), parsers(body=parser(bus_timeout=True)), cs()))

  def test_interceptor_answering_means_another_fault(self):
    # bus 1 alive but CarState invalid: a car-bus problem, not the startup cycle
    self.assertFalse(ti_startup_pending(cp(), parsers(pt=parser(states=[State(ok=False)])), cs(valid=False, timeout=False)))

  def test_unhealthy_car_bus_is_not_pending(self):
    silent = parser(bus_timeout=True)
    self.assertFalse(ti_startup_pending(cp(), parsers(pt=parser(bus_timeout=True), body=silent), cs()))
    self.assertFalse(ti_startup_pending(cp(), parsers(pt=parser(states=[State(ok=False)]), body=silent), cs()))
    self.assertFalse(ti_startup_pending(cp(), parsers(cam=parser(bus_timeout=True), body=silent), cs()))

  def test_missing_body_parser(self):
    self.assertFalse(ti_startup_pending(cp(), {Bus.pt: parser(states=[State()])}, cs()))


if __name__ == "__main__":
  unittest.main()
