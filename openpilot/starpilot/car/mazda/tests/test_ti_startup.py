import unittest
from types import SimpleNamespace

from opendbc.car import Bus
from opendbc.car.mazda.values import MazdaSafetyFlags

from openpilot.starpilot.car.mazda.ti_startup import ti_startup_pending


def parser(bus_timeout=False, invalid_cnt=0):
  return SimpleNamespace(bus_timeout=bus_timeout, can_invalid_cnt=invalid_cnt)


def parsers(pt=None, body=None, cam=None):
  return {Bus.pt: pt or parser(), Bus.body: body or parser(), Bus.cam: cam or parser()}


def cp(brand="mazda", ti=True):
  return SimpleNamespace(brand=brand, flags=MazdaSafetyFlags.TORQUE_INTERCEPTOR.value if ti else 0)


def cs(valid=False, timeout=True):
  return SimpleNamespace(canValid=valid, canTimeout=timeout)


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
    self.assertFalse(ti_startup_pending(cp(), parsers(pt=parser(invalid_cnt=3)), cs(valid=False, timeout=False)))

  def test_unhealthy_car_bus_is_not_pending(self):
    self.assertFalse(ti_startup_pending(cp(), parsers(pt=parser(bus_timeout=True), body=parser(bus_timeout=True)), cs()))
    self.assertFalse(ti_startup_pending(cp(), parsers(pt=parser(invalid_cnt=1), body=parser(bus_timeout=True)), cs()))
    self.assertFalse(ti_startup_pending(cp(), parsers(cam=parser(bus_timeout=True), body=parser(bus_timeout=True)), cs()))

  def test_missing_body_parser(self):
    self.assertFalse(ti_startup_pending(cp(), {Bus.pt: parser()}, cs()))


if __name__ == "__main__":
  unittest.main()
