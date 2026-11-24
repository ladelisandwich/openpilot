"""Finite radar identity and one immutable final-CarParams context."""
from dataclasses import dataclass
import hashlib
import math
import struct

from capnp import KjException

from opendbc.car import structs
from opendbc.car.tesla.preap.aol import qualified as stock_qualified


@dataclass(frozen=True)
class RadarStartupContext:
  version: int
  safety_param: int
  car_params_sha256: bytes
  offset: float


def qualified(cp):
  try:
    return bool(stock_qualified(cp) and not cp.radarUnavailable and cp.safetyConfigs[0].safetyParam in (2, 3, 6, 7))
  except (AttributeError, TypeError, ValueError, IndexError, OverflowError):
    return False


def valid_context(cp, context, cp_bytes):
  if (type(context) is not RadarStartupContext or type(context.version) is not int or context.version != 1 or
      type(context.safety_param) is not int or not qualified(cp) or
      type(cp_bytes) is not bytes or not 0 < len(cp_bytes) <= 1024 * 1024 or
      context.safety_param != int(cp.safetyConfigs[0].safetyParam) or
      type(context.car_params_sha256) is not bytes or len(context.car_params_sha256) != 32 or
      hashlib.sha256(cp_bytes).digest() != context.car_params_sha256 or
      type(context.offset) is not float or not math.isfinite(context.offset)):
    return False
  try:
    if struct.unpack('<f', struct.pack('<f', context.offset))[0] != context.offset:
      return False
    with structs.CarParams.from_bytes(cp_bytes) as published:
      return published.to_dict() == cp.to_dict()
  except (AttributeError, ValueError, TypeError, OverflowError, RuntimeError, KjException):
    return False
