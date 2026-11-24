"""Physical left-paddle signal on the standard Ioniq CAN FD button layout."""

from opendbc.car.hyundai.values import CAR, HyundaiFlags

IONIQ_5_CARS = frozenset((CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_IONIQ_5_N, CAR.HYUNDAI_IONIQ_5_PE))


def has_left_paddle(cp) -> bool:
  return bool(getattr(cp, 'brand', None) == 'hyundai' and
              cp.carFingerprint in IONIQ_5_CARS | {CAR.HYUNDAI_IONIQ_6} and
              cp.flags & HyundaiFlags.CANFD and cp.flags & HyundaiFlags.EV and
              not cp.flags & HyundaiFlags.CANFD_ALT_BUTTONS)
