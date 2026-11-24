"""Dom's PCM stop-latch branch for exact existing legacy command owners."""
from opendbc.car.toyota.values import CAR

CARS = frozenset((CAR.TOYOTA_CAMRY, CAR.TOYOTA_CHR, CAR.TOYOTA_AVALON_2019,
                  CAR.LEXUS_LS, CAR.LEXUS_NX, CAR.LEXUS_RX, CAR.LEXUS_CTH))


def enabled(cp):
  from opendbc.car.toyota import dsu_bypass
  if dsu_bypass.enabled(cp):
    return cp.carFingerprint not in dsu_bypass.AUTO_RESUME_CARS
  if cp is None or cp.carFingerprint not in CARS or not cp.openpilotLongitudinalControl:
    return False
  from opendbc.car.toyota import rav4h_filter, advert_filter, radar_filter
  return rav4h_filter.enabled(cp) or advert_filter.enabled(cp) or radar_filter.configured(cp)


def request(previous_standstill, previous_request, standstill, pcm_acc_status, resume, long_active):
  requested = previous_request
  if standstill and not previous_standstill:
    requested = True
  if pcm_acc_status != 8 or resume:
    requested = False
  # Retain the rebased inactive-axis neutral veto around Dom's source branch.
  if not long_active and standstill:
    requested = True
  return requested
