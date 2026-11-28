"""Always-on-lateral admission for the fork's GEN1 with the Torque Interceptor and radar emulation."""
from opendbc.car import structs
from opendbc.car.mazda.stock_aol import TI_RE_SAFETY_PARAM, qualified, stock_qualified, ti_re_qualified
from opendbc.car.mazda.values import MazdaFlags

CarParams = structs.CarParams


def cx9(flags=777, param=265, long=True, pcm=True, source=CarParams.FingerprintSource.fw, experience=0, fingerprint='MAZDA_CX9_2021'):
  cp = CarParams(brand='mazda', carFingerprint=fingerprint, flags=flags, openpilotLongitudinalControl=long, pcmCruise=pcm,
                 fingerprintSource=source, transmissionType=CarParams.TransmissionType.automatic, alternativeExperience=experience)
  cp.safetyConfigs = [CarParams.SafetyConfig(safetyModel=CarParams.SafetyModel.mazda, safetyParam=param)]
  return cp


def test_the_cx9_configuration_qualifies():
  assert TI_RE_SAFETY_PARAM == 265
  for flags in (777, 777 & ~int(MazdaFlags.HYBRID_LONG)):   # with or without hybrid longitudinal
    assert ti_re_qualified(cx9(flags)) and qualified(cx9(flags)) and not stock_qualified(cx9(flags))
  assert not qualified(cx9(), marked_only=True)       # marked only once card adds the experience
  assert qualified(cx9(experience=32), marked_only=True)


def test_everything_else_stays_out():
  assert not qualified(cx9(source=CarParams.FingerprintSource.fixed))   # a forced platform could be the wrong car
  assert not qualified(cx9(param=9))                                      # no radar emulation
  assert not qualified(cx9(flags=777 | int(MazdaFlags.RADAR_INTERCEPTOR)))
  assert not qualified(cx9(long=False))
  assert not qualified(cx9(pcm=False))
  assert not qualified(cx9(fingerprint='MAZDA_3_2019'))
  assert not qualified(cx9(experience=1))
