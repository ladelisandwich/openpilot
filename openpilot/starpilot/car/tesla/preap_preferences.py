from opendbc.car import get_safety_config, structs
from opendbc.car.tesla.values import CAR
from opendbc.car.tesla.preap.aol import qualified
from opendbc.car.tesla.preap.radar_profile import qualified as radar_qualified
from openpilot.starpilot.car.tesla.radar_settings import read_radar_settings


def stock_configuration(params, radar=None, pedal=None):
  try:
    from openpilot.starpilot.car.tesla.pedal_preferences import read_pedal_settings
    pedal = read_pedal_settings(params) if pedal is None else pedal
    radar = read_radar_settings(params) if radar is None else radar
    return pedal.valid and radar.valid
  except (OSError, TypeError, ValueError):
    return False


def prepare_stock(cp, admitted, radar=None, pedal=None):
  if cp.brand != "tesla" or cp.carFingerprint != CAR.TESLA_MODEL_S_PREAP:
    return cp
  if pedal is not None and pedal.enabled:
    from opendbc.car.tesla.preap.pedal_profile import qualified as pedal_qualified
    from opendbc.car.tesla.preap.constants import PEDAL_LONG_K_BP, PEDAL_LONG_KI_V
    desired = 1 | ((6 if radar.behind_nosecone else 2) if radar is not None and radar.enabled else 0)
    if (admitted and pedal.valid and pedal.calibration is not None and pedal.calibration.valid() and
        (qualified(cp) or pedal_qualified(cp))):
      cp.safetyConfigs[0].safetyParam = desired
      cp.radarUnavailable = desired == 1
      cp.openpilotLongitudinalControl = True
      cp.pcmCruise = False
      cp.alphaLongitudinalAvailable = False
      cp.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
      cp.longitudinalTuning.kiV = PEDAL_LONG_KI_V
      cp.longitudinalActuatorDelay = .4
      if pedal_qualified(cp):
        return cp
    cp.dashcamOnly = True
    cp.alternativeExperience = 0
    cp.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
    return cp
  if radar is not None and not radar.valid:
    admitted = False
  if admitted and radar is not None and radar.valid and radar.enabled and (qualified(cp) or radar_qualified(cp)):
    cp.safetyConfigs[0].safetyParam = 6 if radar.behind_nosecone else 2
    cp.radarUnavailable = False
  if not admitted or not (radar_qualified(cp) if radar is not None and radar.enabled else qualified(cp)):
    cp.dashcamOnly = True
    cp.alternativeExperience = 0
    cp.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
  return cp
