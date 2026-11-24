from opendbc.car.pedal import supported_pedal_detected
from opendbc.car.gm.values import CAR, CanBus
from opendbc.car.vin import is_valid_vin


def pedal_candidate(candidate, fingerprint):
  stock_acc = candidate in (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_ACC_2022_2023)
  if supported_pedal_detected(fingerprint, CanBus.POWERTRAIN, supported=stock_acc):
    return CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL
  return candidate


def automatic_bolt_candidate(candidate, fingerprint, camera_cruise_messages, vin=None):
  """Refine Bolt CAN/FW aliases using VIN year and live camera capability."""
  camera_cruise_states = {payload[1] & 0x7 for payload in camera_cruise_messages if len(payload) == 6}
  bolt_ids = (CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_BOLT_CC_2017,
              CAR.CHEVROLET_BOLT_CC_2018_2021, CAR.CHEVROLET_BOLT_CC_2022_2023,
              CAR.CHEVROLET_BOLT_ACC_2022_2023, CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL)
  if candidate is not None and candidate not in bolt_ids:
    return candidate
  pt = fingerprint.get(CanBus.POWERTRAIN, {})
  # Year alone is not vehicle identity. Require the Bolt EV/EUV VIN descriptor
  # and matching generation's live PT layout before recovering a CAN collision.
  bolt_vin = (isinstance(vin, str) and is_valid_vin(vin) and
              vin.startswith('1G1F') and vin[5:8] == '6S0')
  if bolt_vin and vin[9] in 'HJKLMNP':
    if vin[9] in 'HJKLM':
      if any(pt.get(address) != length for address, length in ((0xD3, 2), (0xBE, 6), (0x130, 1))):
        return None
      return CAR.CHEVROLET_BOLT_CC_2017 if vin[9] == 'H' else CAR.CHEVROLET_BOLT_CC_2018_2021
    if any(pt.get(address) != length for address, length in
           ((0xD3, 3), (0xBE, 7), (0x130, 3), (0x1CA, 5), (0x236, 8))):
      return None
    pedal = supported_pedal_detected(fingerprint, CanBus.POWERTRAIN, supported=True)
    if 0x201 in pt and not pedal:
      return None
    if camera_cruise_states and camera_cruise_states <= {2, 3}:
      return CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL if pedal else CAR.CHEVROLET_BOLT_ACC_2022_2023
    if camera_cruise_messages == {bytes(6)}:
      return CAR.CHEVROLET_BOLT_CC_2022_2023
    # Missing camera or non-adaptive ACC mode is not proof of ordinary cruise.
    return None
  if candidate is None:
    return None
  if pt.get(0xD3) != 3:
    return candidate
  # Original Gen2 signature, qualified against the observed route's full DLCs.
  if any(pt.get(address) != length for address, length in
         ((0xBE, 7), (0x130, 3), (0x1CA, 5), (0x236, 8))):
    return None
  pedal = supported_pedal_detected(fingerprint, CanBus.POWERTRAIN, supported=True)
  if 0x201 in pt and not pedal:
    return None
  if camera_cruise_states and camera_cruise_states <= {2, 3}:
    return CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL if pedal else CAR.CHEVROLET_BOLT_ACC_2022_2023
  # Non-adaptive mode is not proof that the hardware lacks ACC. Only an exact
  # Gen2 identity with its corresponding pedal presence may retain ownership.
  if candidate == CAR.CHEVROLET_BOLT_CC_2022_2023:
    return candidate
  if candidate == CAR.CHEVROLET_BOLT_ACC_2022_2023 and not pedal:
    return candidate
  if candidate == CAR.CHEVROLET_BOLT_ACC_2022_2023_PEDAL and pedal:
    return candidate
  return None
