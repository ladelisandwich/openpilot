"""Narrow body-panel bounds for a corroborated US 2023 IONIQ 6.

These bounds exclude mirrors and accessories. They are used only for sampled
road-edge prediction, not vehicle identification or CAN/startup permission.
"""

CAMERA_FW = bytes.fromhex('f100434520204d46432020415420555341204c484420312e303020312e30342039393231312d4b4c30303020323231323133')
RADAR_FW = bytes.fromhex('f10043455f5f20524452202d2d2d2d2d202020202020312e303020312e30312039393131302d4b4c303030202020202020202020')


def _us_my23_vin(vin):
  if not isinstance(vin, str) or len(vin) != 17 or not vin.startswith('KMH') or vin[9] != 'P' or not 'A' <= vin[6] <= 'Z':
    return False
  values = {str(i): i for i in range(10)}
  values.update({c: i for i, c in enumerate('ABCDEFGH', 1)})
  values.update({c: i for i, c in enumerate('JKLMN', 1)})
  values.update({c: i for i, c in enumerate('STUVWXYZ', 2)})
  values.update(P=7, R=9)
  if any(c not in values for c in vin):
    return False
  weights = (8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2)
  remainder = sum(values[c] * w for c, w in zip(vin, weights, strict=True)) % 11
  return vin[8] == ('X' if remainder == 10 else str(remainder))


def dimensions(cp):
  """Read body metadata without changing CarParams or startup eligibility."""
  if (getattr(cp, 'brand', None) != 'hyundai' or getattr(cp, 'carFingerprint', None) != 'HYUNDAI_IONIQ_6'
      or getattr(cp, 'fuzzyFingerprint', True) is not False or not _us_my23_vin(getattr(cp, 'carVin', None))):
    return None
  matched = set()
  try:
    for fw in cp.carFw:
      if fw.logging:
        continue
      kind = str(fw.ecu)
      if kind not in ('fwdCamera', 'fwdRadar'):
        continue
      address, version = (0x7c4, CAMERA_FW) if kind == 'fwdCamera' else (0x7d0, RADAR_FW)
      if (fw.brand != 'hyundai' or fw.address != address or fw.subAddress != 0 or fw.bus not in (0, 1)
          or bytes(fw.fwVersion) != version):
        return None
      matched.add(kind)
  except (AttributeError, TypeError, ValueError, OverflowError):
    return None
  if matched != {'fwdCamera', 'fwdRadar'}:
    return None
  # Hyundai US 2023 owner's manual, printed page 2-9: overall W/L/H.
  # Mirrors are outside this body-panel envelope.
  return 1.880, 4.855, 1.495
