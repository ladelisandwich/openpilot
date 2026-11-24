"""Optional stock-ACC stop retention for exactly the two Grand Cherokee ports."""


def brake_hold_supported(cp):
  from opendbc.car import structs
  from opendbc.car.chrysler.values import CAR, ChryslerFlags
  return bool(cp is not None and cp.brand == "chrysler" and
              cp.carFingerprint in (CAR.JEEP_GRAND_CHEROKEE, CAR.JEEP_GRAND_CHEROKEE_2019) and
              cp.flags & ChryslerFlags.JEEP_BRAKE_HOLD_AVAILABLE and cp.pcmCruise and
              not cp.openpilotLongitudinalControl and not cp.passive and not cp.dashcamOnly and not cp.notCar and
              len(cp.safetyConfigs) == 1 and cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.chrysler and
              cp.safetyConfigs[0].safetyParam in (0, 4))


def apply_brake_hold(cp, requested):
  if brake_hold_supported(cp):
    cp.safetyConfigs[0].safetyParam = 4 if requested else 0


def create_das_3_command(packer, counter_offset, brake_decel, das_3):
  values = das_3.copy()
  values.update(ACC_AVAILABLE=1, ACC_ACTIVE=1, ACC_GO=0, ACC_STANDSTILL=0, ACC_DECEL_REQ=1,
                ACC_DECEL=brake_decel, ACC_BRK_PREP=0, ENGINE_TORQUE_REQUEST_MAX=0, GR_MAX_REQ=2,
                COUNTER=(int(das_3["COUNTER"]) + counter_offset) % 16)
  return packer.make_can_msg("DAS_3", 0, values)
