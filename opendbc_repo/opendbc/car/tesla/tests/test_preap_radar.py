import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.tesla.tests.test_preap_stock import params
from opendbc.car.tesla.radar_interface import RadarInterface


def radar_params(offset=0.0, mount=False):
  cp = params()
  cp.radarUnavailable = False
  cp.safetyConfigs[0].safetyParam = 6 if mount else 2
  return cp


def radar_interface(cp, offset=0.0, cp_bytes=None):
  import hashlib
  from opendbc.car.tesla.preap.radar_profile import RadarStartupContext
  from openpilot.starpilot.car.tesla.radar_settings import offset_float32
  ri = RadarInterface(cp)
  raw = cp.to_bytes() if cp_bytes is None else cp_bytes
  ri.configure_preap_radar(RadarStartupContext(1, int(cp.safetyConfigs[0].safetyParam),
                                              hashlib.sha256(raw).digest(), offset_float32(offset)), raw)
  return ri


def frames(packer, *, tracked=True, probability=75.0, distance=30.0, index=0, measured=True, fault=False):
  result = [packer.make_can_msg("TeslaRadarSguInfo", 1, {"RADC_HWFail": fault})]
  for i in range(32):
    result.append(packer.make_can_msg(f"RadarPoint{i}_A", 1, {
      "Index": 0, "Tracked": tracked if i == 0 else False, "ProbExist": probability,
      "LongDist": distance, "LatDist": 1.0, "LongSpeed": -2.0, "LongAccel": 0.5, "Meas": measured,
    }))
    result.append(packer.make_can_msg(f"RadarPoint{i}_B", 1, {"Index2": index, "LatSpeed": 0.25}))
  return result


@pytest.mark.parametrize("mount", (False, True))
def test_actual_decoder_offset_metadata_and_cycle_trigger(mount):
  cp = radar_params(0.25, mount)
  raw = cp.to_bytes()
  with structs.CarParams.from_bytes(raw) as saved:
    ri = radar_interface(saved, 0.25, cp_bytes=raw)
  packer = CANPacker("tesla_radar_bosch_generated")
  ri.update([])
  for tick in range(1, 5):
    result = ri.update([(tick * 125_000_000, frames(packer))])
  assert len(result.points) == 1
  point = result.points[0]
  assert point.dRel == 30.0 and point.yRel == 1.25 and point.vRel == -2.0
  assert point.deprecated.measured and point.deprecated.aRel == 0.5 and point.deprecated.yvRel == 0.25
  assert ri.update([(625_000_000, frames(packer)[:-1])]) is None


@pytest.mark.parametrize("changes", ({"tracked": False}, {"probability": 46.875}, {"distance": 0.0},
                                     {"distance": 251.0}, {"index": 1}))
def test_original_index_probability_distance_and_track_retirement(changes):
  ri, packer = radar_interface(radar_params()), CANPacker("tesla_radar_bosch_generated")
  ri.update([])
  for tick in range(1, 5):
    result = ri.update([(tick * 125_000_000, frames(packer))])
  assert len(result.points) == 1
  result = ri.update([(625_000_000, frames(packer, **changes))])
  assert not result.points


def test_source_expiry_and_hardware_fault_are_published():
  ri, packer = radar_interface(radar_params()), CANPacker("tesla_radar_bosch_generated")
  ri.update([])
  for tick in range(1, 5):
    result = ri.update([(tick * 125_000_000, frames(packer, fault=True))])
  assert result.errors.radarFault
  trigger = packer.make_can_msg("RadarPoint31_B", 1, {"Index2": 0})
  for step in range(5):
    result = ri.update([(2_000_000_000 + step * 125_000_000, [trigger])])
  assert result.errors.canError
