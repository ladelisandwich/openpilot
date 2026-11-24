"""Camry cluster radar under actual filter caller/native qualification."""
from unittest.mock import patch
import pytest
from opendbc.can import CANPacker
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4_angle_filter_callers as helpers

source_matched_native=helpers.source_matched_native
daemon_gc_scope=helpers.daemon_gc_scope


@pytest.mark.parametrize('alpha,disable',[(False,False),(True,False),(True,True)])
@pytest.mark.parametrize('aol',[False,True])
def test_camry_cluster_actual_filter_callers(alpha,disable,aol,tmp_path):
  physical=helpers.helpers.physical
  publish=helpers.card_module.Car.radar_publish
  packer=CANPacker('toyota_radar_dsu_tssp')
  frames_count=0
  observed=[]
  def frames(*args,**kwargs):
    nonlocal frames_count
    frames_count+=1
    output=physical(*args,**kwargs)
    if frames_count%10==0:
      for address in range(0x680,0x686):
        output.append(packer.make_can_msg(address,1,{'ID':4 if address==0x680 else 63,
          'LONG_DIST':30. if address==0x680 else 0.,'LAT_DIST':1.5,'SPEED':10.,'LAT_SPEED':.2}))
    return output
  def capture(host,rd):
    publish(host,rd)
    if rd is not None and not rd.errors.canError:
      assert len(rd.points)==1
      observed.append(rd.points[0].dRel)
  with patch.object(helpers.helpers,'physical',side_effect=frames),patch.object(helpers.card_module.Car,'radar_publish',capture):
    helpers.run_case(CAR.TOYOTA_CAMRY,alpha,disable,0,tmp_path,aol=aol)
  assert len(observed)>=50
  assert all(distance==pytest.approx(30.,abs=.03) for distance in observed)
