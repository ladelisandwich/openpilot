"""EPS73 actual factory/Card/SelfdriveD/Controls/C++/native cold callers."""
import struct
import pytest
from opendbc.car.toyota import prius_retrofit_advert as owner, prius_advert_filter
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_prius_retrofit_advert_fw import complete_firmware
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge
from opendbc.car import Bus

source_matched_native=helpers.source_matched_native
daemon_gc_scope=helpers.daemon_gc_scope


@pytest.mark.parametrize('candidate',owner.CARS)
@pytest.mark.parametrize('stock',(False,True))
def test_eps73_advert_complete_firmware_actual_callers(candidate,stock,host_heartbeat_bridge):
  def physical(packer,**fields):
    fields['sdsu']=None
    frames=helpers.physical(packer,**fields)
    assert len(next(data for address,data,bus in frames if address==0x262 and bus==0))==8
    frames.extend(((0x2FF,bytes.fromhex('80007409'),0),(0x2FF,bytes.fromhex('80007409'),2)))
    return frames

  def observe(tick,host,cs,cc,safety,native):
    parser=host.CI.can_parsers[Bus.pt]
    assert isinstance(parser,prius_advert_filter.PriusAdvertCANParser)
    assert 0x2FF not in parser.message_states and 'SDSU' not in parser.vl
    assert host.CI.CS.eps_torque_scale==.73
    assert host.CP.steerActuatorDelay==struct.unpack('<f',struct.pack('<f',.14))[0]
    assert host.CP.lateralTuning.torque.steeringAngleDeadzoneDeg==struct.unpack('<f',struct.pack('<f',.3))[0]
    assert owner.configured(host.CP)
    if tick>=220:
      assert not cs.stockAeb and cc.latActive and cc.longActive==(not stock)
      assert safety.aol_get_permission_mask()==(1 if stock else 3)

  helpers.run_case(stock,candidate=candidate,bridge=host_heartbeat_bridge,input_clock=True,
                   sdsu_words=(owner.LONG_WORD,owner.STOCK_WORD),firmware_override=complete_firmware(),
                   fingerprint_frames=[(0x2FF,bytes(4),0)],manual_selection=candidate==CAR.TOYOTA_PRIUS_RETROFIT,manual_filter_length=4,
                   filter_advert=True,physical_frames=physical,on_sample=observe)
