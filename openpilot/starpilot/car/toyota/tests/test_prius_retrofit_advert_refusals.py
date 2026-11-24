"""Focused current EPS73 joins: physical source refusal/recovery and AOL OFF."""
import pytest
from opendbc.car import Bus
from opendbc.car.toyota import prius_advert_filter, prius_retrofit_advert as owner
from opendbc.car.toyota.values import CAR
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers
from openpilot.starpilot.car.toyota.tests.test_prius_retrofit_advert_fw import complete_firmware
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge

source_matched_native=helpers.source_matched_native
daemon_gc_scope=helpers.daemon_gc_scope


@pytest.mark.parametrize('stock',(False,True))
@pytest.mark.parametrize('fault',('main-drop','eps-checksum','ordinary-aol-off'))
def test_eps73_current_source_refusal_recovery_and_ordinary(stock,fault,host_heartbeat_bridge):
  tick=0

  def physical(packer,**fields):
    nonlocal tick
    tick+=1
    fields['sdsu']=None
    frames=helpers.physical(packer,**fields)
    if 260<=tick<311:
      if fault=='main-drop':
        frames=[f for f in frames if f[0]!=0x1D3 or f[2]!=0]
      elif fault=='eps-checksum':
        frames=[(a,data[:-1]+bytes([data[-1]^1]),bus) if a==0x262 and bus==0 else (a,data,bus) for a,data,bus in frames]
    frames.extend(((0x2FF,bytes.fromhex('80007409'),0),(0x2FF,bytes.fromhex('80007409'),2)))
    return frames

  def expected_rx(address,data,bus,default,cp):
    if fault=='eps-checksum' and 260<=tick<311 and address==0x262 and bus==0:
      assert owner.configured(cp) and len(data)==8
      return False
    return default

  def observe(tick,host,cs,cc,safety,native):
    parser=host.CI.can_parsers[Bus.pt]
    assert isinstance(parser,prius_advert_filter.PriusAdvertCANParser)
    assert 'SDSU' not in parser.vl and 0x2FF not in parser.message_states
    assert host.CI.CS.eps_torque_scale==.73 and owner.configured(host.CP)
    if fault!='ordinary-aol-off' and 301<=tick<308:
      assert not cc.latActive and not cc.longActive and safety.aol_get_permission_mask()==0
    if tick>=480 or fault=='ordinary-aol-off' and tick>=220:
      assert cs.canValid and not cs.canTimeout and cc.latActive and cc.longActive==(not stock)
      assert safety.aol_get_permission_mask()==(1 if stock else 3)
    if fault=='ordinary-aol-off' and tick>=220:
      assert not host.aol_card_intent.settings.enabled and not host.aol_card_intent.allowed_latch
      assert cs.cruiseState.enabled and cc.enabled

  helpers.run_case(stock,'nominal' if fault=='ordinary-aol-off' else 'main_lease',candidate=CAR.TOYOTA_PRIUS_RETROFIT,
                   bridge=host_heartbeat_bridge,input_clock=True,aol=fault!='ordinary-aol-off',
                   sdsu_words=(owner.LONG_WORD,owner.STOCK_WORD),firmware_override=complete_firmware(),
                   fingerprint_frames=[(0x2FF,bytes(4),0)],manual_selection=True,manual_filter_length=4,filter_advert=True,
                   physical_frames=physical,native_rx_expected=expected_rx,on_sample=observe)
