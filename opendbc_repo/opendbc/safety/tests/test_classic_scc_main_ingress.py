"""Physical MAIN may authorize only stock SCC LAT under existing profile guards."""
import pytest

from opendbc.safety.tests import test_hyundai_classic_scc_aol as fixture
from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket


@pytest.mark.parametrize('word', [0xC00, 0xC02, 0xC41, 0xE02, 0xC0A])
def test_fresh_physical_main_authorizes_only_lateral(word):
  case = fixture.TestHyundaiClassicSccAol(methodName='runTest')
  case.setUp()
  try:
    case.reset(word)
    for _ in range(6):
      case.feed(word, main=False, source='BCM_PO_11')
    assert case.request(1) == 0
    case.request(0)
    # Raw CLU11 MAIN press captured in Sonata route09 at42.073543942s.
    assert case.safety.safety_rx_hook(make_CANPacket(0x4F1, 0, bytes.fromhex('28557af0')))
    assert case.request(1) == 1
    assert not case.safety.get_controls_allowed()
    assert case.request(3) == 1
    assert not case.safety.safety_tx_hook(case.packer.make_can_msg_safety('SCC12', 2 if word & 8 else 0,
      {'ACCMode': 1, 'aReqRaw': 1, 'aReqValue': 1}))
    case.safety.set_aol_test_heartbeat(False)
    assert case.request(1) == 0
  finally:
    case.tearDown()


@pytest.mark.parametrize('wrong_bus', [1, 2])
def test_main_wrong_bus_does_not_authorize_lateral(wrong_bus):
  case = fixture.TestHyundaiClassicSccAol(methodName='runTest')
  case.setUp()
  try:
    case.reset(0xC02)
    for _ in range(6):
      case.feed(0xC02, source='BCM_PO_11')
    case.safety.safety_rx_hook(make_CANPacket(0x4F1, wrong_bus, bytes.fromhex('28557af0')))
    assert case.request(1) == 0
  finally:
    case.tearDown()


@pytest.mark.parametrize('word', [0xC00, 0xC02, 0xC0A])
def test_held_at_boot_main_needs_release_before_authorization(word, monkeypatch):
  case = fixture.TestHyundaiClassicSccAol(methodName='runTest')
  case.setUp()
  try:
    case.reset(word)
    original_rx = case.rx
    held = [True]

    def physical_rx(name, values, **kwargs):
      if name == 'CLU11':
        values = dict(values, CF_Clu_CruiseSwMain=int(held[0]))
      return original_rx(name, values, **kwargs)
    monkeypatch.setattr(case, 'rx', physical_rx)
    for _ in range(6):
      case.feed(word, source='BCM_PO_11')
    assert case.request(1) == 0
    case.request(0)
    held[0] = False
    case.feed(word, source='BCM_PO_11')
    held[0] = True
    case.feed(word, source='BCM_PO_11')
    assert case.request(1) == 1
    case.safety.set_aol_test_heartbeat(False)
    assert case.request(1) == 0
  finally:
    case.tearDown()
