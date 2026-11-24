"""The actual pandad C++ policy must recognize every admitted host/native profile."""
import pytest
from opendbc.car.toyota.stock_aol import WORDS
from openpilot.cereal import messaging
from openpilot.starpilot.car.toyota.aol import AOL_WORDS
from openpilot.starpilot.car.toyota.tests.test_toyota_cpp_transport import host_heartbeat_bridge as host_heartbeat_bridge


def test_compiled_cpp_profile_inventory_matches_host_for_every_uint16(host_heartbeat_bridge):
  assert {word for word in range(65536) if host_heartbeat_bridge.admits(word)} == AOL_WORDS


@pytest.mark.parametrize('word', sorted(WORDS | {2121, 2633, 6217}))
def test_new_stock_and_secoc_profiles_negotiate_real_cpp_requests(word, host_heartbeat_bridge):
  peer = host_heartbeat_bridge
  peer.reset_host()
  event = messaging.new_message('aolAxisState', valid=True, logMonoTime=1_000_000_000)
  event.aolAxisState.qualified = True
  event.aolAxisState.sessionId = 'toyota-profile-inventory'
  event.aolAxisState.observedMonoTime = 1_000_000_000
  event.aolAxisState.validUntilMonoTime = 1_030_000_000
  event.aolAxisState.desiredLateral = True
  event.aolAxisState.desiredLongitudinal = word == 6217
  raw = event.to_bytes()
  first = peer.plan_host(2, word, 0, 0, 1_000_000_000, raw, len(raw), b'', 0, False, 1)
  assert first & 255 == 0
  peer.complete_host(0, 0)
  plan = peer.plan_host(2, word, 0, 0, 1_010_000_000, raw, len(raw), b'', 0, False, 1)
  assert plan & 255 == (3 if word == 6217 else 1)
  expired = peer.plan_host(2, word, 0, 0, 1_030_000_001, raw, len(raw), b'', 0, False, 1)
  assert expired & 255 == 0
