from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.hyundai.ecu_startup import Outcome
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.tests.test_g90_continuity import ManualKeeper
from openpilot.selfdrive.car.card import Car


def prepared_classic_card(cp, *, now_ns=10_000_000_000):
  if not CarInterface.startup_required(cp):
    return Car(CI=CarInterface(cp), RI=RadarInterface(cp))
  from opendbc.car.hyundai.ordinary_classic_startup import OrdinaryClassicStartup

  sent = []
  diagnostic_commands = []
  owner = CarInterface.startup_owner(cp, (list, sent.extend), requested=True)
  assert isinstance(owner, OrdinaryClassicStartup)
  owner.clock = lambda: now_ns / 1e9
  owner.keeper_factory = ManualKeeper

  def query(sender, receiver, bus, addresses, requests, responses):
    request = requests[0]
    diagnostic_commands.append(request)
    assert bus == 0 and addresses == [(0x7D0, None)]
    assert request in (b'\x10\x03', b'\x28\x83\x01', b'\x28\x00\x01', b'\x10\x01')

    def receive(timeout):
      sender([(0x7D0, bytes([len(request)]) + request + bytes(7 - len(request)), bus)])
      return {} if request == b'\x28\x83\x01' else {(0x7D0, None): b''}

    return SimpleNamespace(get_data=receive)

  owner._query = query
  try:
    with patch('time.clock_gettime_ns', return_value=now_ns), patch('time.sleep'):
      selected = owner.prepare(admission=lambda: True)
      assert owner.outcome is Outcome.SENT_UNCONFIRMED and sent
      assert diagnostic_commands == [b'\x10\x03', b'\x28\x83\x01']
      assert selected.to_dict() == cp.to_dict()
      card = Car(CI=CarInterface(selected), RI=RadarInterface(selected), startup_owner=owner)
    card._classic_startup_requests = tuple(diagnostic_commands)
    assert owner.ready and owner.published and owner.prepared_for(card.CP)
    return card
  except BaseException:
    owner.close()
    raise
