"""Transport receipts must neither revoke hardware identity nor refresh RX leases."""
import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.toyota import camera_2aa_filter, dsu_bypass, prius_advert_filter, prius_sdsu
from opendbc.car.toyota.values import CAR, DBC

PARSERS = (prius_advert_filter.PriusAdvertCANParser, prius_sdsu.PriusSDSUCANParser,
           camera_2aa_filter.CameraTwoAACANParser, dsu_bypass.BypassCANParser)


def ambiguous(parser):
  return getattr(parser, 'hardware_ambiguous', getattr(parser, 'ambiguous', False))


@pytest.mark.parametrize('parser_type', PARSERS)
@pytest.mark.parametrize('bus', (128, 130, 132, 192, 194, 196))
def test_receipts_do_not_change_identity_or_supply_main_eps(parser_type, bus):
  dbc = DBC[CAR.TOYOTA_PRIUS][Bus.pt]
  packer = CANPacker(dbc)
  parser = parser_type(dbc, [('PCM_CRUISE_2', 10), ('EPS_STATUS', 25)], 0)
  main = packer.make_can_msg('PCM_CRUISE_2', 0, {'MAIN_ON': 1})
  eps = packer.make_can_msg('EPS_STATUS', 0, {'LKA_STATE': 1})
  receipts = [(a, data, bus) for a, data, _ in (main, eps)]
  receipts += [(a, bytes(n), bus) for a, n in ((0x201, 6), (0x2AA, 8), (0x2FF, 4), (0x2FF, 8))]
  parser.update([(2_000_000_000, receipts)])
  assert not ambiguous(parser) and not parser.main_valid and not parser.eps_valid
  parser.update([(2_010_000_000, [main, eps])])
  stamp = parser.ts_nanos['PCM_CRUISE_2']['MAIN_ON']
  assert not ambiguous(parser) and parser.main_valid and parser.eps_valid
  parser.update([(2_020_000_000, receipts)])
  assert not ambiguous(parser) and parser.main_valid and parser.eps_valid
  assert parser.ts_nanos['PCM_CRUISE_2']['MAIN_ON'] == stamp


@pytest.mark.parametrize('parser_type', PARSERS)
@pytest.mark.parametrize('bus', (0, 1, 2, 4))
def test_real_conflicting_hardware_stays_latched(parser_type, bus):
  dbc = DBC[CAR.TOYOTA_PRIUS][Bus.pt]
  parser = parser_type(dbc, [('PCM_CRUISE_2', 10), ('EPS_STATUS', 25)], 0)
  parser.update([(2_000_000_000, [(0x201, bytes(6), bus)])])
  assert ambiguous(parser)
  parser.update([(2_010_000_000, [])])
  assert ambiguous(parser)
