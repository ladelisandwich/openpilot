from copy import deepcopy
from typing import Any

from opendbc.car import Bus
from opendbc.can.parser import MAX_BAD_COUNTER
from opendbc.car.hyundai.classic_long_aol import CLASSIC_LONG_IDS, qualified, ordinary_word, aol_word
from opendbc.car.hyundai.classic_scc_aol import qualified as stock_qualified, stock_word, aol_word as stock_aol_word
from opendbc.car.hyundai.ecu_continuity import ContinuousECUStartup
from opendbc.car.hyundai.ecu_startup import Outcome, stock_copy
from opendbc.car.hyundai.g90_startup import G90Startup
from opendbc.car.hyundai.values import CAR, HyundaiFlags

CARS = CLASSIC_LONG_IDS - {CAR.GENESIS_G90}


def required(cp):
  return cp.carFingerprint in CARS and cp.openpilotLongitudinalControl


class OrdinaryClassicStartup(G90Startup):
  def __init__(self, cp, callbacks):
    if not required(cp) or not qualified(cp) or cp.alternativeExperience != 0 or cp.safetyConfigs[0].safetyParam != ordinary_word(cp):
      raise ValueError('Exact unmarked classic LONG configuration required before publication')
    ContinuousECUStartup.__init__(self, cp, callbacks, address=0x7D0, bus=0, label='Classic LONG')
    self.stock_cp = stock_copy(cp)

  def continuity_required(self):
    return required(self.cp)

  def prepare(self, *, admission):
    super().prepare(admission=admission)
    if self.outcome in (Outcome.STOCK_UNTOUCHED, Outcome.STOCK_RESTORED):
      self.cp = self.stock_cp.as_reader().as_builder()
      self.prepared_cp = deepcopy(self.cp.to_dict())
    return self.cp

  def sources_current(self, ci, now_ns):
    if self.outcome not in (Outcome.PENDING, Outcome.SENT_UNCONFIRMED):
      return True
    if not ci.CS.out.canValid or ci.CS.out.canTimeout:
      return False
    parser = ci.can_parsers[Bus.pt]
    gas = 'E_EMS11' if ci.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV) else 'EMS16'
    for name in (gas, 'WHL_SPD11', 'TCS13', 'MDPS12', 'CLU11'):
      source = parser.message_states.get(parser.dbc.name_to_msg[name].address)
      if (
        source is None
        or not source.timestamps
        or source.counter_fail >= MAX_BAD_COUNTER
        or not self.floor_ns < int(source.timestamps[-1]) <= now_ns
        or now_ns - int(source.timestamps[-1]) > source.timeout_threshold
      ):
        return False
    return True

  def before_control(self, **context):
    self.check()
    return self.outcome is not Outcome.PENDING and (self.outcome is not Outcome.SENT_UNCONFIRMED or
      (context['configured'] and context['sources_current'] and context['control_current']))

  def finalize_aol_configuration(self, ci):
    if self.published or self.closed or ci is not self.ci or not self.ready:
      raise RuntimeError('Classic LONG finalization requires configured startup')
    if self.prepared_for(ci.CP):
      return
    if self.outcome in (Outcome.PENDING, Outcome.SENT_UNCONFIRMED) and qualified(ci.CP, marked_only=True):
      before_word, after_word = ordinary_word(ci.CP), aol_word(ci.CP)
    elif self.outcome in (Outcome.STOCK_UNTOUCHED, Outcome.STOCK_RESTORED) and stock_qualified(ci.CP, marked_only=True):
      before_word, after_word = stock_word(ci.CP), stock_aol_word(ci.CP)
    else:
      raise RuntimeError('Classic startup does not admit this AOL configuration')
    expected: Any = deepcopy(self.prepared_cp)
    configs = expected.get('safetyConfigs', [])
    if len(configs) != 1 or configs[0]['safetyModel'] != 'hyundai' or configs[0]['safetyParam'] != before_word or expected['alternativeExperience'] != 0:
      raise RuntimeError('Classic prepared safety profile cannot be finalized')
    configs[0]['safetyParam'] = after_word
    expected['alternativeExperience'] = 32
    if ci.CP.to_dict() != expected:
      raise RuntimeError('Classic finalization changed unrelated CarParams')
    self.prepared_cp = expected
