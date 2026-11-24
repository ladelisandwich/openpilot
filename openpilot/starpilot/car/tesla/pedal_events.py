"""Original Pre-AP pedal chimes/regen prompt from the existing custom companion."""
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import Alert, ET, Priority, VisualAlert
from opendbc.car.tesla.preap.pedal_profile import qualified


class PedalStatusConsumer:
  def __init__(self, cp):
    self.enabled = qualified(cp)
    self.session = None
    self.previous = False
    self.stamp = 0

  def update(self, sm, *, now_ns, session, car_state=None, source_ns=None, source_valid=False):
    try:
      if not self.enabled or not session:
        return []
      cs = sm['carState'] if car_state is None else car_state
      status = sm['starpilotCarState']
      stamp = sm.logMonoTime['carState'] if source_ns is None else source_ns
      valid = (sm.valid['carState'] and sm.alive['carState']) if car_state is None else source_valid
      if (not valid or not sm.valid['starpilotCarState'] or not sm.alive['starpilotCarState'] or
          not cs.canValid or cs.canTimeout or not 0 < stamp == sm.logMonoTime['starpilotCarState'] ==
          status.sourceCarStateMonoTime <= now_ns <= stamp + 100_000_000):
        return []
      if session != self.session:
        self.session, self.previous, self.stamp = session, False, 0
      if stamp <= self.stamp:
        return []
      self.stamp = stamp
      active = bool(cs.cruiseState.enabled and status.pedalLongActive)
      alerts = []
      if active != self.previous:
        alerts.append(pedal_alert('engaged' if active else 'disengaged'))
      if status.pedalMaxRegen and active:
        alerts.append(pedal_alert('maxRegen'))
      self.previous = active
      return alerts
    except (AttributeError, KeyError, TypeError, ValueError, RuntimeError):
      return []


def pedal_alert(kind):
  regen = kind == 'maxRegen'
  active = kind == 'engaged'
  alert = Alert('Max Regen Being Used' if regen else 'Pedal Cruise Engaged' if active else 'Pedal Cruise Disengaged', '',
    log.SelfdriveState.AlertStatus.userPrompt if regen else log.SelfdriveState.AlertStatus.normal,
    log.SelfdriveState.AlertSize.small, Priority.HIGH if regen else Priority.LOW,
    VisualAlert.steerRequired if regen else VisualAlert.none,
    log.SelfdriveState.AudibleAlert.prompt if regen else log.SelfdriveState.AudibleAlert.engage if active else
    log.SelfdriveState.AudibleAlert.disengage, 2. if regen else .8)
  alert.event_type = ET.WARNING
  alert.alert_type = f'teslaPreapPedal/{kind}/{ET.WARNING}'
  return alert
