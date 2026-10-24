"""Mazda radar diagnostic-session ownership, separate from the acceleration controller.

Ported from the approach in zoompilot/opendbc (opendbc/car/mazda/radar_session.py),
adapted to this branch's structure.

Why this exists: the previous implementation fired a blocking UDS programming request
from CarInterface.init(), before any CAN had been read. That lands inside the Forward
Sensing Camera's cold-boot radar-presence check, and a radar that goes silent during
that check makes the FSC latch "Smart City Brake Support Malfunction". It also gave a
single one-shot attempt with no way to retry, no way to notice the radar coming back,
and no ordered restoration.

This runs in the control loop instead, so the teardown can wait for the camera to settle
and for the car to be stopped.

State machine:
  STOCK      radar is the ACC master; we transmit nothing
  SILENCING  programming session requested at 2 Hz, waiting for the radar to go quiet
  SILENCED   radar is ours; tester-present holds the session, we transmit replacement frames
  HANDBACK   default session requested, waiting for stock traffic to resume

Known limitation, stated plainly: our own replacement CRZ_INFO frames may be echoed back
into the bus-0 parser. The radar-liveness witness is therefore only trusted in STOCK and
SILENCING, where we transmit no CRZ_INFO. Once SILENCED, the session is held by the
tester-present cadence rather than by continuous observation.
"""
from __future__ import annotations

from enum import StrEnum

from opendbc.car import make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.mazda.values import CarControllerParams

RADAR_ADDR = 0x764
RADAR_BUS = 0


def create_radar_session_msg(session_type: int) -> CanData:
  """Single-frame UDS DiagnosticSessionControl. Non-blocking, unlike IsoTpParallelQuery."""
  return CanData(RADAR_ADDR, bytes([2, uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, session_type, 0, 0, 0, 0, 0]), RADAR_BUS)


class RadarSessionState(StrEnum):
  STOCK = "stock"
  SILENCING = "silencing"
  SILENCED = "silenced"
  HANDBACK = "handback"


class RadarSessionManager:
  def __init__(self, moving_takeover: bool = False):
    # A takeover while moving pulls the radar out from under a system that may be actively
    # braking. Default to parked only; this is the conservative path.
    self.moving_open = moving_takeover
    self.state = RadarSessionState.STOCK
    self.state_frames = 0
    self.silencing_failed = False
    self.handback_completed = False
    self.handback_failed = False
    self.programming_sent = False
    self.default_sent = False
    self.stock_frames = 0
    self.replacement_active = False
    self.diagnostic_message: CanData | None = None
    self.attempt_moving = False
    self.frame = 0

  def _transition(self, state: RadarSessionState, reason: str) -> None:
    if state != self.state:
      carlog.info({"event": "mazdaRadarSession", "from": str(self.state), "to": str(state), "reason": reason})
      self.state = state
      self.state_frames = 0
      if state == RadarSessionState.SILENCING:
        self.programming_sent = False
      elif state == RadarSessionState.HANDBACK:
        self.default_sent = False
        self.stock_frames = 0

  def _silencing_gave_up(self, reason: str) -> None:
    # A moving attempt that failed says nothing about the parked path, which is the one
    # every car has on record: leave that open. Parked, a failure is definitive for the drive.
    if self.attempt_moving:
      carlog.warning({"event": "mazdaRadarMovingTakeoverClosed", "reason": reason})
      self.moving_open = False
    else:
      self.silencing_failed = True
      carlog.error({"event": "mazdaRadarSilencingFailed", "reason": reason})
    self._transition(RadarSessionState.HANDBACK, f"programming {reason}")

  def update(self, fsc_settled: bool, stock_radar_alive: bool, handback: bool,
             standstill: bool, *, bus_healthy: bool = True, frame: int = 0,
             stock_engaged: bool = False) -> RadarSessionState:
    self.frame = frame
    self.diagnostic_message = None
    self.state_frames += 1
    self.stock_frames = self.stock_frames + 1 if bus_healthy and stock_radar_alive else 0

    limit_frames = CarControllerParams.RADAR_SESSION_LIMIT_FRAMES
    restore_frames = CarControllerParams.RADAR_RESTORE_FRAMES

    if handback:
      if self.state in (RadarSessionState.SILENCING, RadarSessionState.SILENCED):
        self._transition(RadarSessionState.HANDBACK, "requested")
    else:
      self.handback_completed = False

    # The takeover gate: camera settled past its boot radar-presence check, and no stock
    # ACC engagement to pull the radar out from under.
    gate_open = fsc_settled and not stock_engaged

    if self.state == RadarSessionState.HANDBACK:
      if self.default_sent and self.stock_frames >= restore_frames:
        self.handback_completed = handback
        self.handback_failed = False
        self._transition(RadarSessionState.STOCK, "stock traffic restored")
      elif self.state_frames >= limit_frames and not self.handback_failed:
        # A timeout is a failure, never proof that stock control recovered.
        self.handback_failed = True
        carlog.error({"event": "mazdaRadarRestoreFailed", "reason": "stock traffic did not recover"})
      if self.state == RadarSessionState.HANDBACK and not self.handback_failed and \
         (not self.default_sent or not stock_radar_alive) and self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.DEFAULT)
        self.default_sent = True

    elif not handback:
      takeover_allowed = standstill or self.moving_open

      if self.state == RadarSessionState.STOCK and gate_open and bus_healthy and not self.silencing_failed:
        if takeover_allowed and stock_radar_alive:
          self.attempt_moving = not standstill
          self._transition(RadarSessionState.SILENCING, "moving takeover" if self.attempt_moving else "parked takeover")

      if self.state == RadarSessionState.SILENCING:
        if not bus_healthy or not gate_open or not takeover_allowed:
          # A request may already be in flight: undo it rather than abandoning it.
          self._transition(RadarSessionState.HANDBACK if self.programming_sent else RadarSessionState.STOCK,
                           "takeover prerequisites lost")
        elif self.programming_sent and not stock_radar_alive:
          self._transition(RadarSessionState.SILENCED, "requested radar silence")
        elif self.state_frames >= limit_frames:
          self._silencing_gave_up("timed out")
        elif self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
          self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING)
          self.programming_sent = True

      if self.state == RadarSessionState.SILENCED and self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = make_tester_present_msg(RADAR_ADDR, RADAR_BUS, suppress_response=True)

    self._update_replacement(stock_radar_alive, bus_healthy)
    return self.state

  def _update_replacement(self, stock_radar_alive: bool, bus_healthy: bool) -> None:
    """Only transmit replacement frames when the radar is genuinely ours.

    This is the safety property that matters: if the stock radar is still awake, two ACC
    masters would be commanding the PCM at 50 Hz each.
    """
    if self.state == RadarSessionState.STOCK:
      self.replacement_active = False
    elif self.state == RadarSessionState.SILENCED:
      self.replacement_active = True
    elif self.state == RadarSessionState.HANDBACK:
      # Hold replacement traffic through the handback while the radar is still quiet: a gap
      # the camera can see is worse than a slightly late stop.
      self.replacement_active = bus_healthy and self.programming_sent and not stock_radar_alive
    else:
      self.replacement_active = False
