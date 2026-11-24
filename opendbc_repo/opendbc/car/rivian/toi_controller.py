from enum import IntEnum


TOI_MAX_ANGLE_DEG = 90
TOI_REARM_ANGLE_DEG = 80
TOI_MAX_ANGLE_FRAMES = 89
TOI_ACK_FRAMES = 2
TOI_RECOVERY_TIMEOUT_FRAMES = 50


class ToiState(IntEnum):
  INACTIVE = 0
  TORQUE = 1
  RELEASING = 2
  REARMING = 3
  PREARMING = 4
  ACTIVATING = 5
  HIGH_ANGLE_LOCKOUT = 6


class ToiController:
  """Handshake Rivian's torque-overlay request with the EPAS status feedback."""

  def __init__(self):
    self.state = ToiState.INACTIVE
    self.high_angle_frames = 0
    self.ack_frames = 0
    self.recovery_frames = 0
    self.recovery_failed = False

  @property
  def recovering(self) -> bool:
    return self.state in (ToiState.RELEASING, ToiState.REARMING, ToiState.HIGH_ANGLE_LOCKOUT)

  def _reset(self) -> None:
    self.state = ToiState.INACTIVE
    self.high_angle_frames = 0
    self.ack_frames = 0
    self.recovery_frames = 0
    self.recovery_failed = False

  def _start_release(self) -> None:
    self.state = ToiState.RELEASING
    self.high_angle_frames = 0
    self.ack_frames = 0
    self.recovery_frames = 0

  def _start_rearm(self) -> None:
    self.state = ToiState.REARMING
    self.ack_frames = 0
    self.recovery_frames = 0

  def _start_prearm(self) -> None:
    self.state = ToiState.PREARMING
    self.ack_frames = 0
    self.recovery_frames = 0

  def _start_activation(self) -> None:
    self.state = ToiState.ACTIVATING
    self.ack_frames = 0
    self.recovery_frames = 0

  def _start_high_angle_lockout(self) -> None:
    self.state = ToiState.HIGH_ANGLE_LOCKOUT
    self.ack_frames = 0
    self.recovery_frames = 0

  def _finish_activation(self) -> None:
    self.state = ToiState.TORQUE
    self.high_angle_frames = 0
    self.ack_frames = 0
    self.recovery_frames = 0
    self.recovery_failed = False

  def update(self, requested: bool, high_angle: bool, toi_fault: bool,
             toi_active: bool, toi_unavailable: bool, prearming: bool = False,
             high_angle_rearm: bool | None = None, hold_high_angle_release: bool = False) -> tuple[bool, bool]:
    """Return the request bit and whether non-zero torque may be sent."""
    high_angle_rearm = not high_angle if high_angle_rearm is None else high_angle_rearm
    if not requested:
      self._reset()
      return False, False

    # A live angle->torque selection ramps the rate-limited torque command while
    # EAC is still active. ToiActive cannot acknowledge until angle releases, so
    # prearm must be distinguished from an ordinary zero-torque rearm.
    if self.state == ToiState.INACTIVE:
      if prearming:
        self._start_prearm()
      else:
        self._start_rearm()
    elif prearming and self.state == ToiState.REARMING:
      self._start_prearm()
    elif not prearming and self.state == ToiState.PREARMING:
      self._start_activation()

    if self.state == ToiState.TORQUE:
      if toi_fault or toi_unavailable:
        self._start_release()
      else:
        self.high_angle_frames = self.high_angle_frames + 1 if high_angle else 0
        if self.high_angle_frames > TOI_MAX_ANGLE_FRAMES:
          self._start_release()

    if self.state == ToiState.RELEASING:
      self.recovery_frames += 1
      released = not toi_active and not toi_fault and not toi_unavailable
      self.ack_frames = self.ack_frames + 1 if released else 0
      self.recovery_failed |= self.recovery_frames >= TOI_RECOVERY_TIMEOUT_FRAMES
      if self.ack_frames >= TOI_ACK_FRAMES:
        if hold_high_angle_release and not high_angle_rearm:
          self._start_high_angle_lockout()
        elif prearming:
          self._start_prearm()
        else:
          self._start_rearm()
      return False, False

    if self.state == ToiState.HIGH_ANGLE_LOCKOUT:
      if toi_fault or toi_unavailable:
        self.recovery_frames += 1
        self.recovery_failed |= self.recovery_frames >= TOI_RECOVERY_TIMEOUT_FRAMES
      elif not high_angle_rearm:
        self.recovery_frames = 0
      elif prearming:
        self._start_prearm()
      else:
        self._start_rearm()
      return False, False

    if self.state == ToiState.PREARMING:
      if toi_fault or toi_unavailable:
        self._start_release()
        return False, False

      self.high_angle_frames = self.high_angle_frames + 1 if high_angle else 0
      if self.high_angle_frames > TOI_MAX_ANGLE_FRAMES:
        self._start_release()
        return False, False
      return True, True

    if self.state == ToiState.ACTIVATING:
      if toi_fault or toi_unavailable:
        self._start_release()
        return False, False

      self.high_angle_frames = self.high_angle_frames + 1 if high_angle else 0
      if self.high_angle_frames > TOI_MAX_ANGLE_FRAMES:
        self._start_release()
        return False, False

      self.recovery_frames += 1
      self.ack_frames = self.ack_frames + 1 if toi_active else 0
      if self.ack_frames >= TOI_ACK_FRAMES:
        self._finish_activation()
      elif self.recovery_frames >= TOI_RECOVERY_TIMEOUT_FRAMES:
        self.recovery_failed = True
        self._start_release()
        return False, False
      return True, True

    if self.state == ToiState.REARMING:
      if toi_fault or toi_unavailable:
        self._start_release()
        return False, False

      self.recovery_frames += 1
      self.ack_frames = self.ack_frames + 1 if toi_active else 0
      if self.ack_frames >= TOI_ACK_FRAMES:
        self._finish_activation()
      elif self.recovery_frames >= TOI_RECOVERY_TIMEOUT_FRAMES:
        # Drop the request before retrying instead of leaving an unacknowledged
        # torque command asserted indefinitely.
        self.recovery_failed = True
        self._start_release()
        return False, False
      return True, False

    return True, True
