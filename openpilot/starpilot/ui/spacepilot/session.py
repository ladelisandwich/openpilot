"""Parked game lifecycle: observe, request supported Offroad, then verify shutdown.

The game never starts or stops a managed process itself. A successful onroad
entry leaves the existing DriveStateOwner in Offroad even after game exit; only
a later explicit drive-state selection or verified physical key-off releases it.
"""

from enum import StrEnum
import time

from openpilot.starpilot.parked_vehicle import ParkedVehicleReader
from openpilot.starpilot.ui.spacepilot.guard import ParkGate, read_offroad_witness, read_witness, sample_clocks
from openpilot.starpilot.ui.spacepilot.state import WheelInput
from openpilot.starpilot.ui.spacepilot.ignition import IgnitionReturn

TRANSITION_NS = 8_000_000_000


class Stage(StrEnum):
  VERIFYING = 'verifying'
  STOPPING = 'stopping'
  PLAYING = 'playing'
  CLOSED = 'closed'


class GameSession:
  def __init__(self, ui, drive_owner, *, native_device: bool, reader_factory=ParkedVehicleReader):
    self.ui, self.drive_owner, self.native_device = ui, drive_owner, native_device
    self.stage = Stage.VERIFYING
    self.started_ns = time.monotonic_ns()
    self.requested_offroad = False
    self.ignition_return: IgnitionReturn | None = None
    self.vehicle = None
    self.wheel = WheelInput()
    self.gate = ParkGate()
    self.reason = ''
    self._reader_factory = reader_factory
    self._was_ignition = None
    self._request_revision = None
    self._clock_offset = None

  def _clocks(self):
    mono, boot = sample_clocks()
    offset = boot - mono
    if self._clock_offset is not None and abs(offset - self._clock_offset) > 5_000_000:
      raise ValueError('Device resumed; re-enter SpacePilot')
    self._clock_offset = offset
    return mono, boot

  def _ignition(self):
    return any(p.ignitionLine or p.ignitionCan for p in self.ui.sm['pandaStates'])

  def _onroad_witness(self):
    mono, boot = self._clocks()
    return read_witness(self.ui, mono, boot, c4=self.native_device)

  def step(self) -> bool:
    """Return one debounced boost only after actual process/noOutput admission."""
    if self.stage == Stage.CLOSED:
      return False
    try:
      mono, boot = self._clocks()
      ignition = self._ignition()
      if self._was_ignition is not None and ignition != self._was_ignition:
        # Never reuse a parser or old admission across a physical ignition cycle.
        raise ValueError('Vehicle ignition changed')
      self._was_ignition = ignition
      if self.stage != Stage.PLAYING and mono - self.started_ns > TRANSITION_NS:
        raise ValueError('Driving services did not finish entering parked game mode')

      if self.vehicle is None and ignition:
        sm = self.ui.sm
        if (not sm.valid['carParams'] or sm.recv_frame['carParams'] <= self.ui.started_frame or
            not 0 < int(sm.logMonoTime['carParams']) <= mono):
          raise ValueError('Current vehicle identity is unavailable')
        self.vehicle = self._reader_factory(sm['carParams'])
        # Cold imports/parser construction can outlast this frame's UI sample.
        # Let the normal collector refresh next frame before any admission,
        # request or input; all original dwell and freshness checks still run.
        return False
      samples = self.vehicle.poll() if self.vehicle is not None else ()
      mono, boot = self._clocks()
      vehicle_ready = bool(self.vehicle is not None and self.vehicle.ready(mono, boot))

      if self.stage in (Stage.STOPPING, Stage.PLAYING) and ignition and not vehicle_ready:
        raise ValueError('Physical Park or fresh CAN was lost')

      if self.stage == Stage.VERIFYING and self.ui.started:
        witness = self._onroad_witness()
        if witness is None:
          raise ValueError('Park and disengage before entering SpacePilot')
        if not self.gate.update(witness) or not vehicle_ready:
          return False
        state = self.drive_owner.snapshot()
        if not state.available:
          raise ValueError('Drive-state owner is unavailable')
        def parked_now():
          now, boot_now = self._clocks()
          return bool(self.stage == Stage.VERIFYING and self._onroad_witness() is not None and
                      self.vehicle is not None and self.vehicle.ready(now, boot_now))
        result = self.drive_owner.request('offroad', expected_revision=state.revision,
                                          authorized=parked_now, override_allowed=parked_now)
        if str(result.mode) != 'offroad' or not result.available:
          raise ValueError('Offroad request was not accepted')
        self._request_revision = result.revision
        self.requested_offroad = True
        self.ignition_return = IgnitionReturn(self.ui, self.drive_owner, result.revision, native_device=self.native_device)
        self.stage = Stage.STOPPING
        self.started_ns = mono
        self.gate.reset()
        self.wheel = WheelInput()
        return False

      if self.requested_offroad:
        state = self.drive_owner.snapshot()
        if not state.available or str(state.mode) != 'offroad' or state.revision != self._request_revision:
          raise ValueError('Drive mode changed; SpacePilot has ended')
      if self.stage == Stage.STOPPING and self.ui.started:
        # No game/input until all normal services have actually stopped. Keep
        # the original fresh physical admission while they are still onroad.
        if self._onroad_witness() is None or not vehicle_ready:
          raise ValueError('Physical parked evidence changed during shutdown')
        return False

      witness = read_offroad_witness(self.ui, mono, boot, native_device=self.native_device, vehicle=self.vehicle)
      ready = self.gate.update(witness)
      if self.stage == Stage.PLAYING and not ready:
        raise ValueError('Parked game evidence is no longer trustworthy')
      if not ready:
        return False
      if self.stage != Stage.PLAYING:
        self.stage = Stage.PLAYING
        # Pre-admission presses/releases cannot arm a wheel action.
        self.wheel = WheelInput(self.vehicle.sample.source_boot_ns if self.vehicle is not None else 0)
        return False
      fire = False
      for sample in samples:
        fire = self.wheel.update(sample.source_boot_ns, sample.buttons, mono / 1e9) or fire
      return fire
    except (AttributeError, KeyError, OSError, RuntimeError, TypeError, ValueError, ImportError) as exc:
      self.close(str(exc))
      return False

  def playable(self) -> bool:
    """Touch/controller/render must revalidate, never rely on last frame alone."""
    if self.stage != Stage.PLAYING:
      return False
    try:
      mono, boot = self._clocks()
      if self._ignition() != self._was_ignition:
        return False
      if self.requested_offroad:
        state = self.drive_owner.snapshot()
        if not state.available or str(state.mode) != 'offroad' or state.revision != self._request_revision:
          return False
      return self.gate.update(read_offroad_witness(self.ui, mono, boot, native_device=self.native_device, vehicle=self.vehicle))
    except (AttributeError, KeyError, OSError, RuntimeError, TypeError, ValueError):
      return False

  def close(self, reason: str = '') -> None:
    self.reason = reason
    self.stage = Stage.CLOSED
    if self.vehicle is not None:
      self.vehicle.close()
      self.vehicle = None
    self.gate.reset()
    self.wheel = WheelInput()
    # Deliberately no request(Auto/Onroad): closing/crashing must not restart
    # driving services or restore engagement processing behind a game button.
