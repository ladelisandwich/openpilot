"""Torque Interceptor (TI1 by default), on the harness's AUX lines (panda bus 1 in OBD mode).

The TI sits in the EPS torque-sensor line. When openpilot commands it (CAM_LKAS2, 0x249) it adds a fake torque to
the sensor signal, which the EPS assists like a driver's input, so steering works at any speed and the EPS never
sees "hands off". It reports the driver's *real* torque back in TI_FEEDBACK (0x24A).

The firmware's behaviour here is a model, not a dump of the real TI firmware:
  DISCOVER     power-on sensor discovery (DISCOVER_T), injects nothing
  OFF          no valid command stream; injects nothing
  RUN          valid commands arriving; injects the commanded torque
  DRIVER_OVER  the driver's torque is above the override threshold; injects nothing until it drops back
VIOL (bitfield, live): 0x01 command timeout, 0x02 bad KEY, 0x04 CHKSUM != LKAS_REQUEST, 0x08 command step too large
ERROR (bitfield): 0x01 sensor fault, 0x02 supply fault (only through fault injection)
RAMP_DOWN: TI2 and later only; set while the injection fades out ahead of a driver override.
"""
from __future__ import annotations

from ..common.config import PlantOptions
from ..common.wire import Frame, NET_AUX
from .canbus import Ecu, Periodic, addr_of, decode

TI_KEY = 3294744160
STATE_DISCOVER, STATE_OFF, STATE_DRIVER_OVER, STATE_RUN = 0, 1, 2, 3
STATE_NAMES = {0: "DISCOVER", 1: "OFF", 2: "DRIVER_OVER", 3: "RUN"}

VIOL_TIMEOUT = 0x01
VIOL_KEY = 0x02
VIOL_CHECKSUM = 0x04
VIOL_RATE = 0x08
ERROR_SENSOR = 0x01
ERROR_SUPPLY = 0x02

CMD_ADDR = addr_of("CAM_LKAS2")
FEEDBACK_ADDR = addr_of("TI_FEEDBACK")


class TorqueInterceptor(Ecu):
  name = "ti"
  net = NET_AUX
  DISCOVER_T = 1.5        # s after power-on before the TI takes commands
  TIMEOUT_T = 0.1         # s without a valid command before it drops to OFF
  ARM_FRAMES = 5          # consecutive valid commands needed to enter RUN
  RELEASE_T = 0.25        # s below the release threshold before it leaves DRIVER_OVER
  RAMP_T = 0.3            # s, TI2 fade-out ahead of an override

  def __init__(self, plant: PlantOptions, version: int = 1):
    super().__init__()
    self.p = plant
    self.version = version
    self.state = STATE_DISCOVER
    self.viol = 0
    self.error = 0
    self.ramp_down = False
    self.t_on = 0.0
    self.since_valid = 1e9
    self.valid_run = 0
    self.release_timer = 0.0
    self.cmd = 0            # last accepted command, LKAS counts
    self.inject_units = 0.0
    self.ramp_scale = 1.0
    self.driver_units = 0.0
    self.fault_unplugged = False
    self.fault_error = 0
    self.fault_ignore_commands = False
    self.periodics = [Periodic(FEEDBACK_ADDR, 50.0, self._feedback)]

  def power(self, on: bool) -> None:
    if on and not self.powered:
      self.state, self.t_on, self.viol = STATE_DISCOVER, 0.0, 0
    self.powered = on

  @property
  def injected_nm(self) -> float:
    """Torque the TI currently fakes on the EPS sensor line."""
    if not self.powered or self.fault_unplugged:
      return 0.0
    return self.inject_units / 800.0 * self.p.ti_inject_full_nm

  def set_driver_torque(self, nm: float) -> None:
    self.driver_units = nm / self.p.sensor_nm_per_unit

  def on_frame(self, f: Frame) -> None:
    if f.addr != CMD_ADDR or self.fault_ignore_commands or not self.powered or len(f.dat) != 8:
      return
    sig = decode("CAM_LKAS2", f.dat)
    req = int(sig["LKAS_REQUEST"])
    viol = 0
    if int(sig["KEY"]) != TI_KEY:
      viol |= VIOL_KEY
    if int(sig["CHKSUM"]) != req:
      viol |= VIOL_CHECKSUM
    if self.state == STATE_RUN and abs(req - self.cmd) > self.p.ti_rate_limit_units:
      viol |= VIOL_RATE
    self.viol = viol
    if viol:
      self.valid_run = 0
      return
    self.cmd = req
    self.since_valid = 0.0
    self.valid_run += 1

  def step(self, dt: float) -> None:
    if not self.powered:
      return
    self.t_on += dt
    self.since_valid += dt
    self.error = self.fault_error
    driver = abs(self.driver_units)

    if self.since_valid > self.TIMEOUT_T:
      self.valid_run = 0
      if self.state in (STATE_RUN, STATE_DRIVER_OVER):
        self.viol |= VIOL_TIMEOUT
    elif self.viol & VIOL_TIMEOUT:
      self.viol &= ~VIOL_TIMEOUT

    if self.state == STATE_DISCOVER:
      if self.t_on >= self.DISCOVER_T:
        self.state = STATE_OFF
    elif self.error:
      self.state = STATE_OFF
    elif self.state == STATE_OFF:
      if self.valid_run >= self.ARM_FRAMES:
        self.state = STATE_RUN
    elif self.state == STATE_RUN:
      if self.since_valid > self.TIMEOUT_T:
        self.state = STATE_OFF
      elif driver > self.p.ti_driver_over_units:
        self.state = STATE_DRIVER_OVER
        self.release_timer = 0.0
    elif self.state == STATE_DRIVER_OVER:
      if self.since_valid > self.TIMEOUT_T:
        self.state = STATE_OFF
      elif driver < self.p.ti_driver_release_units:
        self.release_timer += dt
        if self.release_timer >= self.RELEASE_T:
          self.state = STATE_RUN
      else:
        self.release_timer = 0.0

    # TI2+: fade the injection out while the driver's torque climbs towards the override threshold
    self.ramp_down = False
    if self.version > 1 and self.state == STATE_RUN and driver > self.p.ti_driver_release_units:
      self.ramp_scale = max(0.0, self.ramp_scale - dt / self.RAMP_T)
      self.ramp_down = True
    else:
      self.ramp_scale = min(1.0, self.ramp_scale + dt / self.RAMP_T)

    target = self.cmd * self.ramp_scale if self.state == STATE_RUN else 0.0
    self.inject_units = target

  def _feedback(self) -> bytes | None:
    if self.fault_unplugged:
      return None
    d = bytearray(8)
    d[0] = max(0, min(255, int(round(self.driver_units)) + 127))
    d[2] = self.version & 0xFF
    d[3] = self.state
    d[4] = self.viol & 0xFF
    d[5] = self.error & 0xFF
    d[6] = int(self.ramp_down) if self.version > 1 else 0
    d[1] = (d[0] + sum(d[2:8])) & 0xFF
    return bytes(d)

  def telemetry(self) -> dict:
    return {"state": STATE_NAMES.get(self.state, str(self.state)), "viol": self.viol, "error": self.error,
            "rampDown": self.ramp_down, "cmd": self.cmd, "injectNm": round(self.injected_nm, 3),
            "driverUnits": round(self.driver_units, 1), "version": self.version, "unplugged": self.fault_unplugged}
