#!/usr/bin/env python3
"""The comma's end of the harness: a virtual panda (in place of pandad) and the comma's own sensors.

Runs inside the comma container, against the build under test (its cereal, params and panda safety code).
  car  -> comma: CAN per network, ignition, IMU/GPS, ground truth
  comma -> car:  CAN the panda let through or forwarded, panda state, what openpilot is doing
"""
from __future__ import annotations

import argparse
import os
import queue
import socket
import threading
import time

import cereal.messaging as messaging
from cereal import car, log
from openpilot.common.params import Params
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp

from ..common import wire
from . import libsafety
from .virtual_panda import SAFETY_ELM327, SAFETY_NOOUTPUT, VirtualPanda

GT_UDP = ("127.0.0.1", 7011)


def mono_us() -> int:
  return int(time.monotonic() * 1e6)


class PandadLogic:
  """pandad's safety-mode handshake with card (selfdrive/pandad/panda_safety.cc), driving the virtual panda."""

  def __init__(self, vp: VirtualPanda, params: Params):
    self.vp = vp
    self.params = params
    self.initialized = False
    self.configured = False
    self.prev_obd = False

  def update(self, is_onroad: bool) -> None:
    vp = self.vp
    if not vp.ignition or not is_onroad:
      if vp.mode != SAFETY_NOOUTPUT:
        vp.set_mode(SAFETY_NOOUTPUT, 0)
      self.initialized = self.configured = False
      return
    if self.configured:
      return
    if not self.initialized:
      self.prev_obd = False
      vp.set_mode(SAFETY_ELM327, 1)
      self.initialized = True
    obd = self.params.get_bool("ObdMultiplexingEnabled")
    if obd != self.prev_obd:
      vp.set_mode(SAFETY_ELM327, 0 if obd else 1)
      self.prev_obd = obd
      self.params.put_bool("ObdMultiplexingChanged", True)
    if not self.params.get_bool("FirmwareQueryDone") or not self.params.get_bool("ControlsReady"):
      return
    cp_bytes = self.params.get("CarParams")
    if not cp_bytes:
      return
    with car.CarParams.from_bytes(cp_bytes) as cp:
      cfgs = list(cp.safetyConfigs)
      model = int(cfgs[0].safetyModel.raw) if cfgs else 0
      param = int(cfgs[0].safetyParam) if cfgs else 0
      alt = int(cp.alternativeExperience)
    fp_bytes = self.params.get("StarPilotCarParams")
    if fp_bytes:
      try:
        from cereal import custom
        with custom.StarPilotCarParams.from_bytes(fp_bytes) as fp:
          if len(fp.safetyConfigs):
            param |= int(fp.safetyConfigs[0].safetyParam)
          alt |= int(fp.alternativeExperience)
      except Exception as e:  # older builds have no StarPilotCarParams
        print(f"harnessd: StarPilotCarParams not used ({e})")
    vp.set_alternative_experience(alt)
    vp.set_mode(model, param)
    self.configured = True
    print(f"harnessd: safety model {model} param {param} alt {alt}, relay intercept {vp.relay_intercept}, bus 1 on OBD {vp.obd_mode}")


class Harness:
  def __init__(self, car_host: str, opendbc_root: str, build_dir: str):
    self.car_host = car_host
    lib = libsafety.build(opendbc_root, build_dir)
    self.vp = VirtualPanda(libsafety.Safety(lib))
    self.params = Params()
    self.logic = PandadLogic(self.vp, self.params)
    self.lock = threading.Lock()
    self.pm = messaging.PubMaster(['can', 'pandaStates', 'peripheralState', 'accelerometer', 'gyroscope',
                                   'gpsLocation', 'gpsLocationExternal', 'driverStateV2', 'driverMonitoringState'])
    # a comma 3X has no u-blox: its GNSS arrives as gpsLocation from qcomgpsd, unless the build says otherwise
    self.gps_service = "gpsLocationExternal" if self.params.get_bool("UbloxAvailable") else "gpsLocation"
    self.sm = messaging.SubMaster(['selfdriveState', 'carState', 'carControl', 'carOutput', 'controlsState', 'onroadEvents',
                                   'deviceState', 'longitudinalPlan', 'modelV2'], poll=None)
    self.link: wire.Link | None = None
    self.inbox: queue.Queue = queue.Queue()
    self.voltage_mv = 12600
    self.gt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sensor_n = 0
    self.dm_distracted = False

  # ---- car link ----
  def connect(self) -> None:
    while True:
      try:
        self.link = wire.connect(self.car_host, wire.HARNESS_PORT)
        self.link.send(wire.MSG_HELLO, wire.pack_json({"role": "comma", "proto": wire.PROTO_VERSION}))
        print(f"harnessd: plugged into the car at {self.car_host}:{wire.HARNESS_PORT}")
        return
      except OSError:
        time.sleep(1.0)

  def reader(self) -> None:
    while True:
      try:
        self.inbox.put(self.link.recv())
      except wire.ConnectionClosed:
        self.inbox.put((None, b""))
        return

  def send_car(self, msg_type: int, payload: bytes) -> None:
    try:
      if self.link is not None and not self.link.closed:
        self.link.send(msg_type, payload)
    except wire.ConnectionClosed:
      pass

  # ---- openpilot -> car ----
  def sendcan_thread(self) -> None:
    sock = messaging.sub_sock('sendcan', timeout=100)
    while True:
      dat = sock.receive()
      if dat is None:
        continue
      with log.Event.from_bytes(dat) as evt:
        if time.monotonic_ns() - evt.logMonoTime > 1e9:
          continue
      msgs = []
      for _, frames in can_capnp_to_list([dat], msgtype='sendcan'):
        msgs.extend((a, bytes(d), s) for a, d, s in frames)
      with self.lock:
        self.vp.advance(mono_us())
        out = self.vp.send(msgs)
      if out:
        self.send_car(wire.MSG_CAN, wire.pack_can(mono_us(), out))

  # ---- main loop ----
  def handle(self, msg_type: int, payload: bytes) -> None:
    if msg_type == wire.MSG_CAN:
      _, frames = wire.unpack_can(payload)
      with self.lock:
        self.vp.advance(mono_us())
        fwd = self.vp.receive(frames)
        host = self.vp.drain_host()
      if fwd:
        self.send_car(wire.MSG_CAN, wire.pack_can(mono_us(), fwd))
      self.pm.send('can', can_list_to_can_capnp(host, msgtype='can', valid=True))
    elif msg_type == wire.MSG_CAR_STATE:
      st = wire.unpack_json(payload)
      self.vp.ignition = bool(st.get("ignition", True))
      self.voltage_mv = int(st.get("voltageMv", 12600))
      self.dm_distracted = bool(st.get("driverDistracted", False))
      if "relayStuck" in st:
        self.vp.relay_stuck = bool(st["relayStuck"])
    elif msg_type == wire.MSG_SENSORS:
      self.publish_sensors(wire.unpack_sensors(payload))
    elif msg_type == wire.MSG_TRUTH:
      try:
        self.gt_sock.sendto(bytes(payload), GT_UDP)
      except OSError:
        pass

  def publish_sensors(self, s: wire.Sensors) -> None:
    dat = messaging.new_message('accelerometer', valid=True)
    dat.accelerometer.sensor = 4
    dat.accelerometer.type = 0x10
    dat.accelerometer.timestamp = dat.logMonoTime
    dat.accelerometer.init('acceleration')
    dat.accelerometer.acceleration.v = list(s.accel)
    self.pm.send('accelerometer', dat)
    dat = messaging.new_message('gyroscope', valid=True)
    dat.gyroscope.sensor = 5
    dat.gyroscope.type = 0x10
    dat.gyroscope.timestamp = dat.logMonoTime
    dat.gyroscope.init('gyroUncalibrated')
    dat.gyroscope.gyroUncalibrated.v = list(s.gyro)
    self.pm.send('gyroscope', dat)
    self.sensor_n += 1
    if self.sensor_n % 10 == 0:
      lat, lon, alt, speed, bearing, vn, ve = s.gps
      svc = self.gps_service
      dat = messaging.new_message(svc, valid=True)
      source = log.GpsLocationData.SensorSource.ublox if svc == "gpsLocationExternal" else log.GpsLocationData.SensorSource.qcomdiag
      setattr(dat, svc, {
        "unixTimestampMillis": int(time.time() * 1000),  # wall clock, as the GNSS reports
        "flags": 1, "hasFix": True, "horizontalAccuracy": 1.0, "verticalAccuracy": 1.0, "speedAccuracy": 0.1,
        "bearingAccuracyDeg": 0.1, "vNED": [vn, ve, 0.0], "bearingDeg": bearing, "latitude": lat, "longitude": lon,
        "altitude": alt, "speed": speed, "source": source,
      })
      self.pm.send(svc, dat)

  def publish_dm(self) -> None:
    # PC builds run no driver monitoring model: stand in for dmonitoringmodeld + dmonitoringd, as tools/sim does
    dat = messaging.new_message('driverStateV2')
    for side in (dat.driverStateV2.leftDriverData, dat.driverStateV2.rightDriverData):
      side.faceOrientation = [0.0, 0.0, 0.0]
      side.faceProb = 1.0
    self.pm.send('driverStateV2', dat)
    dat = messaging.new_message('driverMonitoringState', valid=True)
    distracted = self.dm_distracted
    dat.driverMonitoringState = {
      "alertLevel": log.DriverMonitoringState.AlertLevel.none,
      "activePolicy": log.DriverMonitoringState.MonitoringPolicy.vision,
      "isRHD": False,
      "visionPolicyState": {"awarenessPercent": 0 if distracted else 100, "isDistracted": distracted, "faceDetected": True},
      "wheeltouchPolicyState": {"awarenessPercent": 100},
    }
    self.pm.send('driverMonitoringState', dat)

  def publish_panda_states(self) -> None:
    vp, s = self.vp, self.vp.safety
    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    ps = dat.pandaStates[0]
    ps.ignitionLine = vp.ignition
    ps.pandaType = log.PandaState.PandaType.tres
    ps.controlsAllowed = s.controls_allowed
    ps.safetyModel = vp.mode
    ps.safetyParam = vp.param
    ps.alternativeExperience = vp.alternative_experience
    ps.safetyTxBlocked = vp.counters.tx_blocked
    ps.safetyRxInvalid = vp.counters.rx_invalid
    ps.safetyRxChecksInvalid = s.rx_checks_invalid
    ps.uptime = int(time.monotonic())
    ps.voltage = self.voltage_mv
    ps.harnessStatus = log.PandaState.HarnessStatus.normal
    ps.faultStatus = log.PandaState.FaultStatus.faultPerm if s.relay_malfunction else log.PandaState.FaultStatus.none
    if s.relay_malfunction:
      ps.faults = [log.PandaState.FaultType.relayMalfunction]
    self.pm.send('pandaStates', dat)

  def publish_peripheral(self) -> None:
    dat = messaging.new_message('peripheralState', valid=True)
    dat.peripheralState = {"pandaType": log.PandaState.PandaType.tres, "voltage": self.voltage_mv, "current": 5678,
                           "fanSpeedRpm": 1500}
    self.pm.send('peripheralState', dat)

  def openpilot_summary(self) -> dict:
    sm = self.sm
    sm.update(0)
    ss, cs, cc, co = sm['selfdriveState'], sm['carState'], sm['carControl'], sm['carOutput']
    events = []
    try:
      events = [str(e.name) for e in sm['onroadEvents']][:8]
    except Exception:
      pass
    return {
      "connected": True,
      "started": bool(sm['deviceState'].started),
      "enabled": bool(ss.enabled), "active": bool(ss.active), "state": str(ss.state), "experimental": bool(ss.experimentalMode),
      "alert1": ss.alertText1, "alert2": ss.alertText2, "alertStatus": str(ss.alertStatus),
      "latActive": bool(cc.latActive), "longActive": bool(cc.longActive),
      "torque": round(cc.actuators.torque, 3), "accel": round(cc.actuators.accel, 3),
      "torqueOut": round(co.actuatorsOutput.torque, 3), "torqueOutCan": round(co.actuatorsOutput.torqueOutputCan, 1),
      "vEgo": round(cs.vEgo, 2), "steeringPressed": bool(cs.steeringPressed), "steeringTorque": round(cs.steeringTorque, 1),
      "cruiseEnabled": bool(cs.cruiseState.enabled), "cruiseAvailable": bool(cs.cruiseState.available),
      "cruiseSpeed": round(cs.cruiseState.speed, 2), "canValid": bool(cs.canValid),
      "steerFaultTemporary": bool(cs.steerFaultTemporary), "steerFaultPermanent": bool(cs.steerFaultPermanent),
      "desiredCurvature": round(sm['modelV2'].action.desiredCurvature, 5), "curvature": round(sm['controlsState'].curvature, 5),
      "events": events, "panda": self.vp.state(),
    }

  def run(self) -> None:
    self.connect()
    threading.Thread(target=self.reader, daemon=True).start()
    threading.Thread(target=self.sendcan_thread, daemon=True).start()
    next_10hz = next_2hz = next_dm = time.monotonic()
    while True:
      try:
        msg_type, payload = self.inbox.get(timeout=0.02)
        if msg_type is None:
          print("harnessd: unplugged from the car, reconnecting")
          with self.lock:
            self.vp.ignition = False
          self.connect()
          threading.Thread(target=self.reader, daemon=True).start()
          continue
        self.handle(msg_type, payload)
      except queue.Empty:
        pass
      now = time.monotonic()
      if now >= next_10hz:
        next_10hz = now + 0.1
        with self.lock:
          self.logic.update(self.params.get_bool("IsOnroad"))
          self.publish_panda_states()
          state = self.vp.state()
        self.send_car(wire.MSG_PANDA, wire.pack_json(state))
        try:
          self.send_car(wire.MSG_OPENPILOT, wire.pack_json(self.openpilot_summary()))
        except Exception as e:
          print(f"harnessd: summary failed: {e}")
      if now >= next_dm:
        next_dm = now + 0.05
        self.publish_dm()
      if now >= next_2hz:
        next_2hz = now + 0.5
        self.publish_peripheral()


def main() -> None:
  ap = argparse.ArgumentParser(description="virtual panda + comma sensors for the Mazda simulator")
  ap.add_argument("--car", default=os.environ.get("MAZDA_SIM_CAR_HOST", "mazda"))
  ap.add_argument("--opendbc", default=os.environ.get("MAZDA_SIM_OPENDBC", "/openpilot/opendbc_repo"))
  ap.add_argument("--build-dir", default=os.environ.get("MAZDA_SIM_SAFETY_BUILD", "/tmp/mazda_sim_safety"))
  args, _ = ap.parse_known_args()
  Harness(args.car, args.opendbc, args.build_dir).run()


if __name__ == "__main__":
  main()
