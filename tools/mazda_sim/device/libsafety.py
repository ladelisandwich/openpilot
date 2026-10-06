"""The panda's safety code, built from the build under test and loaded in-process.

The virtual panda runs exactly the C safety modes the comma's panda would flash from this checkout
(opendbc/safety/*), compiled for the PC: the same rx/tx/forwarding hooks, controls_allowed logic, relay
malfunction detection and rx-check timeouts. A change to modes/mazda.h is therefore exercised in the sim.
"""
from __future__ import annotations

import hashlib
import os
import subprocess

from cffi import FFI

_WRAPPER = r"""
#include "opendbc/safety/tests/libsafety/safety.c"

#include <stddef.h>
int sim_sizeof_canpacket(void) { return (int)sizeof(CANPacket_t); }
int sim_offsetof_data(void) { return (int)offsetof(CANPacket_t, data); }
bool sim_rx_checks_invalid(void) { return safety_rx_checks_invalid; }
"""

_CDEF = """
typedef struct {
  unsigned char fd : 1;
  unsigned char bus : 3;
  unsigned char data_len_code : 4;
  unsigned char rejected : 1;
  unsigned char returned : 1;
  unsigned char extended : 1;
  unsigned int addr : 29;
  unsigned char checksum;
  unsigned char data[64];
} CANPacket_t;
"""

_FUNCS = """
bool safety_rx_hook(CANPacket_t *msg);
bool safety_tx_hook(CANPacket_t *msg);
int safety_fwd_hook(int bus_num, int addr);
int set_safety_hooks(uint16_t mode, uint16_t param);
void set_controls_allowed(bool c);
bool get_controls_allowed(void);
bool get_longitudinal_allowed(void);
void set_alternative_experience(int mode);
int get_alternative_experience(void);
bool get_relay_malfunction(void);
bool get_gas_pressed_prev(void);
bool get_brake_pressed_prev(void);
bool get_acc_main_on(void);
bool get_vehicle_moving(void);
bool get_cruise_engaged_prev(void);
int get_current_safety_mode(void);
int get_current_safety_param(void);
int get_torque_driver_min(void);
int get_torque_driver_max(void);
void set_timer(uint32_t t);
void safety_tick_current_safety_config(void);
bool safety_config_valid(void);
void init_tests(void);
int sim_sizeof_canpacket(void);
int sim_offsetof_data(void);
bool sim_rx_checks_invalid(void);
"""

LEN_TO_DLC = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 12: 9, 16: 10, 20: 11, 24: 12, 32: 13, 48: 14, 64: 15}


def build(opendbc_root: str, out_dir: str) -> str:
  """Compile (or reuse) libsafety for the opendbc tree at opendbc_root (the directory containing 'opendbc/')."""
  safety_dir = os.path.join(opendbc_root, "opendbc", "safety")
  h = hashlib.sha256(_WRAPPER.encode())
  for root, _, files in os.walk(safety_dir):
    if "tests" in root.split(os.sep) and "libsafety" not in root:
      continue
    for fn in sorted(files):
      if fn.endswith((".h", ".c")):
        with open(os.path.join(root, fn), "rb") as fh:
          h.update(fn.encode() + fh.read())
  os.makedirs(out_dir, exist_ok=True)
  lib = os.path.join(out_dir, f"libsafety_sim_{h.hexdigest()[:12]}.so")
  if os.path.isfile(lib):
    return lib
  src = os.path.join(out_dir, "sim_safety.c")
  with open(src, "w") as fh:
    fh.write(_WRAPPER)
  cmd = ["gcc", "-shared", "-fPIC", "-O2", "-std=gnu11", "-w", "-DALLOW_DEBUG", f"-I{opendbc_root}", "-o", lib + ".tmp", src]
  subprocess.run(cmd, check=True)
  os.replace(lib + ".tmp", lib)
  return lib


class Safety:
  def __init__(self, lib_path: str):
    self.ffi = FFI()
    self.ffi.cdef(_CDEF, packed=True)
    self.ffi.cdef(_FUNCS)
    self.lib = self.ffi.dlopen(lib_path)
    # the C struct is packed, aligned(4): same field offsets, padded size
    c_off, py_off = self.lib.sim_offsetof_data(), self.ffi.offsetof("CANPacket_t", "data")
    c_size = self.lib.sim_sizeof_canpacket()
    if c_off != py_off or c_size < self.ffi.sizeof("CANPacket_t"):
      raise RuntimeError(f"CANPacket_t layout changed in this build (data at {c_off}, expected {py_off}); update device/libsafety.py")
    self.lib.init_tests()
    self._buf = self.ffi.new("char[]", c_size)
    self._pkt = self.ffi.cast("CANPacket_t *", self._buf)

  def packet(self, bus: int, addr: int, dat: bytes):
    p = self._pkt
    p[0].fd = 0
    p[0].rejected = 0
    p[0].returned = 0
    p[0].extended = 1 if addr >= 0x800 else 0
    p[0].addr = addr
    p[0].bus = bus
    p[0].data_len_code = LEN_TO_DLC.get(len(dat), 8)
    self.ffi.memmove(p[0].data, dat, len(dat))
    return p

  def set_mode(self, mode: int, param: int) -> bool:
    return self.lib.set_safety_hooks(mode, param) == 0

  def rx(self, bus: int, addr: int, dat: bytes) -> bool:
    return bool(self.lib.safety_rx_hook(self.packet(bus, addr, dat)))

  def tx(self, bus: int, addr: int, dat: bytes) -> bool:
    return bool(self.lib.safety_tx_hook(self.packet(bus, addr, dat)))

  def fwd(self, bus: int, addr: int) -> int:
    return int(self.lib.safety_fwd_hook(bus, addr))

  def set_timer(self, us: int) -> None:
    self.lib.set_timer(us & 0xFFFFFFFF)

  def tick(self) -> None:
    self.lib.safety_tick_current_safety_config()

  @property
  def controls_allowed(self) -> bool:
    return bool(self.lib.get_controls_allowed())

  @property
  def relay_malfunction(self) -> bool:
    return bool(self.lib.get_relay_malfunction())

  @property
  def rx_checks_invalid(self) -> bool:
    return bool(self.lib.sim_rx_checks_invalid())
