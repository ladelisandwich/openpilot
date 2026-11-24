"""Mazda Torque Interceptor (TI1, GEN1) quirks the car controller works around.

The TI faults when its command is held at the 800-count scale maximum. After 1.84-1.99 s of an unbroken
800 it reports STATE OFF with VIOL 0x11 (22 of 22 trips, CX-9 routes 00000022-00000035, at 9-77 mph) and
stops steering for ~31 s: the violation clears ~29.5 s later and RUN returns 2 s after that. Nothing else
tripped it: a command between 780 and 800, or one held at the ceiling for under 1.84 s, never did, and a
single 20 Hz sample dipped to 790 restarted its count (route 00000033 held 790-800 for 3.1 s with two such
dips). The same drives showed the TI's input to the EPS flat from about 650 counts up (the EPS torque
sensor reads 32-34 whatever is sent above that), so the command is clamped at CarControllerParams.TI_CEILING
(700), which never tripped the fault and gives up nothing measurable.

TiCeilingGuard is the backstop under that clamp: the data cannot tell a fault on "800 held" from one on
"any value held" (no other nonzero command was ever held longer than 0.74 s), so a command held above
TI_CEILING_DIP is dipped to it for a moment, well before 1.84 s.

The TI's own documentation (MoreTorque, "TI General Documentation", Part 2) settles that: 0x11 is
VIOL_LKAS_STUK, "LKAS request input does not change when not 0 or +/-600", so any nonzero value held unchanged
trips it. The VIOL byte carries the violation's green-LED code as two hex digits (0x11 is code 11); the table
is VIOLATIONS below. Class 2 violations recover by themselves 30 s after the condition is gone, then RUN after
the 2 s start delay, which is the ~31 s outage seen on the car; class 1 need a power cycle or a reset key.
Route 00000097 tripped it away from the ceiling: -1 count held for 2 s while creeping at 0.5 mph. So the guard
also watches for any nonzero command held unchanged, and lowers it toward zero by `step` (to zero when it is
smaller) for the same moment. Across 22 drives no command between 3 and 675 counts was held for 0.5 s, so
outside the ceiling and a creep this does not act.
"""

# VIOL byte -> (mnemonic, class, what trips it), from the TI documentation's violation code table
VIOLATIONS: dict[int, tuple[str, int, str]] = {
  0x11: ("LKAS_STUK", 2, "the LKAS request did not change while not 0 or +/-600"),
  0x12: ("CAN_TMO", 3, "CAN bus off-line for more than 100 ms"),
  0x13: ("LKAS_ERR", 2, "LKAS torque request out of the window"),
  0x14: ("LOW_VOLT", 3, "CPU voltage too low"),
  0x15: ("TMR_ERR", 1, "the periodic timer interrupt was not detected"),
  0x16: ("MAIN_ERR", 1, "main loop not running within its time limits"),
  0x17: ("TORQUE_H", 2, "computed torque output above the high limit window"),
  0x18: ("TORQUE_L", 2, "computed torque output below the low limit window"),
  0x19: ("WDT_ERR", 1, "internal watchdog triggered"),
  0x21: ("CPU_REG", 1, "CPU pin or peripheral configuration error"),
  0x22: ("HIGH_TMP", 3, "CPU die above 65 C"),
  0x23: ("DATA_MEM", 1, "EEPROM memory CRC error"),
  0x25: ("TK_TRNSF", 2, "calculated torque output differs from the measured output beyond the window"),
  0x26: ("ADC_STUK", 1, "ADC channel failed"),
  0x31: ("HIN_ERR", 2, "EPS hall sensor input out of the window (normal while the car powers up or down)"),
  0x32: ("TK_NLOAD", 2, "feedback torque output out of the window under no load"),
  0x33: ("TK_WLOAD", 2, "feedback torque output out of the window under load"),
  0x34: ("CAN_ERR", 3, "a CAN bus error other than CAN_TMO"),
  0x35: ("STCK_ERR", 1, "stack over- or underflow"),
  0x36: ("PANIC_MON", 1, "the panic monitor output is high"),
  0x37: ("PANIC", 1, "panic test injection"),
  0x41: ("LOGIC", 1, "code logic error (see the ERROR byte)"),
  0x42: ("DBG_STOP", 1, "debug console switched the TI off"),
  0x43: ("CLK_STOP", 1, "CPU main oscillator stopped"),
}


def violation_name(code: int) -> str:
  """'0x33 TK_WLOAD (class 2)' for a known code, '0x5a (unknown)' otherwise, '' for none."""
  if not code:
    return ""
  name = VIOLATIONS.get(code)
  return f"0x{code:02x} {name[0]} (class {name[1]})" if name else f"0x{code:02x} (unknown)"


class TiCeilingGuard:
  def __init__(self, dip: int, hold_frames: int, dip_frames: int, step: int = 20):
    self.dip = dip                    # magnitude of a dipped command, and the level above which the count runs
    self.hold_frames = hold_frames    # frames above `dip`, or of one unchanged nonzero command, before a dip
    self.dip_frames = dip_frames
    self.step = step                  # how far an unchanged command is lowered toward zero (the ceiling's own dip)
    self.held = 0                     # consecutive frames above `dip`
    self.same = 0                     # consecutive frames of one unchanged nonzero command
    self.prev = 0
    self.dipping = 0                  # frames left in the current dip
    self.cap = dip                    # the magnitude the current dip allows

  def update(self, torque: int) -> int | None:
    """Fed every frame with the command about to go to the TI (0 while it isn't steering).
    Returns the magnitude the TI command may have this frame, or None for no limit."""
    if self.dipping == 0:
      self.held = self.held + 1 if abs(torque) > self.dip else 0
      self.same = self.same + 1 if torque != 0 and torque == self.prev else 0
      if self.held >= self.hold_frames:
        self.dipping, self.cap = self.dip_frames, self.dip
      elif self.same >= self.hold_frames:
        self.dipping, self.cap = self.dip_frames, max(abs(torque) - self.step, 0)
    self.prev = torque
    if self.dipping > 0:
      self.dipping -= 1
      self.held = self.same = 0
      return self.cap
    return None
