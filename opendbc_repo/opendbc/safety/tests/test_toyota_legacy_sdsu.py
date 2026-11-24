import pytest
from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.toyota import legacy_sdsu
from opendbc.safety.tests.libsafety import libsafety_py
from openpilot.starpilot.car.toyota.tests import test_rav4h_caller_native as helpers

source_matched_native = helpers.source_matched_native
WORDS = sorted(legacy_sdsu.WORDS)


class Physical:
  def __init__(self, word, experience=32, request=3, *, reset=True, start=1_000_000):
    self.native = libsafety_py.libsafety
    if reset:
      self.native.init_tests()
    self.native.set_alternative_experience(experience)
    assert self.native.set_safety_hooks(structs.CarParams.SafetyModel.toyota, word) == 0
    self.native.set_aol_test_heartbeat(True)
    self.word, self.now, self.request = word, start, request
    self.packer = CANPacker("toyota_new_mc_pt_generated" if word & 0x100 else "toyota_tnga_k_pt_generated")

  def tick(self, cruise=True, status=0, bad=None, **fields):
    self.now += 10_000
    self.native.set_timer(self.now)
    frames = helpers.physical(self.packer, cruise=cruise, sdsu=status, **fields)
    if self.word & 0x4000:
      main = self.packer.make_can_msg("DSU_CRUISE", 0, {"MAIN_ON": fields.get("main", True), "SET_SPEED": 90})
      if bad != "main_drop":
        frames.append((main[0], main[1][:6] if bad == "main_length" else main[1], 2 if bad == "main_bus" else 0))
    for address, data, bus in frames:
      if address == 0x2FF and bad == "status_length":
        data = data[:7]
      if address == 0x2FF and bad == "status_bus":
        bus = 2
      if address == 0x262 and bad == "eps_checksum":
        data = data[:-1] + bytes([data[-1] ^ 1])
      if address == 0x262 and bad == "eps_length":
        data = data + bytes(3)
      if address == 0x224 and bad == "brake_address":
        address = 0x226
      self.native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
    self.native.safety_tick()
    self.native.aol_set_host_request(self.request)

  def accel(self, value=0.1, cancel=0):
    return self.tx(self.packer.make_can_msg("ACC_CONTROL", 0, {"ACCEL_CMD": value, "CANCEL_REQ": cancel}))

  def steer(self, value=1):
    return self.tx(self.packer.make_can_msg("STEERING_LKA", 0, {"STEER_TORQUE_CMD": value, "STEER_REQUEST": value != 0}))

  def tx(self, frame):
    return bool(self.native.safety_tx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1])))

  def warm(self):
    for _ in range(25):
      self.tick(cruise=False)
    for _ in range(25):
      self.tick()

  def close(self):
    self.native.set_alternative_experience(0)
    self.native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)


@pytest.mark.parametrize("word", WORDS)
def test_exact_paired_sources_stock_and_aeb(word):
  case = Physical(word)
  try:
    case.warm()
    expected = 1 if word & 0x200 else 3
    assert case.native.aol_get_permission_mask() == expected
    assert case.steer()
    assert case.accel() == (not bool(word & 0x200))
    assert case.accel(0, 1)
    case.tick(status=7)
    assert case.native.aol_get_permission_mask() == 1
    assert not case.accel()
    case.tick(status=2)
    assert case.native.aol_get_permission_mask() == 0
    assert not case.accel() and not case.steer()
    case.tick()
    # A healthy source restores independent LAT. LONG still needs a real
    # disengage/reengage edge after a required-source fault.
    assert case.native.aol_get_permission_mask() == 1
    case.tick(cruise=False)
    case.tick(cruise=True)
    assert case.native.aol_get_permission_mask() == expected
  finally:
    case.close()


@pytest.mark.parametrize("word", WORDS)
@pytest.mark.parametrize("bad", ["status_length", "status_bus", "eps_checksum", "eps_length"])
def test_malformed_sources_cannot_renew_ownership(word, bad):
  case = Physical(word)
  try:
    case.warm()
    assert case.native.aol_get_permission_mask() == (1 if word & 0x200 else 3)
    for _ in range(45):
      case.tick(bad=bad)
    assert case.native.aol_get_permission_mask() == 0
    assert not case.accel()
  finally:
    case.close()


@pytest.mark.parametrize("word", [w for w in WORDS if w & 0x4000])
@pytest.mark.parametrize("bad", ["main_drop", "main_length", "main_bus"])
@pytest.mark.parametrize("experience,host_request", [(0, 0), (32, 3)])
def test_main_deadline_holds_even_with_ordinary_cruise_permission(word, bad, experience, host_request):
  case = Physical(word, experience, host_request)
  try:
    case.warm()
    assert case.native.get_controls_allowed()
    assert case.steer()
    for _ in range(45):
      case.tick(bad=bad)
    # PCM_CRUISE is still fresh and engaged. Missing physical DSU MAIN must
    # independently veto TX in both ordinary and protocol-owned control.
    assert case.native.get_controls_allowed()
    assert not case.steer() and not case.accel()
    assert not case.tx(case.packer.make_can_msg("DSU_CRUISE", 0, {"MAIN_ON": 1, "SET_SPEED": 90}))
  finally:
    case.close()


@pytest.mark.parametrize("word", [w for w in WORDS if w & 0x100])
def test_alternate_brake_source_is_mandatory(word):
  case = Physical(word)
  try:
    case.warm()
    for _ in range(40):
      case.tick(bad="brake_address")
    assert not case.accel() and not case.steer()
  finally:
    case.close()


@pytest.mark.parametrize("word", WORDS)
def test_actual_eps_measurement_uses_original_scale(word):
  case = Physical(word)
  try:
    frame = case.packer.make_can_msg("STEER_TORQUE_SENSOR", 0, {"STEER_TORQUE_EPS": 100})
    for _ in range(8):
      case.native.safety_rx_hook(libsafety_py.make_CANPacket(frame[0], frame[2], frame[1]))
    assert case.native.get_torque_meas_min() == (word & 0xFF) - 1
    assert case.native.get_torque_meas_max() == (word & 0xFF) + 1
  finally:
    case.close()


def test_warm_switches_clear_both_required_sources():
  now = 1_000_000
  sequence = [0x9049, *WORDS, 0x9249, *reversed(WORDS), 0x9049]
  case = None
  try:
    for index, word in enumerate(sequence):
      case = Physical(word, reset=index == 0, start=now)
      assert case.native.aol_get_permission_mask() == 0
      assert not case.steer() and not case.accel()
      case.warm()
      assert case.native.aol_get_permission_mask() == (1 if word & 0x200 else 3)
      now = case.now
  finally:
    if case is not None:
      case.close()


@pytest.mark.parametrize("word", WORDS)
def test_no_radar_diagnostics(word):
  case = Physical(word)
  try:
    case.warm()
    for payload in ("0f023e0000000000", "0f02280100000000"):
      assert not case.tx((0x750, bytes.fromhex(payload), 0))
  finally:
    case.close()
