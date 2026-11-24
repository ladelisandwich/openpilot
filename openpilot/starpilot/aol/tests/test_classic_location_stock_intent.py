import dataclasses
import json
import pytest
import importlib
import os
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.classic_scc_aol import qualified as qualified_classic
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.starpilot.aol.intent import AOL_TOGGLE, read_settings
from openpilot.starpilot.aol.runtime import current_native

from openpilot.starpilot.aol.tests import test_ioniq_availability as caller_fixture
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety
from openpilot.starpilot.car.hyundai.classic_scc_intent import ClassicSccCardIntent
from openpilot.cereal import log

disable_ecu_module = importlib.import_module('opendbc.car.disable_ecu')


def classic_frames(ci, tick, *, main=0, lkas=0, button=0, engaged=False):

  assert ci.CP.carFingerprint == CAR.HYUNDAI_SONATA_HYBRID
  packer = CANPacker('hyundai_can_generated')


  hz = {'E_EMS11':100, 'ELECT_GEAR':20, 'SCC11':50, 'SCC12':50, 'WHL_SPD11':100, 'TCS13':100, 'MDPS12':50, 'CLU11':50,
        'BCM_PO_11':50, 'LKAS11':100}
  scheduled = {(packer.dbc.name_to_msg[name].address, 0): name for name in hz if name != 'LKAS11'}
  for parser in ci.can_parsers.values():
    for address in parser.addresses:
      scheduled[(address, parser.bus)] = parser.dbc.addr_to_msg[address].name
  frames = []
  for (address,bus),name in sorted(scheduled.items()):
    parser = next((p for p in ci.can_parsers.values() if p.bus == bus), None)
    declared = parser.message_states[address].frequency if parser is not None and address in parser.addresses else 0
    frequency = hz.get(name, min(declared,100) if 0 < declared <= 100 else 100)
    period = max(1,round(100/frequency))
    if tick % period:
      continue
    counter = tick // period
    values = {}
    if name == 'CLU11':
      values.update(CF_Clu_AliveCnt1=counter%16, CF_Clu_CruiseSwState=button, CF_Clu_CruiseSwMain=main)
    elif name == 'E_EMS11':
      values.update(CR_Vcu_AccPedDep_Pos=0)
    elif name == 'WHL_SPD11':
      values.update(WHL_SPD_AliveCounter_LSB=counter%4, WHL_SPD_AliveCounter_MSB=(counter//4)%4,
                    WHL_SPD_FL=40,WHL_SPD_FR=40,WHL_SPD_RL=40,WHL_SPD_RR=40)
    elif name == 'TCS13':
      values.update(AliveCounterTCS=counter%8, ACCEnable=0, ACC_REQ=int(engaged), DriverOverride=0)
    elif name == 'MDPS12':
      values.update(CR_Mdps_StrColTq=0, CR_Mdps_OutTq=0)
    elif name == 'BCM_PO_11':
      values['LDA_BTN']=lkas
    elif name == 'CGW1':
      values['CF_Gway_DrvSeatBeltSw']=1
    elif name == 'ELECT_GEAR':
      values['Elect_Gear_Shifter']=5
    elif name == 'SCC11':
      values['MainMode_ACC']=main
    elif name == 'SCC12':
      values.update(ACCMode=int(engaged), CR_VSM_Alive=counter%16)
    elif name == 'SAS11':
      values['SAS_Angle']=1.
    elif name == 'CLU15':
      values['CF_Clu_VehicleSpeed']=40
    frame = packer.make_can_msg(name,bus,values)
    if name in ('SCC12','WHL_SPD11','TCS13'):
      address, data, bus = checksum(frame)
      frame = (address, bytes(data), bus)
    frames.append(frame)
  assert not ci.CP.openpilotLongitudinalControl and ci.CP.pcmCruise
  assert all(address!=0x340 or bus==2 for address,_,bus in frames), 'Never echo outgoing LKAS on protected RX bus'
  return frames


def _invalid_controls_emits_no_commands(received):
  if 'sendcan' not in received:
    return True
  envelope = received['sendcan']
  return envelope.which() == 'sendcan' and not envelope.valid and len(envelope.sendcan) == 0



def assert_stock_location_warning_contract(sd, card, state, now_ns):
  from copy import copy
  from openpilot.starpilot.aol.wire import decode_safety

  class View:
    def __init__(self, base):
      self.base = base
      self.values = {'onroadEvents': sd.pm.messages['onroadEvents'].onroadEvents}
      self.valid = dict(base.valid)
      self.alive = dict(base.alive)
      self.seen = dict(base.seen)
      self.logMonoTime = dict(base.logMonoTime)
      self.logMonoTime['onroadEvents'] = int(sd.pm.messages['onroadEvents'].logMonoTime)

    def __getitem__(self, key):
      return self.values[key] if key in self.values else self.base[key]

    def __getattr__(self, key):
      return getattr(self.base, key)

  original = sd.pm.messages['aolAxisState']
  stamp = int(original.logMonoTime)
  feedback = card.aol_transport_feedback
  faults = ['none', 'door', 'seatbelt', 'brake', 'regen', 'gas', 'cancel', 'gear', 'permanent_eps', 'temporary_eps',
            'invalid_can', 'can_timeout', 'acc_fault', 'main_off', 'steering_disengage', 'extra_user', 'hard_event',
            'epoch', 'session', 'old_source', 'future_source', 'no_baseline', 'native_invalid', 'native_denial', 'panda_rx', 'relay']
  for fault in faults:
    current = copy(feedback)
    view = View(card.sm)
    cs = state.as_reader().as_builder()
    proof = original.as_builder().aolAxisState
    events = sd.pm.messages['onroadEvents'].as_builder()
    cs.doorOpen = fault == 'door'
    cs.seatbeltUnlatched = fault == 'seatbelt'
    cs.brakePressed = fault == 'brake'
    cs.regenBraking = fault == 'regen'
    cs.gasPressed = fault == 'gas'
    cs.steerFaultPermanent = fault == 'permanent_eps'
    cs.steerFaultTemporary = fault == 'temporary_eps'
    cs.canValid = fault != 'invalid_can'
    cs.canTimeout = fault == 'can_timeout'
    cs.accFaulted = fault == 'acc_fault'
    cs.cruiseState.available = fault != 'main_off'
    cs.steeringDisengage = fault == 'steering_disengage'
    if fault == 'cancel':
      cs.buttonEvents = [structs.CarState.ButtonEvent(type='cancel', pressed=True)]
    if fault == 'gear':
      cs.gearShifter = 'reverse'
    if fault == 'extra_user':
      events.onroadEvents = [x.to_dict() for x in events.onroadEvents] + [{'name': 'buttonCancel', 'userDisable': True}]
    if fault == 'hard_event':
      events.onroadEvents[0].immediateDisable = True
    view.values['onroadEvents'] = events.as_reader().onroadEvents
    if fault == 'epoch':
      proof.intentIntegrityEpoch += 1
    if fault == 'session':
      proof.sessionId = proof.faultSessionId = 'wrong-session'
    if fault == 'old_source':
      proof.sourceCarStateMonoTime = now_ns - 200_000_001
    if fault == 'future_source':
      proof.sourceCarStateMonoTime = now_ns + 1
    if fault == 'no_baseline':
      current.lateral_baseline_ns = 0
    if fault == 'native_invalid':
      view.valid['aolSafetyWire'] = False
    if fault in ('native_denial', 'panda_rx', 'relay'):
      native = decode_safety(view['aolSafetyWire'])
      assert native is not None
      if fault == 'native_denial':
        native = dataclasses.replace(native, requestedLateral=True, lateralAllowed=False)
      else:
        panda = messaging.new_message('pandaStates', len(card.CP.safetyConfigs), valid=True,
                                      logMonoTime=view.logMonoTime['pandaStates'])
        panda.pandaStates = [x.to_dict() for x in view['pandaStates']]
        inventory = list(native.pandaInventory)
        if fault == 'panda_rx':
          panda.pandaStates[0].safetyRxChecksInvalid = True
          inventory[0] = dataclasses.replace(inventory[0], safetyRxChecksInvalid=True)
        else:
          panda.pandaStates[0].faults = [str(x) for x in panda.pandaStates[0].faults] + ['relayMalfunction']
          inventory[0] = dataclasses.replace(inventory[0], faults=inventory[0].faults | 1)
        view.values['pandaStates'] = panda.as_reader().pandaStates
        native = dataclasses.replace(native, pandaInventory=tuple(inventory))
      view.values['aolSafetyWire'] = encode_safety(native)
    # Production dispatches location SOFT withdrawals through observe() and
    # the shared warning owner, rather than the older critical-only helper.
    view.values['aolAxisState'] = proof.as_reader()
    view.logMonoTime['aolAxisState'] = stamp
    before = (current.lateral_baseline_ns, current.lateral_native_ns)
    retained = not current.observe(view, card.CP, cs, now_ns,
      latched=card.aol_card_intent.allowed_latch, intent_producer_session=feedback.availability_producer)
    # Pedals, belt and temporary EPS conditions preserve only armed intent.
    # A denied native LAT receipt while both host axes are withdrawn is also
    # a pause; it supplies no native permission and never resumes commands.
    # Classic independent AOL does not require the cruise MAIN level. This
    # is not a new MAIN button edge and cannot replay the configured toggle.
    expected = fault in ('none', 'seatbelt', 'brake', 'regen', 'gas', 'cancel', 'temporary_eps', 'native_denial') or (
      fault == 'main_off' and qualified_classic(card.CP, marked_only=True))
    assert retained == expected, (fault, current.__dict__)
    assert not original.aolAxisState.lateralActive and not original.aolAxisState.longitudinalActive
    assert (current.lateral_baseline_ns, current.lateral_native_ns) == before



@pytest.fixture(scope='module')
def host_heartbeat_bridge(tmp_path_factory):
  import hashlib
  import re
  import capnproto
  import subprocess
  import struct
  from pathlib import Path

  root = Path(__file__).resolve().parents[4]
  tmp = tmp_path_factory.mktemp('hkg-location-host')
  pandad = (root / 'openpilot/selfdrive/pandad/pandad.cc').read_text()
  matched = re.search(r'      if \(aol_write_ok\) \{.*?\n      \}', pandad, re.S)
  assert matched is not None
  code = r'''
#include <cstring>
#include <vector>
#include "selfdrive/pandad/aol_wire.h"
#include "openpilot/cereal/gen/cpp/log.capnp.h"
#include "openpilot/starpilot/car/hyundai/aol_policy.h"
static const AolSafetyProfile profiles[] = {HYUNDAI_AOL_PROFILE, HYUNDAI_CLASSIC_AOL_PROFILE, HYUNDAI_LEGACY_AOL_PROFILE};
static const AolProfileRegistry registry{profiles, std::size(profiles)};
static AolAxisNegotiator negotiator(registry);
static AolWritePlan pending;
static aol_safety_health_t snapshot;
extern "C" void reset_host() { negotiator = AolAxisNegotiator(registry); }
extern "C" uint32_t plan_host(uint8_t mode, uint16_t word, uint8_t request, uint8_t permission, uint64_t now,
 const uint8_t *axis_bytes, size_t axis_size, const uint8_t *intent_bytes, size_t intent_size, bool engaged) {
 snapshot = {AOL_SAFETY_PROTOCOL_MAGIC, AOL_SAFETY_PROTOCOL_VERSION, request, permission, mode, word, 1U};
 AolAxisInput input;
 if (axis_size != 0U) {
  std::vector<capnp::word> words(axis_size / 8U);
  std::memcpy(words.data(), axis_bytes, axis_size);
  capnp::FlatArrayMessageReader reader(kj::ArrayPtr<const capnp::word>(words.data(), words.size()));
  auto event = reader.getRoot<cereal::Event>();
  auto axis = event.getAolAxisState();
  input = {true, axis.getQualified(), axis.getSessionId().cStr(), axis.getObservedMonoTime(),
           axis.getValidUntilMonoTime(), event.getLogMonoTime(), axis.getDesiredLateral(), axis.getDesiredLongitudinal()};
  if (intent_size != 0U) {
   std::vector<capnp::word> intent_words(intent_size / 8U);
   std::memcpy(intent_words.data(), intent_bytes, intent_size);
   capnp::FlatArrayMessageReader intent_reader(kj::ArrayPtr<const capnp::word>(intent_words.data(), intent_words.size()));
   auto intent = intent_reader.getRoot<cereal::Event>();
   input.retain_lateral_arm = aol_armed_intent(intent.getAolIntentWire(), intent.getLogMonoTime(),
        axis.getSourceCarStateMonoTime(), now, false, axis.getIntentProducerSessionId().cStr());
  }
 }
 auto plan = negotiator.prepare(snapshot, input, now, mode);
 pending = plan;
 const bool aol_write_ok = plan.capable;
''' + matched[0] + r'''
 return uint32_t(plan.request_mask) | (uint32_t(engaged) << 8U) | (uint32_t(plan.retain_lateral_arm) << 9U);
}
extern "C" void complete_host(uint8_t request, uint8_t permission) {
 snapshot.request_mask = request; snapshot.permission_mask = permission;
 negotiator.complete(pending, true, snapshot);
}
'''
  source = tmp / 'host.cc'
  code += r'''
#include <iostream>
template <typename T> bool read_value(T &value) { return bool(std::cin.read(reinterpret_cast<char *>(&value), sizeof(T))); }
static void respond(uint32_t value) { std::cout.write(reinterpret_cast<const char *>(&value), sizeof(value)); std::cout.flush(); }
int main() {
 uint8_t op;
 while (read_value(op)) {
  if (op == 0U) { reset_host(); respond(0U); }
  else if (op == 1U) {
   uint8_t mode, request, permission, engaged; uint16_t word; uint64_t now; uint32_t axis_size, intent_size;
   if (!read_value(mode) || !read_value(word) || !read_value(request) || !read_value(permission) || !read_value(now) ||
       !read_value(axis_size) || !read_value(intent_size) || !read_value(engaged)) return 2;
   std::vector<uint8_t> axis(axis_size), intent(intent_size);
   if (!std::cin.read(reinterpret_cast<char *>(axis.data()), axis_size) ||
       !std::cin.read(reinterpret_cast<char *>(intent.data()), intent_size)) return 3;
   respond(plan_host(mode, word, request, permission, now, axis.data(), axis_size, intent.data(), intent_size, engaged != 0U));
  } else if (op == 2U) {
   uint8_t request, permission; if (!read_value(request) || !read_value(permission)) return 4;
   complete_host(request, permission); respond(0U);
  } else if (op == 3U) { respond(0U); return 0; }
  else return 5;
 }
 return 0;
}
'''
  source.write_text(code)
  library = tmp / 'host-peer'
  generated = [root / ('openpilot/cereal/gen/cpp/' + name + '.capnp.c++') for name in ('log', 'car', 'custom', 'deprecated')]
  capnp_flags = ['-I', capnproto.INCLUDE_DIR, '-L', capnproto.LIB_DIR, '-lcapnp', '-lkj']
  subprocess.run(['c++', '-std=c++17', '-O1', '-Wall', '-Werror',
                  '-I', str(root), '-I', str(root / 'openpilot'), '-I', str(root / 'opendbc_repo'), '-I', str(root / 'openpilot/cereal/gen/cpp'),
                  str(source), *map(str, generated), *capnp_flags, '-o', str(library)], check=True, timeout=35)

  class Peer:
    def __init__(self):
      self.process = subprocess.Popen([str(library)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def exchange(self, raw):
      assert self.process.stdin is not None and self.process.stdout is not None
      assert self.process.stdin.write(raw) == len(raw)
      self.process.stdin.flush()
      result = self.process.stdout.read(4)
      assert len(result) == 4, self.process.poll()
      return struct.unpack('<I', result)[0]

    def reset_host(self):
      assert self.exchange(b'\x00') == 0

    def plan_host(self, mode, word, request, permission, now, axis, axis_size, intent, intent_size, engaged):
      assert axis_size == len(axis) and intent_size == len(intent)
      return self.exchange(b'\x01' + struct.pack('<BHBBQIIB', mode, word, request, permission, now,
                                               axis_size, intent_size, int(engaged)) + axis + intent)

    def complete_host(self, request, permission):
      assert self.exchange(b'\x02' + struct.pack('<BB', request, permission)) == 0

    def close(self):
      try:
        if self.process.poll() is None:
          assert self.exchange(b'\x03') == 0
        assert self.process.wait(timeout=5) == 0
      finally:
        if self.process.poll() is None:
          self.process.kill()
          self.process.wait(timeout=5)
        if self.process.stdin is not None:
          self.process.stdin.close()
        if self.process.stdout is not None:
          self.process.stdout.close()

  paths = ['openpilot/selfdrive/pandad/pandad.cc', 'openpilot/selfdrive/pandad/aol_wire.h',
           'openpilot/selfdrive/pandad/aol_protocol.h', 'openpilot/starpilot/car/hyundai/aol_policy.h',
           'opendbc_repo/opendbc/safety/modes/hyundai_canfd_aol.h', 'opendbc_repo/opendbc/safety/modes/hyundai_canfd_angle_aol.h']
  paths += [str(f.relative_to(root)) for f in generated]
  paths += [str(f.relative_to(root)) for f in (root / 'openpilot/cereal/gen/cpp').glob('*.capnp.h')]
  queued = [root / name for name in paths]
  closure = {}
  include_roots = [root, root / 'openpilot', root / 'opendbc_repo', root / 'openpilot/cereal/gen/cpp']
  while queued:
    path = queued.pop().resolve()
    name = str(path.relative_to(root))
    if name in closure:
      continue
    closure[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    if name == 'openpilot/selfdrive/pandad/pandad.cc':
      continue
    for quoted in re.findall(r'^\s*#include\s+"([^"\n]+)"', path.read_text(), re.M):
      target = next((directory / quoted for directory in [path.parent, *include_roots] if (directory / quoted).is_file()), None)
      assert target is not None, (name, quoted)
      queued.append(target)
  print(json.dumps({'hostHeartbeatSourceClosure': closure, 'artifactSha256': hashlib.sha256(library.read_bytes()).hexdigest()}))
  peer = Peer()
  try:
    yield peer
  finally:
    peer.close()


@pytest.mark.parametrize('raw_fault', [0, 8])
@pytest.mark.parametrize('unavailable_ticks', [3, 8])
def test_real_classic_stock_engagement_then_location_loss_keeps_intent(raw_fault, unavailable_ticks, monkeypatch, host_heartbeat_bridge):
  shared_long = False
  native = libsafety_py.libsafety
  case = caller_fixture.IoniqAvailabilityTests(methodName='runTest')
  card = None
  with OpenpilotPrefix(), patch.dict(os.environ, {'SIMULATION': '0', 'AOL_REPLAY_RUNTIME': '1'}):
    saved = Params()
    saved.put_bool('OpenpilotEnabledToggle', True, block=True)
    saved.put_bool('AlwaysOnLateral', True, block=True)
    saved.put('LKASButtonControl', AOL_TOGGLE, block=True)
    saved.put('MainCruiseButtonControl', AOL_TOGGLE, block=True)
    now = [10_000_000_000]
    for key in ('CancelButtonControl', 'LongCancelButtonControl', 'VeryLongCancelButtonControl'):
      saved.put(key, 0, block=True)
    try:
      saved.put_bool('AlphaLongitudinalEnabled', False, block=True)
      saved.put_bool('IsReleaseBranch', False, block=True)
      fp = gen_empty_fingerprint()
      fp[0].update({0x391: 8, 0x38d: 8})
      fp[2][0x485] = 4
      fp[1][0x500] = 8
      cp = CarInterface.get_params(CAR.HYUNDAI_SONATA_HYBRID, fp, [], False, False, False)
      assert int(cp.flags) == 0x1011580
      assert qualified_classic(cp) and cp.pcmCruise and not cp.openpilotLongitudinalControl
      assert cp.lateralTuning.which() == 'torque' and not CarInterface.startup_required(cp)
      card = card_module.Car(CI=CarInterface(cp), RI=RadarInterface(cp))
      assert qualified_classic(card.CP, marked_only=True)
      assert card.CP.safetyConfigs[0].safetyParam == 0xC02 and card.CP.alternativeExperience == 32
      assert isinstance(card.aol_card_intent, ClassicSccCardIntent)
      assert not card.ci_initialized and not saved.get_bool('ControlsReady')
      card.slc_producer_session = 'authority-card'
      card.CI.update([])
      sd, controls, drive = case._authority_callers(False, cp=card.CP.as_reader().as_builder(), genuine_events=True)

      sd.aol_settings = read_settings(saved)
      assert sd.aol_settings.lkas_action == AOL_TOGGLE and sd.aol_settings.main_action == AOL_TOGGLE
      assert sd.CP.to_dict() == card.CP.to_dict() == controls.CP.to_dict()
      assert len(card.CP.safetyConfigs) == 1 and bool(card.CP.openpilotLongitudinalControl) == shared_long
      config = card.CP.safetyConfigs[0]
      model, word = int(config.safetyModel.raw), int(config.safetyParam)
      native.set_alternative_experience(card.CP.alternativeExperience)
      assert native.set_safety_hooks(model, word) == 0
      native.init_tests()
      host_heartbeat_bridge.reset_host()
      native.set_alternative_experience(card.CP.alternativeExperience)
      queued, received, previous_sd = [], {}, {}
      tick = 0
      native_counts = {'accepted': 0, 'blocked': 0, 'nonzero': 0, 'lateral_only_nonzero': 0}
      freeze_native = [None]
      last_native = [None]
      telemetry_rate_bad = [False]
      patch_scope = monkeypatch.context()
      local = patch_scope.__enter__()
      try:
        diagnostic_requests = []
        def diagnostic_query(sender, receiver, bus, addresses, requests, responses):
          assert bus == 0 and addresses == [(0x7d0, None)]
          assert len(requests) == 1
          request = requests[0]
          assert request in (b'\x10\x03', b'\x28\x83\x01')
          diagnostic_requests.append(request)
          def receive(timeout):
            sender([(0x7d0, bytes([len(request)]) + request + bytes(7-len(request)), bus)])
            return {(0x7d0, None): b'\x50\x03'} if request == b'\x10\x03' else {}
          return SimpleNamespace(get_data=receive)
        local.setattr(disable_ecu_module, 'IsoTpParallelQuery', diagnostic_query)
        local.setattr(card_module, 'REPLAY', False)
        local.setattr(card_module.time, 'monotonic_ns', lambda: now[0])
        local.setattr(card_module.time, 'clock_gettime_ns', lambda clock: now[0])
        local.setattr(card_module.messaging, 'drain_sock_raw', lambda *args, **kwargs: list(queued))
        local.setattr(card.sm, 'update', lambda *args: None)
        def panda_cadence(original):
          due = [0.0]
          def update(stamp, messages):
            delivered = []
            for message in messages:
              if message.which() == 'pandaStates':
                if stamp + 1e-9 < due[0]:
                  continue
                due[0] = stamp + (0.0 if telemetry_rate_bad[0] else 0.1)
              delivered.append(message)
            return original(stamp, delivered)
          return update
        local.setattr(sd.sm, 'update_msgs', panda_cadence(sd.sm.update_msgs))
        motion_unavailable = [False]
        update_messages = sd.sm.update_msgs
        def motion_messages(timestamp, messages):
          delivered = []
          for message in messages:
            if message.which() == 'deviceMotion':
              message = message.as_builder()
              message.deviceMotion.inputsOK = not motion_unavailable[0]
              message = message.as_reader()
            delivered.append(message)
          return update_messages(timestamp, delivered)
        local.setattr(sd.sm, 'update_msgs', motion_messages)

        local.setattr(card.sm, 'update_msgs', panda_cadence(card.sm.update_msgs))
        create_message = messaging.new_message
        def clocked_message(service=None, size=None, **kwargs):
          kwargs.setdefault('logMonoTime', now[0])
          event = create_message(service, size, **kwargs)
          if service == 'aolSafetyWire' and freeze_native[0] is not None:
            event.logMonoTime = freeze_native[0].observedMonoTime
          if service == 'pandaStates' and raw_fault:
            for panda in event.pandaStates:
              panda.faults = ['interruptRateCan2']
          return event
        local.setattr(messaging, 'new_message', clocked_message)
        original_encode = caller_fixture.encode_safety
        def recorded_fault_wire(state):
          state = dataclasses.replace(state, pandaInventory=tuple(
            dataclasses.replace(slot, faults=slot.faults | raw_fault) for slot in state.pandaInventory))
          last_native[0] = state
          return original_encode(freeze_native[0] or state)
        local.setattr(caller_fixture, 'encode_safety', recorded_fault_wire)
        send = card.pm.send
        def capture(service, event):
          raw = event if isinstance(event, bytes) else event.to_bytes()
          received[service] = messaging.log_from_bytes(raw).as_builder()
          return send(service, raw)
        local.setattr(card.pm, 'send', capture)

        def step(*, main=1, lkas=0, button=0, engaged=False, withdraw_request=False, bad_checksum=False,
                 heartbeat_override=None, inject_relay=False, drop_wheel=False):
          nonlocal tick
          tick += 1
          now[0] = source_ns = 10_000_000_000 + tick * 10_000_000
          prior_axis = previous_sd.get('aolAxisState')
          prior_state = previous_sd.get('selfdriveState')
          native.set_timer(source_ns // 1000)
          if tick % 10 == 0:
            axis_bytes = prior_axis.as_builder().to_bytes() if prior_axis is not None else b''
            intent_message = received.get('aolIntentWire')
            intent_bytes = intent_message.to_bytes() if intent_message is not None else b''
            packed = host_heartbeat_bridge.plan_host(model, word, native.aol_get_request_mask(), native.aol_get_permission_mask(),
              source_ns, axis_bytes, len(axis_bytes), intent_bytes, len(intent_bytes),
              prior_state is not None and prior_state.selfdriveState.enabled)
            request = (packed & 0xFF) if not withdraw_request else 0
            heartbeat = bool(packed & 0x100) if heartbeat_override is None else heartbeat_override
            native.aol_set_host_request(request)
            native.set_aol_test_heartbeat(heartbeat)
            host_heartbeat_bridge.complete_host(native.aol_get_request_mask(), native.aol_get_permission_mask())
          frames = classic_frames(card.CI, tick, main=main, lkas=lkas, button=button, engaged=engaged)
          if drop_wheel:
            frames = [frame for frame in frames if frame[0] != 0x386]
          if bad_checksum:
            frames = [(address, bytes([data[0] ^ 1]) + data[1:] if address == 0x386 else data, bus)
                      for address, data, bus in frames]
          if inject_relay:
            camera = next(frame for frame in frames if frame[0] == 0x340 and frame[2] == 2)
            frames.append((camera[0], camera[1], 0))
          for address, data, bus in frames:
            accepted = native.safety_rx_hook(libsafety_py.make_CANPacket(address, bus, data))
            assert bool(accepted) == (not bad_checksum or address != 0x386), (hex(address), bus)
          if tick % 100 == 0:
            native.safety_tick()
          actual_request = native.aol_get_request_mask()
          mask = native.aol_get_permission_mask()



          rx_invalid = bool(native.safety_test_toyota_rx_summary_invalid())
          relay_fault = bool(native.get_relay_malfunction())
          assert not relay_fault or inject_relay
          batch = messaging.log_from_bytes(card_module.can_list_to_can_capnp(frames)).as_builder()
          batch.logMonoTime = source_ns
          queued[:] = [batch.to_bytes()]
          device = messaging.new_message('deviceState', valid=True, logMonoTime=source_ns)
          device.deviceState.started, device.deviceState.startedMonoTime = True, 9_000_000_000
          pandas = messaging.new_message('pandaStates', 1, valid=True, logMonoTime=source_ns)
          panda = pandas.pandaStates[0]
          panda.safetyModel, panda.safetyParam = config.safetyModel, word
          panda.alternativeExperience = card.CP.alternativeExperience
          panda.controlsAllowed = native.get_controls_allowed()
          panda.safetyRxChecksInvalid = rx_invalid
          panda.ignitionLine = True
          panda.faults = (['interruptRateCan2'] if raw_fault else []) + (['relayMalfunction'] if relay_fault else [])
          slot = PandaSlot(0, 'authority-panda', model, word, card.CP.alternativeExperience,
                           native.get_controls_allowed(), safetyRxChecksInvalid=rx_invalid, faults=raw_fault | int(relay_fault))
          wire = messaging.new_message('aolSafetyWire', 0, valid=True, logMonoTime=source_ns)
          wire.aolSafetyWire = encode_safety(SafetyState(1, True, source_ns, source_ns + 30_000_000,
            model, word, bool(mask & 1), bool(mask & 2), bool(actual_request & 1), bool(actual_request & 2),
            'authority-panda', card.aol_axis_session(), source_ns, (slot,)))
          messages = [device.as_reader(), pandas.as_reader(), wire.as_reader()]
          messages += [previous_sd[name] for name in ('selfdriveState', 'aolAxisState', 'onroadEvents') if name in previous_sd]
          if 'onroadEvents' not in previous_sd:
            messages.append(messaging.new_message('onroadEvents', 0, valid=True, logMonoTime=source_ns).as_reader())
          card.sm.update_msgs(source_ns / 1e9, messages)
          state, _ = card.state_update()
          if state.canValid and tick % 2 == 0:
            assert state.cruiseState.available == bool(main)
          for service in ('carState', 'carOutput', 'aolIntentWire', 'starpilotCarState'):
            received.pop(service, None)
          card.state_publish(state, None)
          state_envelope = received['carState']
          output_envelope = received['carOutput']
          assert state_envelope.logMonoTime == source_ns and state_envelope.valid == state.canValid
          assert output_envelope.logMonoTime == source_ns

          axis, host, command = drive(tick, intent_message=received['aolIntentWire'],
            car_state_message=state_envelope, car_output_message=output_envelope,
            companion=False, authority_message=received.get('starpilotCarState'),
            allowed=bool(mask & 1), long_allowed=bool(mask & 2), native_request_lat=bool(actual_request & 1),
            native_request_long=bool(actual_request & 2), panda_controls_allowed=native.get_controls_allowed(),
            panda_fault=rx_invalid, relay_fault=relay_fault)
          if sd.conditional_car_state_valid:
            assert sd.aol_car_state_log_ns == state_envelope.logMonoTime
          assert controls.sm.logMonoTime['carState'] == state_envelope.logMonoTime
          assert controls.sm.valid['carState'] == state_envelope.valid
          assert controls.sm['carState'].to_dict() == state_envelope.carState.to_dict()
          assert controls.sm.logMonoTime['carOutput'] == output_envelope.logMonoTime
          assert controls.sm.valid['carOutput'] == output_envelope.valid
          assert controls.sm['carOutput'].to_dict() == output_envelope.carOutput.to_dict()
          previous_sd.update(sd.pm.messages)
          validated = current_native(sd.sm, sd.CP, now_ns=source_ns, axis_session_id=sd.aol_session_id)
          if not rx_invalid and not relay_fault and freeze_native[0] is None and state.canValid and native.safety_config_valid():
            assert validated is not None, 'RECORDED_IRQ_ONLY_CURRENT_NATIVE_REJECTED'
            assert validated is not None and validated.lateralAllowed == bool(mask & 1)
            assert validated.requestedLateral == bool(actual_request & 1)
            assert validated.pandaInventory[0].faults == raw_fault
            assert list(sd.sm['pandaStates'][0].faults) == ([log.PandaState.FaultType.interruptRateCan2] if raw_fault else [])
          else:
            assert not axis.lateralActive and not axis.longitudinalActive
            assert not command.latActive and not command.longActive

          now[0] += 1
          card.sm.update_msgs(now[0] / 1e9, [controls.pm.messages['carControl']])
          received.pop('sendcan', None)
          card.controls_update(state, card.sm['carControl'])
          steering = []
          if state.canValid:
            assert 'sendcan' in received and received['sendcan'].valid
            steering = [frame for frame in received['sendcan'].sendcan
                        if frame.address == 0x340 and frame.src == 0]
            assert steering
            for frame in received['sendcan'].sendcan:
              accepted = bool(native.safety_tx_hook(libsafety_py.make_CANPacket(frame.address, frame.src, frame.dat)))
              assert accepted == (not relay_fault)
              native_counts['accepted' if accepted else 'blocked'] += 1
              if accepted and frame.address == 0x340 and not command.latActive:
                assert (int.from_bytes(bytes(frame.dat)[2:4], 'little') & 0x7ff) == 1024
                assert not bytes(frame.dat)[3] & 8
              if accepted and frame.address == 0x340 and (int.from_bytes(bytes(frame.dat)[2:4], 'little') & 0x7ff) != 1024:
                native_counts['nonzero'] += 1
                native_counts['lateral_only_nonzero'] += int(not native.get_controls_allowed())
          return state, axis, host, command, steering

        for _ in range(120):
          step(main=0)
        assert native.safety_config_valid()
        for _ in range(2):
          step(main=1, button=2, engaged=False)
        for _ in range(30):
          state, axis, host, command, steering = step(main=1, engaged=True)
        assert native.get_controls_allowed() and state.cruiseState.enabled
        assert axis.lateralActive and command.latActive and host.enabled, axis.to_dict()
        assert not axis.longitudinalActive and not command.longActive
        nonzero = False
        for _ in range(30):
          state, axis, host, command, steering = step(main=1, engaged=True)
          assert host.enabled and native.get_controls_allowed()
          assert axis.lateralActive and command.latActive
          assert not axis.longitudinalActive and not command.longActive
          nonzero |= abs(command.actuators.torque) > 0
          assert steering
        assert nonzero
        assert card.aol_card_intent.physical_latch and card.aol_card_intent.allowed_latch
        assert not card.aol_card_intent.fault_rearm
        for _ in range(20):
          state, axis, host, command, steering = step(main=1, engaged=False)
          if not state.cruiseState.enabled:
            assert state.cruiseState.available and not native.get_controls_allowed()
            assert native.aol_get_permission_mask() == 1
            assert axis.lateralActive and command.latActive, 'FULL_ENGAGEMENT_LAT_WITHDRAWN'
            assert not axis.longitudinalActive and not command.longActive and not host.enabled
            assert card.aol_card_intent.allowed_latch and not card.aol_card_intent.pause_lateral
        assert not state.cruiseState.enabled and state.cruiseState.available
        assert native_counts['lateral_only_nonzero'] > 0
        motion_unavailable[0] = True
        fault_ticks = 0
        baseline = None
        for _ in range(15):
          state, axis, host, command, _ = step(main=1, engaged=False)
          if log.OnroadEvent.EventName.locationdTemporaryError not in sd.events.names:
            continue
          fault_ticks += 1
          if fault_ticks == 1:
            assert_stock_location_warning_contract(sd, card, state, now[0])
          assert log.OnroadEvent.EventName.pcmDisable in sd.events.names
          assert not axis.lateralActive and not axis.longitudinalActive
          assert not command.latActive and not command.longActive
          assert card.aol_card_intent.allowed_latch, 'CLASSIC_LOCATION_PCM_OFF_CLEARED_ESTABLISHED_INTENT'
          observed = (card.aol_transport_feedback.lateral_baseline_ns, card.aol_transport_feedback.lateral_native_ns)
          if baseline is None:
            baseline = observed
          else:
            assert observed == baseline
          if fault_ticks == unavailable_ticks:
            break
        assert fault_ticks == unavailable_ticks
        motion_unavailable[0] = False
        for _ in range(20):
          state, axis, host, command, _ = step(main=1, engaged=False)
          assert card.aol_card_intent.allowed_latch
          assert not axis.longitudinalActive and not command.longActive and not host.enabled
        assert axis.lateralActive and command.latActive
      finally:
        patch_scope.__exit__(None, None, None)
    finally:
      native.init_tests()
      native.set_alternative_experience(0)
      native.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)
      case.doCleanups()
      if card is not None:
        card.vehicle_startup.close()
