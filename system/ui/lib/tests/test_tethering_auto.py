from openpilot.system.ui.lib import tethering_auto as ta


class _Wifi:
  def __init__(self, available=True, active=False, follows=True):
    self.available = available
    self.active = active
    self.follows = follows          # the hotspot really changes when asked
    self.calls = []

  def tethering_available(self):
    return self.available

  def is_tethering_active(self):
    return self.active

  def set_ipv4_forward(self, enabled):
    self.calls.append(("forward", enabled))

  def set_tethering_active(self, active):
    self.calls.append(("tether", active))
    if self.follows:
      self.active = active


class _Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def _switch(wifi, state, clock=None, share=True):
  return ta.TetheringAutoSwitch(wifi, read_mode=lambda: state["mode"], read_onroad=lambda: state["onroad"],
                                share_allowed=lambda: share, clock=clock or _Clock())


def test_wanted_table():
  assert ta.tethering_wanted(ta.OFF, True) is False
  assert ta.tethering_wanted(ta.OFF, False) is False
  assert ta.tethering_wanted(ta.ALWAYS, False) is True
  assert ta.tethering_wanted(ta.ONROAD, True) is True
  assert ta.tethering_wanted(ta.ONROAD, False) is False
  assert ta.tethering_wanted(3, True) is False   # the old Qt "until reboot" value, or anything unknown: off


def test_onroad_mode_follows_the_drive():
  wifi, state = _Wifi(), {"mode": ta.ONROAD, "onroad": False}
  sw = _switch(wifi, state)
  assert sw.step() is None and wifi.calls == []
  state["onroad"] = True
  assert sw.step() == "on"
  assert wifi.calls == [("forward", True), ("tether", True)]   # forwarding decided before the hotspot comes up
  assert sw.step() is None
  state["onroad"] = False
  assert sw.step() == "off"                                     # a new target acts at once, no retry wait
  assert wifi.calls[-1] == ("tether", False) and wifi.active is False


def test_waits_for_the_wifi_backend():
  wifi, state = _Wifi(available=False), {"mode": ta.ALWAYS, "onroad": False}
  sw = _switch(wifi, state)
  assert sw.step() is None and wifi.calls == []
  wifi.available = True
  assert sw.step() == "on"


def test_retries_an_attempt_that_did_not_take_after_the_retry_time():
  clock = _Clock()
  wifi, state = _Wifi(follows=False), {"mode": ta.ONROAD, "onroad": True}
  sw = _switch(wifi, state, clock=clock)
  assert sw.step() == "on"
  clock.t += ta.RETRY_S - 1
  assert sw.step() is None
  clock.t += 2
  assert sw.step() == "on"
  assert [c for c in wifi.calls if c[0] == "tether"] == [("tether", True), ("tether", True)]


def test_off_mode_turns_a_running_hotspot_off_and_sharing_rule_is_passed_through():
  wifi, state = _Wifi(active=True), {"mode": ta.OFF, "onroad": True}
  assert _switch(wifi, state).step() == "off"
  assert ("forward", True) not in wifi.calls
  wifi2, state2 = _Wifi(), {"mode": ta.ALWAYS, "onroad": False}
  _switch(wifi2, state2, share=False).step()
  assert wifi2.calls[0] == ("forward", False)
