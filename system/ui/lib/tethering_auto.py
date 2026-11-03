"""Hotspot by driving state: the TetheringEnabled setting picks Off, Always, or Only Onroad.

Only Onroad shares the LTE connection over the comma's Wi-Fi while the car is on and turns the hotspot off when it
goes offroad, so the device rejoins its saved Wi-Fi (home) when parked for uploads and updates. The comma has one
Wi-Fi radio: while the hotspot is on it is not a Wi-Fi client.

The switch runs on its own thread: the network panel only updates while it is on screen, and the Wi-Fi backend
initialises in the background after the UI starts. Each step compares the wanted state with the hotspot's and acts
on a mismatch; an attempt that has not taken effect is retried after RETRY_S, a change of target acts at once.
"""
import threading
import time
from collections.abc import Callable

OFF, ALWAYS, ONROAD = 0, 1, 2
MODE_PARAM = "TetheringEnabled"
PERIOD_S = 2.0
RETRY_S = 20.0


def tethering_wanted(mode: int, onroad: bool) -> bool:
  return mode == ALWAYS or (mode == ONROAD and onroad)


class TetheringAutoSwitch:
  def __init__(self, wifi, read_mode: Callable[[], int], read_onroad: Callable[[], bool], share_allowed: Callable[[], bool],
               log: Callable[[dict], None] | None = None, clock: Callable[[], float] = time.monotonic):
    self._wifi = wifi
    self._read_mode = read_mode
    self._read_onroad = read_onroad
    self._share_allowed = share_allowed
    self._log = log or (lambda _msg: None)
    self._clock = clock
    self._last_target: bool | None = None
    self._last_action = 0.0
    self._thread: threading.Thread | None = None

  def step(self) -> str | None:
    """One reconcile. Returns "on" or "off" when it asked the Wi-Fi manager to change the hotspot, else None."""
    if not self._wifi.tethering_available():
      return None
    mode = self._read_mode()
    onroad = self._read_onroad()
    wanted = tethering_wanted(mode, onroad)
    if wanted == self._wifi.is_tethering_active():
      return None
    now = self._clock()
    if wanted == self._last_target and now - self._last_action < RETRY_S:
      return None
    self._last_target, self._last_action = wanted, now
    if wanted:
      self._wifi.set_ipv4_forward(self._share_allowed())   # before activation: the manager reads it 5 s after
    self._wifi.set_tethering_active(wanted)
    action = "on" if wanted else "off"
    self._log({"event": "tethering auto", "action": action, "mode": mode, "onroad": onroad})
    return action

  def start(self) -> None:
    if self._thread is not None:
      return

    def loop():
      while True:
        try:
          self.step()
        except Exception as e:  # never take the UI down over the hotspot
          self._log({"event": "tethering auto error", "error": repr(e)})
        time.sleep(PERIOD_S)

    self._thread = threading.Thread(target=loop, name="tethering_auto", daemon=True)
    self._thread.start()
