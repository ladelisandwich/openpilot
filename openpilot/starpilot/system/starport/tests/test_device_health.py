from openpilot.starpilot.system.starport import device_health


def test_wireless_link_reads_quality_signal_and_noise(monkeypatch):
  text = "Inter-| sta-|   Quality   |\n face | tus | link level noise |\n wlan0: 0000   58.  -52.  -256.        0\n"
  monkeypatch.setattr(device_health, "_read", lambda path: text if path == "/proc/net/wireless" else None)
  assert device_health.wireless_link() == {"wifi_link": 58, "wifi_dbm": -52, "wifi_noise": -256}
  assert device_health.wireless_link("wlan1") == {}


def test_sample_leaves_out_what_the_device_does_not_have(monkeypatch):
  monkeypatch.setattr(device_health, "_read", lambda path: "0.75 0.5 0.4 1/200 99\n" if path == "/proc/loadavg" else None)
  monkeypatch.setattr(device_health.glob, "glob", lambda pattern: [])
  assert device_health.sample() == {"load1": 0.75}
