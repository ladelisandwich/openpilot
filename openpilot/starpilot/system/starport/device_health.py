"""Cheap device load readings for the 30 s Android Auto stats line: plain file reads, no D-Bus and no subprocesses."""

from __future__ import annotations

import glob


def _read(path: str) -> str | None:
  try:
    with open(path) as handle:
      return handle.read(4096)
  except OSError:
    return None


def _first_number(path: str) -> float | None:
  text = _read(path)
  try:
    return float(text.split()[0]) if text else None
  except (ValueError, IndexError):
    return None


def _meminfo_available_mb() -> int | None:
  for line in (_read("/proc/meminfo") or "").splitlines():
    if line.startswith("MemAvailable:"):
      try:
        return int(line.split()[1]) // 1024
      except (ValueError, IndexError):
        return None
  return None


def wireless_link(interface: str = "wlan0") -> dict:
  """Link quality, signal and noise of the joined network from /proc/net/wireless."""
  for line in (_read("/proc/net/wireless") or "").splitlines():
    name, _, rest = line.partition(":")
    if name.strip() != interface:
      continue
    fields = rest.split()
    try:
      return {"wifi_link": int(float(fields[1].rstrip("."))), "wifi_dbm": int(float(fields[2].rstrip("."))),
              "wifi_noise": int(float(fields[3].rstrip(".")))}
    except (ValueError, IndexError):
      return {}
  return {}


def sample(interface: str = "wlan0") -> dict:
  """Load, free memory, CPU clocks, hottest thermal zone and Wi-Fi link; absent readings are left out."""
  load = _read("/proc/loadavg")
  temps = [value / 1000 for path in glob.glob("/sys/class/thermal/thermal_zone*/temp") if (value := _first_number(path)) is not None]
  clocks = {f"cpu{core}_mhz": round(khz / 1000) for core in (0, 4)
            if (khz := _first_number(f"/sys/devices/system/cpu/cpu{core}/cpufreq/scaling_cur_freq")) is not None}
  values = {"load1": float(load.split()[0]) if load else None, "mem_available_mb": _meminfo_available_mb(),
            "temp_max_c": round(max(temps), 1) if temps else None, **clocks, **wireless_link(interface)}
  return {key: value for key, value in values.items() if value is not None}
