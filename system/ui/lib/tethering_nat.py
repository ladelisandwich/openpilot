"""The masquerade for the hotspot, done by hand.

NetworkManager's shared mode (the Hotspot connection, ipv4.method shared) runs dnsmasq for the clients and installs a
MASQUERADE rule through /usr/sbin/iptables so their traffic leaves over the cellular link. On AGNOS that binary is the
nf_tables build, and the comma three's 4.9 kernel has no nf_tables: every call fails ("iptables: command
/usr/sbin/iptables failed: Child process exited with code 4" in NetworkManager's journal), so tethered devices get an
address and DNS but no route out. The legacy tool talks to the same kernel's xtables, so install NetworkManager's
rules through it when tethering comes up, and take them out again when it goes down.
"""
import shutil
import subprocess
from collections.abc import Callable

TETHERING_SUBNET = "192.168.43.0/24"   # the Hotspot connection's ipv4.addresses (TETHERING_IP_ADDRESS/24)

# (table, chain, rule): what NetworkManager's shared mode installs for a shared subnet
RULES: tuple[tuple[str, str, list[str]], ...] = (
  ("nat", "POSTROUTING", ["-s", TETHERING_SUBNET, "!", "-d", TETHERING_SUBNET, "-j", "MASQUERADE"]),
  ("filter", "FORWARD", ["-s", TETHERING_SUBNET, "-j", "ACCEPT"]),
  ("filter", "FORWARD", ["-d", TETHERING_SUBNET, "-m", "state", "--state", "ESTABLISHED,RELATED", "-j", "ACCEPT"]),
)

Runner = Callable[..., subprocess.CompletedProcess]


def iptables_tool() -> str:
  return shutil.which("iptables-legacy") or "iptables"


def _run(run: Runner, tool: str, table: str, op: str, chain: str, rule: list[str]) -> int:
  return run(["sudo", tool, "-t", table, op, chain, *rule], check=False, capture_output=True, timeout=10).returncode


def ensure_tethering_nat(enabled: bool, run: Runner = subprocess.run, tool: str | None = None) -> dict[str, str]:
  """Make the rules present (enabled) or absent (not enabled). Returns what was done per rule, for the log."""
  tool = tool or iptables_tool()
  done: dict[str, str] = {}
  if enabled:
    run(["sudo", "sysctl", "-w", "net.ipv4.ip_forward=1"], check=False, capture_output=True, timeout=10)
  for table, chain, rule in RULES:
    key = f"{table}/{chain} {' '.join(rule)}"
    present = _run(run, tool, table, "-C", chain, rule) == 0
    if enabled and not present:
      done[key] = "added" if _run(run, tool, table, "-A", chain, rule) == 0 else "add failed"
    elif not enabled and present:
      done[key] = "removed" if _run(run, tool, table, "-D", chain, rule) == 0 else "remove failed"
    else:
      done[key] = "present" if present else "absent"
  return done
