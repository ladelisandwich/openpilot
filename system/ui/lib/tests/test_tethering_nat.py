import subprocess

from openpilot.system.ui.lib import tethering_nat as tn


class _Kernel:
  """A fake iptables: remembers which rules are present, answers -C / -A / -D like the real tool."""

  def __init__(self, present=(), fail_add=False):
    self.rules = set(present)
    self.fail_add = fail_add
    self.calls = []

  def __call__(self, cmd, **kwargs):
    self.calls.append(cmd)
    if cmd[1] == "sysctl":
      return subprocess.CompletedProcess(cmd, 0)
    table, op, chain, rule = cmd[3], cmd[4], cmd[5], tuple(cmd[6:])
    key = (table, chain, rule)
    if op == "-C":
      return subprocess.CompletedProcess(cmd, 0 if key in self.rules else 1)
    if op == "-A":
      if self.fail_add:
        return subprocess.CompletedProcess(cmd, 3)
      self.rules.add(key)
      return subprocess.CompletedProcess(cmd, 0)
    if op == "-D":
      self.rules.discard(key)
      return subprocess.CompletedProcess(cmd, 0)
    raise AssertionError(cmd)


def _expected_rules():
  return {(table, chain, tuple(rule)) for table, chain, rule in tn.RULES}


def test_enable_adds_networkmanagers_rules_through_the_given_tool_and_turns_forwarding_on():
  k = _Kernel()
  done = tn.ensure_tethering_nat(True, run=k, tool="iptables-legacy")
  assert k.rules == _expected_rules()
  assert set(done.values()) == {"added"}
  assert k.calls[0] == ["sudo", "sysctl", "-w", "net.ipv4.ip_forward=1"]
  assert all(c[1] == "iptables-legacy" and c[0] == "sudo" for c in k.calls[1:])
  assert ["sudo", "iptables-legacy", "-t", "nat", "-A", "POSTROUTING", "-s", tn.TETHERING_SUBNET, "!", "-d", tn.TETHERING_SUBNET,
          "-j", "MASQUERADE"] in k.calls


def test_enable_is_idempotent_when_rules_are_already_there():
  k = _Kernel(present=_expected_rules())
  done = tn.ensure_tethering_nat(True, run=k, tool="iptables-legacy")
  assert set(done.values()) == {"present"}
  assert not any(c[4] in ("-A", "-D") for c in k.calls if c[1] != "sysctl")


def test_disable_removes_only_what_is_present():
  nat_only = {r for r in _expected_rules() if r[0] == "nat"}
  k = _Kernel(present=nat_only)
  done = tn.ensure_tethering_nat(False, run=k, tool="iptables-legacy")
  assert k.rules == set()
  assert sorted(done.values()) == ["absent", "absent", "removed"]
  assert not any(c[1] == "sysctl" for c in k.calls)


def test_add_failure_is_reported_not_raised():
  k = _Kernel(fail_add=True)
  done = tn.ensure_tethering_nat(True, run=k, tool="iptables-legacy")
  assert set(done.values()) == {"add failed"}


def test_tool_prefers_the_legacy_binary(monkeypatch):
  monkeypatch.setattr(tn.shutil, "which", lambda name: "/usr/sbin/iptables-legacy" if name == "iptables-legacy" else None)
  assert tn.iptables_tool() == "/usr/sbin/iptables-legacy"
  monkeypatch.setattr(tn.shutil, "which", lambda name: None)
  assert tn.iptables_tool() == "iptables"
