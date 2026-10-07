#!/usr/bin/env python3
"""Mazda sim launcher: the program you open on your PC.

  python tools/mazda_sim/launcher/mazdasim.py            (then http://localhost:8765 opens)

Pick the openpilot build (a branch on GitHub, or a checkout on this PC), set up the car, press Start. The launcher
runs two containers, the simulated CX-9 2023 + TI1 and the comma running that build, and the page becomes the
car's dashboard: steering, pedals, stalk buttons, faults, telemetry, the comma's screen.

Backends
  docker  (default) Docker Desktop on Windows/macOS or Docker Engine on Linux. Nothing else to install.
  native  Linux (Ubuntu 24.04) without containers: the same two halves as local processes; the comma's UI opens
          as a window on your desktop. Needs openpilot's Ubuntu dependencies (tools/install_ubuntu_dependencies.sh).

Stdlib only, so any Python 3.10+ runs it.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from collections import deque
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.parse import parse_qs, urlparse
from urllib.request import urlopen

HERE = Path(__file__).resolve().parent
SIM = HERE.parent
TOOLS = SIM.parent
REPO = TOOLS.parent
WEB = HERE / "web"
DOCKER = SIM / "docker"
sys.path.insert(0, str(TOOLS))

from mazda_sim.common.config import SimConfig  # stdlib only, like this launcher

PROJECT = "mazda-sim"
GPU_OVERRIDES = {"nvidia": "compose.nvidia.yml", "dri": "compose.dri.yml", "amd": "compose.amd.yml"}
DEFAULT_SETTINGS = {"backend": "docker", "gpu": "none", "bind": "127.0.0.1"}


ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
PROCESS_LIST = re.compile(r"^[a-z0-9_ ]+$")


class Logs:
  """Recent output from the launcher, docker and both halves of the sim, for the page's log view."""

  def __init__(self, size: int = 4000):
    self.lines: deque = deque(maxlen=size)
    self.seq = 0
    self.lock = threading.Lock()
    self.last_process_list = ""

  def add(self, source: str, text: str) -> None:
    with self.lock:
      for raw in text.rstrip("\n").splitlines() or [""]:
        line = ANSI.sub("", raw).rstrip()
        if raw != line and PROCESS_LIST.match(line):
          # the manager prints its (coloured) list of running processes every second: keep only the changes
          if line == self.last_process_list:
            continue
          self.last_process_list = line
          line = "running: " + line
        self.seq += 1
        self.lines.append((self.seq, source, line))

  def since(self, seq: int) -> tuple[int, list]:
    with self.lock:
      return self.seq, [x for x in self.lines if x[0] > seq]


class Backend:
  name = ""

  def __init__(self, home: Path, logs: Logs, settings: dict):
    self.home = home
    self.logs = logs
    self.settings = settings
    self.busy = ""        # what a start/stop in progress is doing

  def config(self) -> SimConfig:
    return SimConfig.load(str(self.home / "sim.json"))

  def _pump(self, source: str, proc: subprocess.Popen) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
      self.logs.add(source, line)

  def _task(self, label: str, fn) -> bool:
    if self.busy:
      return False
    self.busy = label

    def go():
      try:
        fn()
      except Exception as e:  # report, never kill the launcher
        self.logs.add("launcher", f"{label} failed: {e}")
      finally:
        self.busy = ""
    threading.Thread(target=go, daemon=True).start()
    return True


class DockerBackend(Backend):
  name = "docker"

  def __init__(self, *a, **k):
    super().__init__(*a, **k)
    self.follower: subprocess.Popen | None = None

  def env(self) -> dict:
    cfg = self.config()
    local = cfg.build.local_path if cfg.build.source == "local" and cfg.build.local_path else ""
    if local and not Path(local).is_dir():
      raise RuntimeError(f"local build {local} does not exist")
    (self.home / "no-local-build").mkdir(exist_ok=True)
    return {**os.environ, "MAZDA_SIM_HOME": str(self.home), "MAZDA_SIM_BIND": self.settings.get("bind", "127.0.0.1"),
            "MAZDA_SIM_LOCAL_BUILD": str(Path(local).resolve()) if local else str(self.home / "no-local-build")}

  def compose(self, *args: str) -> list[str]:
    files = ["-f", str(DOCKER / "compose.yml")]
    gpu = self.settings.get("gpu", "none")
    if gpu in GPU_OVERRIDES:
      files += ["-f", str(DOCKER / GPU_OVERRIDES[gpu])]
    return ["docker", "compose", "-p", PROJECT, *files, *args]

  def run(self, *args: str, source: str = "docker") -> int:
    cmd = self.compose(*args)
    self.logs.add("launcher", "$ " + " ".join(cmd[2:]))
    p = subprocess.Popen(cmd, env=self.env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         encoding="utf-8", errors="replace")
    self._pump(source, p)
    rc = p.wait()
    if rc != 0:
      self.logs.add("launcher", f"docker compose {args[0]} exited with {rc}")
    return rc

  def follow(self) -> None:
    """Stream both containers' output into the log view (restarted whenever they come back)."""
    def loop():
      tail = "200"
      while True:
        if self.follower is None or self.follower.poll() is not None:
          try:
            self.follower = subprocess.Popen(self.compose("logs", "-f", "--no-color", "--tail", tail), env=self.env(),
                                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                                             encoding="utf-8", errors="replace")
            tail = "0"
            assert self.follower.stdout is not None
            for line in self.follower.stdout:
              svc, _, text = line.partition("|")
              self.logs.add(svc.strip().rsplit("-", 1)[0] or "docker", text[1:] if text.startswith(" ") else text)
          except Exception:
            pass
        time.sleep(2.0)
    threading.Thread(target=loop, daemon=True).start()

  def start(self) -> bool:
    def go():
      if shutil.which("docker") is None:
        raise RuntimeError("docker not found: install Docker Desktop (Windows/macOS) or Docker Engine (Linux)")
      if self.run("up", "-d", "--build", "--remove-orphans") == 0:
        self.logs.add("launcher", "started: the comma prepares its build first (the first time takes a while)")
        self.follow()
    return self._task("starting", go)

  def stop(self, wipe: bool = False) -> bool:
    return self._task("stopping", lambda: self.run("down", *(["-v"] if wipe else [])))

  def reload_build(self) -> bool:
    # the comma's entrypoint re-syncs the build and lets scons rebuild what changed; the car keeps running
    return self._task("reloading the build", lambda: self.run("up", "-d", "--force-recreate", "--no-deps", "comma"))

  def restart_car(self) -> bool:
    return self._task("restarting the car", lambda: self.run("up", "-d", "--force-recreate", "--no-deps", "mazda"))

  def status(self) -> dict:
    out = {"backend": self.name, "busy": self.busy, "services": {}}
    try:
      r = subprocess.run(self.compose("ps", "-a", "--format", "json"), env=self.env(), capture_output=True, text=True,
                         timeout=10)
      rows = []
      for line in r.stdout.strip().splitlines():
        line = line.strip()
        if line.startswith("["):
          rows += json.loads(line)
        elif line:
          rows.append(json.loads(line))
      for row in rows:
        out["services"][row.get("Service")] = {"state": row.get("State"), "status": row.get("Status")}
    except (OSError, ValueError, subprocess.TimeoutExpired, RuntimeError) as e:
      out["error"] = str(e)
    return out


def process_tree(pid: int) -> list[int]:
  """pid and all its descendants (Linux /proc). openpilot's manager forks itself into a new session (a pty for its
  output), so a process-group signal alone does not reach everything it starts."""
  children: dict[int, list[int]] = {}
  for d in os.listdir("/proc"):
    if d.isdigit():
      try:
        with open(f"/proc/{d}/stat") as fh:
          ppid = int(fh.read().rsplit(")", 1)[1].split()[1])
        children.setdefault(ppid, []).append(int(d))
      except (OSError, ValueError, IndexError):
        pass
  out, todo = [], [pid]
  while todo:
    p = todo.pop()
    out.append(p)
    todo += children.get(p, [])
  return out


def alive(pid: int) -> bool:
  try:
    os.kill(pid, 0)
    with open(f"/proc/{pid}/stat") as fh:
      return fh.read().rsplit(")", 1)[1].split()[0] != "Z"
  except (OSError, IndexError):
    return False


class NativeBackend(Backend):
  """Both halves as local processes (Linux). The build is prepared in ~/.mazda_sim/work like the container does."""
  name = "native"

  def __init__(self, *a, **k):
    super().__init__(*a, **k)
    self.procs: dict[str, subprocess.Popen] = {}

  @property
  def work(self) -> Path:
    return self.home / "work"

  def spawn(self, name: str, cmd: list[str], cwd: Path, env: dict) -> subprocess.Popen:
    self.logs.add("launcher", f"$ {' '.join(cmd)}")
    p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         encoding="utf-8", errors="replace", start_new_session=True)
    self.procs[name] = p
    threading.Thread(target=self._pump, args=(name, p), daemon=True).start()
    return p

  def start(self) -> bool:
    def go():
      if sys.platform != "linux":
        raise RuntimeError("the native backend runs on Linux only: use the docker backend")
      cfg = self.config()
      base = {**os.environ, "MAZDA_SIM_CONFIG": str(self.home / "sim.json"), "PYTHONUNBUFFERED": "1"}
      prep = [sys.executable, str(SIM / "device" / "prepare_build.py"), "--work", str(self.work)]
      if cfg.build.source == "local":
        prep += ["--src", cfg.build.local_path]
      if self.spawn("comma", prep, TOOLS, base).wait() != 0:
        raise RuntimeError("preparing the build failed (see the comma log)")
      py = str(self.work / "venv" / "bin" / "python")
      self.spawn("mazda", [py, "-m", "mazda_sim.car.main"], TOOLS,
                 {**base, "PYTHONPATH": f"{TOOLS}:{REPO / 'opendbc_repo'}"})
      time.sleep(2.0)
      build = self.work / "openpilot"
      self.spawn("comma", [py, "-m", "mazda_sim.device.sim_manager"], build,
                 {**base, "PYTHONPATH": f"{build}:{TOOLS}", "MAZDA_SIM_CAR_HOST": "127.0.0.1",
                  "MAZDA_SIM_OPENDBC": str(build / "opendbc_repo"), "MAZDA_SIM_SAFETY_BUILD": str(self.work / "safety_build")})
    return self._task("starting", go)

  def _stop(self, names: tuple[str, ...]) -> None:
    for name in names:
      p = self.procs.get(name)
      if p is None or p.poll() is not None:
        continue
      tree = process_tree(p.pid)
      try:
        os.killpg(p.pid, signal.SIGINT if name == "comma" else signal.SIGTERM)
      except ProcessLookupError:
        pass
      deadline = time.monotonic() + 60.0    # the manager stops its ~25 processes one after another (~35 s)
      while time.monotonic() < deadline and any(alive(x) for x in tree):
        time.sleep(0.5)
        p.poll()
      for x in tree:
        if alive(x):
          try:
            os.kill(x, signal.SIGKILL)
          except ProcessLookupError:
            pass
      p.poll()

  def stop(self, wipe: bool = False) -> bool:
    def go():
      self._stop(("comma", "mazda"))
      if wipe:
        shutil.rmtree(self.work, ignore_errors=True)
    return self._task("stopping", go)

  def reload_build(self) -> bool:
    def go():
      self._stop(("comma",))
      cfg = self.config()
      base = {**os.environ, "MAZDA_SIM_CONFIG": str(self.home / "sim.json"), "PYTHONUNBUFFERED": "1"}
      prep = [sys.executable, str(SIM / "device" / "prepare_build.py"), "--work", str(self.work)]
      if cfg.build.source == "local":
        prep += ["--src", cfg.build.local_path]
      if self.spawn("comma", prep, TOOLS, base).wait() != 0:
        raise RuntimeError("preparing the build failed")
      build = self.work / "openpilot"
      self.spawn("comma", [str(self.work / "venv" / "bin" / "python"), "-m", "mazda_sim.device.sim_manager"], build,
                 {**base, "PYTHONPATH": f"{build}:{TOOLS}", "MAZDA_SIM_CAR_HOST": "127.0.0.1",
                  "MAZDA_SIM_OPENDBC": str(build / "opendbc_repo"), "MAZDA_SIM_SAFETY_BUILD": str(self.work / "safety_build")})
    return self._task("reloading the build", go)

  def restart_car(self) -> bool:
    def go():
      self._stop(("mazda",))
      self.spawn("mazda", [str(self.work / "venv" / "bin" / "python"), "-m", "mazda_sim.car.main"], TOOLS,
                 {**os.environ, "MAZDA_SIM_CONFIG": str(self.home / "sim.json"), "PYTHONUNBUFFERED": "1",
                  "PYTHONPATH": f"{TOOLS}:{REPO / 'opendbc_repo'}"})
    return self._task("restarting the car", go)

  def status(self) -> dict:
    return {"backend": self.name, "busy": self.busy,
            "services": {n: {"state": "running" if p.poll() is None else "exited", "status": f"pid {p.pid}"}
                         for n, p in self.procs.items()}}


class Launcher:
  def __init__(self, home: Path):
    self.home = home
    home.mkdir(parents=True, exist_ok=True)
    self.logs = Logs()
    self.settings_path = home / "launcher.json"
    self.settings = {**DEFAULT_SETTINGS, **(json.loads(self.settings_path.read_text()) if self.settings_path.is_file() else {})}
    if not (home / "sim.json").is_file():
      SimConfig().save(str(home / "sim.json"))
    self.backend = self.make_backend()

  def make_backend(self) -> Backend:
    cls = NativeBackend if self.settings.get("backend") == "native" else DockerBackend
    return cls(self.home, self.logs, self.settings)

  def save_settings(self, new: dict) -> None:
    for k in DEFAULT_SETTINGS:
      if k in new:
        self.settings[k] = str(new[k])
    self.settings_path.write_text(json.dumps(self.settings, indent=2))
    if self.settings["backend"] != self.backend.name:
      self.backend = self.make_backend()
    else:
      self.backend.settings = self.settings

  def car_reachable(self) -> bool:
    try:
      with urlopen("http://127.0.0.1:8770/status", timeout=0.5) as r:
        return r.status == 200
    except (URLError, OSError):
      return False

  def state(self) -> dict:
    cfg = SimConfig.load(str(self.home / "sim.json"))
    return {"config": cfg.to_dict(), "defaults": SimConfig().to_dict(), "settings": self.settings,
            "status": {**self.backend.status(), "carReachable": self.car_reachable()},
            "home": str(self.home), "platform": sys.platform}


def make_handler(launcher: Launcher):
  class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
      super().__init__(*a, directory=str(WEB), **k)

    def log_message(self, *_a) -> None:
      pass

    def reply(self, obj, code: int = 200) -> None:
      body = json.dumps(obj).encode()
      self.send_response(code)
      self.send_header("Content-Type", "application/json")
      self.send_header("Content-Length", str(len(body)))
      self.send_header("Cache-Control", "no-store")
      self.end_headers()
      self.wfile.write(body)

    def do_GET(self):
      url = urlparse(self.path)
      if url.path == "/api/state":
        return self.reply(launcher.state())
      if url.path == "/api/logs":
        since = int(parse_qs(url.query).get("since", ["0"])[0])
        seq, lines = launcher.logs.since(since)
        return self.reply({"seq": seq, "lines": lines[-1500:]})
      return super().do_GET()

    def do_POST(self):
      # a custom header: browsers only send it from this page (other sites would need a CORS preflight we refuse)
      if self.headers.get("X-Mazda-Sim") != "1":
        return self.reply({"error": "missing X-Mazda-Sim header"}, HTTPStatus.FORBIDDEN)
      n = int(self.headers.get("Content-Length") or 0)
      try:
        body = json.loads(self.rfile.read(n) or b"{}")
      except ValueError:
        return self.reply({"error": "bad json"}, HTTPStatus.BAD_REQUEST)
      path = urlparse(self.path).path
      b = launcher.backend
      try:
        if path == "/api/config":
          cfg = SimConfig.from_dict(body)
          if cfg.build.source == "local" and cfg.build.local_path and not Path(cfg.build.local_path).is_dir():
            return self.reply({"error": f"{cfg.build.local_path} is not a folder on this PC"}, HTTPStatus.BAD_REQUEST)
          cfg.save(str(launcher.home / "sim.json"))
          return self.reply({"ok": True, "config": cfg.to_dict()})
        if path == "/api/settings":
          launcher.save_settings(body)
          return self.reply({"ok": True, "settings": launcher.settings})
        actions = {"/api/start": b.start, "/api/reload": b.reload_build, "/api/restart-car": b.restart_car,
                   "/api/stop": lambda: b.stop(bool(body.get("wipe")))}
        if path in actions:
          ok = actions[path]()
          return self.reply({"ok": ok, "busy": b.busy} if ok else {"error": f"busy: {b.busy}"},
                            200 if ok else HTTPStatus.CONFLICT)
      except (TypeError, ValueError, RuntimeError) as e:
        return self.reply({"error": str(e)}, HTTPStatus.BAD_REQUEST)
      return self.reply({"error": "not found"}, HTTPStatus.NOT_FOUND)

  return Handler


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--port", type=int, default=8765)
  ap.add_argument("--host", default="127.0.0.1", help="listen address (0.0.0.0 to reach it from another device)")
  ap.add_argument("--home", default=os.environ.get("MAZDA_SIM_HOME", str(Path.home() / ".mazda_sim")),
                  help="where sim.json and launcher settings live")
  ap.add_argument("--no-browser", action="store_true")
  args = ap.parse_args()

  launcher = Launcher(Path(args.home).expanduser().resolve())
  server = ThreadingHTTPServer((args.host, args.port), make_handler(launcher))
  url = f"http://localhost:{args.port}/"
  keeps = "the sim keeps running" if launcher.backend.name == "docker" else "the sim stops with it"
  print(f"Mazda sim launcher on {url}  (settings in {launcher.home}; Ctrl+C quits, {keeps})")
  signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
  if launcher.backend.name == "docker" and shutil.which("docker"):
    launcher.backend.follow()   # pick up the logs of a sim that is already running
  if not args.no_browser:
    threading.Timer(0.5, lambda: webbrowser.open(url)).start()
  try:
    server.serve_forever()
  except KeyboardInterrupt:
    if isinstance(launcher.backend, NativeBackend):
      print("stopping the sim")
      launcher.backend._stop(("comma", "mazda"))


if __name__ == "__main__":
  main()
