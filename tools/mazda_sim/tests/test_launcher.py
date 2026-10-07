"""The launcher's HTTP API, without starting anything."""
import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from mazda_sim.launcher.mazdasim import Launcher, Logs, make_handler


@pytest.fixture
def server(tmp_path):
  launcher = Launcher(tmp_path)
  httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(launcher))
  threading.Thread(target=httpd.serve_forever, daemon=True).start()
  yield f"http://127.0.0.1:{httpd.server_address[1]}", launcher
  httpd.shutdown()


def call(url, body=None, header=True):
  req = urllib.request.Request(url, data=None if body is None else json.dumps(body).encode(),
                               headers={"X-Mazda-Sim": "1"} if header else {})
  try:
    with urllib.request.urlopen(req, timeout=20) as r:
      return r.status, json.loads(r.read())
  except urllib.error.HTTPError as e:
    return e.code, json.loads(e.read())


def test_state_and_config(server, tmp_path):
  url, launcher = server
  code, st = call(url + "/api/state")
  assert code == 200 and st["config"]["car"]["torque_interceptor"] is True and st["settings"]["backend"] == "docker"
  cfg = st["config"]
  cfg["car"]["radar_emulation"] = True
  assert call(url + "/api/config", cfg)[0] == 200
  assert json.loads((tmp_path / "sim.json").read_text())["car"]["radar_emulation"] is True
  cfg["build"] = {"source": "local", "local_path": str(tmp_path / "nope")}
  assert call(url + "/api/config", cfg)[0] == 400
  assert call(url + "/api/config", cfg, header=False)[0] == 403   # other web pages can't drive it


def test_page_is_served(server):
  url, _ = server
  with urllib.request.urlopen(url + "/", timeout=5) as r:
    assert b"Mazda Sim" in r.read()


def test_logs_collapse_the_process_list():
  logs = Logs()
  for _ in range(3):
    logs.add("comma", "\x1b[32mpandad\x1b[0m \x1b[32mcard\x1b[0m")
  logs.add("comma", "\x1b[32mpandad\x1b[0m \x1b[31mcard\x1b[0m")
  logs.add("comma", "hello")
  _, lines = logs.since(0)
  assert [line for _s, _src, line in lines] == ["running: pandad card", "hello"]


def test_local_build_must_be_a_checkout_top(tmp_path):
  from mazda_sim.launcher.mazdasim import checkout_problem
  repo = tmp_path / "op"
  (repo / "tools" / "x").mkdir(parents=True)
  assert "not a folder" in checkout_problem(str(tmp_path / "missing"))
  assert "not an openpilot checkout" in checkout_problem(str(repo / "tools" / "x"))
  (repo / "SConstruct").write_text("")
  (repo / "pyproject.toml").write_text("")
  assert checkout_problem(str(repo)) is None
