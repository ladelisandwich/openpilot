#!/usr/bin/env python3
"""Gets an openpilot build ready to run on a PC as the simulated comma: source, Python env, scons, driving model.

Runs in the comma container before the sim starts (also usable without docker). Everything lands in a work
directory that persists between runs, so starting the same build again only re-syncs the source and lets uv and
scons confirm nothing changed.

  source   git: shallow fetch of BuildOptions.repo @ ref (a branch, tag or commit)
           local: a checkout on your PC (mounted read-only) copied in, uncommitted changes included. Files your
           .gitignore excludes are neither copied nor deleted, so x86 build outputs survive between runs.
  python   uv sync --frozen against the build's own uv.lock (all extras but speedvision's torch)
  native   scons, x86 this time (the repo's committed aarch64 binaries are rebuilt in the copy, never in yours)
  model    vision mode only: the build's driving_vision/driving_policy ONNX compiled for this PC's tinygrad
           device, installed in place of the device-only artifact (cached per ONNX + device)

Stdlib only: runs on the system python before the build's venv exists.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

PC_MODEL_BEHAVIOR_VERSION = "v10"   # the repo's split vision/policy nets: a plan output, no `action` head


def log(msg: str) -> None:
  print(f"[prepare] {msg}", flush=True)


def run(cmd: list[str], cwd: Path | None = None, env: dict | None = None, log_path: Path | None = None,
        check: bool = True) -> int:
  log("$ " + " ".join(str(c) for c in cmd))
  full_env = {**os.environ, **(env or {})}
  if log_path is None:
    rc = subprocess.call([str(c) for c in cmd], cwd=cwd, env=full_env)
  else:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "w") as fh:
      p = subprocess.Popen([str(c) for c in cmd], cwd=cwd, env=full_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
      assert p.stdout is not None
      last = 0.0
      for line in p.stdout:
        fh.write(line)
        if time.monotonic() - last > 10.0:   # a heartbeat for the launcher's log view, not the whole firehose
          last = time.monotonic()
          print("  " + line.rstrip()[:160], flush=True)
      rc = p.wait()
    if rc != 0:
      tail = log_path.read_text(errors="replace").splitlines()[-40:]
      print("\n".join("  | " + t for t in tail), flush=True)
  if check and rc != 0:
    raise SystemExit(f"[prepare] failed ({rc}): {' '.join(str(c) for c in cmd)}")
  return rc


def is_device_binary(path: Path) -> bool:
  """An aarch64 ELF file or static library: the comma's prebuilt binaries this fork commits. scons does not notice
  a target replaced behind its back, so these must never sit where it expects its x86 outputs."""
  try:
    with open(path, "rb") as fh:
      head = fh.read(20)
      if head[:4] == b"\x7fELF":
        return int.from_bytes(head[18:20], "little") == 0xB7
      if head[:8] == b"!<arch>\n":
        fh.seek(8)
        for _ in range(4):   # symbol table, long-name table, then the first object file
          hdr = fh.read(60)
          if len(hdr) < 60:
            return False
          size = int(hdr[48:58].strip() or 0)
          data = fh.read(min(size, 20))
          if data[:4] == b"\x7fELF":
            return int.from_bytes(data[18:20], "little") == 0xB7
          fh.seek(size - len(data) + (size & 1), 1)
  except (OSError, ValueError):
    pass
  return False


def drop_device_binaries(build: Path, files: list[str]) -> None:
  n = 0
  for f in files:
    p = build / f
    if p.is_file() and not p.is_symlink() and is_device_binary(p):
      p.unlink()
      n += 1
  if n:
    log(f"removed {n} committed aarch64 binaries from the work copy: scons builds x86 ones in their place")


def tracked_files(root: Path, untracked: bool = False) -> list[str] | None:
  cmd = ["git", "-C", root, "ls-files", "-z", "--cached"] + (["--others", "--exclude-standard"] if untracked else [])
  r = subprocess.run(cmd, capture_output=True)
  return sorted({f for f in r.stdout.decode().split("\0") if f}) if r.returncode == 0 else None


def sync_git(repo: str, ref: str, dst: Path) -> None:
  if not (dst / ".git").exists():
    dst.mkdir(parents=True, exist_ok=True)
    run(["git", "init", "-q", dst])
  has_origin = subprocess.run(["git", "-C", dst, "remote", "get-url", "origin"], capture_output=True).returncode == 0
  run(["git", "-C", dst, "remote", "set-url" if has_origin else "add", "origin", repo])
  run(["git", "-C", dst, "fetch", "--depth=1", "--no-tags", "origin", ref])
  # -f: tracked files the last x86 build replaced (the committed aarch64 libraries) go back, scons rebuilds them
  run(["git", "-C", dst, "checkout", "-q", "-f", "FETCH_HEAD"])
  # keep a branch name: StarPilot reads it for its update and version checks
  if not all(c in "0123456789abcdef" for c in ref.lower()) or len(ref) < 7:
    run(["git", "-C", dst, "checkout", "-q", "-B", ref.split("/")[-1]])
  drop_device_binaries(dst, tracked_files(dst) or [])


def sync_local(src: Path, dst: Path) -> None:
  """Copy a checkout as git sees it: tracked files (with your uncommitted edits) and untracked files that are not
  ignored. Ignored files are left alone on both sides, so x86 build outputs here survive the next sync. (rsync's own
  .gitignore support can't do this: it has no `!` re-includes, which panda's firmware objects rely on.)"""
  if not src.is_dir() or not any(src.iterdir()):
    raise SystemExit(f"[prepare] local build {src} is empty: is BuildOptions.local_path mounted?")
  if shutil.which("rsync") is None:
    raise SystemExit("[prepare] rsync is needed to copy a local build")
  dst.mkdir(parents=True, exist_ok=True)
  files = tracked_files(src, untracked=True)
  if files is None:
    log(f"{src} is not a git checkout: copying everything in it")
    run(["rsync", "-a", f"{src}/", f"{dst}/"])
    return
  copy = [f for f in files if not is_device_binary(src / f)]
  list_path = dst.parent / "local-files.txt"
  list_path.write_text("\0".join(copy))
  run(["rsync", "-a", "--from0", f"--files-from={list_path}", "--ignore-missing-args", f"{src}/", f"{dst}/"])
  drop_device_binaries(dst, sorted(set(files) - set(copy)))   # only where an aarch64 copy is still in the way
  # git metadata too: the build reads its branch and commit
  run(["rsync", "-a", "--delete", f"{src}/.git/", f"{dst}/.git/"], check=False)
  manifest = dst.parent / "local-manifest.txt"
  if manifest.is_file():
    gone = set(manifest.read_text().split("\0")) - set(files)
    for f in gone:
      if f and (dst / f).is_file():
        (dst / f).unlink()
    if gone:
      log(f"removed {len(gone)} file(s) no longer in the checkout")
  manifest.write_text("\0".join(files))


def python_env(build: Path, venv: Path, cache: Path) -> None:
  if shutil.which("uv") is None:
    raise SystemExit("[prepare] uv is needed: https://docs.astral.sh/uv/getting-started/installation/")
  env = {"UV_PROJECT_ENVIRONMENT": str(venv), "UV_CACHE_DIR": str(cache / "uv"), "UV_PYTHON": "3.12",
         "UV_LINK_MODE": "copy"}
  # as tools/install_python_dependencies.sh does (the build imports some packages only its extras bring, e.g.
  # python-dateutil via matplotlib), minus speedvision's torch
  run(["uv", "sync", "--frozen", "--all-extras", "--no-extra", "speedvision"], cwd=build, env=env,
      log_path=build.parent / "logs" / "uv.log")


def build_native(build: Path, venv: Path, jobs: int) -> None:
  env = {"VIRTUAL_ENV": str(venv), "PATH": f"{venv}/bin:{os.environ['PATH']}"}
  run([venv / "bin" / "scons", f"-j{jobs}"], cwd=build, env=env, log_path=build.parent / "logs" / "scons.log")


def file_hash(paths: list[Path], extra: str) -> str:
  h = hashlib.sha256(extra.encode())
  for p in paths:
    with open(p, "rb") as fh:
      for chunk in iter(lambda: fh.read(1 << 20), b""):
        h.update(chunk)
  return h.hexdigest()[:16]


def pc_model(build: Path, venv: Path, cache: Path, device: str) -> None:
  models = build / "selfdrive" / "modeld" / "models"
  vision, policy = models / "driving_vision.onnx", models / "driving_policy.onnx"
  compiler = build / "selfdrive" / "modeld" / "compile_modeld.py"
  if not (vision.is_file() and policy.is_file() and compiler.is_file()):
    raise SystemExit("[prepare] vision mode needs selfdrive/modeld/models/driving_{vision,policy}.onnx and " +
                     "selfdrive/modeld/compile_modeld.py in the build")
  # compiled kernels are specific to the OpenCL driver too (compose.nvidia.yml / compose.dri.yml pick it)
  key = file_hash([vision, policy, compiler], f"{device}|{os.environ.get('OCL_ICD_VENDORS', '')}")
  out = cache / "models" / key / "driving_tinygrad.pkl"
  if not out.is_file():
    log(f"compiling the driving model for {device} (first time for this model + device; can take a while)")
    out.parent.mkdir(parents=True, exist_ok=True)
    env = {"DEV": device, "JIT_BATCH_SIZE": "0", "PYTHONPATH": f"{build}:{build}/tinygrad_repo",
           "PATH": f"{venv}/bin:{os.environ['PATH']}"}
    tmp = out.with_suffix(".tmp")
    run([venv / "bin" / "python", compiler, "--model-type", "vision_policy", "--model-size", "512x256",
         "--camera-resolutions", "1928x1208", "--image-history-pipeline", "policy",
         "--behavior-version", PC_MODEL_BEHAVIOR_VERSION, "--vision-onnx", vision, "--policy-onnx", policy,
         "--output", tmp], cwd=build, env=env, log_path=build.parent / "logs" / "model.log")
    tmp.replace(out)
  # the device artifact is chunked (and compiled for the comma's GPU); the manifest wins over a plain file
  for p in models.glob("driving_tinygrad.pkl*"):
    p.unlink()
  shutil.copyfile(out, models / "driving_tinygrad.pkl")
  log(f"driving model for {device} installed ({out.stat().st_size / 1e6:.0f} MB)")


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--config", default=os.environ.get("MAZDA_SIM_CONFIG", ""))
  ap.add_argument("--work", default=os.environ.get("MAZDA_SIM_WORK", "/work"))
  ap.add_argument("--src", default=os.environ.get("MAZDA_SIM_SRC", "/src"), help="local build mount (source=local)")
  ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
  ap.add_argument("--skip-build", action="store_true", help="source + python only (scons already done)")
  args = ap.parse_args()

  cfg = json.load(open(args.config)) if args.config and os.path.isfile(args.config) else {}
  b = {"source": "git", "repo": "https://github.com/ladelisandwich/openpilot.git", "ref": "mazda-long-ti1-testing",
       **cfg.get("build", {})}
  world = cfg.get("world", {})
  work = Path(args.work)
  build, venv, cache = work / "openpilot", work / "venv", work / "cache"

  t0 = time.monotonic()
  if b["source"] == "local":
    log(f"build: local checkout ({b.get('local_path') or args.src})")
    sync_local(Path(args.src), build)
  else:
    log(f"build: {b['repo']} @ {b['ref']}")
    sync_git(b["repo"], b["ref"], build)
  head = subprocess.run(["git", "-C", build, "log", "-1", "--format=%h %s"], capture_output=True, text=True).stdout.strip()
  log(f"source ready: {head or '(no git metadata)'}")

  python_env(build, venv, cache)
  if not args.skip_build:
    log("scons (first build of a checkout takes a while; later ones only rebuild what changed)")
    build_native(build, venv, args.jobs)
  if world.get("model") == "vision":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from mazda_sim.device.sim_manager import vision_device
    pc_model(build, venv, cache, vision_device(world.get("vision_device", "auto")))
  (work / "ready.json").write_text(json.dumps({"head": head, "t": time.time(), "build": b}))
  log(f"ready in {time.monotonic() - t0:.0f} s")


if __name__ == "__main__":
  main()
