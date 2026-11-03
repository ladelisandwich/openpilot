#!/usr/bin/env python3
"""Event timeline of a can_census --dump: what changed, when.

  python3 tools/mazda/can_dump_events.py /data/can_dump_<route>_<T0>-<T1>.csv.gz [--window T0-T1] [--max-events 60]
                                         [--exclude REGEX] [--out /data/can_events.txt]

Every change of a decoded mazda_2017 signal, plus every change of an undecoded byte that no decoded signal accounts for,
merged by time. Counters and checksums are skipped. A byte cycling through a few values is treated as a page index
when the other bytes differ page to page (a multiplexed message), and ignored when they do not (a counter nibble).
Items that change more often than --max-events in the window (analog values, noise) are listed by count at the end
instead of in the timeline. Read it against a note of what was operated when: a switch is the byte that moved
within a few tenths of a second of the action and nowhere else."""
import argparse
import csv
import gzip
import re
import sys
from collections import defaultdict

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("dump")
ap.add_argument("--window", default="", help="T0-T1 in route seconds (default: the whole dump)")
ap.add_argument("--max-events", type=int, default=60)
ap.add_argument("--exclude", default="", help="regex of item labels to leave out of the timeline")
ap.add_argument("--out", default="", help="write here instead of stdout")
args = ap.parse_args()
dbc = DBC("mazda_2017")
fn, MAXEV = args.dump, args.max_events
t0, t1 = (float(x) for x in args.window.split("-")) if args.window else (0.0, float("inf"))
EXCL = re.compile(args.exclude) if args.exclude else None
SKIP_SIG = re.compile(r"CTR|CHKSUM|CHECKSUM|COUNTER|counter|NEW_SIGNAL|^S\d$|^SIGNAL$", re.I)
out = open(args.out, "w") if args.out else sys.stdout

frames = defaultdict(list)
with gzip.open(fn, "rt") as f:
  rd = csv.reader(f)
  next(rd)
  for t, bus, addr, data in rd:
    t = float(t)
    if t0 <= t <= t1:
      frames[(int(bus), int(addr, 16))].append((t, bytes.fromhex(data)))

events, counts = [], defaultdict(int)
explained = set()   # (bus, addr, t, byte) changes a decoded signal accounts for

def sigval(s, dat):
  raw = get_raw_value(dat, s)
  if s.is_signed and raw & (1 << (s.size - 1)):
    raw -= 1 << s.size
  return raw * s.factor + s.offset

def sig_bytes(s):
  """byte indexes a signal touches (big-endian DBC bit numbering, as opendbc stores start_bit for msb)"""
  lo, hi = s.lsb // 8, s.msb // 8
  return set(range(min(lo, hi), max(lo, hi) + 1))

for (bus, addr), fr in frames.items():
  msg = dbc.addr_to_msg.get(addr)
  if msg is None:
    continue
  sigs = list(msg.sigs.values())
  last, prev = {}, None
  ev = defaultdict(list)
  for t, d in fr:
    if d == prev:
      continue
    prev = d
    for s in sigs:
      try:
        v = sigval(s, d)
      except Exception:
        continue
      if s.name in last and last[s.name] != v:
        ev[s.name].append((t, last[s.name], v))
        if SKIP_SIG.search(s.name):
          for b in sig_bytes(s):
            explained.add((bus, addr, t, b))
      last[s.name] = v
  for name, lst in ev.items():
    if SKIP_SIG.search(name):
      continue
    label = f"bus{bus} {msg.name}.{name}"
    counts[label] = len(lst)
    if len(lst) <= MAXEV:
      for t, a, b in lst:
        events.append((t, label, f"{a:g}>{b:g}"))
        s = msg.sigs[name]
        for byte in sig_bytes(s):
          explained.add((bus, addr, t, byte))

def page_byte(fr, ctr_bytes=()):
  """a byte with 3..32 values changing on most frames, whose pages carry different content in the bytes that are
  otherwise quiet (a counter nibble leaves them alone; an analog byte next to it must not fool the test)"""
  n = len(fr)
  dlc = max(len(d) for _, d in fr)
  chg = [sum(1 for i in range(1, n) if b < len(fr[i][1]) and b < len(fr[i - 1][1]) and fr[i][1][b] != fr[i - 1][1][b]) for b in range(dlc)]
  for b in range(dlc):
    if b in ctr_bytes:
      continue
    vals = {d[b] for _, d in fr if b < len(d)}
    if not (3 <= len(vals) <= 32) or chg[b] < 0.5 * n:
      continue
    quiet = [x for x in range(dlc) if x != b and chg[x] < 0.2 * n]
    if not quiet:
      continue
    same = sum(1 for i in range(1, n) if all(fr[i][1][x] == fr[i - 1][1][x] for x in quiet if x < len(fr[i][1]) and x < len(fr[i - 1][1])))
    if same < 0.5 * n:
      return b
  return None

def byte_events(bus, addr, label, fr, skip=()):
  n = len(fr)
  if n < 2:
    return
  dlc = max(len(d) for _, d in fr)
  for b in range(dlc):
    if b in skip:
      continue
    lst = [(fr[i][0], fr[i - 1][1][b], fr[i][1][b]) for i in range(1, n)
           if b < len(fr[i][1]) and b < len(fr[i - 1][1]) and fr[i][1][b] != fr[i - 1][1][b]]
    lst = [x for x in lst if (bus, addr, x[0], b) not in explained]
    if not lst:
      continue
    key = f"{label}.b{b}"
    counts[key] = len(lst)
    if len(lst) > 0.2 * n or len(lst) > MAXEV:
      continue
    for t, a, c in lst:
      bits = "".join(f"{'+' if (c >> k) & 1 else '-'}{k}" for k in range(8) if ((a ^ c) >> k) & 1)
      events.append((t, key, f"{a:02x}>{c:02x} bit{bits}"))

for (bus, addr), fr in sorted(frames.items()):
  msg = dbc.addr_to_msg.get(addr)
  label = f"bus{bus} 0x{addr:03x} {msg.name if msg else '-'}"
  ctr_bytes = set()
  if msg is not None:
    for sg in msg.sigs.values():
      if re.search(r"CTR|COUNTER|counter|CHKSUM|CHECKSUM", sg.name):
        ctr_bytes |= sig_bytes(sg)
  pb = page_byte(fr, ctr_bytes) if len(fr) > 30 else None
  if pb is None:
    byte_events(bus, addr, label, fr)
  else:
    pages = defaultdict(list)
    for t, d in fr:
      pages[d[pb]].append((t, d))
    for p, pfr in pages.items():
      byte_events(bus, addr, f"{label}[p{p}]", pfr, skip=(pb,))

events.sort()
if EXCL:
  events = [e for e in events if not EXCL.search(e[1])]
print(f"=== {len(events)} events in {t0:g}..{t1:g} s", file=out)
cur = None
for t, label, text in events:
  sec = int(t)
  if sec != cur:
    print(f"{t:9.2f}", file=out)
    cur = sec
  print(f"    {t:9.2f}  {label:36s} {text}", file=out)
print("\n=== busy items (changes in the window; not in the timeline)", file=out)
for k, v in sorted(counts.items(), key=lambda kv: -kv[1]):
  if v > MAXEV and not (EXCL and EXCL.search(k)):
    print(f"  {v:6d}  {k}", file=out)
if args.out:
  out.close()
  print(f"  -> {args.out}")
