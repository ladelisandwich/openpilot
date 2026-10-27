"""Parse a can_census --watch timeline in a window. Usage: python3 -I watch_parse2.py <can_watch.txt> t0 t1
Mux-style addresses are analysed page by page (page byte given in MUX); busy bytes are tracked as 1-s means and
reported on level shifts instead of per-frame changes."""
# ruff: noqa: E701, B007  -- one-off analysis scripts, kept as written for the write-ups
import ast
import re
import sys
import statistics
from collections import defaultdict

MUX = {0x4fd: 0, 0x45b: 0, 0x3d2: 0, 0x9f: 6}
LINE = re.compile(r"^\s*(\d+\.\d+) bus(\d) 0x([0-9a-f]+) ((?:[0-9a-f]{2} ?)+?)\s+(first\s+)?(\{.*\})$")
frames = defaultdict(list)
ctx_tl = []
with open(sys.argv[1]) as f:
  for line in f:
    m = LINE.match(line.rstrip("\n"))
    if not m:
      continue
    t, bus, addr, hexs, first, ctx = m.groups()
    frames[(int(bus), int(addr, 16))].append((float(t), bytes.fromhex(hexs.replace(" ", "")), ast.literal_eval(ctx)))
    ctx_tl.append((float(t), frames[(int(bus), int(addr, 16))][-1][2]))
t0, t1 = float(sys.argv[2]), float(sys.argv[3])

def cs(c):
  return f"L{c['L']}R{c['R']}hb{c['hb']}g{c['gear']}br{c['brake']}k{c['kph']}"

print(f"=== decoded context in {t0:.0f}..{t1:.0f} s (hazard = L and R flashing together)")
last = {}
for t, c in ctx_tl:
  if not (t0 <= t <= t1):
    continue
  for k in ("L", "R", "hb", "gear", "brake", "acc", "kph", "gas"):
    v = c[k]
    if k == "kph": v = int(v > 0)
    if k == "gas": v = int(v > 5)
    if last.get(k) != v:
      if k in last and (k not in ("L", "R") or True):
        print(f"  {t:8.2f} {k}={v}")
      last[k] = v

def report_changes(label, fr, skip_byte=None):
  n = len(fr)
  if n == 0:
    return
  dlc = max(len(d) for _, d, _ in fr)
  changes = [0] * dlc
  prev = None
  for _, d, _ in fr:
    if prev is not None:
      for b in range(min(len(d), len(prev))):
        if d[b] != prev[b]:
          changes[b] += 1
    prev = d
  busy = [b for b in range(dlc) if n > 20 and changes[b] > 0.2 * n and b != skip_byte]
  print(f"\n{label}: {n} frames, change counts {changes}, busy {busy}, first {fr[0][1].hex(' ')}")
  prev = None
  shown = 0
  for t, d, c in fr:
    if prev is not None:
      diffs = [(b, prev[b], d[b]) for b in range(min(len(d), len(prev))) if d[b] != prev[b] and b not in busy and b != skip_byte]
      if diffs:
        shown += 1
        if shown <= 120:
          print(f"  {t:8.2f} {' '.join(f'b{b}:{o:02x}>{nv:02x}' for b, o, nv in diffs):34s} {d.hex(' ')}  {cs(c)}")
    prev = d
  if shown > 120:
    print(f"  ... {shown - 120} more")
  # level shifts of busy bytes: 1-s means vs the median of the previous 5 s
  for b in busy:
    sec = defaultdict(list)
    for t, d, c in fr:
      if b < len(d):
        sec[int(t)].append(d[b])
    secs = sorted(sec)
    means = {s: statistics.fmean(sec[s]) for s in secs}
    hits = []
    for i, s in enumerate(secs):
      base = [means[x] for x in secs[max(0, i - 5):i]]
      if len(base) >= 3 and abs(means[s] - statistics.median(base)) > 6:
        hits.append((s, means[s], statistics.median(base)))
    if hits:
      print(f"  busy b{b} level shifts (sec, mean, prev-5s median): " + " ".join(f"{s}:{m:.0f}<-{bm:.0f}" for s, m, bm in hits[:40]))

for key in sorted(frames):
  fr = [x for x in frames[key] if t0 <= x[0] <= t1]
  if not fr:
    continue
  bus, addr = key
  if addr in MUX:
    pb = MUX[addr]
    pages = defaultdict(list)
    for x in fr:
      pages[x[1][pb]].append(x)
    print(f"\n##### bus{bus} 0x{addr:03x}: {len(fr)} frames, page byte b{pb}, pages {sorted(pages)}")
    for p in sorted(pages):
      report_changes(f"  page {p:3d}", pages[p], skip_byte=pb)
  else:
    report_changes(f"##### bus{bus} 0x{addr:03x}", fr)
