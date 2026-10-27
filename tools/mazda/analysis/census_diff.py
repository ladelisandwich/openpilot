"""Which bytes gained values between two census reports. Usage: python3 -I census_diff.py old.txt new.txt"""

import re
import sys

def load(fn):
  out, cur = {}, None
  for line in open(fn):
    m = re.match(r"^(bus\d) 0x([0-9a-f]+) (\S+)\s+([\d.]+) Hz", line)
    if m:
      cur = (m.group(1), int(m.group(2), 16), m.group(3))
      out[cur] = {"rate": float(m.group(4)), "bytes": line.split("Hz", 1)[1].strip() if "bytes:" not in line else ""}
      if "b0" in line and "bytes:" not in line:      # the named section puts the bytes on the same line
        out[cur]["bytes"] = re.sub(r"^.*Hz\s+", "", line.strip())
      continue
    if cur and line.strip().startswith("bytes:"):
      out[cur]["bytes"] = line.strip()[len("bytes:"):].strip()
  return out

def nvals(desc):
  """distinct-value count per byte from a descriptor like 'b3:16v' / 'b3=0x20' / 'b3=noisy ..' / 'b3=MUX(..' / 'b3=ctr..'"""
  res = {}
  for tok in re.findall(r"b(\d)[=:]([^\s]+(?:\s[\d-]+)?)", desc):
    b, d = int(tok[0]), tok[1]
    if d.startswith("0x"):
      res[b] = 1
    elif re.match(r"^\d+v", d):
      res[b] = int(d.split("v")[0])
    elif d.startswith("toggle"):
      res[b] = 2
    else:
      res[b] = 256   # counters, checksums, noisy, mux: already busy
  return res

old, new = load(sys.argv[1]), load(sys.argv[2])
rows = []
for key, n in sorted(new.items(), key=lambda kv: (kv[0][0], kv[0][1])):
  if key[0] != "bus0" or key[1] >= 0x700:
    continue
  o = old.get(key)
  ov = nvals(o["bytes"]) if o else {}
  nv = nvals(n["bytes"])
  grew = [(b, ov.get(b), nv[b]) for b in sorted(nv) if nv[b] > (ov.get(b, 0) if o else 0) and nv[b] < 256 and (ov.get(b, 1) if o else 1) < 256]
  if grew or not o:
    tag = "NEW " if not o else ""
    g = " ".join(f"b{b}:{a if a is not None else '-'}>{c}" for b, a, c in grew)
    rows.append(f"{tag}{key[0]} 0x{key[1]:03x} {key[2]:18s} {n['rate']:5.1f} Hz  {g}")
print("\n".join(rows))
