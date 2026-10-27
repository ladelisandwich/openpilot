import struct

from mazda_sim.device.prepare_build import is_device_binary


def elf(machine: int) -> bytes:
  return b"\x7fELF\x02\x01\x01" + bytes(11) + struct.pack("<H", machine) + bytes(40)


def ar(*members: bytes) -> bytes:
  out = b"!<arch>\n"
  for i, m in enumerate(members):
    name = (b"/" if i == 0 else f"m{i}.o/".encode()).ljust(16)
    out += name + b"0".ljust(12) + b"0".ljust(6) + b"0".ljust(6) + b"644".ljust(8) + str(len(m)).encode().ljust(10) + b"`\n"
    out += m + (b"\n" if len(m) % 2 else b"")
  return out


def test_device_binaries(tmp_path):
  cases = {"arm.so": (elf(0xB7), True), "x86.so": (elf(0x3E), False), "text.py": (b"print('hi')\n", False),
           "arm.a": (ar(b"\x00\x00\x00\x01symtab", elf(0xB7)), True), "x86.a": (ar(b"\x00" * 9, elf(0x3E)), False)}
  for name, (data, want) in cases.items():
    p = tmp_path / name
    p.write_bytes(data)
    assert is_device_binary(p) is want, name
