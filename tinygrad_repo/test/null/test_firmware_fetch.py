import unittest, hashlib, io, pathlib
from unittest.mock import patch
from tinygrad.helpers import fetch_fw

class TestFirmwareFetch(unittest.TestCase):
  def test_local_zstd_unknown_content_size_and_hash(self):
    from zstandard import ZstdCompressor
    firmware = b"verified AMD firmware fixture" * 20
    digest = hashlib.sha256(firmware).hexdigest()
    for write_content_size in (False, True):
      compressed = ZstdCompressor(write_content_size=write_content_size).compress(firmware)
      with self.subTest(write_content_size=write_content_size), patch.object(pathlib.Path, "is_file", return_value=True), \
           patch.object(pathlib.Path, "open", return_value=io.BytesIO(compressed)), \
           patch("tinygrad.helpers.fetch", side_effect=AssertionError("network fallback")):
        self.assertEqual(fetch_fw("amdgpu", "fixture.bin", digest), firmware)

  def test_local_zstd_wrong_hash_uses_pinned_fallback(self):
    from zstandard import ZstdCompressor
    compressed = ZstdCompressor(write_content_size=False).compress(b"wrong firmware")
    with patch.object(pathlib.Path, "is_file", return_value=True), patch.object(pathlib.Path, "open", return_value=io.BytesIO(compressed)), \
         patch("tinygrad.helpers.fetch", side_effect=RuntimeError("hash mismatch fallback")) as fallback:
      with self.assertRaisesRegex(RuntimeError, "hash mismatch fallback"):
        fetch_fw("amdgpu", "fixture.bin", hashlib.sha256(b"expected firmware").hexdigest())
    fallback.assert_called_once_with(
      "https://gitlab.com/kernel-firmware/linux-firmware/-/raw/0a6871b19abf5d6e024b5d208b101ae53e7fa0de/amdgpu/fixture.bin",
      subdir="fw", sha256=hashlib.sha256(b"expected firmware").hexdigest())

if __name__ == '__main__':
  unittest.main()
