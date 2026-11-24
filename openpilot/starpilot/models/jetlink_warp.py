"""Jetlink's warp (jetlink.openpilot.warp.Warp's interface) over the small model's own camera warp.

The stock driving warp Jetlink ships with is a legacy9d pickle, which a native02e modeld cannot load. A catalog
model on the policy image pipeline already runs the same warp every frame, compiled for whichever runtime it was
selected under, and makes the frame the link sends: both cameras as (2, 6, model_h/2, model_w/2) uint8, the same
YUV layout as examples/openpilot/compile_warp.py's yuv420 warp. Reusing it keeps the link on the user's small
model's runtime and loads no second warp.

Only one of the small and the large model runs a frame (JoiningModelState), so the warp JIT, its inputs and its
output buffer are never in use by both at once. The small model rewrites its transforms every frame it runs.

tinygrad maps a QCOM output write-combined, which the CPU reads uncached: 2.9 ms to copy the frame out. As
Jetlink's own warp does (jetlink.openpilot.warp.Warp), the small model moves its warp's output to IO-coherent
write-back memory when it loads (CatalogModelState, with Jetlink on), and the link sends straight from it.
"""
from __future__ import annotations

import numpy as np

PROBE = 77


class CatalogWarp:
  def __init__(self, small, geometry: tuple[int, int]):
    descriptor = small.camera_warp_descriptor
    if descriptor is None:
      raise RuntimeError("the small model's image history is not on the policy pipeline: it has no shared warp")
    model_w, model_h = geometry
    shape = (2, 6, model_h // 2, model_w // 2)
    if tuple(descriptor.output_shape) != shape or descriptor.dtype.itemsize != 1:
      raise RuntimeError(f"the small model's warp makes {descriptor.output_shape} {descriptor.dtype}, the link sends {shape} uint8")
    self._small = small
    # before modeld goes realtime: a blank frame finds the output buffer every frame writes (the small model's
    # warmup captured the JIT, which returns the same one every call), then a flat frame proves `output` reads it
    # live. as_memoryview() hands back a one-off copy where it finds no mapping, which would send the blank frame
    # forever; a flat frame under the identity samples to itself
    frames = [np.zeros(small.frame_buf_size, dtype=np.uint8) for _ in small.vision_input_names]
    pointers = tuple(frame.ctypes.data for frame in frames)
    eye = np.eye(3, dtype=np.float32)
    buffer = small.warp_frames(pointers, eye, eye).uop.base.buffer
    mapped = np.frombuffer(buffer.as_memoryview(allow_zero_copy=True), dtype=np.uint8)
    expected = int(np.prod(shape))
    if buffer.nbytes != expected or mapped.nbytes < expected:
      raise RuntimeError(f"the warp's output is {buffer.nbytes} bytes ({mapped.nbytes} mapped), not {expected}")
    mapped, synchronize = mapped[:expected], buffer.allocator.dev.synchronize
    if small.warp_copied is None:
      self.sending = "from coherent memory"
      self.output, self.wait = memoryview(mapped), synchronize
    else:
      self.sending = f"copied out: {small.warp_copied}"
      host = np.empty(expected, dtype=np.uint8)
      self.output = memoryview(host)

      def wait():
        synchronize()
        np.copyto(host, mapped)
      self.wait = wait
    for frame in frames:
      frame.fill(PROBE)
    self.start(pointers[0], pointers[1], eye, eye)
    self.wait()
    # the frames go with this call; their cache entries must not outlive them
    for key in zip(small.vision_input_names, pointers, strict=True):
      small._blob_cache.pop(key, None)
    if not (np.frombuffer(self.output, dtype=np.uint8) == PROBE).all():
      raise RuntimeError("the small model's warp output is not read live: the link would send a stale frame")

  def start(self, frame: int, big_frame: int, tfm, big_tfm) -> None:
    """Warp the camera buffers at these addresses under their transforms; `output` holds it once wait() returns."""
    self._small.warp_frames((frame, big_frame), tfm, big_tfm)
