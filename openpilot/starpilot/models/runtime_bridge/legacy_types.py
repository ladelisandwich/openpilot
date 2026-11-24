"""Type-only facets of the selected, pinned 9d runtime's legacy APIs."""
from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
  from pickle import PickleBuffer
  from tinygrad.device import BufferSpec
  from tinygrad.dtype import DType
  from tinygrad.uop.ops import UOp


class LegacyBuffer(Protocol):
  @property
  def nbytes(self) -> int: ...
  def copy_from(self, src: LegacyBuffer) -> LegacyBuffer: ...
  def view(self, size: int, dtype: DType, offset: int) -> LegacyBuffer: ...
  def ensure_allocated(self) -> LegacyBuffer: ...


class LegacyBufferFactory(Protocol):
  def __call__(self, device: str, size: int, dtype: DType, *, options: BufferSpec | None = None,
               initial_value: bytes | PickleBuffer | None = None, preallocate: bool = False) -> LegacyBuffer: ...


class LegacyUOpFactory(Protocol):
  def from_buffer(self, opaque: LegacyBuffer, device: str | tuple[str, ...] | None = None) -> UOp: ...


class LegacyCall(Protocol):
  def call(self, *srcs: UOp, ret_dtype: DType | None = None) -> UOp: ...


class LegacyHCQModule(Protocol):
  lower_and_compile: Callable[[UOp], UOp]


class LegacyPersistentUnpickler(Protocol):
  persistent_load: Callable[[tuple[int, DType, int]], LegacyBuffer]
