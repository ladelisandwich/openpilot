"""Protect one pinned stock QCOM capture; no artifact selection or metadata bypass."""
from collections import Counter
import hashlib
from tinygrad.uop.ops import Ops
from tinygrad.runtime.support.hcq2 import HCQInfo


def protect_capture(jit, protection):
  cap = getattr(jit, 'captured', None)
  if cap is None or 'linear' in cap.__dict__:
    raise ValueError('Expected a never-executed stored capture')
  for entry in cap._linear.src:
    call = entry.without_after
    if call.op is not Ops.CALL or not (call.body.op is Ops.COPY or isinstance(call.arg.aux, HCQInfo)):
      raise ValueError('Unknown stock capture schedule')
  def inventory(root):
    cpu = {u.src[-1] for u in root.toposort() if u.op is Ops.PROGRAM and u.arg.target.device == 'CPU'}
    return Counter(hashlib.sha256(u.arg).hexdigest() for u in root.toposort() if u.op is Ops.BINARY and u not in cpu)
  replacements = {}
  for u in cap._linear.toposort():
    if u.op is Ops.CALL and isinstance(u.arg.aux, HCQInfo):
      if u.arg.aux.device != ('QCOM',):
        raise ValueError('Stock capture is not the declared QCOM backend')
      replacements[u] = protection.protect_call(u)
  if not replacements:
    raise ValueError('No native QCOM submission')
  proposed = cap._linear.substitute(replacements, enter_calls=True)
  if inventory(cap._linear) != inventory(proposed):
    raise ValueError('Stock capture opaque device payload changed')
  cap._linear = proposed
  return jit
