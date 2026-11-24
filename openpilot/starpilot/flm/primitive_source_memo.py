"""Lossless primitive memo sharing for level1 transport, never evidence authority.

Wire remains the existing classless primitive pickle. All dictionary/list/tuple
occurrences are independently rebuilt on decode. Identity is never a digest or
permission proof. Float keys use binary64 bits, never numeric equality.
"""
import math
import struct
from openpilot.starpilot.flm.native_primitives import try_native as _try_native, FALLBACK as _NATIVE_FALLBACK

VERSION='lossless-source-primitive-memo-v1'
MAX_LOGICAL_BYTES=128*1024*1024
MAX_DEPTH=128


def _scan(value):
  nodes=[]
  sizes=[]
  heights=[]
  keys={}
  original={}
  active=set()
  def visit(item,depth,recurse):
    if depth>MAX_DEPTH:
      raise ValueError('primitive memo depth bound')
    kind=type(item)
    if item is None:
      key=('none',)
      cost=1
      children=None
    elif kind is bool:
      key=('bool',item)
      cost=2
      children=None
    elif kind is int:
      key=('int',item)
      cost=9+(abs(item).bit_length()+7)//8
      children=None
    elif kind is float:
      if not math.isfinite(item):
        raise ValueError('nonfinite primitive memo input')
      key=('float',struct.pack('!d',item))
      cost=9
      children=None
    elif kind in (str,bytes):
      key=('str' if kind is str else 'bytes',item)
      cost=9+len(item.encode('utf8','surrogatepass') if kind is str else item)
      children=None
    elif kind in (tuple,list,dict):
      ident=id(item)
      if ident in active:
        raise ValueError('cyclic primitive memo input')
      if ident in original:
        index=original[ident]
        if depth+heights[index]>MAX_DEPTH:
          raise ValueError('primitive memo depth bound')
        return index
      if kind is dict and any(type(k) is not str for k in item):
        raise ValueError('primitive memo dictionary key')
      active.add(ident)
      try:
        children=tuple(recurse(child,depth+1,recurse) for child in (tuple(v for pair in item.items() for v in pair) if kind is dict else item))
      finally:
        active.remove(ident)
      cost=9+sum(sizes[index] for index in children)
      key=('dict' if kind is dict else 'tuple' if kind is tuple else 'list',children)
    else:
      raise ValueError('primitive memo input type')
    height=0 if children is None else 1+max((heights[index] for index in children),default=-1)
    if depth+height>MAX_DEPTH:
      raise ValueError('primitive memo depth bound')
    if cost>MAX_LOGICAL_BYTES:
      raise ValueError('primitive memo logical expansion bound')
    index=keys.get(key)
    if index is None:
      if children is None:
        out=item
      elif kind is tuple:
        out=tuple(nodes[i] for i in children)
      elif kind is list:
        out=[nodes[i] for i in children]
      else:
        out={nodes[children[i]]:nodes[children[i+1]] for i in range(0,len(children),2)}
      index=len(nodes)
      nodes.append(out)
      sizes.append(cost)
      heights.append(height)
      keys[key]=index
    if children is not None:
      original[id(item)]=index
    return index
  index=visit(value,0,visit)
  return nodes[index],sizes[index]


def intern_primitive_graph(value):
  """Exact typed full-key equality; shared composites exist ONLY during encoding."""
  return _scan(value)[0]


def _logical_limits(value):
  """Validation-only local immutable/owned-object memo; no canonical graph built."""
  memo={}
  active=set()
  def visit(item,depth,recurse):
    if depth>MAX_DEPTH:
      raise ValueError('primitive memo depth bound')
    ident=id(item)
    if ident in active:
      raise ValueError('cyclic primitive memo input')
    if ident in memo:
      cost,height=memo[ident]
      if depth+height>MAX_DEPTH:
        raise ValueError('primitive memo depth bound')
      return cost,height
    kind=type(item)
    height=0
    if item is None:
      cost=1
    elif kind is bool:
      cost=2
    elif kind is int:
      cost=9+(abs(item).bit_length()+7)//8
    elif kind is float:
      if not math.isfinite(item):
        raise ValueError('nonfinite primitive memo input')
      cost=9
    elif kind in (str,bytes):
      cost=9+len(item.encode('utf8','surrogatepass') if kind is str else item)
    elif kind in (tuple,list,dict):
      if kind is dict and any(type(k) is not str for k in item):
        raise ValueError('primitive memo dictionary key')
      active.add(ident)
      cost=9
      maximum=-1
      try:
        children=(v for pair in item.items() for v in pair) if kind is dict else item
        for child in children:
          child_cost,child_height=recurse(child,depth+1,recurse)
          cost+=child_cost
          maximum=max(maximum,child_height)
          if cost>MAX_LOGICAL_BYTES:
            raise ValueError('primitive memo logical expansion bound')
      finally:
        active.remove(ident)
      height=1+maximum
    else:
      raise ValueError('primitive memo input type')
    if depth+height>MAX_DEPTH:
      raise ValueError('primitive memo depth bound')
    if cost>MAX_LOGICAL_BYTES:
      raise ValueError('primitive memo logical expansion bound')
    memo[ident]=(cost,height)
    return cost,height
  return visit(value,0,visit)


def _independent_primitive_graph_python(value):
  """Validate logical expansion BEFORE allocating independently owned containers."""
  _logical_limits(value)
  def copy(item,recurse):
    kind=type(item)
    if kind is dict:
      return {key:recurse(child,recurse) for key,child in item.items()}
    if kind is list:
      return [recurse(child,recurse) for child in item]
    if kind is tuple:
      return tuple(recurse(child,recurse) for child in item)
    return item
  return copy(value,copy)


def independent_primitive_graph(value, *, reserved_bytes=0):
  if type(reserved_bytes) is not int or not 0 <= reserved_bytes <= 512 * 1024:
    raise ValueError('bounded normalized transient reservation')
  if reserved_bytes:
    # Declared bounded allocation padding, never encoded semantic trace data.
    return independent_primitive_graph((value, bytes(reserved_bytes)))[0]
  result = _try_native('independent_primitive_graph', value)
  return _independent_primitive_graph_python(value) if result is _NATIVE_FALLBACK else result
