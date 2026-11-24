"""Persistent StarPort enable state."""

PARAM = "StarPortEnabled"


def enabled(params) -> bool:
  return params.get_bool(PARAM)


def set_enabled(params, value: bool) -> None:
  params.put_bool(PARAM, value, block=True)
