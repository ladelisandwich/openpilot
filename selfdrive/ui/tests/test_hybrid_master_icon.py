import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

MAZDA_HYBRID = 256 | 512  # MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG


def _load_icon(monkeypatch, memory=None):
  draws = []
  rl = SimpleNamespace(
    Color=lambda r, g, b, a=255: SimpleNamespace(r=r, g=g, b=b, a=a),
    Rectangle=lambda x=0, y=0, width=0, height=0: SimpleNamespace(x=x, y=y, width=width, height=height),
    Vector2=lambda x, y: SimpleNamespace(x=x, y=y),
    draw_circle_v=lambda *args: draws.append(("circle", args)),
    draw_texture_ex=lambda texture, pos, rot, scale, color: draws.append(("texture", texture, color)),
  )
  monkeypatch.setitem(sys.modules, "pyray", rl)

  def module(name, **attributes):
    result = ModuleType(name)
    for key, value in attributes.items():
      setattr(result, key, value)
    monkeypatch.setitem(sys.modules, name, result)

  class Widget:
    def __init__(self):
      self._is_visible = True
      self._rect = rl.Rectangle()

    @property
    def is_visible(self):
      return self._is_visible() if callable(self._is_visible) else self._is_visible

    def set_visible(self, visible):
      self._is_visible = visible

    def set_rect(self, rect):
      self._rect = rect

    @property
    def rect(self):
      return self._rect

    def render(self):
      self._update_state()
      if self.is_visible:
        self._render(self._rect)

  def texture(path, w, h):
    return SimpleNamespace(name=path.split("/")[-1], width=w, height=h)

  memory = {} if memory is None else memory
  ui_state = SimpleNamespace(
    started=True,
    CP=SimpleNamespace(brand="mazda", flags=MAZDA_HYBRID),
    params_memory=SimpleNamespace(get_int=lambda key, default=0: memory.get(key, default)),
  )
  module("openpilot.system.ui.widgets", Widget=Widget)
  module("openpilot.system.ui.lib.application", gui_app=SimpleNamespace(texture=texture))
  module("openpilot.selfdrive.ui.ui_state", ui_state=ui_state)

  path = Path(__file__).parents[1] / "onroad/starpilot/widgets/hybrid_master_icon.py"
  spec = importlib.util.spec_from_file_location("hybrid_master_icon_under_test", path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod, ui_state, memory, draws


def test_only_shown_onroad_for_a_mazda_with_hybrid_long(monkeypatch):
  mod, ui_state, _, _ = _load_icon(monkeypatch)
  icon = mod.HybridMasterIcon()
  assert icon.is_visible
  ui_state.started = False
  assert not icon.is_visible
  ui_state.started = True
  for cp in (None, SimpleNamespace(brand="mazda", flags=256), SimpleNamespace(brand="mazda", flags=512),
             SimpleNamespace(brand="toyota", flags=MAZDA_HYBRID)):
    ui_state.CP = cp
    assert not icon.is_visible, cp


def test_shows_the_master_the_car_controller_reports(monkeypatch):
  mod, _, memory, draws = _load_icon(monkeypatch)
  icon = mod.HybridMasterIcon()
  icon.set_rect(SimpleNamespace(x=0, y=0, width=144, height=144))

  for status, name, solid in ((0, "mazda_mrcc_radar.png", True), (1, "mazda_comma_vision.png", True),
                              (2, "mazda_mrcc_radar.png", False), (3, "mazda_comma_vision.png", False)):
    memory["MazdaHybridMaster"] = status
    icon._last_read = 0.0
    draws.clear()
    icon.render()
    textures = [d for d in draws if d[0] == "texture"]
    assert len(textures) == 1 and textures[0][1].name == name, status
    if solid:
      assert textures[0][2].a == 255, status
    else:
      assert 110 <= textures[0][2].a <= 255, status  # pending: pulsing


def test_sits_left_of_the_wheel_centred_on_it(monkeypatch):
  mod, _, _, _ = _load_icon(monkeypatch)
  icon = mod.HybridMasterIcon()
  wheel = SimpleNamespace(x=1800, y=75, width=192, height=192)
  icon.place_left_of(wheel, 15)
  assert icon.rect.x + icon.rect.width == wheel.x - 15
  assert icon.rect.y + icon.rect.height / 2 == wheel.y + wheel.height / 2


def test_hidden_icon_does_not_read_params(monkeypatch):
  mod, ui_state, _, _ = _load_icon(monkeypatch)
  reads = []
  ui_state.params_memory = SimpleNamespace(get_int=lambda key, default=0: reads.append(key) or 0)
  ui_state.CP = SimpleNamespace(brand="honda", flags=0)
  icon = mod.HybridMasterIcon()
  icon.render()
  assert reads == []
