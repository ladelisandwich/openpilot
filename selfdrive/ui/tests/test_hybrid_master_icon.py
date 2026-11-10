import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

MAZDA_HYBRID = 256 | 512  # MazdaSafetyFlags.RADAR_EMULATION | MazdaSafetyFlags.HYBRID_LONG
EMULATING, PENDING, ACTIVE = 1, 2, 4


def _load_icon(monkeypatch, memory=None):
  draws = []
  rl = SimpleNamespace(
    Color=lambda r, g, b, a=255: SimpleNamespace(r=r, g=g, b=b, a=a),
    Rectangle=lambda x=0, y=0, width=0, height=0: SimpleNamespace(x=x, y=y, width=width, height=height),
    Vector2=lambda x, y: SimpleNamespace(x=x, y=y),
    Texture=object,
    draw_circle_v=lambda *args: draws.append(("circle", args)),
    draw_texture_pro=lambda texture, src, dest, origin, rot, color: draws.append(("texture", texture, src, dest, color)),
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


def _render_at(mod, icon, memory, draws, status, t, monkeypatch):
  monkeypatch.setattr(mod, "time", SimpleNamespace(monotonic=lambda: t))
  memory["MazdaHybridMaster"] = status
  icon._last_read = -1e9
  draws.clear()
  icon.render()
  return [d for d in draws if d[0] == "texture"]


def _icon(mod):
  icon = mod.HybridMasterIcon()
  icon.set_rect(SimpleNamespace(x=0, y=0, width=144, height=144))
  return icon


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


def test_mrcc_is_the_car_and_its_waves_still_unless_mrcc_drives(monkeypatch):
  mod, _, memory, draws = _load_icon(monkeypatch)
  icon = _icon(mod)
  for t in (0.0, 0.37, 1.1):
    textures = _render_at(mod, icon, memory, draws, 0, t, monkeypatch)
    assert [d[1].name for d in textures] == ["mazda_mrcc_car.png", "mazda_mrcc_wave1.png", "mazda_mrcc_wave2.png",
                                             "mazda_mrcc_wave3.png"]
    assert all(d[4].a == 255 for d in textures), t   # not driving: no animation


def test_mrcc_driving_sends_the_waves_out_smallest_first(monkeypatch):
  mod, _, memory, draws = _load_icon(monkeypatch)
  icon = _icon(mod)
  rest = round(255 * mod.WAVE_REST_ALPHA)
  for wave, peak in enumerate(mod.WAVE_PEAKS):
    textures = _render_at(mod, icon, memory, draws, ACTIVE, peak * mod.WAVE_PERIOD_S, monkeypatch)
    alphas = [d[4].a for d in textures]
    assert alphas[0] == 255                                    # the car itself never fades
    assert alphas[1 + wave] == 255, (wave, alphas)             # this wave at its brightest...
    others = [a for i, a in enumerate(alphas[1:]) if i != wave]
    assert all(rest <= a < 128 for a in others), (wave, alphas)   # ...the others resting, the next one only starting
  assert list(mod.WAVE_PEAKS) == sorted(mod.WAVE_PEAKS)


def test_eye_is_centred_unless_openpilot_drives(monkeypatch):
  mod, _, memory, draws = _load_icon(monkeypatch)
  icon = _icon(mod)
  centre = mod.eye_frame(0.0)
  assert centre == 12
  for t in (0.0, 1.4, 3.0):
    textures = _render_at(mod, icon, memory, draws, EMULATING, t, monkeypatch)
    assert len(textures) == 1 and textures[0][1].name == "mazda_comma_vision_frames.png"
    src = textures[0][2]
    assert (src.x, src.y) == ((centre % 5) * 104, (centre // 5) * 104), t
    assert textures[0][4].a == 255


def test_eye_looks_left_and_right_while_openpilot_drives(monkeypatch):
  mod, _, memory, draws = _load_icon(monkeypatch)
  icon = _icon(mod)
  seen = {}
  for t in (0.3, 1.4, 3.0):
    src = _render_at(mod, icon, memory, draws, EMULATING | ACTIVE, t, monkeypatch)[0][2]
    seen[t] = int(src.y // 104) * 5 + int(src.x // 104)
  assert seen == {0.3: 12, 1.4: 0, 3.0: 24}   # straight ahead, full left, full right


def test_pending_pulses_whichever_icon(monkeypatch):
  mod, _, memory, draws = _load_icon(monkeypatch)
  icon = _icon(mod)
  trough = 0.5 / mod.PULSE_HZ   # cos = -1
  for status in (PENDING, EMULATING | PENDING, PENDING | ACTIVE):
    textures = _render_at(mod, icon, memory, draws, status, trough, monkeypatch)
    assert textures[0][4].a == 110, status


def test_pure_animation_curves_are_smooth_and_bounded(monkeypatch):
  mod, _, _, _ = _load_icon(monkeypatch)
  step = 0.01
  looks = [mod.eye_look(i * step) for i in range(int(2 * mod.EYE_PERIOD_S / step))]
  assert min(looks) == -1.0 and max(looks) == 1.0
  assert max(abs(b - a) for a, b in zip(looks, looks[1:], strict=False)) < 0.08   # no jump, also across the wrap
  waves = [mod.wave_alphas(i * step) for i in range(int(2 * mod.WAVE_PERIOD_S / step))]
  for w in range(3):
    series = [x[w] for x in waves]
    assert min(series) == mod.WAVE_REST_ALPHA and max(series) <= 1.0
    assert max(abs(b - a) for a, b in zip(series, series[1:], strict=False)) < 0.06
  assert all(0 <= mod.eye_frame(x) < mod.EYE_FRAMES for x in (-1.5, -1.0, 0.0, 1.0, 1.5))


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
