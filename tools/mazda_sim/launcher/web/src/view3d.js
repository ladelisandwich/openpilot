// The dashboard's 3D road view: drawn in the browser (WebGL, your GPU) from the road the car sends once and the
// telemetry it streams at 20 Hz. Nothing here feeds the comma: its cameras come from the car (MetaDrive) or the
// ground-truth model. Bundled with three.js into ../view3d.js by ../build.sh.
//
// Frames: the car's road frame is x east, y north, z up, heading counter-clockwise from x. three.js is y-up, so a
// road point (x, y) sits at (x, 0, -y) and a heading `yaw` is rotation.y = yaw. Car models face local +x, with
// local +z on their right.
import * as THREE from "three";

const CAR = { length: 5.07, width: 1.97, wheelbase: 2.93, wheelR: 0.37, steerRatio: 17.6 };
const DASH_ON = 3.0, DASH_PERIOD = 12.0;
const MODES = {
  chase: { back: 7.5, up: 2.7, ahead: 7.0, lookUp: 1.0, fov: 60 },
  far: { back: 15.0, up: 5.5, ahead: 14.0, lookUp: 0.8, fov: 55 },
  overhead: { back: 0.01, up: 45.0, ahead: 0.0, lookUp: 0.0, fov: 55 },
};

function noiseTexture(base, spread, size = 256, streaks = false) {
  const c = document.createElement("canvas");
  c.width = c.height = size;
  const g = c.getContext("2d");
  const img = g.createImageData(size, size);
  let seed = 1234567;
  const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < size * size; i++) {
    const n = (rnd() - 0.5) * spread + (streaks ? Math.sin(i / size * 0.9) * spread * 0.15 : 0);
    img.data[i * 4] = Math.max(0, Math.min(255, base[0] + n));
    img.data[i * 4 + 1] = Math.max(0, Math.min(255, base[1] + n));
    img.data[i * 4 + 2] = Math.max(0, Math.min(255, base[2] + n));
    img.data[i * 4 + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

// what shiny surfaces reflect: sky above, a bright horizon, grass below
function environmentTexture(renderer) {
  const c = document.createElement("canvas");
  c.width = 64; c.height = 32;
  const g = c.getContext("2d");
  const grad = g.createLinearGradient(0, 0, 0, 32);
  grad.addColorStop(0, "#4f86c9");
  grad.addColorStop(0.47, "#d8e7f3");
  grad.addColorStop(0.53, "#8d9a7c");
  grad.addColorStop(1, "#3f5a2e");
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 32);
  const t = new THREE.CanvasTexture(c);
  t.mapping = THREE.EquirectangularReflectionMapping;
  t.colorSpace = THREE.SRGBColorSpace;
  const pmrem = new THREE.PMREMGenerator(renderer);
  const env = pmrem.fromEquirectangular(t).texture;
  pmrem.dispose();
  t.dispose();
  return env;
}

function skyTexture() {
  const c = document.createElement("canvas");
  c.width = 4; c.height = 256;
  const g = c.getContext("2d");
  const grad = g.createLinearGradient(0, 0, 0, 256);
  grad.addColorStop(0, "#3b78c4");
  grad.addColorStop(0.55, "#8fbde8");
  grad.addColorStop(1, "#dbe9f4");
  g.fillStyle = grad;
  g.fillRect(0, 0, 4, 256);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// ---------------------------------------------------------------- car model
function carModel(color, opts = {}) {
  const root = new THREE.Group();
  const paint = new THREE.MeshPhysicalMaterial({ color, metalness: 0.35, roughness: 0.38, clearcoat: 1.0, clearcoatRoughness: 0.08 });
  const glass = new THREE.MeshPhysicalMaterial({ color: 0x1a222b, metalness: 0.1, roughness: 0.05, clearcoat: 1.0 });
  const trim = new THREE.MeshStandardMaterial({ color: 0x15171a, roughness: 0.7 });
  const L = CAR.length / 2, W = CAR.width - 0.06;

  // SUV side profile (x along the car, y up), extruded across its width
  const profile = [[-L, 0.32], [-L - 0.03, 0.86], [-L + 0.08, 1.06], [-L + 0.42, 1.70], [0.45, 1.74], [1.28, 1.14],
                   [L - 0.12, 0.98], [L + 0.02, 0.70], [L - 0.06, 0.32]];
  const body = new THREE.Shape();
  profile.forEach(([x, y], i) => i ? body.lineTo(x, y) : body.moveTo(x, y));
  const bodyGeo = new THREE.ExtrudeGeometry(body, { depth: W, bevelEnabled: false });
  bodyGeo.translate(0, 0, -W / 2);
  const bodyMesh = new THREE.Mesh(bodyGeo, paint);
  bodyMesh.castShadow = true;
  root.add(bodyMesh);

  // the glasshouse: the cabin part of the profile grown a little, so glass shows on every side, then the roof
  // painted back over it
  const cabin = [[-L + 0.08, 1.06], [-L + 0.42, 1.70], [0.45, 1.74], [1.28, 1.14]];
  const gx = cabin.reduce((a, q) => a + q[0], 0) / cabin.length, gy = cabin.reduce((a, q) => a + q[1], 0) / cabin.length;
  const gh = new THREE.Shape();
  cabin.forEach(([x, y], i) => {
    const px = gx + (x - gx) * 1.02, py = gy + (y - gy) * 1.06;
    i ? gh.lineTo(px, py) : gh.moveTo(px, py);
  });
  const ghGeo = new THREE.ExtrudeGeometry(gh, { depth: W + 0.04, bevelEnabled: false });
  ghGeo.translate(0, 0, -(W + 0.04) / 2);
  root.add(new THREE.Mesh(ghGeo, glass));
  const roofX0 = -L + 0.5, roofX1 = 0.40;
  const roof = new THREE.Mesh(new THREE.BoxGeometry(roofX1 - roofX0, 0.05, W - 0.16), paint);
  roof.position.set((roofX0 + roofX1) / 2, 1.77, 0);
  roof.castShadow = true;
  root.add(roof);

  // black lower cladding and bumpers
  const clad = new THREE.Mesh(new THREE.BoxGeometry(CAR.length - 0.25, 0.16, W + 0.12), trim);
  clad.position.y = 0.36;
  root.add(clad);

  // wheels: tire + rim, front pair steers
  const tireGeo = new THREE.CylinderGeometry(CAR.wheelR, CAR.wheelR, 0.26, 28);
  tireGeo.rotateX(Math.PI / 2);
  const rimGeo = new THREE.CylinderGeometry(CAR.wheelR * 0.62, CAR.wheelR * 0.62, 0.27, 6);
  rimGeo.rotateX(Math.PI / 2);
  const tireMat = new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.9 });
  const rimMat = new THREE.MeshStandardMaterial({ color: 0xb8bcc2, metalness: 0.9, roughness: 0.25 });
  const wheels = [], steer = [];
  for (const [x, front] of [[CAR.wheelbase / 2, true], [-CAR.wheelbase / 2, false]]) {
    for (const z of [-0.84, 0.84]) {
      const knuckle = new THREE.Group();
      knuckle.position.set(x, CAR.wheelR, z);
      const spin = new THREE.Group();
      const tire = new THREE.Mesh(tireGeo, tireMat);
      tire.castShadow = true;
      spin.add(tire, new THREE.Mesh(rimGeo, rimMat));
      knuckle.add(spin);
      root.add(knuckle);
      wheels.push(spin);
      if (front) steer.push(knuckle);
    }
  }

  // lamps
  const lamp = (c, emissive, w, h, d) => new THREE.Mesh(new THREE.BoxGeometry(w, h, d),
    new THREE.MeshStandardMaterial({ color: c, emissive, emissiveIntensity: 0.0, roughness: 0.3 }));
  const head = [], tail = [], blinkL = [], blinkR = [];
  for (const z of [-0.68, 0.68]) {
    const h = lamp(0xdde6f0, 0xffffff, 0.08, 0.1, 0.42); h.position.set(L + 0.0, 0.92, z); root.add(h); head.push(h);
    const t = lamp(0x5a0a0a, 0xff1a1a, 0.08, 0.12, 0.46); t.position.set(-L - 0.02, 1.0, z); root.add(t); tail.push(t);
    for (const x of [L - 0.02, -L - 0.03]) {
      const b = lamp(0x6a4a10, 0xffa21a, 0.08, 0.08, 0.14);
      b.position.set(x, x > 0 ? 0.80 : 0.86, z * 1.18);
      root.add(b);
      (z < 0 ? blinkL : blinkR).push(b);
    }
  }
  head.forEach(h => (h.material.emissiveIntensity = opts.headlights ? 1.2 : 0.35));
  tail.forEach(t => (t.material.emissiveIntensity = 0.4));
  root.userData = { wheels, steer, tail, blinkL, blinkR };
  return root;
}

// ---------------------------------------------------------------- road geometry
class RoadPath {
  constructor(road) {
    this.x = road.x; this.y = road.y; this.closed = road.closed;
    const n = this.x.length;
    this.s = new Float64Array(n);
    for (let i = 1; i < n; i++) this.s[i] = this.s[i - 1] + Math.hypot(this.x[i] - this.x[i - 1], this.y[i] - this.y[i - 1]);
    this.length = this.s[n - 1] + (this.closed ? Math.hypot(this.x[0] - this.x[n - 1], this.y[0] - this.y[n - 1]) : 0);
  }

  // (x, y, heading) at arc length s, shifted left by `offset` (the car's convention: lanes have negative offsets)
  at(s, offset = 0) {
    const S = this.s, n = S.length;
    if (this.closed) s = ((s % this.length) + this.length) % this.length;
    else s = Math.max(0, Math.min(S[n - 1], s));
    let lo = 0, hi = n - 1;
    if (this.closed && s >= S[n - 1]) { lo = n - 1; hi = 0; }
    else {
      while (hi - lo > 1) { const m = (lo + hi) >> 1; if (S[m] <= s) lo = m; else hi = m; }
    }
    const s0 = S[lo], s1 = hi === 0 ? this.length : S[hi];
    const f = s1 > s0 ? (s - s0) / (s1 - s0) : 0;
    const x = this.x[lo] + f * (this.x[hi] - this.x[lo]), y = this.y[lo] + f * (this.y[hi] - this.y[lo]);
    const h = Math.atan2(this.y[hi] - this.y[lo], this.x[hi] - this.x[lo]);
    return [x - Math.sin(h) * offset, y + Math.cos(h) * offset, h];
  }
}

// a flat ribbon between two lateral offsets, sampled every `step` metres from s0 to s1
function ribbon(path, offLeft, offRight, s0, s1, step, uvScale, lift = 0) {
  const pos = [], uv = [], idx = [];
  const n = Math.max(2, Math.ceil((s1 - s0) / step) + 1);
  for (let i = 0; i < n; i++) {
    const s = Math.min(s1, s0 + i * step);
    const [xl, yl] = path.at(s, offLeft), [xr, yr] = path.at(s, offRight);
    pos.push(xl, lift, -yl, xr, lift, -yr);
    uv.push(0, s / uvScale, (offLeft - offRight) / uvScale, s / uvScale);
    if (i) { const a = (i - 1) * 2; idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2); }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

function mergeRibbons(geos) {
  let nv = 0, ni = 0;
  for (const g of geos) { nv += g.getAttribute("position").count; ni += g.getIndex().count; }
  const pos = new Float32Array(nv * 3), uv = new Float32Array(nv * 2), idx = new Uint32Array(ni);
  let v = 0, k = 0;
  for (const g of geos) {
    pos.set(g.getAttribute("position").array, v * 3);
    uv.set(g.getAttribute("uv").array, v * 2);
    const gi = g.getIndex().array;
    for (let j = 0; j < gi.length; j++) idx[k + j] = gi[j] + v;
    v += g.getAttribute("position").count;
    k += gi.length;
    g.dispose();
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  out.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
  out.setIndex(new THREE.BufferAttribute(idx, 1));
  out.computeVertexNormals();
  return out;
}

// ---------------------------------------------------------------- the view
export class View3D {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(this.renderer.domElement);
    this.renderer.domElement.style.display = "block";

    this.scene = new THREE.Scene();
    this.scene.background = skyTexture();
    this.scene.environment = environmentTexture(this.renderer);
    this.scene.fog = new THREE.Fog(0xc9dcec, 120, 700);
    this.camera = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 3000);

    this.scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x4a5a3a, 1.1));
    this.sun = new THREE.DirectionalLight(0xfff2dd, 2.4);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    Object.assign(this.sun.shadow.camera, { left: -30, right: 30, top: 30, bottom: -30, near: 1, far: 200 });
    this.sun.shadow.bias = -0.0005;
    this.scene.add(this.sun, this.sun.target);

    this.ego = carModel(0x9b0c18);         // Soul Red Crystal-ish
    this.lead = carModel(0x8d96a3);
    this.lead.visible = false;
    this.scene.add(this.ego, this.lead);
    this.world = new THREE.Group();
    this.scene.add(this.world);

    this.samples = [];       // [{t: wall ms, pose, tel}]
    this.mode = "chase";
    this.orbit = { yaw: 0, pitch: 0, zoom: 1 };
    this.camYaw = null;
    this.wheelSpin = 0;
    this.lastFrame = performance.now();
    this._bindMouse();
    this._resize = () => this.resize();
    window.addEventListener("resize", this._resize);
    this.resize();
    this.running = false;
  }

  _bindMouse() {
    const el = this.renderer.domElement;
    let drag = null;
    el.addEventListener("pointerdown", e => { drag = { x: e.clientX, y: e.clientY, ...this.orbit }; el.setPointerCapture(e.pointerId); });
    el.addEventListener("pointermove", e => {
      if (!drag) return;
      this.orbit.yaw = drag.yaw - (e.clientX - drag.x) * 0.008;
      this.orbit.pitch = Math.max(-0.35, Math.min(1.2, drag.pitch + (e.clientY - drag.y) * 0.006));
    });
    el.addEventListener("pointerup", () => { drag = null; });
    el.addEventListener("wheel", e => { this.orbit.zoom = Math.max(0.35, Math.min(4, this.orbit.zoom * (e.deltaY > 0 ? 1.1 : 1 / 1.1))); e.preventDefault(); }, { passive: false });
    el.addEventListener("dblclick", () => { this.orbit = { yaw: 0, pitch: 0, zoom: 1 }; });
    el.title = "drag to look around the car · wheel to zoom · double-click to reset";
  }

  setMode(mode) { if (MODES[mode]) { this.mode = mode; this.orbit = { yaw: 0, pitch: 0, zoom: 1 }; } }

  resize() {
    const r = this.container.getBoundingClientRect();
    const w = Math.max(10, r.width), h = Math.max(10, r.height);
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = "100%";
    this.renderer.domElement.style.height = "100%";
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  setRoad(road) {
    for (const c of [...this.world.children]) {
      this.world.remove(c);
      c.traverse(o => { if (o.geometry) o.geometry.dispose(); });
    }
    const path = this.path = new RoadPath(road);
    const w = road.laneWidth, lanes = road.lanes, width = w * lanes;
    this.laneWidth = w;
    const L = path.length, step = 1.5;
    const end = path.closed ? L : L;

    // ground: one big plane under the whole track
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (let i = 0; i < road.x.length; i++) {
      minX = Math.min(minX, road.x[i]); maxX = Math.max(maxX, road.x[i]);
      minY = Math.min(minY, road.y[i]); maxY = Math.max(maxY, road.y[i]);
    }
    const pad = 1500, gw = maxX - minX + 2 * pad, gh = maxY - minY + 2 * pad;
    const grassTex = noiseTexture([78, 112, 52], 40);
    grassTex.repeat.set(gw / 6, gh / 6);
    const ground = new THREE.Mesh(new THREE.PlaneGeometry(gw, gh),
      new THREE.MeshStandardMaterial({ map: grassTex, roughness: 1.0 }));
    ground.rotation.x = -Math.PI / 2;
    ground.position.set((minX + maxX) / 2, -0.02, -(minY + maxY) / 2);
    ground.receiveShadow = true;
    this.world.add(ground);

    // shoulders (gravel), asphalt, then the paint on top
    const asphalt = noiseTexture([62, 64, 68], 26, 256, true);
    const gravel = noiseTexture([120, 112, 98], 50);
    const shoulders = new THREE.Mesh(mergeRibbons([ribbon(path, 1.6, 0, 0, end, step, 4, 0.0), ribbon(path, -width, -width - 2.2, 0, end, step, 4, 0.0)]),
      new THREE.MeshStandardMaterial({ map: gravel, roughness: 1.0 }));
    shoulders.receiveShadow = true;
    const surface = new THREE.Mesh(ribbon(path, 0.25, -width - 0.25, 0, end, step, 6, 0.01),
      new THREE.MeshStandardMaterial({ map: asphalt, roughness: 0.92 }));
    surface.receiveShadow = true;
    this.world.add(shoulders, surface);

    const paint = (color) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, polygonOffset: true, polygonOffsetFactor: -2 });
    const lines = [ribbon(path, 0.0, -0.15, 0, end, step, 1, 0.02)];                 // left edge
    const white = [ribbon(path, -width + 0.15, -width, 0, end, step, 1, 0.02)];        // right edge
    for (let k = 1; k < lanes; k++) {
      for (let s = 0; s < end - DASH_ON; s += DASH_PERIOD) {
        white.push(ribbon(path, -k * w + 0.075, -k * w - 0.075, s, s + DASH_ON, 0.75, 1, 0.02));
      }
    }
    const yellowMesh = new THREE.Mesh(mergeRibbons(lines), paint(0xe2b33b));
    const whiteMesh = new THREE.Mesh(mergeRibbons(white), paint(0xeeeeee));
    yellowMesh.receiveShadow = whiteMesh.receiveShadow = true;
    this.world.add(yellowMesh, whiteMesh);

    // roadside: reflector posts every 50 m on the right, trees further out (deterministic, never on the road)
    const near = (x, y, clear) => {
      for (let i = 0; i < road.x.length; i += 2) {
        if (Math.abs(road.x[i] - x) < clear + 60 && Math.abs(road.y[i] - y) < clear + 60 &&
            Math.hypot(road.x[i] - x, road.y[i] - y) < clear) return true;
      }
      return false;
    };
    const postGeo = new THREE.BoxGeometry(0.12, 1.0, 0.12);
    const postMat = new THREE.MeshStandardMaterial({ color: 0xf2f2f2, roughness: 0.5 });
    const posts = new THREE.InstancedMesh(postGeo, postMat, Math.ceil(end / 50) + 2);
    const m = new THREE.Matrix4();
    let np = 0;
    for (let s = 0; s < end; s += 50) {
      const [x, y] = path.at(s, -width - 2.6);
      m.makeTranslation(x, 0.5, -y);
      posts.setMatrixAt(np++, m);
    }
    posts.count = np;
    posts.castShadow = true;
    this.world.add(posts);

    const trunks = [], crowns = [];
    let seed = 99;
    const rnd = () => (seed = (seed * 48271) % 2147483647) / 2147483647;
    for (let s = 0; s < end; s += 18) {
      for (const side of [1, -1]) {
        if (rnd() < 0.35) continue;
        const off = side > 0 ? 7 + rnd() * 30 : -width - 7 - rnd() * 30;
        const [x, y] = path.at(s + rnd() * 10, off);
        if (near(x, y, 9)) continue;
        const scale = 0.7 + rnd() * 0.8;
        trunks.push([x, y, scale]);
        crowns.push([x, y, scale, rnd()]);
      }
    }
    const trunkMesh = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.18, 0.26, 2.4, 6),
      new THREE.MeshStandardMaterial({ color: 0x5b4127, roughness: 1 }), Math.max(1, trunks.length));
    const crownMesh = new THREE.InstancedMesh(new THREE.ConeGeometry(2.0, 5.5, 8),
      new THREE.MeshStandardMaterial({ color: 0x2f5e2a, roughness: 0.95 }), Math.max(1, crowns.length));
    const q = new THREE.Quaternion(), sv = new THREE.Vector3(), pv = new THREE.Vector3();
    trunks.forEach(([x, y, k], i) => { trunkMesh.setMatrixAt(i, m.compose(pv.set(x, 1.2 * k, -y), q.identity(), sv.set(k, k, k))); });
    crowns.forEach(([x, y, k, c], i) => {
      crownMesh.setMatrixAt(i, m.compose(pv.set(x, (2.4 + 2.75) * k, -y), q.identity(), sv.set(k, k, k)));
      crownMesh.setColorAt(i, new THREE.Color().setHSL(0.28 + c * 0.06, 0.45, 0.22 + c * 0.08));
    });
    trunkMesh.count = trunks.length; crownMesh.count = crowns.length;
    trunkMesh.castShadow = crownMesh.castShadow = true;
    this.world.add(trunkMesh, crownMesh);
  }

  // one telemetry message (20 Hz)
  push(tel) {
    if (!tel || !tel.pose) return;
    const now = performance.now();
    this.samples.push({ t: now, tel });
    while (this.samples.length > 8) this.samples.shift();
  }

  _pose(now) {
    // render ~1.5 telemetry periods behind real time and interpolate, so motion is smooth at any frame rate
    const t = now - 75, S = this.samples;
    if (!S.length) return null;
    if (S.length === 1 || t <= S[0].t) return { ...S[0].tel.pose, tel: S[0].tel };
    for (let i = S.length - 1; i > 0; i--) {
      const a = S[i - 1], b = S[i];
      if (t >= a.t) {
        const f = Math.min(1.5, (t - a.t) / Math.max(1, b.t - a.t));
        const pa = a.tel.pose, pb = b.tel.pose;
        let dy = pb.yaw - pa.yaw;
        dy = Math.atan2(Math.sin(dy), Math.cos(dy));
        return { x: pa.x + (pb.x - pa.x) * f, y: pa.y + (pb.y - pa.y) * f, yaw: pa.yaw + dy * f, s: pa.s + (pb.s - pa.s) * f, tel: b.tel };
      }
    }
    return { ...S[S.length - 1].tel.pose, tel: S[S.length - 1].tel };
  }

  start() {
    if (this.running) return;
    this.running = true;
    const loop = () => {
      if (!this.running) return;
      requestAnimationFrame(loop);
      if (!this.container.offsetParent) return;   // hidden tab
      this.frame();
    };
    requestAnimationFrame(loop);
  }

  stop() { this.running = false; }

  frame() {
    const now = performance.now();
    const dt = Math.min(0.1, (now - this.lastFrame) / 1000);
    this.lastFrame = now;
    const r = this.container.getBoundingClientRect();
    if (Math.abs(r.width - this.renderer.domElement.clientWidth) > 1 || Math.abs(r.height - this.renderer.domElement.clientHeight) > 1) this.resize();
    const p = this._pose(now);
    if (p) this._place(p, dt, now);
    this.renderer.render(this.scene, this.camera);
  }

  _place(p, dt, now) {
    const tel = p.tel;
    const ego = this.ego, u = ego.userData;
    ego.position.set(p.x, 0, -p.y);
    ego.rotation.y = p.yaw;
    const roadWheel = THREE.MathUtils.degToRad((tel.steer ? tel.steer.angleDeg : 0) / CAR.steerRatio);
    u.steer.forEach(k => (k.rotation.y = roadWheel));
    this.wheelSpin -= (tel.speedKph / 3.6) * dt / CAR.wheelR;
    u.wheels.forEach(w => (w.rotation.z = this.wheelSpin));
    const braking = (tel.pedals && tel.pedals.brake > 0.02) || (tel.pcm && tel.pcm.brakeDecel > 0.3) || tel.ax < -1.2;
    u.tail.forEach(t => (t.material.emissiveIntensity = braking ? 3.0 : 0.4));
    const blink = tel.body ? tel.body.blinker : "";
    const on = Math.floor(now / 380) % 2 === 0;
    u.blinkL.forEach(b => (b.material.emissiveIntensity = on && (blink === "left" || blink === "hazard") ? 3.5 : 0));
    u.blinkR.forEach(b => (b.material.emissiveIntensity = on && (blink === "right" || blink === "hazard") ? 3.5 : 0));

    // lead car, on our lane at the reported gap
    if (tel.lead && this.path) {
      const lane = tel.track ? tel.track.lane : 0;
      const [lx, ly, lh] = this.path.at(p.s + tel.lead.gap + CAR.length, -(lane + 0.5) * this.laneWidth);
      this.lead.visible = true;
      this.lead.position.set(lx, 0, -ly);
      this.lead.rotation.y = lh;
      this.lead.userData.tail.forEach(t => (t.material.emissiveIntensity = 0.4));
    } else {
      this.lead.visible = false;
    }

    // camera: behind the car, swinging after it with a little lag; the mouse orbits around it
    const m = MODES[this.mode];
    if (this.camYaw === null) this.camYaw = p.yaw;
    let d = p.yaw - this.camYaw;
    d = Math.atan2(Math.sin(d), Math.cos(d));
    this.camYaw += d * Math.min(1, dt * 4.0);
    const yaw = this.camYaw + this.orbit.yaw;
    const back = m.back * this.orbit.zoom, up = m.up * this.orbit.zoom + this.orbit.pitch * m.back * this.orbit.zoom;
    const cx = p.x - Math.cos(yaw) * back, cy = p.y - Math.sin(yaw) * back;
    this.camera.position.set(cx, Math.max(0.6, up), -cy);
    const aim = m.ahead * Math.max(0, Math.cos(this.orbit.yaw));   // orbiting round to the side or front: look at the car
    const ax = p.x + Math.cos(this.camYaw) * aim, ay = p.y + Math.sin(this.camYaw) * aim;
    if (this.mode === "overhead") {
      this.camera.up.set(Math.cos(this.camYaw), 0, -Math.sin(this.camYaw));
    } else {
      this.camera.up.set(0, 1, 0);
    }
    this.camera.lookAt(ax, m.lookUp, -ay);
    if (this.camera.fov !== m.fov) { this.camera.fov = m.fov; this.camera.updateProjectionMatrix(); }

    // keep the sun's shadow box on the car
    this.sun.position.set(p.x + 40, 80, -p.y + 25);
    this.sun.target.position.set(p.x, 0, -p.y);
  }
}
