// Mazda sim dashboard: talks to the car over its websocket (:8770) and, when the launcher serves this page, to the
// launcher's API for setup, start/stop and logs.
"use strict";

const HOST = location.hostname || "localhost";
const CAR_WS = `ws://${HOST}:8770/ws`;
const CAR_HTTP = `http://${HOST}:8770`;
const SCREEN_URL = `http://${HOST}:6080/vnc.html?autoconnect=1&resize=scale&reconnect=1&reconnect_delay=2000`;
const GALAXY_URL = `http://${HOST}:8082/`;

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];
const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v)) ? "--" : (typeof v === "number" ? v.toFixed(d) : String(v));

let launcher = null;          // /api/state when the launcher serves this page
let ws = null;
let tel = null;               // latest telemetry
let road = null;              // road shape {x, y, lanes, laneWidth, closed, s}
let carCfg = null;            // the car's config (from the road message)
let screenWanted = true;      // show the comma's screen (not in native runs: it is a desktop window there)

// ------------------------------------------------------------------ tabs + view layout
$$(".tab").forEach(b => b.addEventListener("click", () => {
  $$(".tab").forEach(x => x.classList.toggle("active", x === b));
  $$(".tab-body").forEach(x => x.classList.toggle("active", x.id === "tab-" + b.dataset.tab));
  if (b.dataset.tab === "logs") scrollLog();
}));
$$("input[name=view]").forEach(r => r.addEventListener("change", () => {
  $("#view-area").className = "view-area " + r.value;
  try { localStorage.setItem("mazdasim.view", r.value); } catch (e) { /* private mode */ }
  resizeRoad();
}));
try {
  const v = localStorage.getItem("mazdasim.view");
  if (v) { const r = $(`input[name=view][value=${v}]`); if (r) { r.checked = true; $("#view-area").className = "view-area " + v; } }
} catch (e) { /* ignore */ }
$("#link-screen").href = SCREEN_URL;
$("#link-galaxy").href = GALAXY_URL;

// ------------------------------------------------------------------ car websocket
function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function connect() {
  ws = new WebSocket(CAR_WS);
  ws.onmessage = ev => {
    const m = JSON.parse(ev.data);
    if (m.type === "tel") { tel = m; onTelemetry(); }
    else if (m.type === "road") { setRoad(m.road); carCfg = m.config; onCarConfig(); }
    else if (m.type === "error") console.warn("car:", m.error);
  };
  ws.onclose = () => { tel = null; setChip("#chip-car", "car: offline", "bad"); setTimeout(connect, 1500); };
  ws.onopen = () => setChip("#chip-car", "car", "ok");
}

function setChip(sel, text, cls) {
  const el = $(sel);
  el.textContent = text;
  el.className = "chip" + (cls ? " " + cls : "");
}

// ------------------------------------------------------------------ inputs: wheel, pedals
const input = { steer: 0, gas: 0, brake: 0 };       // what the page sends (steer: -1..1, positive left)
const keys = new Set();
let kbSteer = 0;
let lastAxes = "";

function sliderVal(id) { return Number($(id).value) / 100; }

function springBack(el, latchId) {
  const back = () => { if (!latchId || !$(latchId).checked) el.value = 0; };
  el.addEventListener("pointerup", back);
  el.addEventListener("pointercancel", back);
  el.addEventListener("blur", back);
}
springBack($("#in-steer"), "#in-steer-latch");
springBack($("#in-gas"));
springBack($("#in-brake"));

const KEYMAP = { ArrowLeft: "left", KeyA: "left", ArrowRight: "right", KeyD: "right", ArrowUp: "gas", KeyW: "gas",
                 ArrowDown: "brake", KeyS: "brake" };
const KEY_BUTTONS = { Digit1: "set_minus", Digit2: "set_plus", Digit3: "resume", Digit4: "cancel", Digit5: "distance", Digit6: "mode" };

document.addEventListener("keydown", e => {
  if (e.target.matches("input[type=text], input[type=number], textarea, select")) return;
  if (!$("#tab-drive").classList.contains("active")) return;
  if (KEYMAP[e.code]) { keys.add(KEYMAP[e.code]); e.preventDefault(); }
  else if (KEY_BUTTONS[e.code] && !e.repeat) pressButton(KEY_BUTTONS[e.code]);
  else if (e.code === "KeyQ" && !e.repeat) toggleBlinker("left");
  else if (e.code === "KeyE" && !e.repeat) toggleBlinker("right");
});
document.addEventListener("keyup", e => { if (KEYMAP[e.code]) keys.delete(KEYMAP[e.code]); });
window.addEventListener("blur", () => keys.clear());

let padButtons = [];
function readGamepad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (const p of pads) {
    if (!p) continue;
    const ax = Math.abs(p.axes[0]) > 0.06 ? p.axes[0] : 0;
    const gas = p.buttons[7] ? p.buttons[7].value : 0, brake = p.buttons[6] ? p.buttons[6].value : 0;
    const map = { 0: "set_minus", 1: "cancel", 2: "resume", 3: "distance" };
    p.buttons.forEach((b, i) => {
      if (b.pressed && !padButtons[i]) {
        if (map[i]) pressButton(map[i]);
        if (i === 4) toggleBlinker("left");
        if (i === 5) toggleBlinker("right");
      }
      padButtons[i] = b.pressed;
    });
    return { steer: -ax, gas, brake };
  }
  return null;
}

function inputLoop() {
  const dt = 1 / 30;
  // keyboard steering ramps to full torque in ~0.6 s, and back to zero quickly
  const want = (keys.has("left") ? 1 : 0) - (keys.has("right") ? 1 : 0);
  kbSteer = want ? Math.max(-1, Math.min(1, kbSteer + want * dt / 0.6)) : (Math.abs(kbSteer) < 0.1 ? 0 : kbSteer * 0.6);
  const pad = readGamepad();
  // slider: right is a right turn (negative torque)
  input.steer = clamp(-sliderVal("#in-steer") + kbSteer + (pad ? pad.steer : 0), -1, 1);
  input.gas = clamp(sliderVal("#in-gas") + (keys.has("gas") ? 0.6 : 0) + (pad ? pad.gas : 0), 0, 1);
  input.brake = clamp(sliderVal("#in-brake") + (keys.has("brake") ? 0.6 : 0) + (pad ? pad.brake : 0), 0, 1);
  const s = `${input.steer.toFixed(2)}|${input.gas.toFixed(2)}|${input.brake.toFixed(2)}`;
  if (s !== lastAxes || (input.steer || input.gas || input.brake)) {
    send({ type: "axes", steer: input.steer, gas: input.gas, brake: input.brake });
    lastAxes = s;
  }
}
setInterval(inputLoop, 1000 / 30);
function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }

// ------------------------------------------------------------------ buttons, body, driver, lead, faults
function pressButton(name) {
  const btn = $(`[data-btn=${name}]`);
  if ($("#in-hold").checked) {
    const on = !btn.classList.contains("on");
    btn.classList.toggle("on", on);
    send({ type: "button", name, hold: on });
  } else {
    send({ type: "button", name, duration: 0.3 });
    btn.classList.add("on");
    setTimeout(() => btn.classList.remove("on"), 250);
  }
}
$$("[data-btn]").forEach(b => b.addEventListener("click", () => pressButton(b.dataset.btn)));

function toggleBlinker(side) {
  const cur = tel ? tel.body.blinker : "";
  send({ type: "set", key: "blinker", value: cur === side ? "" : side });
}
$$("[data-blink]").forEach(b => b.addEventListener("click", () => toggleBlinker(b.dataset.blink)));
$$("[data-gear]").forEach(b => b.addEventListener("click", () => send({ type: "set", key: "gear", value: b.dataset.gear })));
$$("[data-set]").forEach(c => c.addEventListener("change", () => send({ type: "set", key: c.dataset.set, value: c.checked })));
$$("[data-fault]").forEach(c => c.addEventListener("change", () => send({ type: "fault", name: c.dataset.fault, value: c.checked })));
$$("[data-fault-once]").forEach(b => b.addEventListener("click", () => send({ type: "fault", name: b.dataset.faultOnce, value: true })));
$("#in-driver-mode").addEventListener("change", e => send({ type: "set", key: "driver_mode", value: e.target.value }));
$("#in-auto-speed").addEventListener("change", e => send({ type: "set", key: "auto_speed_kph", value: Number(e.target.value) }));
$("#in-lane").addEventListener("change", e => send({ type: "set", key: "lane", value: Number(e.target.value) }));
$("#btn-reset-car").addEventListener("click", () => send({ type: "reset" }));
$("#btn-lead").addEventListener("click", () => send({ type: "lead", mode: $("#in-lead-mode").value,
  speed_kph: Number($("#in-lead-speed").value), gap_m: Number($("#in-lead-gap").value) }));

function onCarConfig() {
  const w = carCfg.world;
  $("#hud-unit").textContent = carCfg.car.imperial ? "mph" : "km/h";
  $("#in-lead-mode").value = w.lead;
  $("#in-lead-speed").value = w.lead_speed_kph;
  $("#in-lead-gap").value = w.lead_gap_m;
  const lane = $("#in-lane");
  lane.innerHTML = "";
  for (let i = 0; i < w.lanes; i++) {
    const o = document.createElement("option");
    o.value = i; o.textContent = i === 0 ? `${i + 1} (left)` : i === w.lanes - 1 ? `${i + 1} (right)` : `${i + 1}`;
    lane.appendChild(o);
  }
  const rendered = w.world === "metadrive";
  $("#chase").hidden = !rendered;
  $("#road").hidden = rendered;
  if (rendered && !$("#chase").src) $("#chase").src = `${CAR_HTTP}/chase.mjpg`;
}

// ------------------------------------------------------------------ telemetry
function dl(id, rows) {
  const el = $(id);
  if (el.childElementCount !== rows.length * 2) {
    el.innerHTML = rows.map(() => "<dt></dt><dd></dd>").join("");
  }
  const dts = el.querySelectorAll("dt"), dds = el.querySelectorAll("dd");
  rows.forEach(([k, v, cls], i) => {
    if (dts[i].textContent !== k) dts[i].textContent = k;
    const t = v === undefined || v === null ? "--" : String(v);
    if (dds[i].textContent !== t) dds[i].textContent = t;
    dds[i].className = cls || "";
  });
}
const yes = (b, good = true) => [b ? "yes" : "no", b === good ? "ok" : (b ? "warn" : "")];
function flag(k, b, good = true) { const [t, c] = yes(b, good); return [k, t, c]; }

function syncControl(el, value) {
  if (document.activeElement !== el && el.checked !== undefined && el.checked !== value) el.checked = value;
}

function onTelemetry() {
  const t = tel, op = t.op, p = t.panda || {};
  const imperial = carCfg ? carCfg.car.imperial : true;
  const toUnit = kph => imperial ? kph / 1.609 : kph;
  $("#hud-speed").textContent = fmt(toUnit(t.speedKph), 0);
  $("#hud-set").textContent = t.pcm.setSpeedKph ? fmt(toUnit(t.pcm.setSpeedKph), 0) : "--";
  const alert = op && (op.alert1 || op.alert2) ? `${op.alert1} ${op.alert2}`.trim() : "";
  $("#hud-alert").textContent = alert;
  $("#hud-alert").className = "alert " + (op ? op.alertStatus : "");
  $("#wheel").style.transform = `rotate(${-t.steer.angleDeg}deg)`;
  $("#t-sw").textContent = fmt(t.steer.angleDeg, 1);
  $("#t-drvnm").textContent = fmt(t.steer.driverNm, 2);
  $("#t-tinm").textContent = fmt(t.steer.tiNm, 2);
  $("#t-lkasnm").textContent = fmt(t.eps.lkasNm, 2);

  const commaOn = t.comma && t.comma.connected;
  setChip("#chip-comma", commaOn ? "comma" : "comma: unplugged", commaOn ? "ok" : "warn");
  if (!op) setChip("#chip-op", "openpilot: --", "");
  else if (op.enabled) setChip("#chip-op", op.experimental ? "engaged · experimental" : "engaged", "engaged");
  else if (op.latActive) setChip("#chip-op", "lateral only", "ok");
  else setChip("#chip-op", op.started ? "openpilot: ready" : "openpilot: offroad", op.started ? "ok" : "warn");

  dl("#tel-op", op ? [
    ["state", op.state], flag("enabled", op.enabled), flag("lat active", op.latActive), flag("long active", op.longActive),
    ["torque cmd", fmt(op.torque, 3)], ["torque CAN", fmt(op.torqueOutCan, 0)], ["accel cmd", fmt(op.accel, 2)],
    ["curvature want / is", `${fmt(op.desiredCurvature, 4)} / ${fmt(op.curvature, 4)}`],
    flag("CAN valid", op.canValid), flag("steering pressed", op.steeringPressed, false),
    ["cruise", op.cruiseEnabled ? `on ${fmt(op.cruiseSpeed * 3.6, 0)} km/h` : (op.cruiseAvailable ? "available" : "off")],
  ] : [["openpilot", "not connected"]]);
  $("#tel-events").innerHTML = op ? op.events.map(e => `<span>${e}</span>`).join("") : "";

  dl("#tel-car", [
    ["speed", `${fmt(t.speedKph, 1)} km/h`], ["lane offset", `${fmt(t.pose.laneOffset, 2)} m`],
    ["lat / long accel", `${fmt(t.ay, 2)} / ${fmt(t.ax, 2)}`], ["steer rate", `${fmt(t.steer.rateDeg, 0)} °/s`],
    ["gear", t.body.gear], ["gas / brake", `${fmt(t.pedals.gas, 2)} / ${fmt(t.pedals.brake, 2)}`],
    ["blinker", t.body.blinker || "off"], ["driver", t.driver.mode + (t.driver.takeover ? " (took over)" : "")],
    ["track", `${t.track.name} · ${fmt(t.pose.s / 1000, 2)} / ${fmt(t.track.length / 1000, 1)} km`],
    ["lead", t.lead ? `${fmt(t.lead.gap, 0)} m · ${fmt(t.lead.v, 0)} km/h` : "none"],
  ]);
  dl("#tel-ti", [
    ["state", t.ti.state, t.ti.state === "RUN" ? "ok" : (t.ti.state === "OFF" ? "" : "warn")], ["command", t.ti.cmd],
    ["injected", `${fmt(t.ti.injectNm, 2)} Nm`], ["driver units", fmt(t.ti.driverUnits, 0)],
    ["violations", t.ti.viol, t.ti.viol ? "bad" : ""], ["error", t.ti.error, t.ti.error ? "bad" : ""],
    ["version", t.ti.version], flag("unplugged", t.ti.unplugged, false),
  ]);
  dl("#tel-eps", [
    ["LKAS request", t.eps.lkasReq ?? "--"], flag("LKAS active", t.eps.lkasActive), flag("speed ok", t.eps.speedOk),
    flag("locked out", t.eps.lockedOut, false), ["hands-off", `${fmt(t.eps.handsOffT, 1)} s`],
    flag("fault", t.eps.fault, false), ["bad frames", t.eps.badFrames, t.eps.badFrames ? "warn" : ""],
    ["sensor / assist", `${fmt(t.eps.sensorNm, 2)} / ${fmt(t.eps.assistNm, 2)}`], ["motor", `${fmt(t.eps.motorNm, 2)} Nm`],
  ]);
  dl("#tel-pcm", [
    flag("main on", t.pcm.mainOn), flag("engaged", t.pcm.engaged), flag("set allowed", t.pcm.setAllowed),
    ["set speed", `${fmt(t.pcm.setSpeedKph, 0)} km/h`], ["accel cmd / out", `${fmt(t.pcm.aCmd, 2)} / ${fmt(t.pcm.aOut, 2)}`],
    flag("standstill hold", t.pcm.hold, false), ["last cancel", t.pcm.lastCancel || "--"],
    ["radar", `${t.radar.state} · ${t.radar.session}`, t.radar.state === "silent" ? "warn" : ""],
    ["radar lead", t.radar.lead ? `${fmt(t.radar.lead.dist, 0)} m` : "none"], ["distance bars", t.radar.distanceBars],
    flag("SCBS malfunction", t.fsc.scbsMalfunction, false),
  ]);
  const blocked = p.blockedByAddr ? Object.entries(p.blockedByAddr).map(([k, v]) => `${k}×${v}`).join(" ") : "";
  dl("#tel-panda", [
    ["safety", `${p.safetyModel ?? "--"} / ${p.safetyParam ?? "--"}`], flag("controls allowed", p.controlsAllowed),
    flag("relay intercept", p.relayIntercept), flag("bus 1 → OBD", p.obdMode),
    flag("relay malfunction", p.relayMalfunction, false), ["tx sent", p.txSent],
    ["tx blocked", p.txBlocked, p.txBlocked ? "warn" : ""], ["blocked", blocked || "--"],
    ["rx invalid", p.rxInvalid, p.rxInvalid ? "warn" : ""], ["link", `${fmt(t.comma.loopMs, 1)} ms loop`],
  ]);

  // reflect the car's state in the controls (without fighting the user's own clicks)
  $$("[data-gear]").forEach(b => b.classList.toggle("on", b.dataset.gear === t.body.gear));
  $$("[data-blink]").forEach(b => b.classList.toggle("on", b.dataset.blink === t.body.blinker));
  const setMap = { ignition: t.body.ignition, seatbelt: t.body.seatbelt, door_open: t.body.doorOpen,
                   high_beams: t.body.highBeams, driver_distracted: t.driverDistracted, paused: t.paused };
  $$("[data-set]").forEach(c => { if (c.dataset.set in setMap) syncControl(c, setMap[c.dataset.set]); });
  if (document.activeElement !== $("#in-driver-mode")) $("#in-driver-mode").value = t.driver.mode;
  if (document.activeElement !== $("#in-auto-speed")) $("#in-auto-speed").value = t.driver.autoSpeedKph;
  if (document.activeElement !== $("#in-lane")) $("#in-lane").value = t.track.lane;
  const faultMap = { ti_unplugged: t.ti.unplugged, eps_lockout: t.eps.lockedOut, relay_stuck: t.relayStuck,
                     radar_refuse_programming: t.radar.refuseProgramming, radar_restart_in_standby: t.radar.restartInStandby };
  $$("[data-fault]").forEach(c => { if (c.dataset.fault in faultMap) syncControl(c, faultMap[c.dataset.fault]); });

  chartPush(t);
}

// ------------------------------------------------------------------ top-down road (lite world)
const canvas = $("#road");
const ctx = canvas.getContext("2d");
let zoom = 9;   // px per m
canvas.addEventListener("wheel", e => { zoom = clamp(zoom * (e.deltaY < 0 ? 1.15 : 1 / 1.15), 0.5, 40); e.preventDefault(); }, { passive: false });

function setRoad(r) {
  const n = r.x.length, s = new Float64Array(n);
  for (let i = 1; i < n; i++) s[i] = s[i - 1] + Math.hypot(r.x[i] - r.x[i - 1], r.y[i] - r.y[i - 1]);
  road = { ...r, s };
}

function resizeRoad() {
  const r = canvas.getBoundingClientRect();
  canvas.width = Math.max(10, Math.round(r.width * devicePixelRatio));
  canvas.height = Math.max(10, Math.round(r.height * devicePixelRatio));
}
window.addEventListener("resize", resizeRoad);

function roadPoint(s, offset) {
  // point at arc length s along the reference (left edge), shifted left by offset
  const S = road.s, n = S.length;
  if (road.closed) s = ((s % S[n - 1]) + S[n - 1]) % S[n - 1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (S[m] <= s) lo = m; else hi = m; }
  const f = S[hi] > S[lo] ? (s - S[lo]) / (S[hi] - S[lo]) : 0;
  const x = road.x[lo] + f * (road.x[hi] - road.x[lo]), y = road.y[lo] + f * (road.y[hi] - road.y[lo]);
  const h = Math.atan2(road.y[hi] - road.y[lo], road.x[hi] - road.x[lo]);
  return [x - Math.sin(h) * offset, y + Math.cos(h) * offset, h];
}

function drawRoad() {
  requestAnimationFrame(drawRoad);
  if (canvas.hidden || !canvas.offsetParent) return;
  if (canvas.width !== Math.round(canvas.getBoundingClientRect().width * devicePixelRatio)) resizeRoad();
  const W = canvas.width, H = canvas.height, k = zoom * devicePixelRatio;
  ctx.fillStyle = "#2f4a2a";
  ctx.fillRect(0, 0, W, H);
  if (!road || !tel) return;
  const pose = tel.pose;
  // car-centred, heading up, car a third of the way up from the bottom
  ctx.save();
  ctx.translate(W / 2, H * 0.68);
  ctx.scale(k, -k);
  ctx.rotate(Math.PI / 2 - pose.yaw);
  ctx.translate(-pose.x, -pose.y);
  const reach = Math.hypot(W, H) / k;
  const s0 = pose.s - reach * 0.6, s1 = pose.s + reach;
  const w = road.laneWidth, lanes = road.lanes;
  const strip = (off, from, to, step) => { const pts = []; for (let s = from; s <= to; s += step) pts.push(roadPoint(s, off)); return pts; };
  const step = Math.max(0.5, 2 / zoom);
  const left = strip(0, s0, s1, step), right = strip(-lanes * w, s0, s1, step);
  ctx.beginPath();
  left.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
  for (let i = right.length - 1; i >= 0; i--) ctx.lineTo(right[i][0], right[i][1]);
  ctx.closePath();
  ctx.fillStyle = "#3c4148";
  ctx.fill();
  const line = (pts, color, width, dash) => {
    ctx.beginPath();
    pts.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []); ctx.stroke();
  };
  line(left, "#e8c341", 0.15);
  line(right, "#f2f2f2", 0.15);
  for (let i = 1; i < lanes; i++) line(strip(-i * w, s0, s1, step), "#f2f2f2", 0.15, [3, 9]);
  ctx.setLineDash([]);
  if (tel.lead) {
    const [lx, ly, lh] = roadPoint(pose.s + tel.lead.gap + 4.9, -(tel.track.lane + 0.5) * w);
    car(lx, ly, lh, "#8d99a8");
  }
  car(pose.x, pose.y, pose.yaw, "#c8102e");
  ctx.restore();
}
function car(x, y, h, color) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(h);
  ctx.fillStyle = color;
  ctx.fillRect(-2.55, -0.98, 5.1, 1.96);
  ctx.fillStyle = "#0008";
  ctx.fillRect(0.6, -0.85, 1.1, 1.7);   // windscreen
  ctx.restore();
}
requestAnimationFrame(drawRoad);

// ------------------------------------------------------------------ strip chart
const chart = $("#chart"), cctx = chart.getContext("2d");
const hist = [];
function chartPush(t) {
  const op = t.op;
  hist.push([t.t, t.pose.laneOffset, t.steer.driverNm, op ? op.torque : 0, t.speedKph, t.pcm.setSpeedKph]);
  while (hist.length && hist[hist.length - 1][0] - hist[0][0] > 30) hist.shift();
}
function drawChart() {
  requestAnimationFrame(drawChart);
  if (!chart.offsetParent) return;
  const W = chart.width = Math.round(chart.getBoundingClientRect().width * devicePixelRatio);
  const H = chart.height = Math.round(140 * devicePixelRatio);
  cctx.fillStyle = "#0b0e12"; cctx.fillRect(0, 0, W, H);
  if (hist.length < 2) return;
  const t0 = hist[hist.length - 1][0] - 30, mid = H / 2, sy = H / 8;   // ±4 units full height
  cctx.strokeStyle = "#2a313b"; cctx.beginPath(); cctx.moveTo(0, mid); cctx.lineTo(W, mid); cctx.stroke();
  const series = [[1, "#4aa3ff"], [2, "#f2b134"], [3, "#37c871"], [4, "#d17cff"], [5, "#d17cff80"]];
  for (const [i, color] of series) {
    cctx.beginPath();
    hist.forEach((r, j) => {
      // speeds: 0..160 km/h across the full height; the rest: ±4 around the middle
      const v = i >= 4 ? r[i] / 20 - 4 : r[i];
      const x = (r[0] - t0) / 30 * W, y = clamp(mid - v * sy, 0, H);
      j ? cctx.lineTo(x, y) : cctx.moveTo(x, y);
    });
    cctx.strokeStyle = color; cctx.lineWidth = devicePixelRatio * 1.3; cctx.stroke();
  }
}
requestAnimationFrame(drawChart);

// ------------------------------------------------------------------ launcher: state, setup, actions, logs
async function api(path, body) {
  const opts = body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json", "X-Mazda-Sim": "1" },
                                           body: JSON.stringify(body) };
  const r = await fetch(path, opts);
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}

function get(obj, path) { return path.split(".").reduce((o, k) => (o ? o[k] : undefined), obj); }
function put(obj, path, v) { const ks = path.split("."); const last = ks.pop(); ks.reduce((o, k) => (o[k] = o[k] || {}), obj)[last] = v; }

function fillForm(cfg, settings) {
  const all = { ...cfg, settings };
  $$("#setup [name]").forEach(el => {
    const v = get(all, el.name);
    if (el.type === "checkbox") el.checked = !!v;
    else if (el.type === "radio") el.checked = el.value === v;
    else if (v !== undefined) el.value = v;
  });
  $("#json").value = JSON.stringify(cfg, null, 2);
  showSource();
}
function showSource() {
  const local = ($('input[name="build.source"]:checked') || {}).value === "local";
  $$(".src-git").forEach(e => e.hidden = local);
  $$(".src-local").forEach(e => e.hidden = !local);
}
$$('input[name="build.source"]').forEach(r => r.addEventListener("change", showSource));

function readForm() {
  let cfg;
  try { cfg = JSON.parse($("#json").value); } catch (e) { throw new Error("the JSON box isn't valid JSON: " + e.message); }
  const settings = {};
  $$("#setup [name]").forEach(el => {
    if (el.type === "radio" && !el.checked) return;
    let v = el.type === "checkbox" ? el.checked : el.value;
    if (el.type === "number") v = Number(v);
    if (el.name.startsWith("settings.")) settings[el.name.slice(9)] = v;
    else put(cfg, el.name, v);
  });
  return { cfg, settings };
}
// edits in the form flow into the JSON box, so either can be used
$$("#setup [name]").forEach(el => el.addEventListener("change", () => {
  try { const { cfg } = readForm(); $("#json").value = JSON.stringify(cfg, null, 2); } catch (e) { /* JSON box being edited */ }
}));
$("#json").addEventListener("change", () => { try { fillForm(JSON.parse($("#json").value), launcher.settings); } catch (e) { /* keep typing */ } });

$("#setup").addEventListener("submit", async e => {
  e.preventDefault();
  try {
    const { cfg, settings } = readForm();
    await api("/api/config", cfg);
    await api("/api/settings", settings);
    $("#save-msg").textContent = "saved";
    await refreshState(true);
  } catch (err) { $("#save-msg").textContent = err.message; }
  setTimeout(() => $("#save-msg").textContent = "", 4000);
});

async function action(path, body = {}) {
  try { await api(path, body); } catch (e) { alert(e.message); }
  refreshState();
}
$("#btn-start").addEventListener("click", () => action("/api/start"));
$("#btn-stop").addEventListener("click", () => action("/api/stop"));
$("#btn-reload").addEventListener("click", () => action("/api/reload"));
$("#btn-restart-car").addEventListener("click", () => action("/api/restart-car"));
$("#btn-wipe").addEventListener("click", () => {
  if (confirm("Stop the sim and delete the comma's build cache (source copy, venv, x86 build, compiled model, device state)?")) action("/api/stop", { wipe: true });
});

async function refreshState(refill = false) {
  try {
    const first = launcher === null;
    launcher = await api("/api/state");
    document.body.classList.remove("no-launcher");
    if (first || refill) fillForm(launcher.config, launcher.settings);
    const st = launcher.status, svc = st.services || {};
    const running = Object.values(svc).some(s => s.state === "running");
    $("#busy").textContent = st.busy ? st.busy + "…" : "";
    $("#btn-start").disabled = !!st.busy;
    $("#btn-stop").disabled = !!st.busy || !running;
    $("#btn-reload").disabled = !!st.busy || !running;
    $("#btn-restart-car").disabled = !!st.busy || !running;
    const comma = svc.comma;
    const native = launcher.settings.backend === "native";
    $("#screen-note").textContent = native ? "native run: the comma's screen is a window on your desktop" :
      (!comma ? "not running: press Start" : (tel && tel.op ? "" : "the comma is getting its build ready (see Logs); its screen appears when openpilot starts"));
    screenWanted = !native;
  } catch (e) {
    if (launcher === null) {
      document.body.classList.add("no-launcher");
      $("#readonly-note").hidden = false;
      if (carCfg) fillForm(carCfg, {});
    }
  }
}

// the comma's screen (noVNC) comes up with its manager, after the build is ready: load it then, drop it when it goes
function updateScreen() {
  const iframe = $("#screen"), up = screenWanted && tel && tel.comma && tel.comma.connected;
  if (up && !iframe.getAttribute("src")) iframe.setAttribute("src", SCREEN_URL);
  else if (!up && iframe.getAttribute("src")) iframe.removeAttribute("src");
}
setInterval(updateScreen, 1500);
setInterval(refreshState, 2000);
refreshState();

// logs
let logSeq = 0;
const logEl = $("#log");
async function pollLogs() {
  if (document.body.classList.contains("no-launcher")) return;
  try {
    const r = await api(`/api/logs?since=${logSeq}`);
    if (r.seq < logSeq) logSeq = 0;
    const filter = $("#log-filter").value;
    const frag = document.createDocumentFragment();
    for (const [seq, src, line] of r.lines) {
      logSeq = Math.max(logSeq, seq);
      const div = document.createElement("div");
      div.dataset.src = src;
      if (filter && src !== filter) div.hidden = true;
      div.innerHTML = `<span class="src src-${src}">${src.padEnd(8)}</span> `;
      div.appendChild(document.createTextNode(line));
      frag.appendChild(div);
    }
    logEl.appendChild(frag);
    while (logEl.childElementCount > 5000) logEl.firstElementChild.remove();
    scrollLog();
  } catch (e) { /* launcher restarting */ }
}
function scrollLog() { if ($("#log-follow").checked) logEl.scrollTop = logEl.scrollHeight; }
$("#log-filter").addEventListener("change", () => {
  const f = $("#log-filter").value;
  $$("#log > div").forEach(d => d.hidden = !!f && d.dataset.src !== f);
  scrollLog();
});
$("#log-clear").addEventListener("click", () => { logEl.innerHTML = ""; });
setInterval(pollLogs, 1000);

connect();
resizeRoad();
