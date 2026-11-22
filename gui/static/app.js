// Draws what the server read from the logs; computes nothing the metrics depend on.
"use strict";
const $ = (id) => document.getElementById(id);
const LIVENESS_COLOR = {alive: "#3fb950", suspect: "#d29922", lost: "#f85149", dead: "#8b949e"};
const FIELD = {ground: "#f7f7f7", grid: "#d0d0d0", label: "#6e7681", border: "#f2c200", zone: "#e00000"};
const droneColor = (id) => geo.colors[(id - 1) % geo.colors.length];   // gui/model.py DRONE_COLORS
const [T, X, Y, Z, YAW, STATUS, EPOCH, SLOT, NBR_AGE] = [0, 1, 2, 3, 4, 5, 6, 7, 8];
const MAP_MARGIN_PX = 28;

let geo = null;                       // footprint and area numbers from swarm/sensor.py, config.py
let liveRun = emptyRun(), replayRun = emptyRun(), liveDir = null;
let mode = "live", follow = true, playing = true, speed = 1, t = 0, lastFrameMs = 0;
let heat = null, heatBusy = false, logCount = -1;
const query = new URLSearchParams(location.search);   // ?run=DIR&t=MISSION_S&paused=1

function emptyRun() { return {agents: {}, events: [], series: [], t0: null, end_t: 0}; }
function run() { return mode === "live" ? liveRun : replayRun; }

function merge(target, delta) {
  for (const [id, rows] of Object.entries(delta.agents)) (target.agents[id] ||= []).push(...rows);
  target.events.push(...delta.events);
  target.series.push(...delta.series);
  for (const k of ["t0", "kill_t", "victim", "outcome", "mission", "seed", "radio", "end_t", "zone", "loss"])
    if (delta[k] != null) target[k] = delta[k];
}

// ---------------------------------------------------------------- lookups at time t
function rowAt(rows, tNow) {          // last row at or before tNow (binary search)
  let lo = 0, hi = rows.length - 1;
  if (hi < 0 || rows[0][T] > tNow) return null;
  while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (rows[mid][T] <= tNow) lo = mid; else hi = mid - 1; }
  return rows[lo];
}
function eventsUntil(r, tNow) { return r.events.filter((e) => e.t <= tNow); }
function killed(r, id) { return r.victim == +id && r.kill_t != null && t >= r.kill_t; }   // the truth
function liveness(r, id, past) {
  if (killed(r, id)) return "dead";
  return groundSays(id, past);
}
function groundSays(id, past) {       // what the ground believes, from its liveness log
  let state = "alive";
  for (const e of past) if (e.event === "liveness" && e.agent == id) state = e.to;
  return state;
}
function assignFor(past, epoch) {     // the plan the drone flies: its own epoch, else the newest
  const assigns = past.filter((e) => e.event === "assign");
  return assigns.find((e) => e.epoch === epoch) || assigns[assigns.length - 1];
}
function shortDir(dir) { return dir.split("/").slice(-3).join("/"); }
function missionTime(r, tNow) { return r.t0 == null ? null : tNow - r.t0; }
function fmtT(r, tNow) { const m = missionTime(r, tNow); return m == null ? "warm-up" : (m >= 0 ? "+" : "") + m.toFixed(1) + " s"; }

// ---------------------------------------------------------------- map
const map = $("map"), mctx = map.getContext("2d");
function scalePx() { return (map.width - 2 * MAP_MARGIN_PX) / (geo.area[1] - geo.area[0]); }
function px(x, y) { const s = scalePx(); return [MAP_MARGIN_PX + (x - geo.area[0]) * s, map.height - MAP_MARGIN_PX - (y - geo.area[2]) * s]; }

function drawMap(r, past) {               // as the Gazebo top view: white field, yellow border
  mctx.fillStyle = FIELD.ground; mctx.fillRect(0, 0, map.width, map.height);
  drawHeat();
  drawGrid();
  drawRect(geo.area[0], geo.area[2], geo.area[1], geo.area[3], FIELD.border, 2.5);
  const zone = r.zone;                    // [x0, y0, x1, y1], from the run's meta.json
  if (zone) drawRect(zone[0], zone[1], zone[2], zone[3], FIELD.zone, 2);
  const rows = Object.entries(r.agents).map(([id, rs]) => [id, rowAt(rs, t)]).filter(([, row]) => row);
  const alive = rows.filter(([id]) => liveness(r, id, past) !== "dead");   // a dead drone has no camera
  const dead = rows.filter((r) => !alive.includes(r));                     // old loops first, under the live ones
  for (const [id, row] of dead) drawPlan(id, row, assignFor(past, row[EPOCH]), true);
  for (const [id, row] of alive) drawPlan(id, row, assignFor(past, row[EPOCH]), false);
  for (const [id, row] of alive) drawFootprint(id, row);
  for (const [id, row] of rows) drawDrone(id, row, liveness(r, id, past));
}

function drawRect(x0, y0, x1, y1, color, width) {
  const [ax, ay] = px(x0, y1), [bx, by] = px(x1, y0);
  mctx.strokeStyle = color; mctx.lineWidth = width; mctx.setLineDash([]);
  mctx.strokeRect(ax, ay, bx - ax, by - ay);
}

function drawHeat() {
  if (!heat) return;
  const img = new ImageData(heat.cols, heat.rows);
  heat.age_s.forEach((age, i) => {    // row 0 of the grid is the south edge: flip for the canvas
    const k = ((heat.rows - 1 - Math.floor(i / heat.cols)) * heat.cols + (i % heat.cols)) * 4;
    img.data.set([...heatRgb(age, geo.window_s), 255], k);
  });
  const off = new OffscreenCanvas(heat.cols, heat.rows);
  off.getContext("2d").putImageData(img, 0, 0);
  const [x0, y1] = px(geo.area[0], geo.area[3]), [x1, y0] = px(geo.area[1], geo.area[2]);
  mctx.imageSmoothingEnabled = false;
  mctx.drawImage(off, x0, y1, x1 - x0, y0 - y1);
}

function drawGrid() {
  mctx.strokeStyle = FIELD.grid; mctx.fillStyle = FIELD.label; mctx.font = "11px sans-serif"; mctx.lineWidth = 1;
  for (let m = 0; m <= geo.area[1]; m += 50) {
    const [gx, gy0] = px(m, geo.area[2]), [, gy1] = px(m, geo.area[3]);
    const [hx0, hy] = px(geo.area[0], m), [hx1] = px(geo.area[1], m);
    mctx.globalAlpha = 0.6;
    mctx.beginPath(); mctx.moveTo(gx, gy0); mctx.lineTo(gx, gy1); mctx.moveTo(hx0, hy); mctx.lineTo(hx1, hy); mctx.stroke();
    mctx.globalAlpha = 1;
    mctx.fillText(m, gx - 8, gy0 + 14); mctx.fillText(m, 2, hy + 4);
  }
}

function drawPlan(id, row, assign, dead) {   // in the drone's colour; a dead drone's old loop grey
  const plan = assign && assign.assignments[id];
  if (!plan || !plan.slots.length) return;
  const pts = plan.slots.map(([x, y]) => px(x, y)), from = plan.loop_from || 0;
  mctx.strokeStyle = dead ? "#9a9a9a" : droneColor(id); mctx.lineWidth = 1.5; mctx.globalAlpha = dead ? 0.6 : 0.85;
  mctx.setLineDash([]); mctx.beginPath();                  // the loop it repeats
  pts.slice(from).forEach(([x, y], i) => (i ? mctx.lineTo(x, y) : mctx.moveTo(x, y)));
  mctx.closePath(); mctx.stroke();
  mctx.setLineDash([4, 4]); mctx.beginPath();              // the approach, flown once
  pts.slice(0, from + 1).forEach(([x, y], i) => (i ? mctx.lineTo(x, y) : mctx.moveTo(x, y)));
  mctx.stroke();
  const slot = !dead && pts[row[SLOT]];
  if (slot) {                                               // the slot it is flying to now
    mctx.beginPath(); mctx.moveTo(...px(row[X], row[Y])); mctx.lineTo(...slot); mctx.stroke();
    mctx.setLineDash([]); mctx.globalAlpha = 1;
    mctx.beginPath(); mctx.arc(...slot, 5, 0, 7); mctx.stroke();
  }
  mctx.setLineDash([]); mctx.globalAlpha = 1;
}

function drawFootprint(id, row) {    // same rectangle as sensor.footprint_polygon, an outline
  const corners = footprintCorners(row[X], row[Y], row[Z], row[YAW], geo.tan_half_across, geo.tan_half_along);
  mctx.strokeStyle = droneColor(id); mctx.lineWidth = 1.5; mctx.setLineDash([]); mctx.beginPath();
  corners.forEach(([x, y], i) => (i ? mctx.lineTo(...px(x, y)) : mctx.moveTo(...px(x, y))));
  mctx.closePath(); mctx.stroke();
}

function drawDrone(id, row, state) {
  const [cx, cy] = px(row[X], row[Y]), ringPx = geo.d_min_m * scalePx();
  mctx.strokeStyle = "#9a9a9a"; mctx.setLineDash([2, 3]); mctx.lineWidth = 1;
  mctx.beginPath(); mctx.arc(cx, cy, ringPx, 0, 7); mctx.stroke(); mctx.setLineDash([]);
  if (state === "suspect" || state === "lost") {          // what the ground says, around the drone
    mctx.strokeStyle = LIVENESS_COLOR[state]; mctx.lineWidth = 3;
    mctx.beginPath(); mctx.arc(cx, cy, 15, 0, 7); mctx.stroke();
  }
  const tip = arrowPoints(cx, cy, row[YAW], 12);          // nose along the logged yaw
  mctx.fillStyle = state === "dead" ? "#9a9a9a" : droneColor(id); mctx.strokeStyle = "#fff"; mctx.lineWidth = 1.5;
  mctx.beginPath(); tip.forEach(([x, y], i) => (i ? mctx.lineTo(x, y) : mctx.moveTo(x, y))); mctx.closePath();
  mctx.fill(); mctx.stroke();
  mctx.fillStyle = droneColor(id); mctx.font = "bold 13px sans-serif";
  mctx.fillText(id, cx + 10, cy - 8);
  if (state === "dead") { mctx.strokeStyle = "#f85149"; mctx.lineWidth = 2; mctx.beginPath();
    mctx.moveTo(cx - 8, cy - 8); mctx.lineTo(cx + 8, cy + 8); mctx.moveTo(cx + 8, cy - 8); mctx.lineTo(cx - 8, cy + 8); mctx.stroke(); }
}

// ---------------------------------------------------------------- side panel
const chart = $("chart"), cctx = chart.getContext("2d");
function drawChart(r) {
  cctx.fillStyle = "#0d1117"; cctx.fillRect(0, 0, chart.width, chart.height);
  if (r.t0 == null || !r.series.length) return;
  const tFrom = r.t0 - geo.window_s, tTo = Math.max(r.end_t, r.t0 + 180), pad = 24;
  const cx = (tt) => pad + (tt - tFrom) / (tTo - tFrom) * (chart.width - pad - 4);
  const cy = (pct) => chart.height - 14 - pct / 100 * (chart.height - 20);
  cctx.strokeStyle = "#30363d"; cctx.fillStyle = "#8b949e"; cctx.font = "10px sans-serif";
  for (const p of [0, 50, 100]) { cctx.beginPath(); cctx.moveTo(pad, cy(p)); cctx.lineTo(chart.width, cy(p)); cctx.stroke(); cctx.fillText(p, 2, cy(p) + 3); }
  for (let m = 0; r.t0 + m < tTo - 10; m += 60) cctx.fillText(m + " s", cx(r.t0 + m) - 8, chart.height - 2);
  cctx.strokeStyle = "#58a6ff"; cctx.lineWidth = 1.5; cctx.beginPath();
  r.series.filter((s) => s[0] >= tFrom).forEach((s, i) => (i ? cctx.lineTo(cx(s[0]), cy(s[1])) : cctx.moveTo(cx(s[0]), cy(s[1]))));
  cctx.stroke();
  const mark = (tt, color, label, yPx) => { cctx.strokeStyle = cctx.fillStyle = color; cctx.beginPath();
    cctx.moveTo(cx(tt), 4); cctx.lineTo(cx(tt), chart.height - 14); cctx.stroke(); cctx.fillText(label, cx(tt) + 3, yPx); };
  if (r.kill_t != null) mark(r.kill_t, "#f85149", "kill " + r.victim, 12);
  const rec = lossLevels(r), level = (pct, dash) => { cctx.strokeStyle = "#8b949e"; cctx.setLineDash(dash);
    cctx.beginPath(); cctx.moveTo(cx(r.kill_t - geo.window_s), cy(pct)); cctx.lineTo(chart.width, cy(pct)); cctx.stroke(); cctx.setLineDash([]); };
  if (rec) { level(rec.before, [5, 3]); level(rec.share, [1, 3]); }
  if (rec && rec.back != null) {                     // back within 10 % of the pre-loss coverage
    const [bx, by] = [cx(rec.back), cy(0.9 * rec.before)];
    cctx.fillStyle = "#3fb950"; cctx.beginPath(); cctx.arc(bx, by, 3.5, 0, 7); cctx.fill();
    cctx.fillText(`back +${Math.round(rec.back - r.kill_t)} s`, bx + 5, by + 12);
  }
  r.events.filter((e) => e.event === "liveness" && e.to === "lost").forEach((e) => mark(e.t, "#d29922", "lost " + e.agent, 24));
  mark(t, "#e6edf3", "", 0);
}

function lossLevels(r) {             // analysis/metrics.py's figures if the run was analysed
  if (r.kill_t == null) return null;
  const m = r.loss;
  if (m) return {before: m.before_pct, share: m.share_pct, back: m.recovery_s == null ? null : r.kill_t + m.recovery_s};
  return recovery(r.series, r.kill_t, Object.keys(r.agents).length, geo.window_s);
}

function drawSide(r, past) {
  $("clock").textContent = fmtT(r, t);
  $("pct").textContent = heat ? heat.pct.toFixed(1) + " %" : "";
  const a = assignFor(past);
  $("epoch").innerHTML = a ? `&nbsp; epoch <b>${a.epoch}</b> trigger <b>${a.trigger}</b> changed [${a.changed}]` : "";
  let html = "<tr><th>id</th><th>ground says</th><th>status</th><th>slot</th><th>epoch</th><th>nbr_age</th></tr>";
  for (const [id, rows] of Object.entries(r.agents)) {
    const row = rowAt(rows, t), state = groundSays(id, past);
    if (!row) continue;
    html += `<tr><td><b style="color:#fff;background:${droneColor(id)};padding:0 5px;border-radius:3px">${id}</b></td><td style="color:${LIVENESS_COLOR[state]}">${state}</td>` +
      `<td>${killed(r, id) ? "dead" : row[STATUS]}</td><td>${row[SLOT] ?? ""}</td><td>${row[EPOCH] ?? ""}</td><td>${row[NBR_AGE] == null ? "" : row[NBR_AGE].toFixed(2) + " s"}</td></tr>`;
  }
  $("drones").innerHTML = html;
  const zone = $("zone"), age = heat && heat.zone_age_s;   // the stalest 5 m cell in the zone
  zone.hidden = !r.zone;
  zone.textContent = age == null ? "zone not seen yet" : `zone last seen ${Math.round(age)} s ago`;
  zone.style.color = age == null || age > geo.zone_limit_s ? LIVENESS_COLOR.lost : "";
  const s = rowAt(r.series, t), d = s && s[2];
  $("mind").innerHTML = `min pairwise distance since t = 0: <b>${d == null ? "-" : d.toFixed(1) + " m"}</b> (limit ${geo.d_min_m} m)`;
  const g = r.radio;
  $("radio").textContent = g ? `radio measured: ground links ${g.ground.loss_pct ?? "-"} % lost, ${g.ground.delay_ms ?? "-"} ms; ` +
    `agent links ${g.agents.loss_pct ?? "-"} % lost, ${g.agents.delay_ms ?? "-"} ms` : "radio stats: written at the end of the run";
  drawLog(r, past);
}

function describe(e) {
  if (e.event === "assign") return `assign epoch ${e.epoch} (${e.trigger}) changed [${e.changed}]`;
  if (e.event === "liveness") return `agent ${e.agent} ${e.from} -> ${e.to} (${e.status})`;
  if (e.event === "infeasible") return `infeasible agent ${e.agent} (${e.slot_xy}) ${e.accepted ? "accepted" : "ignored"}: ${e.why}`;
  if (e.event === "blacklist") return e.pairs.length ? `blacklist ${e.pairs.map((p) => "agent " + p.agent + " at " + p.slot_key).join(", ")}` : "blacklist cleared";
  if (e.event === "kill") return `KILL agent ${e.agent}`;
  if (e.event === "start") return `start, t = 0 in 60 s${e.late && e.late.length ? ", late " + e.late : ""}`;
  return e.event + (e.agent != null ? " agent " + e.agent : "");
}
function drawLog(r, past) {
  const items = r.kill_t != null && t >= r.kill_t ? [...past, {event: "kill", agent: r.victim, t: r.kill_t}] : past;
  if (items.length === logCount) return;   // rebuild only when an entry appears or goes
  logCount = items.length;
  items.sort((p, q) => p.t - q.t);
  $("log").textContent = items.map((e) => `${fmtT(r, e.t).padStart(9)}  ${describe(e)}`).join("\n");
  $("log").scrollTop = 1e9;
}

// ---------------------------------------------------------------- time, heat, data
function tRange(r) {
  const starts = Object.values(r.agents).filter((rs) => rs.length).map((rs) => rs[0][T]);
  return [starts.length ? Math.min(...starts) : 0, r.end_t || 0];
}
function wantHeat(dir) {             // the heatmap comes from Python (gui/model.py)
  if (heatBusy || !dir || (heat && Math.abs(heat.t - t) < 0.25)) return;
  heatBusy = true;
  fetch(`/api/heat?dir=${encodeURIComponent(dir)}&t=${t.toFixed(2)}`).then((r) => r.json())
    .then((h) => { if (!h.error) heat = h; }).finally(() => { heatBusy = false; });
}
function frame(nowMs) {
  const r = run(), [tMin, tMax] = tRange(r), dt = Math.min(0.2, (nowMs - lastFrameMs) / 1000);
  lastFrameMs = nowMs;
  if (mode === "live" && follow) t = tMax;
  else if (playing) t = Math.min(tMax, t + dt * speed);
  t = Math.max(tMin, t);
  const slider = $("slider");
  slider.min = tMin; slider.max = tMax; if (document.activeElement !== slider) slider.value = t;
  if (geo) {
    if (!(mode === "live" && follow)) wantHeat(mode === "live" ? liveDir : $("runs").value);
    const past = eventsUntil(r, t);
    drawMap(r, past); drawChart(r); drawSide(r, past);
  }
  requestAnimationFrame(frame);
}

function openReplay(dir, missionS) {
  mode = "replay"; heat = null; logCount = -1; $("runs").value = dir;
  $("live").classList.remove("on"); $("source").textContent = "replay: " + shortDir(dir);
  fetch(`/api/run?dir=${encodeURIComponent(dir)}`).then((r) => r.json()).then((d) => {
    replayRun = emptyRun(); merge(replayRun, d);
    t = missionS != null && d.t0 != null ? Math.min(d.end_t, d.t0 + missionS) : (d.t0 != null ? d.t0 - geo.window_s : 0);
  });
}
function openLive() {
  mode = "live"; follow = true; heat = null; logCount = -1;
  $("live").classList.add("on"); $("runs").value = ""; $("source").textContent = "live: " + (liveDir ? shortDir(liveDir) : "no mission running");
}

function onLive(msg) {
  const st = msg.status;
  $("runstate").textContent = st.dir ? `${st.running ? "running" : "ended (exit " + st.exit + ")"}: ${shortDir(st.dir)}` : "idle";
  $("tail").textContent = st.tail;
  if (st.live_dir !== liveDir) { liveDir = st.live_dir; liveRun = emptyRun(); if (mode === "live") openLive(); }
  if (msg.run) merge(liveRun, msg.run);
  if (msg.heat && mode === "live" && follow) heat = msg.heat;
}

function post(path, body) {
  return fetch(path, {method: "POST", body: JSON.stringify(body || {})}).then((r) => r.json())
    .then((d) => { if (d.error) alert(d.error); return d; });
}
function setup(info) {
  geo = info.geometry;
  $("mission").innerHTML = info.missions.map((m) => `<option>${m}</option>`).join("");
  $("backend").innerHTML = info.backends.map((b) => `<option>${b}</option>`).join("");
  $("runs").innerHTML += info.runs.map((d) => `<option value="${d}">${shortDir(d)}</option>`).join("");
  $("start").onclick = () => post("/api/start", {mission: $("mission").value, seed: +$("seed").value, n: +$("n").value, backend: $("backend").value})
    .then((d) => { if (d.dir) openLive(); });
  $("stop").onclick = () => post("/api/stop");
  $("live").onclick = openLive;
  $("runs").onchange = () => $("runs").value ? openReplay($("runs").value) : openLive();
  $("play").onclick = () => { playing = !playing; $("play").textContent = playing ? "pause" : "play"; };
  $("slider").oninput = () => { t = +$("slider").value; follow = false; };
  document.querySelectorAll("[data-speed]").forEach((b) => (b.onclick = () => {
    speed = +b.dataset.speed; document.querySelectorAll("[data-speed]").forEach((o) => o.classList.toggle("on", o === b)); }));
  new EventSource("/api/live").onmessage = (m) => onLive(JSON.parse(m.data));
  const q = query;
  if (q.get("paused")) $("play").click();
  if (q.get("run")) openReplay(q.get("run"), q.get("t") == null ? null : +q.get("t"));
  else if (info.live || !info.runs.length) openLive();    // cold: the run being written now ...
  else openReplay(info.runs[0], 1e9);                     // ... else the newest one, at its end
  requestAnimationFrame(frame);
}
fetch("/api/runs").then((r) => r.json()).then(setup);
