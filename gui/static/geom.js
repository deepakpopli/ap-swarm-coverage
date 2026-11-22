// Pure drawing maths for app.js; tests/test_gui.py runs it under node.
"use strict";

// Yaw is a compass angle (rad): 0 = north, clockwise. The canvas has y down, so north is -y.
function headingVector(yaw) { return [Math.sin(yaw), -Math.cos(yaw)]; }

// The drone marker: an arrow with its tip `size` px ahead of (cx, cy) along the yaw.
function arrowPoints(cx, cy, yaw, size) {
  const [fx, fy] = headingVector(yaw), [rx, ry] = [-fy, fx];   // ahead, and to the right
  const at = (ahead, right) => [cx + ahead * fx + right * rx, cy + ahead * fy + right * ry];
  return [at(size, 0), at(-0.7 * size, 0.65 * size), at(-0.35 * size, 0), at(-0.7 * size, -0.65 * size)];
}

// Ground corners (east, north) of the camera footprint, as sensor.footprint_polygon.
function footprintCorners(x, y, z, yaw, tanAcross, tanAlong) {
  const across = 2 * z * tanAcross, along = 2 * z * tanAlong;
  const fx = Math.sin(yaw), fy = Math.cos(yaw), rx = Math.cos(yaw), ry = -Math.sin(yaw);
  return [[1, -1], [-1, -1], [-1, 1], [1, 1]].map(([f, c]) =>
    [x + f * along / 2 * fx + c * across / 2 * rx, y + f * along / 2 * fy + c * across / 2 * ry]);
}

// Cell colour on the white field: never seen white, then light blue (just seen) to grey (old).
function heatRgb(ageS, windowS) {
  if (ageS < 0) return [255, 255, 255];
  const f = 1 - Math.min(ageS, windowS) / windowS;   // 1 = just seen, 0 = window or older
  return [228 - f * 78, 228 - f * 30, 228 + f * 22].map(Math.round);
}

// Coverage before a loss (mean over windowS before killT), its (n-1)/n share, and the first time
// from which coverage stays within 10 % of it, held 10 s (analysis/metrics.py); series [[t, %], ...].
function recovery(series, killT, n, windowS) {
  const pre = series.filter(([t]) => t >= killT - windowS && t < killT).map((s) => s[1]);
  if (!pre.length) return null;
  const before = pre.reduce((a, b) => a + b) / pre.length, after = series.filter(([t]) => t >= killT);
  let back = 0;
  after.forEach(([, pct], i) => { if (pct < 0.9 * before) back = i + 1; });
  const held = back < after.length && after[after.length - 1][0] - after[back][0] >= 10;
  return {before, share: (n - 1) / n * before, back: held ? after[back][0] : null};
}

if (typeof module !== "undefined") module.exports = {headingVector, arrowPoints, footprintCorners, heatRgb, recovery};
