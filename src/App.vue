<template>
  <div class="container">
    <header>
      <h1>GeoJSON 障碍图构建</h1>
      <div class="stats">
        <div>节点: {{ stats.nodes }}</div>
        <div>边: {{ stats.edges }}</div>
        <div>构建耗时: {{ stats.buildMs }} ms</div>
      </div>
    </header>

    <section class="controls">
      <div class="row">
        <button :disabled="loading" @click="build">构建障碍图</button>
      </div>
      <div class="row">
        <label>起点经度</label>
        <input v-model.number="startLon" type="number" step="0.000001" />
        <label>起点纬度</label>
        <input v-model.number="startLat" type="number" step="0.000001" />
        <button @click="pickStart">在图上选择起点</button>
      </div>
      <div class="row">
        <label>终点经度</label>
        <input v-model.number="endLon" type="number" step="0.000001" />
        <label>终点纬度</label>
        <input v-model.number="endLat" type="number" step="0.000001" />
        <button @click="pickEnd">在图上选择终点</button>
      </div>
      <div class="row">
        <span class="pick-state">
          当前选择：{{
            picking ? (picking === "start" ? "起点" : "终点") : "无"
          }}
        </span>
      </div>
    </section>

    <section class="canvas-wrap">
      <canvas
        ref="canvasRef"
        width="1000"
        height="700"
        @click="onCanvasClick"
        :style="{ cursor: picking ? 'crosshair' : 'default' }"
      ></canvas>
    </section>
  </div>
</template>

<script setup>
import { ref, reactive } from "vue";
import { buildObstacleGraph } from "./graph.js";
import { bboxFromNodes, fitToCanvas } from "./geo.js";
import { segmentIntersectsAnyObstacle, pointInPolygon, segmentIntersectsPolygon } from "./obstacles.js";

const canvasRef = ref(null);
const ctxRef = ref(null);
const graph = ref(null);
const graphReady = ref(false);
const loading = ref(false);

const stats = reactive({ nodes: 0, edges: 0, buildMs: 0 });
const startLon = ref(0);
const startLat = ref(0);
const endLon = ref(0);
const endLat = ref(0);
const picking = ref(null); // 'start' | 'end' | null
const view = reactive({ scale: 1, tx: 0, ty: 0 });
const pathPoints = ref([]);
const gridCache = ref(null);
const wallSegments = ref([]);
const geojsonRef = ref(null);
const obstacleMetaCache = ref(null);
const wallMetaCache = ref(null);
const workerRef = ref(null);
const busy = ref(false);
function safeClone(data) {
  try {
    // eslint-disable-next-line no-undef
    return structuredClone(data);
  } catch (e) {
    return JSON.parse(JSON.stringify(data));
  }
}
function toPlainObstacles(obstacles) {
  const arr = Array.isArray(obstacles) ? obstacles : [];
  return arr.map(rings => rings.map(r => r.map(([lon, lat]) => [Number(lon), Number(lat)])));
}
function toPlainWalls(walls) {
  const arr = Array.isArray(walls) ? walls : [];
  return arr.map(seg => [[Number(seg[0][0]), Number(seg[0][1])], [Number(seg[1][0]), Number(seg[1][1])]]);
}

async function loadGeoJSON() {
  const url = new URL("./data/lines.geojson", import.meta.url).href;
  const res = await fetch(url);
  if (!res.ok) throw new Error("加载 GeoJSON 失败");
  return await res.json();
}

async function build() {
  loading.value = true;
  try {
    const t0 = performance.now();
    const geojson = await loadGeoJSON();
    const g = await buildObstacleGraph(geojson, {
      precision: 6,
      includeObstacles: true,
      filterEdges: false,
    });
    graph.value = g;
    geojsonRef.value = geojson;
    wallSegments.value = [];
    gridCache.value = null;
    obstacleMetaCache.value = null;
    wallMetaCache.value = null;

    const t1 = performance.now();
    stats.nodes = g.nodes.length;
    stats.edges = g.adjacency.reduce((s, a) => s + a.length, 0) / 2;
    stats.buildMs = Math.round(t1 - t0);
    graphReady.value = true;
    setupCanvas();
    drawNetwork();
    // 默认进入起点选择，提升可用性
    picking.value = "start";
  } catch (e) {
    console.error(e);
    alert(e.message);
  } finally {
    loading.value = false;
  }
}

function setupCanvas() {
  const canvas = canvasRef.value;
  const ctx = canvas.getContext("2d");
  ctxRef.value = ctx;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!graph.value) return;
  const bbox = bboxFromNodes(graph.value.nodes);
  const fit = fitToCanvas(bbox, canvas.width, canvas.height, 30);
  view.scale = fit.scale;
  view.tx = fit.tx;
  view.ty = fit.ty;
}

function toScreen(lon, lat) {
  return {
    x: lon * view.scale + view.tx,
    y: lat * view.scale + view.ty,
  };
}

function drawNetwork() {
  const canvas = canvasRef.value;
  const ctx = ctxRef.value;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!graph.value) return;
  const MAX_DRAW_EDGES = 100000;
  const tooManyEdges = stats.edges > MAX_DRAW_EDGES;
  if (!tooManyEdges) {
    ctx.lineWidth = 1;
    ctx.strokeStyle = "#86a8c0";
    ctx.globalAlpha = 0.6;
    for (let i = 0; i < graph.value.nodes.length; i++) {
      for (const { to } of graph.value.adjacency[i]) {
        if (to < i) continue; // draw once
        const a = graph.value.nodes[i];
        const b = graph.value.nodes[to];
        const p = toScreen(a.lon, a.lat);
        const q = toScreen(b.lon, b.lat);
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(q.x, q.y);
        ctx.stroke();
      }
    }
  }
  ctx.globalAlpha = 1;
  // draw start/end if present
  drawPoint(startLon.value, startLat.value, "#2b9348");
  drawPoint(endLon.value, endLat.value, "#d00000");
  if (busy.value) {
    const ctx = ctxRef.value;
    ctx.fillStyle = "rgba(0,0,0,0.35)";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }
  if (pathPoints.value && pathPoints.value.length > 1) {
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#ff5400";
    ctx.beginPath();
    const p0 = toScreen(pathPoints.value[0].lon, pathPoints.value[0].lat);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < pathPoints.value.length; i++) {
      const pi = toScreen(pathPoints.value[i].lon, pathPoints.value[i].lat);
      ctx.lineTo(pi.x, pi.y);
    }
    ctx.stroke();
  }
}

function drawPoint(lon, lat, color) {
  if (!isFinite(lon) || !isFinite(lat)) return;
  const ctx = ctxRef.value;
  const p = toScreen(lon, lat);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
  ctx.fill();
}

function pickStart() {
  picking.value = "start";
}
function pickEnd() {
  picking.value = "end";
}

function onCanvasClick(ev) {
  if (!graphReady.value || busy.value) return;
  const rect = canvasRef.value.getBoundingClientRect();
  const x = ev.clientX - rect.left;
  const y = ev.clientY - rect.top;
  const lon = (x - view.tx) / view.scale;
  const lat = (y - view.ty) / view.scale;

  // 直接使用用户点击的坐标作为点

  // 若未显式进入选择模式，默认按"起点→终点"轮换
  if (!picking.value) {
    picking.value =
      !isFinite(startLon.value) || !isFinite(startLat.value) ? "start" : "end";
  }

  if (isInsideAnyObstacle(lon, lat)) {
    alert("该位置在障碍区域内，无法作为起终点");
    return;
  }

  if (picking.value === "start") {
    startLon.value = lon;
    startLat.value = lat;
    // 下一次指向终点，便于连续选取
    picking.value = "end";
  } else if (picking.value === "end") {
    endLon.value = lon;
    endLat.value = lat;
    // 选完终点退出选择
    picking.value = null;
  }
  console.log(`选择了点:`, { lon, lat });

  drawNetwork();

  if ((startLon.value !== 0 || startLat.value !== 0) && (endLon.value !== 0 || endLat.value !== 0)) {
    computeAndDrawPath();
  }
}

function isInsideAnyObstacle(lon, lat) {
  const obs = graph.value?.obstacles || [];
  for (const poly of obs) {
    if (pointInPolygon(lon, lat, poly)) return true;
  }
  if (isOnAnyWall(lon, lat)) return true;
  return false;
}

function euclid(a, b) {
  const dx = a.lon - b.lon;
  const dy = a.lat - b.lat;
  return Math.hypot(dx, dy);
}

function ensureGrid() {
  if (gridCache.value) return gridCache.value;
  const obstacles = graph.value?.obstacles || [];
  if (!wallSegments.value.length && geojsonRef.value) {
    wallSegments.value = buildWallSegments(geojsonRef.value);
  }
  const bboxNodes = bboxFromNodes(graph.value.nodes);
  let minLon = Math.min(startLon.value, endLon.value);
  let maxLon = Math.max(startLon.value, endLon.value);
  let minLat = Math.min(startLat.value, endLat.value);
  let maxLat = Math.max(startLat.value, endLat.value);
  if (!isFinite(minLon) || !isFinite(minLat) || !isFinite(maxLon) || !isFinite(maxLat)) {
    minLon = bboxNodes.minLon; maxLon = bboxNodes.maxLon; minLat = bboxNodes.minLat; maxLat = bboxNodes.maxLat;
  }
  let lonSpan = maxLon - minLon;
  let latSpan = maxLat - minLat;
  const padLon = lonSpan * 0.3 + 1e-4;
  const padLat = latSpan * 0.3 + 1e-4;
  minLon = Math.max(bboxNodes.minLon, minLon - padLon);
  maxLon = Math.min(bboxNodes.maxLon, maxLon + padLon);
  minLat = Math.max(bboxNodes.minLat, minLat - padLat);
  maxLat = Math.min(bboxNodes.maxLat, maxLat + padLat);
  lonSpan = Math.max(lonSpan, 1e-6);
  latSpan = Math.max(latSpan, 1e-6);
  const BASE_COLS = 60;
  const MIN_COLS = 40;
  const MAX_COLS = 160;
  const cols = Math.max(MIN_COLS, Math.min(BASE_COLS, MAX_COLS));
  let rows = Math.round((latSpan / lonSpan) * cols);
  const MIN_ROWS = 30;
  const MAX_ROWS = 120;
  rows = Math.max(MIN_ROWS, Math.min(rows, MAX_ROWS));
  const cellLon = lonSpan / cols;
  const cellLat = latSpan / rows;
  const nodes = [];
  const blocked = [];
  const eps = Math.min(cellLon, cellLat) * 0.3;
  if (!obstacleMetaCache.value) {
    obstacleMetaCache.value = (obstacles || []).map(rings => {
      let mLon = Infinity, mLat = Infinity, xLon = -Infinity, xLat = -Infinity;
      for (const ring of rings) {
        for (const [lo, la] of ring) {
          if (lo < mLon) mLon = lo;
          if (lo > xLon) xLon = lo;
          if (la < mLat) mLat = la;
          if (la > xLat) xLat = la;
        }
      }
      return { rings, bbox: { minLon: mLon, minLat: mLat, maxLon: xLon, maxLat: xLat } };
    });
  }
  if (!wallMetaCache.value) {
    wallMetaCache.value = (wallSegments.value || []).map(seg => {
      const a = seg[0], b = seg[1];
      return { a, b, bbox: { minLon: Math.min(a[0], b[0]), maxLon: Math.max(a[0], b[0]), minLat: Math.min(a[1], b[1]), maxLat: Math.max(a[1], b[1]) } };
    });
  }
  function bboxIntersects(a, b) {
    return !(a.minLon > b.maxLon || a.maxLon < b.minLon || a.minLat > b.maxLat || a.maxLat < b.minLat);
  }
  for (let r = 0; r <= rows; r++) {
    for (let c = 0; c <= cols; c++) {
      const lon = minLon + c * cellLon;
      const lat = minLat + r * cellLat;
      const idx = nodes.length;
      nodes.push({ id: idx, lon, lat });
      let inside = false;
      for (const om of obstacleMetaCache.value) {
        if (!bboxIntersects({ minLon: lon, maxLon: lon, minLat: lat, maxLat: lat }, om.bbox)) continue;
        if (pointInPolygon(lon, lat, om.rings)) { inside = true; break; }
      }
      if (!inside && isOnAnyWall(lon, lat, eps)) inside = true;
      blocked[idx] = inside ? 1 : 0;
    }
  }
  const adjacency = new Array(nodes.length);
  const dirs = [
    [0, 1], [1, 0], [0, -1], [-1, 0],
    [1, 1], [1, -1], [-1, 1], [-1, -1]
  ];
  for (let r = 0; r <= rows; r++) {
    for (let c = 0; c <= cols; c++) {
      const idx = r * (cols + 1) + c;
      const list = [];
      if (blocked[idx]) { adjacency[idx] = list; continue; }
      const a = nodes[idx];
      for (const [dr, dc] of dirs) {
        const nr = r + dr, nc = c + dc;
        if (nr < 0 || nr > rows || nc < 0 || nc > cols) continue;
        const j = nr * (cols + 1) + nc;
        if (blocked[j]) continue;
        const b = nodes[j];
        const sb = { minLon: Math.min(a.lon, b.lon), maxLon: Math.max(a.lon, b.lon), minLat: Math.min(a.lat, b.lat), maxLat: Math.max(a.lat, b.lat) };
        let crossesPoly = false;
        for (const om of obstacleMetaCache.value) {
          if (!bboxIntersects(sb, om.bbox)) continue;
          if (segmentIntersectsPolygon([a.lon, a.lat], [b.lon, b.lat], om.rings)) { crossesPoly = true; break; }
        }
        let crossesWall = false;
        if (!crossesPoly) {
          for (const wm of wallMetaCache.value) {
            if (!bboxIntersects(sb, wm.bbox)) continue;
            if (segmentsIntersect(a.lon, a.lat, b.lon, b.lat, wm.a[0], wm.a[1], wm.b[0], wm.b[1])) { crossesWall = true; break; }
          }
        }
        if (!crossesPoly && !crossesWall) list.push({ to: j, w: euclid(a, b) });
      }
      adjacency[idx] = list;
    }
  }
  gridCache.value = { nodes, adjacency, cols, rows, minLon, minLat, cellLon, cellLat };
  return gridCache.value;
}

function buildWallSegments(geojson) {
  const segs = [];
  const features = geojson?.features || [];
  for (const f of features) {
    const g = f.geometry;
    if (!g) continue;
    if (g.type === "LineString") {
      const coords = g.coordinates || [];
      for (let i = 0; i < coords.length - 1; i++) segs.push([coords[i], coords[i + 1]]);
    } else if (g.type === "MultiLineString") {
      for (const line of g.coordinates || []) {
        for (let i = 0; i < line.length - 1; i++) segs.push([line[i], line[i + 1]]);
      }
    }
  }
  return segs;
}

function orientation(ax, ay, bx, by, cx, cy) {
  const val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  if (Math.abs(val) < 1e-12) return 0;
  return val > 0 ? 1 : 2;
}
function onSegment(ax, ay, cx, cy, bx, by) {
  return Math.min(ax, bx) - 1e-12 <= cx && cx <= Math.max(ax, bx) + 1e-12 &&
         Math.min(ay, by) - 1e-12 <= cy && cy <= Math.max(ay, by) + 1e-12;
}
function segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
  const o1 = orientation(ax, ay, bx, by, cx, cy);
  const o2 = orientation(ax, ay, bx, by, dx, dy);
  const o3 = orientation(cx, cy, dx, dy, ax, ay);
  const o4 = orientation(cx, cy, dx, dy, bx, by);
  if (o1 !== o2 && o3 !== o4) return true;
  if (o1 === 0 && onSegment(ax, ay, cx, cy, bx, by)) return true;
  if (o2 === 0 && onSegment(ax, ay, dx, dy, bx, by)) return true;
  if (o3 === 0 && onSegment(cx, cy, ax, ay, dx, dy)) return true;
  if (o4 === 0 && onSegment(cx, cy, bx, by, dx, dy)) return true;
  return false;
}
function segmentIntersectsAnyWall(a, b) {
  const [ax, ay] = a, [bx, by] = b;
  for (const seg of wallSegments.value || []) {
    const c = seg[0], d = seg[1];
    if (segmentsIntersect(ax, ay, bx, by, c[0], c[1], d[0], d[1])) return true;
  }
  return false;
}
function pointSegmentDistance(px, py, ax, ay, bx, by) {
  const vx = bx - ax, vy = by - ay;
  const wx = px - ax, wy = py - ay;
  const c1 = vx * wx + vy * wy;
  const c2 = vx * vx + vy * vy || 1e-12;
  let t = c1 / c2;
  if (t < 0) t = 0; else if (t > 1) t = 1;
  const cx = ax + t * vx, cy = ay + t * vy;
  return Math.hypot(px - cx, py - cy);
}
function isOnAnyWall(lon, lat, eps = 1e-6) {
  for (const seg of wallSegments.value || []) {
    const a = seg[0], b = seg[1];
    if (pointSegmentDistance(lon, lat, a[0], a[1], b[0], b[1]) <= eps) return true;
  }
  return false;
}

function nearestFreeGridIndex(lon, lat) {
  const grid = ensureGrid();
  let best = -1;
  let bestD = Infinity;
  for (let i = 0; i < grid.nodes.length; i++) {
    const n = grid.nodes[i];
    const d = Math.hypot(n.lon - lon, n.lat - lat);
    if (d < bestD && grid.adjacency[i] && grid.adjacency[i].length) { bestD = d; best = i; }
  }
  return best;
}

function aStarGrid(startIdx, goalIdx) {
  const grid = ensureGrid();
  const nodes = grid.nodes;
  const adj = grid.adjacency;
  const open = new Set([startIdx]);
  const came = new Map();
  const g = new Map([[startIdx, 0]]);
  const h = (i) => euclid(nodes[i], nodes[goalIdx]);
  const f = new Map([[startIdx, h(startIdx)]]);
  function popLowest() {
    let m = null, mv = Infinity;
    for (const i of open) { const v = f.get(i) ?? Infinity; if (v < mv) { mv = v; m = i; } }
    return m;
  }
  while (open.size) {
    const cur = popLowest();
    if (cur === goalIdx) {
      const path = [];
      let x = cur;
      while (x !== undefined) { path.push(nodes[x]); x = came.get(x); }
      path.reverse();
      return path;
    }
    open.delete(cur);
    for (const { to, w } of adj[cur] || []) {
      const tentative = (g.get(cur) ?? Infinity) + w;
      if (tentative < (g.get(to) ?? Infinity)) {
        came.set(to, cur);
        g.set(to, tentative);
        f.set(to, tentative + h(to));
        open.add(to);
      }
    }
  }
  return null;
}

function computeAndDrawPath() {
  pathPoints.value = [];
  // 只有当起点和终点都被设置（都不为0）时才进行计算
  if ((startLon.value === 0 && startLat.value === 0) || (endLon.value === 0 && endLat.value === 0)) {
    return;
  }
  if (!isFinite(startLon.value) || !isFinite(startLat.value) || !isFinite(endLon.value) || !isFinite(endLat.value)) {
    drawNetwork();
    return;
  }
  if (isInsideAnyObstacle(startLon.value, startLat.value) || isInsideAnyObstacle(endLon.value, endLat.value)) {
    alert("起点或终点在障碍区域内");
    drawNetwork();
    return;
  }
  const obstacles = graph.value?.obstacles || [];
  if (!wallSegments.value.length && geojsonRef.value) wallSegments.value = buildWallSegments(geojsonRef.value);
  const bboxNodes = bboxFromNodes(graph.value.nodes);
  if (!workerRef.value) workerRef.value = new Worker(new URL('./pathWorker.js', import.meta.url), { type: 'module' });
  busy.value = true;
  const t0 = performance.now();
  const timer = setTimeout(() => { busy.value = false; alert('路径计算超时'); }, 15000);
  workerRef.value.onmessage = (ev) => {
    clearTimeout(timer);
    busy.value = false;
    const data = ev.data;
    if (!data.ok) {
      alert(data.error === 'nearby-grid-fail' ? '无法找到附近可通行格点' : '未找到可通行路径');
      drawNetwork();
      return;
    }
    pathPoints.value = data.path;
    drawNetwork();
    const t1 = performance.now();
    console.log('路径计算耗时(ms):', Math.round(t1 - t0));
  };
  const payload = {
    startLon: Number(startLon.value),
    startLat: Number(startLat.value),
    endLon: Number(endLon.value),
    endLat: Number(endLat.value),
    obstacles: safeClone(toPlainObstacles(obstacles)),
    walls: safeClone(toPlainWalls(wallSegments.value)),
    bboxNodes: safeClone(bboxNodes),
  };
  try {
    workerRef.value.postMessage(payload);
  } catch (e) {
    clearTimeout(timer);
    busy.value = false;
    alert('浏览器消息序列化失败，已取消路径计算');
    console.error(e);
  }
}
</script>

<style scoped>
.container {
  font-family: system-ui, -apple-system, Segoe UI, Roboto, PingFang SC,
    Microsoft YaHei, sans-serif;
  padding: 12px;
}
header {
  display: flex;
  align-items: baseline;
  gap: 16px;
}
h1 {
  font-size: 18px;
  margin: 0;
}
.stats {
  display: flex;
  gap: 12px;
  color: #555;
}
.controls {
  margin: 10px 0;
  display: flex;
  flex-direction: column;
  gap: 8px;
}
.row {
  display: flex;
  gap: 8px;
  align-items: center;
}
label {
  color: #444;
}
input {
  width: 150px;
  padding: 4px 6px;
}
button {
  padding: 6px 10px;
}
.canvas-wrap {
  border: 1px solid #ddd;
}
.pick-state {
  margin-left: 12px;
  color: #666;
}
</style>
