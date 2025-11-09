<template>
  <div class="container">
    <header>
      <h1>GeoJSON 路网寻路分析</h1>
      <div class="stats">
        <div>节点: {{ stats.nodes }}</div>
        <div>边: {{ stats.edges }}</div>
        <div>构建耗时: {{ stats.buildMs }} ms</div>
      </div>
    </header>

    <section class="controls">
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
        <button :disabled="loading" @click="build">构建图结构</button>
        <button :disabled="!graphReady" @click="run">路径查询（A*）</button>
        <span class="pick-state">当前选择：{{ picking ? (picking === 'start' ? '起点' : '终点') : '无' }}</span>
      </div>
      <div class="row">
        <label>优化策略</label>
        <select v-model="strategy">
          <option value="shortest">最短距离</option>
          <option value="fewest_turns">最少转弯</option>
        </select>
        <label>步行速度(m/s)</label>
        <input v-model.number="walkSpeed" type="number" step="0.1" min="0.1" />
      </div>
    </section>

    <section class="canvas-wrap">
      <canvas ref="canvasRef" width="1000" height="700" @click="onCanvasClick" :style="{ cursor: picking ? 'crosshair' : 'default' }"></canvas>
    </section>

    <section class="result" v-if="result.pathCoords.length">
      <h2>路径结果</h2>
      <div>节点数: {{ result.pathIds.length }}</div>
      <div>总长度: {{ (result.length/1000).toFixed(3) }} km</div>
      <div>预计时间: {{ formatTime(result.timeSec) }}</div>
      <div>查询耗时: {{ result.ms }} ms</div>
      <div v-if="strategy === 'shortest'">验证(Dijkstra):
        <span :class="{ ok: validation.ok, bad: !validation.ok }">
          {{ validation.ok ? '通过' : '未通过' }}
        </span>
        <template v-if="validation.ok">
          ，A*={{ (result.length/1000).toFixed(3) }} km，Dijkstra={{ (validation.dijkstraLen/1000).toFixed(3) }} km
        </template>
      </div>
      <details>
        <summary>节点坐标序列</summary>
        <pre>{{ result.pathCoords.map(c => c.map(v => v.toFixed(6))).join('\n') }}</pre>
      </details>
    </section>
  </div>
</template>

<script setup>
import { ref, reactive } from 'vue'
import { buildGraph, buildSpatialIndex, snapToNearestEdge } from './graph.js'
import { aStar, dijkstra, makeRuntimeGraphWithSnap } from './pathfinding.js'
import { bboxFromNodes, fitToCanvas } from './geo.js'

const canvasRef = ref(null)
const ctxRef = ref(null)
const graph = ref(null)
const spatial = ref(null)
const graphReady = ref(false)
const loading = ref(false)

const stats = reactive({ nodes: 0, edges: 0, buildMs: 0 })
const startLon = ref(0)
const startLat = ref(0)
const endLon = ref(0)
const endLat = ref(0)
const picking = ref(null) // 'start' | 'end' | null
const view = reactive({ scale: 1, tx: 0, ty: 0 })

const result = reactive({ pathIds: [], pathCoords: [], length: 0, ms: 0, timeSec: 0 })
const validation = reactive({ ok: false, dijkstraLen: 0 })
const strategy = ref('shortest')
const walkSpeed = ref(1.4) // m/s

async function loadGeoJSON() {
  const url = new URL('./data/lines.geojson', import.meta.url).href
  const res = await fetch(url)
  if (!res.ok) throw new Error('加载 GeoJSON 失败')
  return await res.json()
}

async function build() {
  loading.value = true
  result.pathIds = []
  result.pathCoords = []
  result.length = 0
  validation.ok = false
  validation.dijkstraLen = 0
  try {
    const t0 = performance.now()
    const geojson = await loadGeoJSON()
    const g = buildGraph(geojson, { precision: 6, includeObstacles: true })
    graph.value = g
    spatial.value = buildSpatialIndex(g.nodes, 0.01)
    const t1 = performance.now()
    stats.nodes = g.nodes.length
    stats.edges = g.adjacency.reduce((s, a) => s + a.length, 0) / 2
    stats.buildMs = Math.round(t1 - t0)
    graphReady.value = true
    setupCanvas()
    drawNetwork()
    // 默认进入起点选择，提升可用性
    picking.value = 'start'
  } catch (e) {
    console.error(e)
    alert(e.message)
  } finally {
    loading.value = false
  }
}

function setupCanvas() {
  const canvas = canvasRef.value
  const ctx = canvas.getContext('2d')
  ctxRef.value = ctx
  ctx.clearRect(0, 0, canvas.width, canvas.height)
  if (!graph.value) return
  const bbox = bboxFromNodes(graph.value.nodes)
  const fit = fitToCanvas(bbox, canvas.width, canvas.height, 30)
  view.scale = fit.scale
  view.tx = fit.tx
  view.ty = fit.ty
}

function toScreen(lon, lat) {
  return {
    x: lon * view.scale + view.tx,
    y: lat * view.scale + view.ty,
  }
}

function drawNetwork() {
  const canvas = canvasRef.value
  const ctx = ctxRef.value
  ctx.clearRect(0, 0, canvas.width, canvas.height)
  if (!graph.value) return
  ctx.lineWidth = 1
  ctx.strokeStyle = '#86a8c0'
  ctx.globalAlpha = 0.6
  for (let i = 0; i < graph.value.nodes.length; i++) {
    for (const { to } of graph.value.adjacency[i]) {
      if (to < i) continue // draw once
      const a = graph.value.nodes[i]
      const b = graph.value.nodes[to]
      const p = toScreen(a.lon, a.lat)
      const q = toScreen(b.lon, b.lat)
      ctx.beginPath()
      ctx.moveTo(p.x, p.y)
      ctx.lineTo(q.x, q.y)
      ctx.stroke()
    }
  }
  ctx.globalAlpha = 1
  // draw start/end if present
  drawPoint(startLon.value, startLat.value, '#2b9348')
  drawPoint(endLon.value, endLat.value, '#d00000')
}

function drawPoint(lon, lat, color) {
  if (!isFinite(lon) || !isFinite(lat)) return
  const ctx = ctxRef.value
  const p = toScreen(lon, lat)
  ctx.fillStyle = color
  ctx.beginPath()
  ctx.arc(p.x, p.y, 4, 0, Math.PI * 2)
  ctx.fill()
}

function drawPath(coords) {
  const ctx = ctxRef.value
  if (!coords.length) return
  ctx.lineWidth = 3
  ctx.strokeStyle = '#ff4d4f'
  ctx.beginPath()
  const p0 = toScreen(coords[0][0], coords[0][1])
  ctx.moveTo(p0.x, p0.y)
  for (let i = 1; i < coords.length; i++) {
    const p = toScreen(coords[i][0], coords[i][1])
    ctx.lineTo(p.x, p.y)
  }
  ctx.stroke()
}

function pickStart() { picking.value = 'start' }
function pickEnd() { picking.value = 'end' }

function onCanvasClick(ev) {
  if (!graphReady.value) return
  const rect = canvasRef.value.getBoundingClientRect()
  const x = ev.clientX - rect.left
  const y = ev.clientY - rect.top
  const lon = (x - view.tx) / view.scale
  const lat = (y - view.ty) / view.scale
  // 若未显式进入选择模式，默认按“起点→终点”轮换
  if (!picking.value) {
    picking.value = (!isFinite(startLon.value) || !isFinite(startLat.value)) ? 'start' : 'end'
  }
  if (picking.value === 'start') {
    startLon.value = lon
    startLat.value = lat
    // 下一次指向终点，便于连续选取
    picking.value = 'end'
  } else if (picking.value === 'end') {
    endLon.value = lon
    endLat.value = lat
    // 选完终点退出选择
    picking.value = null
  }
  drawNetwork()
}

function run() {
  if (!graphReady.value) return
  const { graph: rGraph, startId, endId, error } = makeRuntimeGraphWithSnap(
    graph.value,
    [startLon.value, startLat.value],
    [endLon.value, endLat.value],
    { obstacles: graph.value.obstacles }
  )
  if (error === 'point_in_obstacle') {
    alert('起点或终点位于阻碍区域，请重新选择可行点')
    return
  }
  if (startId === -1 || endId === -1) {
    alert('无法定位到最近边（或附近不可行），请检查数据或位置')
    return
  }
  const t0 = performance.now()
  const res = aStar(rGraph, startId, endId, { strategy: strategy.value })
  const t1 = performance.now()
  result.pathIds = res.pathIds
  result.pathCoords = res.pathCoords
  result.length = res.length
  result.ms = Math.round(t1 - t0)
  result.timeSec = res.length / Math.max(0.1, walkSpeed.value)
  // validation via Dijkstra (only for shortest strategy)
  if (strategy.value === 'shortest') {
    const dres = dijkstra(rGraph, startId, endId)
    validation.dijkstraLen = dres.length
    validation.ok = Math.abs(dres.length - res.length) < 1e-6
  } else {
    validation.dijkstraLen = 0
    validation.ok = true
  }
  drawNetwork()
  drawPath(result.pathCoords)
  console.log('查询耗时', result.ms, 'ms')
}

function formatTime(sec) {
  if (!isFinite(sec) || sec <= 0) return '—'
  const m = Math.floor(sec / 60)
  const s = Math.round(sec % 60)
  return `${m}分${s}秒`
}

</script>

<style scoped>
.container { font-family: system-ui, -apple-system, Segoe UI, Roboto, PingFang SC, Microsoft YaHei, sans-serif; padding: 12px; }
header { display: flex; align-items: baseline; gap: 16px; }
h1 { font-size: 18px; margin: 0; }
.stats { display: flex; gap: 12px; color: #555; }
.controls { margin: 10px 0; display: flex; flex-direction: column; gap: 8px; }
.row { display: flex; gap: 8px; align-items: center; }
label { color: #444; }
input { width: 150px; padding: 4px 6px; }
button { padding: 6px 10px; }
.canvas-wrap { border: 1px solid #ddd; }
.result { margin-top: 10px; }
.ok { color: #2b9348; }
.bad { color: #d00000; }
pre { background: #f7f7f7; padding: 8px; max-height: 200px; overflow: auto; }
.pick-state { margin-left: 12px; color: #666; }
</style>