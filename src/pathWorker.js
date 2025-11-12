import { pointInPolygon, segmentIntersectsPolygon } from './obstacles.js'

const MAX_COLS = 120
const MAX_ROWS = 120
const MIN_COLS = 40
const MIN_ROWS = 30

function orientation(ax, ay, bx, by, cx, cy) {
  const val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by)
  if (Math.abs(val) < 1e-12) return 0
  return val > 0 ? 1 : 2
}
function onSegment(ax, ay, cx, cy, bx, by) {
  return Math.min(ax, bx) - 1e-12 <= cx && cx <= Math.max(ax, bx) + 1e-12 &&
         Math.min(ay, by) - 1e-12 <= cy && cy <= Math.max(ay, by) + 1e-12
}
function segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
  const o1 = orientation(ax, ay, bx, by, cx, cy)
  const o2 = orientation(ax, ay, bx, by, dx, dy)
  const o3 = orientation(cx, cy, dx, dy, ax, ay)
  const o4 = orientation(cx, cy, dx, dy, bx, by)
  if (o1 !== o2 && o3 !== o4) return true
  if (o1 === 0 && onSegment(ax, ay, cx, cy, bx, by)) return true
  if (o2 === 0 && onSegment(ax, ay, dx, dy, bx, by)) return true
  if (o3 === 0 && onSegment(cx, cy, ax, ay, dx, dy)) return true
  if (o4 === 0 && onSegment(cx, cy, bx, by, dx, dy)) return true
  return false
}

function euclid(a, b) { return Math.hypot(a.lon - b.lon, a.lat - b.lat) }
function bboxIntersects(a, b) { return !(a.minLon > b.maxLon || a.maxLon < b.minLon || a.minLat > b.maxLat || a.maxLat < b.minLat) }

function buildObstacleMeta(obstacles) {
  const out = []
  for (const rings of obstacles || []) {
    let mLon = Infinity, mLat = Infinity, xLon = -Infinity, xLat = -Infinity
    for (const ring of rings) {
      for (const [lo, la] of ring) {
        if (lo < mLon) mLon = lo
        if (lo > xLon) xLon = lo
        if (la < mLat) mLat = la
        if (la > xLat) xLat = la
      }
    }
    out.push({ rings, bbox: { minLon: mLon, minLat: mLat, maxLon: xLon, maxLat: xLat } })
  }
  return out
}
function buildWallMeta(walls) {
  const out = []
  for (const [a, b] of walls || []) {
    out.push({ a, b, bbox: { minLon: Math.min(a[0], b[0]), maxLon: Math.max(a[0], b[0]), minLat: Math.min(a[1], b[1]), maxLat: Math.max(a[1], b[1]) } })
  }
  return out
}

function ensureGridLocal(cfg) {
  const { startLon, startLat, endLon, endLat, bboxNodes, obstaclesMeta, wallsMeta } = cfg
  let minLon = Math.min(startLon, endLon)
  let maxLon = Math.max(startLon, endLon)
  let minLat = Math.min(startLat, endLat)
  let maxLat = Math.max(startLat, endLat)
  if (!isFinite(minLon) || !isFinite(minLat) || !isFinite(maxLon) || !isFinite(maxLat)) {
    minLon = bboxNodes.minLon; maxLon = bboxNodes.maxLon; minLat = bboxNodes.minLat; maxLat = bboxNodes.maxLat
  }
  let lonSpan = maxLon - minLon
  let latSpan = maxLat - minLat
  const padLon = lonSpan * 0.3 + 1e-4
  const padLat = latSpan * 0.3 + 1e-4
  minLon = Math.max(bboxNodes.minLon, minLon - padLon)
  maxLon = Math.min(bboxNodes.maxLon, maxLon + padLon)
  minLat = Math.max(bboxNodes.minLat, minLat - padLat)
  maxLat = Math.min(bboxNodes.maxLat, maxLat + padLat)
  lonSpan = Math.max(maxLon - minLon, 1e-6)
  latSpan = Math.max(maxLat - minLat, 1e-6)
  let cols = Math.round(60)
  cols = Math.max(MIN_COLS, Math.min(cols, MAX_COLS))
  let rows = Math.round((latSpan / lonSpan) * cols)
  rows = Math.max(MIN_ROWS, Math.min(rows, MAX_ROWS))
  const cellLon = lonSpan / cols
  const cellLat = latSpan / rows
  const nodes = []
  const blocked = []
  const eps = Math.min(cellLon, cellLat) * 0.3
  for (let r = 0; r <= rows; r++) {
    for (let c = 0; c <= cols; c++) {
      const lon = minLon + c * cellLon
      const lat = minLat + r * cellLat
      const idx = nodes.length
      nodes.push({ id: idx, lon, lat })
      let inside = false
      for (const om of obstaclesMeta) {
        if (!bboxIntersects({ minLon: lon, maxLon: lon, minLat: lat, maxLat: lat }, om.bbox)) continue
        if (pointInPolygon(lon, lat, om.rings)) { inside = true; break }
      }
      if (!inside) {
        for (const wm of wallsMeta) {
          const vx = wm.bbox.maxLon - wm.bbox.minLon
          const vy = wm.bbox.maxLat - wm.bbox.minLat
          const d = pointSegmentDistance(lon, lat, wm.a[0], wm.a[1], wm.b[0], wm.b[1])
          if (d <= Math.min(cellLon, cellLat) * 0.3) { inside = true; break }
        }
      }
      blocked[idx] = inside ? 1 : 0
    }
  }
  const adjacency = new Array(nodes.length)
  const dirs = [[0,1],[1,0],[0,-1],[-1,0],[1,1],[1,-1],[-1,1],[-1,-1]]
  for (let r = 0; r <= rows; r++) {
    for (let c = 0; c <= cols; c++) {
      const idx = r * (cols + 1) + c
      const list = []
      if (blocked[idx]) { adjacency[idx] = list; continue }
      const a = nodes[idx]
      for (const [dr, dc] of dirs) {
        const nr = r + dr, nc = c + dc
        if (nr < 0 || nr > rows || nc < 0 || nc > cols) continue
        const j = nr * (cols + 1) + nc
        if (blocked[j]) continue
        const b = nodes[j]
        const sb = { minLon: Math.min(a.lon, b.lon), maxLon: Math.max(a.lon, b.lon), minLat: Math.min(a.lat, b.lat), maxLat: Math.max(a.lat, b.lat) }
        let crossesPoly = false
        for (const om of obstaclesMeta) {
          if (!bboxIntersects(sb, om.bbox)) continue
          if (segmentIntersectsPolygon([a.lon, a.lat], [b.lon, b.lat], om.rings)) { crossesPoly = true; break }
        }
        let crossesWall = false
        if (!crossesPoly) {
          for (const wm of wallsMeta) {
            if (!bboxIntersects(sb, wm.bbox)) continue
            if (segmentsIntersect(a.lon, a.lat, b.lon, b.lat, wm.a[0], wm.a[1], wm.b[0], wm.b[1])) { crossesWall = true; break }
          }
        }
        if (!crossesPoly && !crossesWall) list.push({ to: j, w: euclid(a, b) })
      }
      adjacency[idx] = list
    }
  }
  return { nodes, adjacency, cols, rows, minLon, minLat, cellLon, cellLat }
}

function pointSegmentDistance(px, py, ax, ay, bx, by) {
  const vx = bx - ax, vy = by - ay
  const wx = px - ax, wy = py - ay
  const c1 = vx * wx + vy * wy
  const c2 = vx * vx + vy * vy || 1e-12
  let t = c1 / c2
  if (t < 0) t = 0; else if (t > 1) t = 1
  const cx = ax + t * vx, cy = ay + t * vy
  return Math.hypot(px - cx, py - cy)
}

function nearestFreeGridIndex(grid, lon, lat) {
  let best = -1, bestD = Infinity
  for (let i = 0; i < grid.nodes.length; i++) {
    const n = grid.nodes[i]
    const list = grid.adjacency[i]
    if (!list || !list.length) continue
    const d = Math.hypot(n.lon - lon, n.lat - lat)
    if (d < bestD) { bestD = d; best = i }
  }
  return best
}

function aStarGrid(grid, startIdx, goalIdx) {
  const nodes = grid.nodes
  const adj = grid.adjacency
  const open = new Set([startIdx])
  const came = new Map()
  const g = new Map([[startIdx, 0]])
  const h = (i) => euclid(nodes[i], nodes[goalIdx])
  const f = new Map([[startIdx, h(startIdx)]])
  function popLowest() {
    let m = null, mv = Infinity
    for (const i of open) { const v = f.get(i) ?? Infinity; if (v < mv) { mv = v; m = i } }
    return m
  }
  let steps = 0
  while (open.size) {
    const cur = popLowest()
    if (cur === goalIdx) {
      const path = []
      let x = cur
      while (x !== undefined) { path.push(nodes[x]); x = came.get(x) }
      path.reverse()
      return path
    }
    open.delete(cur)
    for (const { to, w } of adj[cur] || []) {
      const tentative = (g.get(cur) ?? Infinity) + w
      if (tentative < (g.get(to) ?? Infinity)) {
        came.set(to, cur)
        g.set(to, tentative)
        f.set(to, tentative + h(to))
        open.add(to)
      }
    }
    steps++
    if (steps % 500 === 0) { /* yield */ }
  }
  return null
}

self.onmessage = async (ev) => {
  const { startLon, startLat, endLon, endLat, obstacles, walls, bboxNodes } = ev.data
  try {
    const obstaclesMeta = buildObstacleMeta(obstacles)
    const wallsMeta = buildWallMeta(walls)
    const grid = ensureGridLocal({ startLon, startLat, endLon, endLat, bboxNodes, obstaclesMeta, wallsMeta })
    const si = nearestFreeGridIndex(grid, startLon, startLat)
    const gi = nearestFreeGridIndex(grid, endLon, endLat)
    if (si < 0 || gi < 0) { self.postMessage({ ok: false, error: 'nearby-grid-fail' }); return }
    const path = aStarGrid(grid, si, gi)
    if (!path || !path.length) { self.postMessage({ ok: false, error: 'no-path' }); return }
    self.postMessage({ ok: true, path })
  } catch (e) {
    self.postMessage({ ok: false, error: String(e && e.message || e) })
  }
}

