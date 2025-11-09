import { haversineDistance, roundCoordKey } from './geo.js'

export function buildGraph(geojson, options = {}) {
  const precision = options.precision ?? 6;
  const nodeByKey = new Map();
  const nodes = []; // { id, lon, lat, key }
  const adjacency = []; // Array of arrays: adjacency[id] = [{ to, w }]

  function addNode(coord) {
    const key = roundCoordKey(coord, precision);
    let id = nodeByKey.get(key);
    if (id === undefined) {
      id = nodes.length;
      nodeByKey.set(key, id);
      nodes.push({ id, lon: coord[0], lat: coord[1], key });
      adjacency[id] = [];
    }
    return id;
  }

  function addUndirectedEdge(aId, bId, w) {
    if (aId === bId || !isFinite(w)) return;
    adjacency[aId].push({ to: bId, w });
    adjacency[bId].push({ to: aId, w });
  }

  const features = geojson?.features ?? [];
  for (const f of features) {
    const g = f.geometry;
    if (!g) continue;
    if (g.type === 'LineString') {
      const coords = g.coordinates;
      for (let i = 0; i < coords.length - 1; i++) {
        const aId = addNode(coords[i]);
        const bId = addNode(coords[i + 1]);
        const w = haversineDistance(coords[i], coords[i + 1]);
        addUndirectedEdge(aId, bId, w);
      }
    } else if (g.type === 'MultiLineString') {
      for (const line of g.coordinates) {
        for (let i = 0; i < line.length - 1; i++) {
          const aId = addNode(line[i]);
          const bId = addNode(line[i + 1]);
          const w = haversineDistance(line[i], line[i + 1]);
          addUndirectedEdge(aId, bId, w);
        }
      }
    }
  }

  return { nodes, adjacency, nodeByKey };
}

// Lightweight spatial index using lat/lon grid buckets for fast nearest-node lookups.
export function buildSpatialIndex(nodes, cellDeg = 0.01) {
  const buckets = new Map();
  function bucketKey(lon, lat) {
    const ix = Math.floor((lon + 180) / cellDeg);
    const iy = Math.floor((lat + 90) / cellDeg);
    return `${ix}:${iy}`;
  }
  for (const n of nodes) {
    const key = bucketKey(n.lon, n.lat);
    let arr = buckets.get(key);
    if (!arr) {
      arr = [];
      buckets.set(key, arr);
    }
    arr.push(n.id);
  }
  function neighbors(ix, iy, radius) {
    const ids = [];
    for (let dx = -radius; dx <= radius; dx++) {
      for (let dy = -radius; dy <= radius; dy++) {
        const key = `${ix + dx}:${iy + dy}`;
        const arr = buckets.get(key);
        if (arr) ids.push(...arr);
      }
    }
    return ids;
  }
  return {
    nearest(lon, lat, expandMax = 10) {
      const ix = Math.floor((lon + 180) / cellDeg);
      const iy = Math.floor((lat + 90) / cellDeg);
      let bestId = -1;
      let bestDist = Infinity;
      // progressively expand search radius up to a larger cap
      const maxCap = Math.max(expandMax, 30);
      for (let r = 0; r <= maxCap; r++) {
        const candidates = neighbors(ix, iy, r);
        if (candidates.length === 0) continue;
        for (const id of candidates) {
          const n = nodes[id];
          const d = Math.hypot(n.lon - lon, n.lat - lat);
          if (d < bestDist) {
            bestDist = d;
            bestId = id;
          }
        }
        if (bestId !== -1) break; // found something in this ring
      }
      // fallback: global scan if still not found (very sparse or click far away)
      if (bestId === -1 && nodes.length) {
        for (const n of nodes) {
          const d = Math.hypot(n.lon - lon, n.lat - lat);
          if (d < bestDist) {
            bestDist = d;
            bestId = n.id;
          }
        }
      }
      return bestId;
    },
    cellDeg,
  };
}

// Compute nearest point on a segment AB to point P in lon/lat plane
export function nearestPointOnSegment(px, py, ax, ay, bx, by) {
  const vx = bx - ax;
  const vy = by - ay;
  const wx = px - ax;
  const wy = py - ay;
  const denom = vx * vx + vy * vy;
  let t = 0;
  if (denom > 0) t = (wx * vx + wy * vy) / denom;
  if (t < 0) t = 0; else if (t > 1) t = 1;
  const x = ax + t * vx;
  const y = ay + t * vy;
  const dx = x - px;
  const dy = y - py;
  return { t, x, y, dist2: dx * dx + dy * dy };
}

// Snap an arbitrary lon/lat to the nearest point on any graph edge
export function snapToNearestEdge(graph, lon, lat) {
  const { nodes, adjacency } = graph || {};
  if (!nodes || !adjacency || !nodes.length) return null;
  let best = null;
  let bestDist2 = Infinity;
  for (let aId = 0; aId < nodes.length; aId++) {
    const a = nodes[aId];
    const neighbors = adjacency[aId] || [];
    for (const { to: bId } of neighbors) {
      if (bId <= aId) continue; // avoid double-counting edges
      const b = nodes[bId];
      const { t, x, y, dist2 } = nearestPointOnSegment(lon, lat, a.lon, a.lat, b.lon, b.lat);
      if (dist2 < bestDist2) {
        bestDist2 = dist2;
        best = { aId, bId, t, point: [x, y], dist2 };
      }
    }
  }
  return best;
}