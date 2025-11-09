import { haversineDistance } from './geo.js'
import { snapToNearestEdge } from './graph.js'

class MinHeap {
  constructor() { this.arr = []; }
  push(item, priority) {
    this.arr.push({ item, priority });
    this.bubbleUp(this.arr.length - 1);
  }
  bubbleUp(i) {
    const a = this.arr;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].priority <= a[i].priority) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop() {
    const a = this.arr;
    if (a.length === 0) return null;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      this.sinkDown(0);
    }
    return top.item;
  }
  sinkDown(i) {
    const a = this.arr;
    const n = a.length;
    while (true) {
      let left = i * 2 + 1;
      let right = left + 1;
      let smallest = i;
      if (left < n && a[left].priority < a[smallest].priority) smallest = left;
      if (right < n && a[right].priority < a[smallest].priority) smallest = right;
      if (smallest === i) break;
      [a[i], a[smallest]] = [a[smallest], a[i]];
      i = smallest;
    }
  }
  get size() { return this.arr.length; }
}

export function aStar(graph, startId, goalId) {
  const { nodes, adjacency } = graph;
  const n = nodes.length;
  const open = new MinHeap();
  const cameFrom = new Int32Array(n).fill(-1);
  const gScore = new Float64Array(n);
  const fScore = new Float64Array(n);
  for (let i = 0; i < n; i++) { gScore[i] = Infinity; fScore[i] = Infinity; }
  gScore[startId] = 0;
  fScore[startId] = heuristic(nodes[startId], nodes[goalId]);
  open.push(startId, fScore[startId]);

  const closed = new Uint8Array(n);

  while (open.size) {
    const current = open.pop();
    if (current === goalId) {
      return reconstructPath(cameFrom, current, graph);
    }
    if (closed[current]) continue;
    closed[current] = 1;
    for (const { to, w } of adjacency[current]) {
      const tentative = gScore[current] + w;
      if (tentative < gScore[to]) {
        cameFrom[to] = current;
        gScore[to] = tentative;
        fScore[to] = tentative + heuristic(nodes[to], nodes[goalId]);
        open.push(to, fScore[to]);
      }
    }
  }
  return { pathIds: [], pathCoords: [], length: Infinity };
}

export function dijkstra(graph, startId, goalId) {
  const { nodes, adjacency } = graph;
  const n = nodes.length;
  const open = new MinHeap();
  const cameFrom = new Int32Array(n).fill(-1);
  const dist = new Float64Array(n);
  for (let i = 0; i < n; i++) dist[i] = Infinity;
  dist[startId] = 0;
  open.push(startId, 0);
  const closed = new Uint8Array(n);

  while (open.size) {
    const current = open.pop();
    if (current === goalId) {
      return reconstructPath(cameFrom, current, graph, dist[current]);
    }
    if (closed[current]) continue;
    closed[current] = 1;
    for (const { to, w } of adjacency[current]) {
      const tentative = dist[current] + w;
      if (tentative < dist[to]) {
        cameFrom[to] = current;
        dist[to] = tentative;
        open.push(to, tentative);
      }
    }
  }
  return { pathIds: [], pathCoords: [], length: Infinity };
}

function heuristic(aNode, bNode) {
  return haversineDistance([aNode.lon, aNode.lat], [bNode.lon, bNode.lat]);
}

function reconstructPath(cameFrom, current, graph, knownLength) {
  const { nodes, adjacency } = graph;
  const ids = [current];
  while (cameFrom[current] !== -1) {
    current = cameFrom[current];
    ids.push(current);
  }
  ids.reverse();
  const coords = ids.map(id => [nodes[id].lon, nodes[id].lat]);
  let length = 0;
  if (typeof knownLength === 'number') {
    length = knownLength;
  } else {
    for (let i = 0; i < ids.length - 1; i++) {
      const a = ids[i], b = ids[i + 1];
      // find edge weight; small fallback to haversine
      const edge = adjacency[a].find(e => e.to === b);
      length += edge ? edge.w : haversineDistance(
        [nodes[a].lon, nodes[a].lat], [nodes[b].lon, nodes[b].lat]
      );
    }
  }
  return { pathIds: ids, pathCoords: coords, length };
}

// Build a runtime graph by snapping arbitrary start/end coords onto nearest edges and
// inserting temporary nodes in the middle of segments if needed.
export function makeRuntimeGraphWithSnap(graph, startCoord, endCoord, epsilon = 1e-6) {
  const nodes = graph.nodes.map(n => ({ ...n }));
  const adjacency = graph.adjacency.map(arr => arr.map(e => ({ ...e })));

  function ensureEdgeAnchor(aId, bId, t, point) {
    if (t <= epsilon) return aId;
    if (1 - t <= epsilon) return bId;
    const id = nodes.length;
    nodes.push({ id, lon: point[0], lat: point[1], key: `snap:${point[0]},${point[1]}` });
    adjacency[id] = [];
    const a = graph.nodes[aId];
    const b = graph.nodes[bId];
    const wA = haversineDistance([a.lon, a.lat], point);
    const wB = haversineDistance(point, [b.lon, b.lat]);
    adjacency[id].push({ to: aId, w: wA });
    adjacency[aId].push({ to: id, w: wA });
    adjacency[id].push({ to: bId, w: wB });
    adjacency[bId].push({ to: id, w: wB });
    return id;
  }

  function insertClickedCoord(coord) {
    const snap = snapToNearestEdge(graph, coord[0], coord[1]);
    if (!snap) return { coordId: -1, anchorId: -1 };
    const { aId, bId, t, point } = snap;
    const anchorId = ensureEdgeAnchor(aId, bId, t, point);
    const coordId = nodes.length;
    nodes.push({ id: coordId, lon: coord[0], lat: coord[1], key: `clicked:${coord[0]},${coord[1]}` });
    adjacency[coordId] = [];
    const wC = haversineDistance(coord, point);
    adjacency[coordId].push({ to: anchorId, w: wC });
    adjacency[anchorId].push({ to: coordId, w: wC });
    return { coordId, anchorId };
  }

  const { coordId: startId } = insertClickedCoord(startCoord);
  const { coordId: endId } = insertClickedCoord(endCoord);
  return { graph: { nodes, adjacency }, startId, endId };
}