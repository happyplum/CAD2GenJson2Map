// Utilities for obstacle parsing and feasibility checks on GeoJSON
// Polygons are represented as arrays of rings; each ring is an array of [lon, lat] coordinates.

// Extract obstacle polygons from GeoJSON features.
// A feature is considered an obstacle if:
// - properties.walkable === false
// - properties.blocked === true
// - properties.type === 'obstacle' (case-insensitive)
// - properties.obstacle === true
// You can override with options.isObstacle(feature) => boolean.
export function extractObstaclesFromGeoJSON(geojson, options = {}) {
  const features = geojson?.features ?? [];
  const isObstacle = options.isObstacle ?? defaultObstacleClassifier;
  const obstacles = [];
  for (const f of features) {
    if (!f || !f.geometry) continue;
    if (!isObstacle(f)) continue;
    const g = f.geometry;
    if (g.type === 'Polygon') {
      const rings = normalizePolygonRings(g.coordinates);
      if (rings.length) obstacles.push(rings);
    } else if (g.type === 'MultiPolygon') {
      for (const poly of g.coordinates) {
        const rings = normalizePolygonRings(poly);
        if (rings.length) obstacles.push(rings);
      }
    }
  }
  return obstacles;
}

function defaultObstacleClassifier(f) {
  const p = f?.properties || {};
  const type = String(p.type || '').toLowerCase();
  return p.walkable === false || p.blocked === true || p.obstacle === true || type === 'obstacle';
}

function normalizePolygonRings(coords) {
  // coords can be: [ring1, ring2, ...]; each ring is array of [lon, lat]
  const rings = [];
  for (const ring of coords || []) {
    if (!Array.isArray(ring) || ring.length < 3) continue;
    // Ensure the ring is closed; if last != first, append first.
    const last = ring[ring.length - 1];
    const first = ring[0];
    const isClosed = first && last && first[0] === last[0] && first[1] === last[1];
    rings.push(isClosed ? ring : [...ring, first]);
  }
  return rings;
}

// Ray casting point-in-polygon on outer ring; holes are treated as non-walkable as well.
// Returns true if point lies inside any ring (outer or holes). For strict handling, you may
// treat holes differently; here we consider any ring as blocked to be safe.
export function pointInPolygon(lon, lat, polygonRings) {
  for (const ring of polygonRings) {
    if (pointInRing(lon, lat, ring)) return true;
  }
  return false;
}

function pointInRing(lon, lat, ring) {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const xi = ring[i][0], yi = ring[i][1];
    const xj = ring[j][0], yj = ring[j][1];
    const intersect = ((yi > lat) !== (yj > lat)) &&
      (lon < (xj - xi) * (lat - yi) / ((yj - yi) || 1e-12) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

// Check if a segment [a,b] intersects any edge of a polygon (any ring),
// or if either endpoint lies inside the polygon.
export function segmentIntersectsPolygon(a, b, polygonRings) {
  const [ax, ay] = a, [bx, by] = b;
  if (pointInPolygon(ax, ay, polygonRings) || pointInPolygon(bx, by, polygonRings)) return true;
  for (const ring of polygonRings) {
    for (let i = 0; i < ring.length - 1; i++) {
      const c = ring[i];
      const d = ring[i + 1];
      if (segmentsIntersect(ax, ay, bx, by, c[0], c[1], d[0], d[1])) return true;
    }
  }
  return false;
}

// Check against a list of obstacles
export function segmentIntersectsAnyObstacle(a, b, obstacles) {
  for (const poly of obstacles || []) {
    if (segmentIntersectsPolygon(a, b, poly)) return true;
  }
  return false;
}