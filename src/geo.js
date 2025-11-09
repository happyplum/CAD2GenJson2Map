// Geographic utilities: haversine distance and simple projection
const R = 6371000; // Earth radius in meters

export function haversineDistance(a, b) {
  const [lon1, lat1] = a;
  const [lon2, lat2] = b;
  const toRad = Math.PI / 180;
  const φ1 = lat1 * toRad;
  const φ2 = lat2 * toRad;
  const Δφ = (lat2 - lat1) * toRad;
  const Δλ = (lon2 - lon1) * toRad;
  const s = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  return 2 * R * Math.asin(Math.min(1, Math.sqrt(s)));
}

export function roundCoordKey([lon, lat], precision = 6) {
  return `${lon.toFixed(precision)},${lat.toFixed(precision)}`;
}

// Equirectangular projection (sufficient for small area visualization)
export function projectXY([lon, lat]) {
  return { x: lon, y: lat };
}

export function bboxFromNodes(nodes) {
  let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
  for (const n of nodes) {
    if (n.lon < minLon) minLon = n.lon;
    if (n.lon > maxLon) maxLon = n.lon;
    if (n.lat < minLat) minLat = n.lat;
    if (n.lat > maxLat) maxLat = n.lat;
  }
  return { minLon, minLat, maxLon, maxLat };
}

export function fitToCanvas(bbox, width, height, padding = 20) {
  const w = width - padding * 2;
  const h = height - padding * 2;
  const lonSpan = bbox.maxLon - bbox.minLon || 1;
  const latSpan = bbox.maxLat - bbox.minLat || 1;
  const scale = Math.min(w / lonSpan, h / latSpan);
  const tx = padding - bbox.minLon * scale;
  const ty = padding - bbox.minLat * scale;
  return { scale, tx, ty };
}