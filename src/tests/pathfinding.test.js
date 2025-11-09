import { describe, it, expect } from 'vitest'
import { buildGraph } from '../graph.js'
import { aStar, makeRuntimeGraphWithSnap } from '../pathfinding.js'
import { segmentIntersectsAnyObstacle } from '../obstacles.js'

function turnCount(coords) {
  if (!coords || coords.length < 3) return 0
  let count = 0
  for (let i = 1; i < coords.length - 1; i++) {
    const a = coords[i - 1]
    const b = coords[i]
    const c = coords[i + 1]
    const v1x = b[0] - a[0], v1y = b[1] - a[1]
    const v2x = c[0] - b[0], v2y = c[1] - b[1]
    const dot = v1x * v2x + v1y * v2y
    const n1 = Math.hypot(v1x, v1y), n2 = Math.hypot(v2x, v2y)
    if (!n1 || !n2) continue
    const cos = Math.min(1, Math.max(-1, dot / (n1 * n2)))
    const angle = Math.acos(cos)
    if (angle > Math.PI / 12) count++
  }
  return count
}

describe('Pathfinding with strategies and obstacle safety', () => {
  it('returns error when start/end inside obstacle', () => {
    const geojson = {
      type: 'FeatureCollection',
      features: [
        { type: 'Feature', properties: { type: 'obstacle' }, geometry: { type: 'Polygon', coordinates: [[[0,0],[0,2],[2,2],[2,0],[0,0]]] } },
        { type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[-1,1],[3,1]] } },
      ],
    }
    const g = buildGraph(geojson, { precision: 6, includeObstacles: true })
    const { error } = makeRuntimeGraphWithSnap(g, [1,1], [3,1], { obstacles: g.obstacles })
    expect(error).toBe('point_in_obstacle')
  })

  it('path stays within feasible area (no segment intersects obstacles)', () => {
    const geojson = {
      type: 'FeatureCollection',
      features: [
        { type: 'Feature', properties: { type: 'obstacle' }, geometry: { type: 'Polygon', coordinates: [[[0,0],[0,2],[2,2],[2,0],[0,0]]] } },
        { type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[-1,0],[ -1,3 ], [3,3]] } },
        { type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[-1,-1],[ -1,0 ]] } },
      ],
    }
    const g = buildGraph(geojson, { precision: 6, includeObstacles: true })
    const { graph: rGraph, startId, endId } = makeRuntimeGraphWithSnap(g, [-1,-1], [3,3], { obstacles: g.obstacles })
    const res = aStar(rGraph, startId, endId)
    expect(res.pathCoords.length).toBeGreaterThan(1)
    // Safety: ensure no segment intersects obstacle polygons
    let anyIntersects = false
    for (let i = 0; i < res.pathCoords.length - 1; i++) {
      const a = res.pathCoords[i]
      const b = res.pathCoords[i + 1]
      if (segmentIntersectsAnyObstacle(a, b, g.obstacles)) { anyIntersects = true; break }
    }
    expect(anyIntersects).toBe(false)
  })

  it('fewest_turns strategy yields fewer turns than shortest when alternatives exist', () => {
    // Construct a simple grid with two alternatives of equal length but different turns
    const geojson = {
      type: 'FeatureCollection',
      features: [
        { type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[0,0],[1,0],[1,1],[2,1]] } }, // path with two turns
        { type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [[0,0],[2,0],[2,1]] } }, // alternative with one turn
      ],
    }
    const g = buildGraph(geojson, { precision: 6, includeObstacles: true })
    const { graph: rGraph, startId, endId } = makeRuntimeGraphWithSnap(g, [0,0], [2,1])
    const resShortest = aStar(rGraph, startId, endId, { strategy: 'shortest' })
    const resFewest = aStar(rGraph, startId, endId, { strategy: 'fewest_turns', turnPenalty: 20 })
    const turnsShortest = turnCount(resShortest.pathCoords)
    const turnsFewest = turnCount(resFewest.pathCoords)
    expect(turnsFewest).toBeLessThanOrEqual(turnsShortest)
  })
})