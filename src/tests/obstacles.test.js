import { describe, it, expect } from 'vitest'
import { buildGraph } from '../graph.js'
import { extractObstaclesFromGeoJSON, coordInAnyObstacle, segmentIntersectsAnyObstacle } from '../obstacles.js'

describe('Obstacles parsing and graph filtering', () => {
  const squareObstacle = {
    type: 'Feature',
    properties: { type: 'obstacle' },
    geometry: {
      type: 'Polygon',
      coordinates: [
        [ [0,0], [0,2], [2,2], [2,0], [0,0] ]
      ]
    }
  }

  const crossingLines = {
    type: 'Feature',
    properties: {},
    geometry: {
      type: 'LineString',
      coordinates: [ [-1,1], [3,1] ] // horizontal line through obstacle
    }
  }

  const geojson = { type: 'FeatureCollection', features: [squareObstacle, crossingLines] }

  it('extracts obstacle polygons from GeoJSON', () => {
    const obstacles = extractObstaclesFromGeoJSON(geojson)
    expect(obstacles.length).toBe(1)
    expect(coordInAnyObstacle([1,1], obstacles)).toBe(true) // inside
    expect(coordInAnyObstacle([-1,1], obstacles)).toBe(false) // outside
  })

  it('filters edges crossing obstacles', () => {
    const g = buildGraph(geojson, { precision: 6, includeObstacles: true })
    // adjacency on both sides should not connect through the obstacle interior
    // Check that no edge connects a node left of obstacle to a node right of obstacle directly
    let hasCrossing = false
    for (let aId = 0; aId < g.nodes.length; aId++) {
      const a = g.nodes[aId]
      for (const { to: bId } of g.adjacency[aId] || []) {
        const b = g.nodes[bId]
        if (segmentIntersectsAnyObstacle([a.lon, a.lat], [b.lon, b.lat], g.obstacles)) {
          hasCrossing = true
          break
        }
      }
      if (hasCrossing) break
    }
    expect(hasCrossing).toBe(false)
  })
})