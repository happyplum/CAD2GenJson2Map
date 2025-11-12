/**
 * 障碍图构建模块
 * 
 * 该模块提供从GeoJSON数据构建障碍图的功能，以及空间索引的构建和查询。
 * 主要用于将线段数据转换为障碍图结构，识别障碍物并构建可通行区域图。
 * 
 * 主要功能：
 * 1. 从GeoJSON线段数据构建障碍图
 * 2. 识别和提取障碍物多边形
 * 3. 构建可通行区域的图结构
 * 4. 支持空间索引用于快速最近邻查询
 * 5. 提供节点和边的邻接表表示
 * 
 * 坐标系统：使用经纬度坐标 [longitude, latitude]
 * 图结构：使用邻接表表示，节点包含ID和坐标，边包含权重(距离)
 */

import { haversineDistance, roundCoordKey } from './geo.js'
import { extractObstaclesFromGeoJSON, segmentIntersectsPolygon, pointInPolygon } from './obstacles.js'

/**
 * 从GeoJSON数据构建障碍图
 * 
 * @param {Object} geojson - GeoJSON对象，包含线段和障碍物数据
 * @param {Object} [options={}] - 构建选项
 * @param {number} [options.precision=6] - 坐标精度，用于节点去重
 * @param {boolean} [options.includeObstacles=true] - 是否包含障碍物处理
 * @param {Function} [options.obstacleClassifier] - 自定义障碍物分类器
 * @returns {Object} 构建好的障碍图对象，包含nodes(节点数组)、adjacency(邻接表)、nodeByKey(节点映射)和obstacles(障碍物数组)
 */
export async function buildObstacleGraph(geojson, options = {}) {
  // 解析选项参数
  const precision = options.precision ?? 6;
  const includeObstacles = options.includeObstacles ?? true;
  const filterEdges = options.filterEdges ?? true;
  
  // 初始化图数据结构
  const nodeByKey = new Map(); // 坐标键到节点ID的映射，用于节点去重
  const nodes = []; // 节点数组，每个节点包含 { id, lon, lat, key }
  const adjacency = []; // 邻接表，adjacency[id] = [{ to, w }]
  
  // 提取障碍物多边形
  const obstacles = includeObstacles ? extractObstaclesFromGeoJSON(geojson, options) : [];

  // 预计算每个障碍的包围盒，用于快速剔除
  function bboxOfRings(rings) {
    let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
    for (const ring of rings) {
      for (const [lon, lat] of ring) {
        if (lon < minLon) minLon = lon;
        if (lon > maxLon) maxLon = lon;
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;
      }
    }
    return { minLon, minLat, maxLon, maxLat };
  }
  const obstaclesMeta = obstacles.map(rings => ({ rings, bbox: bboxOfRings(rings) }));

  function segBbox(ax, ay, bx, by) {
    return {
      minLon: Math.min(ax, bx),
      maxLon: Math.max(ax, bx),
      minLat: Math.min(ay, by),
      maxLat: Math.max(ay, by),
    };
  }
  function bboxIntersects(a, b) {
    return !(a.minLon > b.maxLon || a.maxLon < b.minLon || a.minLat > b.maxLat || a.maxLat < b.minLat);
  }

  /**
   * 添加节点到图中
   * 如果节点已存在则返回现有ID，否则创建新节点
   * 
   * @param {Array<number>} coord - 节点坐标 [经度, 纬度]
   * @returns {number} 节点ID
   */
  function addNode(coord) {
    // 将坐标四舍五入到指定精度，用于节点去重
    const key = roundCoordKey(coord, precision);
    let id = nodeByKey.get(key);
    
    // 如果节点不存在，创建新节点
    if (id === undefined) {
      id = nodes.length;
      nodeByKey.set(key, id);
      nodes.push({ id, lon: coord[0], lat: coord[1], key });
      adjacency[id] = [];
    }
    return id;
  }

  /**
   * 添加无向边到图中
   * 
   * @param {number} aId - 起始节点ID
   * @param {number} bId - 目标节点ID
   * @param {number} w - 边的权重(距离)
   */
  function addUndirectedEdge(aId, bId, w) {
    // 忽略自环和无效权重
    if (aId === bId || !isFinite(w)) return;
    
    // 添加双向边
    adjacency[aId].push({ to: bId, w });
    adjacency[bId].push({ to: aId, w });
  }

  // 处理GeoJSON中的线段特征
  const features = geojson?.features ?? [];
  for (const f of features) {
    const g = f.geometry;
    if (!g) continue;
    
    // 处理LineString类型的几何对象
    if (g.type === 'LineString') {
      const coords = g.coordinates;
      // 遍历线段中的每对连续点，创建节点和边
      for (let i = 0; i < coords.length - 1; i++) {
        const aId = addNode(coords[i]);
        const bId = addNode(coords[i + 1]);
        const w = haversineDistance(coords[i], coords[i + 1]);
        addUndirectedEdge(aId, bId, w);
      }
    } 
    // 处理MultiLineString类型的几何对象
    else if (g.type === 'MultiLineString') {
      for (const line of g.coordinates) {
        // 遍历每条线段中的每对连续点，创建节点和边
        for (let i = 0; i < line.length - 1; i++) {
          const aId = addNode(line[i]);
          const bId = addNode(line[i + 1]);
          const w = haversineDistance(line[i], line[i + 1]);
          addUndirectedEdge(aId, bId, w);
        }
      }
    }
  }

  // 构建障碍图：识别障碍物并构建可通行区域图
  if (obstacles.length && filterEdges) {
    // 标记位于障碍物内部的节点（这些节点不可通行）
    const blockedNode = new Uint8Array(nodes.length);
    for (let i = 0; i < nodes.length; i++) {
      const n = nodes[i];
      // 检查节点是否在任何障碍物多边形内部
      let isInside = false;
      for (const poly of obstacles) {
        if (pointInPolygon(n.lon, n.lat, poly)) {
          isInside = true;
          break;
        }
      }
      if (isInside) blockedNode[i] = 1;
    }
    
    // 过滤邻接表，构建可通行区域图：移除穿障边和障碍物内部节点
    for (let aId = 0; aId < adjacency.length; aId++) {
      const list = adjacency[aId] || [];
      const a = nodes[aId];
      const filtered = [];

      // 检查每条边是否穿过障碍物或连接到障碍物内部节点
      for (const { to: bId, w } of list) {
        // 如果任一端点在障碍物内部，跳过该边（不可通行）
        if (blockedNode[aId] || blockedNode[bId]) continue;

        const b = nodes[bId];
        // 先用包围盒快速剔除，再做几何相交
        const sb = segBbox(a.lon, a.lat, b.lon, b.lat);
        let maybe = false;
        for (const om of obstaclesMeta) {
          if (bboxIntersects(sb, om.bbox)) { maybe = true; break; }
        }
        if (!maybe) {
          filtered.push({ to: bId, w });
        } else {
          let intersects = false;
          for (const om of obstaclesMeta) {
            if (!bboxIntersects(sb, om.bbox)) continue;
            if (segmentIntersectsPolygon([a.lon, a.lat], [b.lon, b.lat], om.rings)) { intersects = true; break; }
          }
          if (!intersects) filtered.push({ to: bId, w });
        }
      }
      adjacency[aId] = filtered;
      if (aId % 200 === 0) await Promise.resolve();
    }
  }

  // 返回构建好的图对象
  return { nodes, adjacency, nodeByKey, obstacles };
}
