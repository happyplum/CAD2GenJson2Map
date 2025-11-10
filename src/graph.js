/**
 * 路网图构建模块
 * 
 * 该模块提供从GeoJSON数据构建路网图的功能，以及空间索引的构建和查询。
 * 主要用于将线段数据转换为图结构，并支持障碍物过滤和空间查询。
 * 
 * 主要功能：
 * 1. 从GeoJSON线段数据构建无向图
 * 2. 支持障碍物过滤，移除穿障边和障内节点
 * 3. 构建空间索引用于快速最近邻查询
 * 4. 提供节点和边的邻接表表示
 * 
 * 坐标系统：使用经纬度坐标 [longitude, latitude]
 * 图结构：使用邻接表表示，节点包含ID和坐标，边包含权重(距离)
 */

import { haversineDistance, roundCoordKey } from './geo.js'
import { extractObstaclesFromGeoJSON, segmentIntersectsAnyObstacle, pointInPolygon } from './obstacles.js'

/**
 * 从GeoJSON数据构建路网图
 * 
 * @param {Object} geojson - GeoJSON对象，包含线段和障碍物数据
 * @param {Object} [options={}] - 构建选项
 * @param {number} [options.precision=6] - 坐标精度，用于节点去重
 * @param {boolean} [options.includeObstacles=true] - 是否包含障碍物处理
 * @param {Function} [options.obstacleClassifier] - 自定义障碍物分类器
 * @returns {Object} 构建好的图对象，包含nodes(节点数组)、adjacency(邻接表)、nodeByKey(节点映射)和obstacles(障碍物数组)
 */
export function buildGraph(geojson, options = {}) {
  // 解析选项参数
  const precision = options.precision ?? 6;
  const includeObstacles = options.includeObstacles ?? true;
  
  // 初始化图数据结构
  const nodeByKey = new Map(); // 坐标键到节点ID的映射，用于节点去重
  const nodes = []; // 节点数组，每个节点包含 { id, lon, lat, key }
  const adjacency = []; // 邻接表，adjacency[id] = [{ to, w }]
  
  // 提取障碍物多边形
  const obstacles = includeObstacles ? extractObstaclesFromGeoJSON(geojson, options) : [];

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

  // 使用障碍物过滤图：移除穿过障碍物的边和位于障碍物内部的节点
  if (obstacles.length) {
    // 标记位于障碍物内部的节点
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
    
    // 过滤邻接表，移除穿障边和连接到障碍物内部节点的边
    for (let aId = 0; aId < adjacency.length; aId++) {
      const list = adjacency[aId] || [];
      const a = nodes[aId];
      const filtered = [];
      
      // 检查每条边是否穿过障碍物或连接到障碍物内部节点
      for (const { to: bId, w } of list) {
        // 如果任一端点在障碍物内部，跳过该边
        if (blockedNode[aId] || blockedNode[bId]) continue;
        
        const b = nodes[bId];
        // 检查边是否与任何障碍物相交
        const intersects = segmentIntersectsAnyObstacle([a.lon, a.lat], [b.lon, b.lat], obstacles);
        if (!intersects) filtered.push({ to: bId, w });
      }
      adjacency[aId] = filtered;
    }
  }

  // 返回构建好的图对象
  return { nodes, adjacency, nodeByKey, obstacles };
}

/**
 * 构建空间索引用于快速最近邻查询
 * 使用网格分桶方法将节点分配到不同的地理网格中
 * 
 * @param {Array<Object>} nodes - 节点数组，每个节点包含 { id, lon, lat }
 * @param {number} [cellDeg=0.01] - 网格单元大小(度)，默认约1km
 * @returns {Object} 空间索引对象，包含nearest方法用于查询最近节点
 */
export function buildSpatialIndex(nodes, cellDeg = 0.01) {
  // 创建网格桶映射，键为网格坐标，值为该网格中的节点ID数组
  const buckets = new Map();
  
  /**
   * 计算给定经纬度坐标对应的网格键
   * 
   * @param {number} lon - 经度
   * @param {number} lat - 纬度
   * @returns {string} 网格键，格式为"ix:iy"
   */
  function bucketKey(lon, lat) {
    // 将经纬度转换为网格坐标
    const ix = Math.floor((lon + 180) / cellDeg);
    const iy = Math.floor((lat + 90) / cellDeg);
    return `${ix}:${iy}`;
  }
  
  // 将所有节点分配到对应的网格中
  for (const n of nodes) {
    const key = bucketKey(n.lon, n.lat);
    let arr = buckets.get(key);
    if (!arr) {
      arr = [];
      buckets.set(key, arr);
    }
    arr.push(n.id);
  }
  
  /**
   * 获取指定网格坐标周围指定半径内的所有节点ID
   * 
   * @param {number} ix - 中心网格的x坐标
   * @param {number} iy - 中心网格的y坐标
   * @param {number} radius - 搜索半径(网格数)
   * @returns {Array<number>} 节点ID数组
   */
  function neighbors(ix, iy, radius) {
    const ids = [];
    // 遍历周围所有网格
    for (let dx = -radius; dx <= radius; dx++) {
      for (let dy = -radius; dy <= radius; dy++) {
        const key = `${ix + dx}:${iy + dy}`;
        const arr = buckets.get(key);
        if (arr) ids.push(...arr);
      }
    }
    return ids;
  }
  
  // 返回空间索引对象
  return {
    /**
     * 查找距离给定经纬度坐标最近的节点
     * 使用渐进式扩展搜索半径的方法提高效率
     * 
     * @param {number} lon - 查询点的经度
     * @param {number} lat - 查询点的纬度
     * @param {number} [expandMax=10] - 最大扩展半径(网格数)
     * @returns {number} 最近节点的ID，如果没有节点则返回-1
     */
    nearest(lon, lat, expandMax = 10) {
      // 计算查询点所在的网格坐标
      const ix = Math.floor((lon + 180) / cellDeg);
      const iy = Math.floor((lat + 90) / cellDeg);
      
      let bestId = -1;
      let bestDist = Infinity;
      
      // 渐进式扩展搜索半径，直到找到节点或达到最大半径
      const maxCap = Math.max(expandMax, 30);
      for (let r = 0; r <= maxCap; r++) {
        const candidates = neighbors(ix, iy, r);
        if (candidates.length === 0) continue;
        
        // 在当前半径内的所有候选节点中查找最近的
        for (const id of candidates) {
          const n = nodes[id];
          const d = Math.hypot(n.lon - lon, n.lat - lat);
          if (d < bestDist) {
            bestDist = d;
            bestId = id;
          }
        }
        
        // 如果在当前半径内找到节点，停止搜索
        if (bestId !== -1) break;
      }
      
      // 如果在扩展搜索中仍未找到节点，进行全局扫描
      // 这种情况发生在节点非常稀疏或查询点远离所有节点时
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
    cellDeg, // 导出网格单元大小，供外部使用
  };
}