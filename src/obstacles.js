/**
 * 障碍物处理模块
 * 
 * 该模块提供从GeoJSON数据中提取障碍物多边形的功能，以及与障碍物相关的几何计算。
 * 主要用于路网构建过程中过滤掉穿过障碍物的边和移除位于障碍物内部的节点。
 * 
 * 主要功能：
 * 1. 从GeoJSON特征集合中提取障碍物多边形
 * 2. 判断点是否在多边形内部
 * 3. 判断线段是否与多边形相交
 * 4. 判断线段是否与任何障碍物相交
 * 
 * 坐标系统：使用经纬度坐标 [longitude, latitude]
 * 多边形表示：多边形由环(rings)组成，每个环是[lon, lat]坐标数组
 */

// Utilities for obstacle parsing and feasibility checks on GeoJSON
// Polygons are represented as arrays of rings; each ring is an array of [lon, lat] coordinates.

/**
 * 从GeoJSON特征集合中提取障碍物多边形
 * 
 * @param {Object} geojson - GeoJSON格式的数据，包含features数组
 * @param {Object} options - 可选配置项
 * @param {Function} options.isObstacle - 自定义障碍物判断函数，接收feature参数，返回boolean
 * @returns {Array} 障碍物多边形数组，每个障碍物是一个多边形环数组
 * 
 * 障碍物识别标准：
 * - properties.walkable === false
 * - properties.blocked === true
 * - properties.type === 'obstacle' (不区分大小写)
 * - properties.obstacle === true
 */
export function extractObstaclesFromGeoJSON(geojson, options = {}) {
  const features = geojson?.features ?? [];
  const isObstacle = options.isObstacle ?? defaultObstacleClassifier;
  const obstacles = [];
  
  // 遍历所有特征，提取符合条件的障碍物多边形
  for (const f of features) {
    if (!f || !f.geometry) continue;
    if (!isObstacle(f)) continue;
    const g = f.geometry;
    
    // 处理单个多边形
    if (g.type === 'Polygon') {
      const rings = normalizePolygonRings(g.coordinates);
      if (rings.length) obstacles.push(rings);
    } 
    // 处理多个多边形
    else if (g.type === 'MultiPolygon') {
      for (const poly of g.coordinates) {
        const rings = normalizePolygonRings(poly);
        if (rings.length) obstacles.push(rings);
      }
    }
  }
  return obstacles;
}

/**
 * 默认的障碍物分类器
 * 根据特征的属性判断是否为障碍物
 * 
 * @param {Object} f - GeoJSON特征对象
 * @returns {boolean} 如果是障碍物返回true，否则返回false
 */
function defaultObstacleClassifier(f) {
  const p = f?.properties || {};
  const type = String(p.type || '').toLowerCase();
  return p.walkable === false || p.blocked === true || p.obstacle === true || type === 'obstacle';
}

/**
 * 标准化多边形环，确保每个环都是闭合的
 * 
 * @param {Array} coords - 多边形坐标数组，可以是多个环
 * @returns {Array} 标准化后的环数组，每个环都是闭合的
 * 
 * 坐标格式：[ring1, ring2, ...]，每个环是[lon, lat]坐标数组
 */
function normalizePolygonRings(coords) {
  const rings = [];
  for (const ring of coords || []) {
    if (!Array.isArray(ring) || ring.length < 3) continue;
    
    // 检查环是否闭合，如果首尾点不同，则添加首点到末尾
    const last = ring[ring.length - 1];
    const first = ring[0];
    const isClosed = first && last && first[0] === last[0] && first[1] === last[1];
    rings.push(isClosed ? ring : [...ring, first]);
  }
  return rings;
}

/**
 * 使用射线投射算法判断点是否在多边形内部
 * 支持带孔洞的多边形，任何环(外环或内环)都视为不可通行区域
 * 
 * @param {number} lon - 点的经度
 * @param {number} lat - 点的纬度
 * @param {Array} polygonRings - 多边形环数组，每个环是[lon, lat]坐标数组
 * @returns {boolean} 如果点在任何环内返回true，否则返回false
 */
export function pointInPolygon(lon, lat, polygonRings) {
  for (const ring of polygonRings) {
    if (pointInRing(lon, lat, ring)) return true;
  }
  return false;
}

/**
 * 判断点是否在单个多边形环内部
 * 使用射线投射算法(Ray Casting Algorithm)
 * 
 * @param {number} lon - 点的经度
 * @param {number} lat - 点的纬度
 * @param {Array} ring - 多边形环，是[lon, lat]坐标数组
 * @returns {boolean} 如果点在环内返回true，否则返回false
 */
function pointInRing(lon, lat, ring) {
  let inside = false;
  // 遍历环的所有边，从每个顶点向下一个顶点发射射线
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const xi = ring[i][0], yi = ring[i][1];
    const xj = ring[j][0], yj = ring[j][1];
    
    // 检查射线是否与边相交
    const intersect = ((yi > lat) !== (yj > lat)) &&
      (lon < (xj - xi) * (lat - yi) / ((yj - yi) || 1e-12) + xi);
    
    // 每次相交就翻转状态
    if (intersect) inside = !inside;
  }
  return inside;
}

/**
 * 判断线段是否与多边形相交
 * 检查两种情况：
 * 1. 线段的任一端点位于多边形内部
 * 2. 线段与多边形的任何边相交
 * 
 * @param {Array} a - 线段起点坐标 [lon, lat]
 * @param {Array} b - 线段终点坐标 [lon, lat]
 * @param {Array} polygonRings - 多边形环数组，每个环是[lon, lat]坐标数组
 * @returns {boolean} 如果线段与多边形相交返回true，否则返回false
 */
export function segmentIntersectsPolygon(a, b, polygonRings) {
  const [ax, ay] = a, [bx, by] = b;
  
  // 首先检查端点是否在多边形内部
  if (pointInPolygon(ax, ay, polygonRings) || pointInPolygon(bx, by, polygonRings)) return true;
  
  // 检查线段是否与多边形的任何边相交
  for (const ring of polygonRings) {
    for (let i = 0; i < ring.length - 1; i++) {
      const c = ring[i];
      const d = ring[i + 1];
      if (segmentsIntersect(ax, ay, bx, by, c[0], c[1], d[0], d[1])) return true;
    }
  }
  return false;
}

/**
 * 判断两条线段是否相交
 * 使用方向法和共线检查算法
 * 
 * @param {number} ax - 第一条线段起点的x坐标(经度)
 * @param {number} ay - 第一条线段起点的y坐标(纬度)
 * @param {number} bx - 第一条线段终点的x坐标(经度)
 * @param {number} by - 第一条线段终点的y坐标(纬度)
 * @param {number} cx - 第二条线段起点的x坐标(经度)
 * @param {number} cy - 第二条线段起点的y坐标(纬度)
 * @param {number} dx - 第二条线段终点的x坐标(经度)
 * @param {number} dy - 第二条线段终点的y坐标(纬度)
 * @returns {boolean} 如果两条线段相交返回true，否则返回false
 */
function segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
  // 计算方向：三点(abc, abd, cda, cdb)的方向关系
  const o1 = orientation(ax, ay, bx, by, cx, cy);
  const o2 = orientation(ax, ay, bx, by, dx, dy);
  const o3 = orientation(cx, cy, dx, dy, ax, ay);
  const o4 = orientation(cx, cy, dx, dy, bx, by);
  
  // 一般情况：方向不同表示相交
  if (o1 !== o2 && o3 !== o4) return true;
  
  // 特殊情况：共线情况
  if (o1 === 0 && onSegment(ax, ay, cx, cy, bx, by)) return true;
  if (o2 === 0 && onSegment(ax, ay, dx, dy, bx, by)) return true;
  if (o3 === 0 && onSegment(cx, cy, ax, ay, dx, dy)) return true;
  if (o4 === 0 && onSegment(cx, cy, bx, by, dx, dy)) return true;
  
  return false;
}

/**
 * 计算三点的方向关系
 * 
 * @param {number} ax - 点a的x坐标
 * @param {number} ay - 点a的y坐标
 * @param {number} bx - 点b的x坐标
 * @param {number} by - 点b的y坐标
 * @param {number} cx - 点c的x坐标
 * @param {number} cy - 点c的y坐标
 * @returns {number} 0表示共线，1表示顺时针，2表示逆时针
 */
function orientation(ax, ay, bx, by, cx, cy) {
  const val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  if (Math.abs(val) < 1e-12) return 0; // 共线
  return val > 0 ? 1 : 2; // 1: 顺时针, 2: 逆时针
}

/**
 * 检查点c是否在线段ab上
 * 
 * @param {number} ax - 线段起点的x坐标
 * @param {number} ay - 线段起点的y坐标
 * @param {number} cx - 检查点的x坐标
 * @param {number} cy - 检查点的y坐标
 * @param {number} bx - 线段终点的x坐标
 * @param {number} by - 线段终点的y坐标
 * @returns {boolean} 如果点c在线段ab上返回true，否则返回false
 */
function onSegment(ax, ay, cx, cy, bx, by) {
  return Math.min(ax, bx) - 1e-12 <= cx && cx <= Math.max(ax, bx) + 1e-12 &&
         Math.min(ay, by) - 1e-12 <= cy && cy <= Math.max(ay, by) + 1e-12;
}

/**
 * 判断线段是否与任何障碍物相交
 * 遍历所有障碍物多边形，检查线段是否与其中任何一个相交
 * 
 * @param {Array} a - 线段起点坐标 [lon, lat]
 * @param {Array} b - 线段终点坐标 [lon, lat]
 * @param {Array} obstacles - 障碍物多边形数组，每个障碍物是多边形环数组
 * @returns {boolean} 如果线段与任何障碍物相交返回true，否则返回false
 */
export function segmentIntersectsAnyObstacle(a, b, obstacles) {
  for (const poly of obstacles || []) {
    if (segmentIntersectsPolygon(a, b, poly)) return true;
  }
  return false;
}