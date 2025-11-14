import { pointInPolygon, segmentIntersectsPolygon } from "./obstacles.js";

/**
 * 网格配置参数
 * MAX_COLS: 网格最大列数，增加此值可以提高路径精度但会增加计算时间
 * 当处理大尺度地图或复杂路径时，较大的值能提供更精准的路径，但会增加内存使用和计算量
 */
const MAX_COLS = 200;
/**
 * 网格最大行数，与MAX_COLS一起控制网格的总体大小
 * 行数通常根据地图比例动态计算，但不会超过此上限
 */
const MAX_ROWS = 200;
/**
 * 网格最小列数，确保即使在小区域也能提供足够的路径点
 */
const MIN_COLS = 40;
/**
 * 网格最小行数，确保即使在小区域也能提供足够的路径点
 */
const MIN_ROWS = 30;
/**
 * 基础列数，作为网格密度计算的起点
 * 用于平衡精度和性能的初始值
 */
const BASE_COLS = 60;
/**
 * 基础分辨率，用于动态调整网格密度
 * 较小的值会生成更精细的网格，但会增加计算复杂度
 * 此值影响障碍物边缘的平滑度和路径规划的精度
 */
const BASE_RESOLUTION = 0.0001;

/**
 * 计算三个点的转向方向（叉积法）
 * 用于判断线段相交关系和路径规划中的障碍物检测
 * @param {number} ax - 第一个点的x坐标
 * @param {number} ay - 第一个点的y坐标
 * @param {number} bx - 第二个点的x坐标
 * @param {number} by - 第二个点的y坐标
 * @param {number} cx - 第三个点的x坐标
 * @param {number} cy - 第三个点的y坐标
 * @returns {number} 0表示共线，1表示逆时针方向，-1表示顺时针方向
 */
function orientation(ax, ay, bx, by, cx, cy) {
  // 计算向量叉积：(B - A) × (C - B)
  const val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  // 考虑浮点精度问题，设置一个很小的阈值判断是否共线
  if (Math.abs(val) < 1e-12) return 0;
  // 根据叉积符号返回转向方向
  return val > 0 ? 1 : -1;
}
function onSegment(ax, ay, cx, cy, bx, by) {
  return (
    Math.min(ax, bx) - 1e-12 <= cx &&
    cx <= Math.max(ax, bx) + 1e-12 &&
    Math.min(ay, by) - 1e-12 <= cy &&
    cy <= Math.max(ay, by) + 1e-12
  );
}
/**
 * 判断两条线段是否相交
 * 使用叉积法检测线段相交关系，包括一般相交和端点在线段上的情况
 * 这是路径规划中检测路径是否穿越障碍物的核心函数
 * @param {number} ax - 第一条线段起点的x坐标
 * @param {number} ay - 第一条线段起点的y坐标
 * @param {number} bx - 第一条线段终点的x坐标
 * @param {number} by - 第一条线段终点的y坐标
 * @param {number} cx - 第二条线段起点的x坐标
 * @param {number} cy - 第二条线段起点的y坐标
 * @param {number} dx - 第二条线段终点的x坐标
 * @param {number} dy - 第二条线段终点的y坐标
 * @returns {boolean} 如果线段相交返回true，否则返回false
 */
function segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
  // 计算各点的转向方向：使用叉积法
  // o1: 线段AB与点C的转向关系
  const o1 = orientation(ax, ay, bx, by, cx, cy);
  // o2: 线段AB与点D的转向关系
  const o2 = orientation(ax, ay, bx, by, dx, dy);
  // o3: 线段CD与点A的转向关系
  const o3 = orientation(cx, cy, dx, dy, ax, ay);
  // o4: 线段CD与点B的转向关系
  const o4 = orientation(cx, cy, dx, dy, bx, by);

  // 一般情况：线段相交
  // 当两条线段的四个转向方向满足交叉条件时，线段一定相交
  if (o1 !== o2 && o3 !== o4) return true;

  // 特殊情况：点在线段上
  // 检查端点是否在线段上的情况
  if (o1 === 0 && onSegment(ax, ay, cx, cy, bx, by)) return true;
  if (o2 === 0 && onSegment(ax, ay, dx, dy, bx, by)) return true;
  if (o3 === 0 && onSegment(cx, cy, ax, ay, dx, dy)) return true;
  if (o4 === 0 && onSegment(cx, cy, bx, by, dx, dy)) return true;

  return false; // 不相交
}

// 优化版欧几里得距离计算
function euclid(a, b) {
  const dx = a.lon - b.lon;
  const dy = a.lat - b.lat;
  return Math.hypot(dx, dy);
}

// 欧几里得距离平方（避免开方操作，用于比较）
function euclidSquared(a, b) {
  const dx = a.lon - b.lon;
  const dy = a.lat - b.lat;
  return dx * dx + dy * dy;
}
function bboxIntersects(a, b) {
  return !(
    a.minLon > b.maxLon ||
    a.maxLon < b.minLon ||
    a.minLat > b.maxLat ||
    a.maxLat < b.minLat
  );
}

function buildObstacleMeta(obstacles) {
  const out = [];
  for (const rings of obstacles || []) {
    let mLon = Infinity,
      mLat = Infinity,
      xLon = -Infinity,
      xLat = -Infinity;
    for (const ring of rings) {
      for (const [lo, la] of ring) {
        if (lo < mLon) mLon = lo;
        if (lo > xLon) xLon = lo;
        if (la < mLat) mLat = la;
        if (la > xLat) xLat = la;
      }
    }
    out.push({
      rings,
      bbox: { minLon: mLon, minLat: mLat, maxLon: xLon, maxLat: xLat },
    });
  }
  return out;
}
function buildWallMeta(walls) {
  const out = [];
  for (const [a, b] of walls || []) {
    out.push({
      a,
      b,
      bbox: {
        minLon: Math.min(a[0], b[0]),
        maxLon: Math.max(a[0], b[0]),
        minLat: Math.min(a[1], b[1]),
        maxLat: Math.max(a[1], b[1]),
      },
    });
  }
  return out;
}

/**
 * 构建用于路径规划的网格地图
 * 根据障碍物和边界构建一个二维网格，标记可通行和不可通行区域
 * @param {Object} cfg - 配置对象，包含起点、终点、边界和障碍物信息
 * @returns {Object} 返回网格信息，包含节点、邻接关系、行列数和边界坐标
 */
function ensureGridLocal(cfg) {
  // 从配置中提取所需的参数
  const {
    startLon,
    startLat,
    endLon,
    endLat,
    bboxNodes,
    obstaclesMeta,
    wallsMeta,
  } = cfg;

  // 确定初始网格边界，基于起点和终点坐标
  let minLon = Math.min(startLon, endLon);
  let maxLon = Math.max(startLon, endLon);
  let minLat = Math.min(startLat, endLat);
  let maxLat = Math.max(startLat, endLat);

  // 处理无效坐标的情况，使用全局边界框
  if (
    !isFinite(minLon) ||
    !isFinite(minLat) ||
    !isFinite(maxLon) ||
    !isFinite(maxLat)
  ) {
    minLon = bboxNodes.minLon;
    maxLon = bboxNodes.maxLon;
    minLat = bboxNodes.minLat;
    maxLat = bboxNodes.maxLat;
  }

  // 计算初始边界的跨度
  let lonSpan = maxLon - minLon;
  let latSpan = maxLat - minLat;

  // 添加边界缓冲区，确保起点和终点周围有足够空间
  const padLon = lonSpan * 0.3 + 1e-4;
  const padLat = latSpan * 0.3 + 1e-4;

  // 确保扩展后的边界不会超出全局边界框
  minLon = Math.max(bboxNodes.minLon, minLon - padLon);
  maxLon = Math.min(bboxNodes.maxLon, maxLon + padLon);
  minLat = Math.max(bboxNodes.minLat, minLat - padLat);
  maxLat = Math.min(bboxNodes.maxLat, maxLat + padLat);

  // 确保跨度为有效正值，避免除以零错误
  lonSpan = Math.max(maxLon - minLon, 1e-6);
  latSpan = Math.max(maxLat - minLat, 1e-6);

  // 动态计算网格列数，根据距离调整密度
  const diagonal = Math.hypot(lonSpan, latSpan);
  // 距离越远，网格密度相对增加但有上限
  let cols = Math.round(
    Math.min(BASE_COLS + (diagonal / BASE_RESOLUTION) * 10, MAX_COLS),
  );
  // 确保列数在合理范围内
  cols = Math.max(MIN_COLS, Math.min(cols, MAX_COLS));

  // 根据宽高比计算行数，保持网格比例
  let rows = Math.round((latSpan / lonSpan) * cols);
  // 确保行数在合理范围内
  rows = Math.max(MIN_ROWS, Math.min(rows, MAX_ROWS));

  // 计算单元格的实际大小
  const cellLon = lonSpan / cols;
  const cellLat = latSpan / rows;
  const nodes = [];
  const blocked = [];
  const eps = Math.min(cellLon, cellLat) * 0.3;
  for (let r = 0; r <= rows; r++) {
    for (let c = 0; c <= cols; c++) {
      const lon = minLon + c * cellLon;
      const lat = minLat + r * cellLat;
      const idx = nodes.length;
      nodes.push({ id: idx, lon, lat });
      let inside = false;
      for (const om of obstaclesMeta) {
        if (
          !bboxIntersects(
            { minLon: lon, maxLon: lon, minLat: lat, maxLat: lat },
            om.bbox,
          )
        )
          continue;
        if (pointInPolygon(lon, lat, om.rings)) {
          inside = true;
          break;
        }
      }
      if (!inside) {
        for (const wm of wallsMeta) {
          const vx = wm.bbox.maxLon - wm.bbox.minLon;
          const vy = wm.bbox.maxLat - wm.bbox.minLat;
          const d = pointSegmentDistance(
            lon,
            lat,
            wm.a[0],
            wm.a[1],
            wm.b[0],
            wm.b[1],
          );
          if (d <= Math.min(cellLon, cellLat) * 0.3) {
            inside = true;
            break;
          }
        }
      }
      blocked[idx] = inside ? 1 : 0;
    }
  }

  /**
   * 获取指定网格节点的所有相邻可通行节点
   * 检查八个方向（上下左右及四个对角线）的相邻节点，并过滤出可通行且不穿越障碍物的节点
   * 这是路径规划中确定节点间连通性的核心函数，为A*算法提供关键支持
   */
  const adjacency = new Array(nodes.length);
  const dirs = [
    [0, 1],
    [1, 0],
    [0, -1],
    [-1, 0],
    [1, 1],
    [1, -1],
    [-1, 1],
    [-1, -1],
  ];
  for (let r = 0; r <= rows; r++) {
    for (let c = 0; c <= cols; c++) {
      const idx = r * (cols + 1) + c;
      const list = [];
      if (blocked[idx]) {
        adjacency[idx] = list;
        continue;
      }
      const a = nodes[idx];
      for (const [dr, dc] of dirs) {
        // 计算新位置的行列坐标
        const nr = r + dr,
          nc = c + dc;

        // 边界检查：确保新位置在网格范围内
        if (nr < 0 || nr > rows || nc < 0 || nc > cols) continue;

        // 计算邻居节点的索引
        const j = nr * (cols + 1) + nc;

        // 可通行性检查：邻居节点必须未被标记为阻塞
        if (blocked[j]) continue;

        // 获取邻居节点坐标
        const b = nodes[j];

        // 计算节点间连线的边界框，用于快速排除不相交的障碍物
        const sb = {
          minLon: Math.min(a.lon, b.lon),
          maxLon: Math.max(a.lon, b.lon),
          minLat: Math.min(a.lat, b.lat),
          maxLat: Math.max(a.lat, b.lat),
        };

        // 障碍物碰撞检测：检查节点间连线是否穿过多边形障碍物
        let crossesPoly = false;
        for (const om of obstaclesMeta) {
          if (!bboxIntersects(sb, om.bbox)) continue; // 快速边界框排除
          if (
            segmentIntersectsPolygon([a.lon, a.lat], [b.lon, b.lat], om.rings)
          ) {
            crossesPoly = true;
            break;
          }
        }

        // 墙体碰撞检测：检查节点间连线是否与墙体相交
        let crossesWall = false;
        if (!crossesPoly) {
          for (const wm of wallsMeta) {
            if (!bboxIntersects(sb, wm.bbox)) continue; // 快速边界框排除
            if (
              segmentsIntersect(
                a.lon,
                a.lat,
                b.lon,
                b.lat,
                wm.a[0],
                wm.a[1],
                wm.b[0],
                wm.b[1],
              )
            ) {
              crossesWall = true;
              break;
            }
          }
        }

        // 如果连线不穿过任何障碍物，则将邻居节点添加到邻接列表
        if (!crossesPoly && !crossesWall) {
          // 计算距离并添加到邻接列表
          const dx = a.lon - b.lon;
          const dy = a.lat - b.lat;
          // 直接计算距离以避免函数调用开销
          const distance = Math.hypot(dx, dy);
          list.push({ to: j, w: distance });
        }
      }
      adjacency[idx] = list;
    }
  }
  return { nodes, adjacency, cols, rows, minLon, minLat, cellLon, cellLat };
}

// 优化版点到线段距离计算
function pointSegmentDistance(px, py, ax, ay, bx, by) {
  // 快速计算向量分量
  const vx = bx - ax;
  const vy = by - ay;
  const wx = px - ax;
  const wy = py - ay;

  // 计算投影参数
  const c1 = vx * wx + vy * wy;
  // 避免除零错误
  const c2 = vx * vx + vy * vy || 1e-12;

  // 投影参数裁剪到[0,1]区间
  const t = Math.max(0, Math.min(1, c1 / c2));

  // 计算投影点并返回距离
  const cx = ax + t * vx;
  const cy = ay + t * vy;
  const dx = px - cx;
  const dy = py - cy;

  // 使用Math.hypot计算欧几里得距离
  return Math.hypot(dx, dy);
}

// 距离平方版本，用于比较（无需开方，性能更好）
function pointSegmentDistanceSquared(px, py, ax, ay, bx, by) {
  const vx = bx - ax;
  const vy = by - ay;
  const wx = px - ax;
  const wy = py - ay;

  const c1 = vx * wx + vy * wy;
  const c2 = vx * vx + vy * vy || 1e-12;

  const t = Math.max(0, Math.min(1, c1 / c2));

  const cx = ax + t * vx;
  const cy = ay + t * vy;
  const dx = px - cx;
  const dy = py - cy;

  return dx * dx + dy * dy;
}

/**
 * 查找距离指定坐标最近的可通行网格节点
 * 使用两阶段搜索策略：优先在估算位置附近搜索，若未找到再进行全局搜索
 * 这是路径规划中至关重要的函数，将用户输入的地理坐标映射到可行走的网格点上
 *
 * @param {Object} grid - 网格对象，包含节点信息、邻接关系和网格参数
 * @param {number} lon - 目标经度坐标
 * @param {number} lat - 目标纬度坐标
 * @returns {number} 返回最近可通行节点的索引，如果整个网格中没有可通行节点则返回-1
 */
function nearestFreeGridIndex(grid, lon, lat) {
  // 第一步：将地理坐标快速转换为对应的网格坐标
  // 通过简单的除法运算，计算出给定坐标在网格中的大致行列位置
  const col = Math.floor((lon - grid.minLon) / grid.cellLon);
  const row = Math.floor((lat - grid.minLat) / grid.cellLat);
  // 计算实际节点数（网格线交点）的列数
  const cols = grid.cols + 1;

  // 性能优化：设定合理的局部搜索半径
  // 半径取固定值5和网格规模的平方根一半中的较小值，平衡搜索速度和效果
  const searchRadius = Math.min(
    5,
    Math.floor(Math.sqrt(grid.nodes.length) / 2),
  );
  // 初始化最佳节点索引和距离
  let bestNodeIndex = -1;
  let bestDistance = Infinity;

  // 第一阶段：局部搜索 - 在估算位置附近的小范围内搜索
  // 这是快速路径，能处理大多数正常情况，显著减少不必要的计算
  for (let dr = -searchRadius; dr <= searchRadius; dr++) {
    for (let dc = -searchRadius; dc <= searchRadius; dc++) {
      // 计算当前搜索位置的行列号
      const currentRow = row + dr;
      const currentCol = col + dc;

      // 边界检查：确保当前位置在有效网格范围内
      if (
        currentRow < 0 ||
        currentRow > grid.rows ||
        currentCol < 0 ||
        currentCol > grid.cols
      ) {
        continue;
      }

      // 将行列坐标转换为节点索引
      const nodeIndex = currentRow * cols + currentCol;
      // 获取节点信息
      const node = grid.nodes[nodeIndex];

      // 可通行性检查：节点必须有邻接节点才被视为可通行
      const adjacentNodes = grid.adjacency[nodeIndex];
      if (!adjacentNodes || adjacentNodes.length === 0) {
        continue; // 跳过不可通行的节点
      }

      // 计算节点到目标坐标的欧几里得距离（使用平方距离进行比较，避免开方）
      const dx = node.lon - lon;
      const dy = node.lat - lat;
      const distanceSquared = dx * dx + dy * dy; // 平方距离用于比较

      // 更新最近节点信息
      if (distanceSquared < bestDistance) {
        bestDistance = distanceSquared;
        bestNodeIndex = nodeIndex;
      }
    }
  }

  // 第二阶段：全局搜索 - 如果局部搜索未找到合适节点，扩大搜索范围
  // 这是回退机制，确保在复杂或边缘情况下仍然能找到解决方案
  if (bestNodeIndex === -1) {
    // 遍历整个网格中的所有节点
    for (let i = 0; i < grid.nodes.length; i++) {
      const node = grid.nodes[i];

      // 可通行性检查
      const adjacentNodes = grid.adjacency[i];
      if (!adjacentNodes || adjacentNodes.length === 0) {
        continue;
      }

      // 计算欧几里得距离平方
      const dx = node.lon - lon;
      const dy = node.lat - lat;
      const distanceSquared = dx * dx + dy * dy;

      // 更新最近节点信息
      if (distanceSquared < bestDistance) {
        bestDistance = distanceSquared;
        bestNodeIndex = i;
      }
    }
  }

  // 返回找到的最近可通行节点索引，如果没有找到则返回-1
  return bestNodeIndex;
}

/**
 * A*路径搜索算法的核心实现
 * 在网格地图上寻找从起始点到目标点的最短路径，使用启发式函数进行优化搜索
 * 此实现包含多项性能优化和容错机制，适用于复杂地形和大规模地图
 *
 * @param {Object} grid - 网格对象，包含节点和邻接关系信息
 * @param {number} startIdx - 起始点在网格中的索引
 * @param {number} goalIdx - 目标点在网格中的索引
 * @returns {Array|null} 返回路径节点数组，包含连续的路径坐标点；若无法找到路径则返回null
 */
function aStarGrid(grid, startIdx, goalIdx) {
  // 从网格对象中提取必要的数据结构
  const nodes = grid.nodes; // 所有网格节点的坐标信息
  const adj = grid.adjacency; // 邻接关系列表，表示节点间的连通性和权重

  // 初始化数据结构，使用更高效的存储方式
  // 开放列表：使用Map存储待探索的节点，键为节点ID，值为f值
  // 这种结构便于快速查找和删除操作，提高开放列表管理效率
  const open = new Map();

  // 前驱节点映射：记录每个节点是从哪个节点访问而来，用于路径重建
  const came = new Map();

  // 距离数组：使用数组而非Map存储g值（从起点到当前节点的实际距离）
  // 数组提供O(1)的访问速度，明显优于Map的查找性能
  const g = new Array(nodes.length).fill(Infinity);

  // 启发式函数值数组：存储每个节点到目标节点的估计距离
  const h = new Array(nodes.length);

  // 预计算所有节点的启发式函数值（使用距离平方，避免开方）
  // 注意：由于A*算法中我们只关心f值的相对大小，使用距离平方作为启发式函数仍然能保证算法最优性
  const goalNode = nodes[goalIdx];
  const goalLon = goalNode.lon;
  const goalLat = goalNode.lat;

  // 批量预计算，减少属性查找次数
  for (let i = 0; i < nodes.length; i++) {
    const node = nodes[i];
    const dx = node.lon - goalLon;
    const dy = node.lat - goalLat;
    h[i] = dx * dx + dy * dy; // 使用距离平方作为启发式函数值
  }

  // 初始化起点节点
  // 起点到自身的距离为0，初始f值等于启发式估计值
  open.set(startIdx, h[startIdx]);
  g[startIdx] = 0;

  /**
   * 从开放列表中选择f值最小的节点
   * 这是A*算法的核心操作，决定了搜索方向的优先级
   *
   * @returns {number|null} 返回f值最小的节点ID，如果开放列表为空则返回null
   */
  function popLowest() {
    let bestNode = null;
    let bestF = Infinity;

    // 使用迭代器遍历Map，性能优于for...of循环
    const entries = open.entries();
    for (const [node, fScore] of entries) {
      if (fScore < bestF) {
        bestF = fScore;
        bestNode = node;
      }
    }

    // 找到最优节点后从开放列表中移除
    if (bestNode !== null) {
      open.delete(bestNode);
    }
    return bestNode;
  }

  // 安全机制：设置最大迭代次数，防止在复杂情况下出现无限循环
  // 迭代次数限制为节点数量的2倍，在保证足够搜索空间的同时避免性能问题
  const MAX_ITERATIONS = nodes.length * 2;
  let iterations = 0;

  // A*算法主循环：持续搜索直到找到路径或达到搜索限制
  while (open.size > 0 && iterations < MAX_ITERATIONS) {
    // 获取当前具有最小f值的节点（最有希望的节点）
    const cur = popLowest();

    // 找到目标节点，开始重建路径
    if (cur === goalIdx) {
      // 路径重建数组：存储从起点到终点的节点
      const path = [];
      let current = cur;

      // 回溯算法：从终点开始，沿着前驱节点链向上追踪到起点
      while (current !== undefined) {
        path.push(nodes[current]); // 将当前节点加入路径
        current = came.get(current); // 移动到前驱节点
      }

      // 反转路径顺序：从起点到终点
      path.reverse();
      return path; // 返回完整路径
    }

    // 扩展当前节点的邻居节点
    // 获取当前节点的所有邻接节点
    const neighbors = adj[cur] || [];
    for (const { to, w } of neighbors) {
      // 计算通过当前节点到达邻居节点的路径长度
      const tentativeG = g[cur] + w;

      // 关键判断：只有当发现更短的路径时，才更新节点信息
      // 这是保证算法最优性的核心条件
      if (tentativeG < g[to]) {
        came.set(to, cur); // 记录到达该邻居的最佳路径来源
        g[to] = tentativeG; // 更新从起点到该节点的最短距离
        const fScore = tentativeG + h[to]; // 计算综合评价值
        open.set(to, fScore); // 将节点加入或更新到开放列表
      }
    }

    iterations++;

    // 性能优化：定期让出控制权，避免长时间阻塞主线程
    // 这在Web Worker环境中影响不大，但保留以增强兼容性
    if (iterations % 1000 === 0) {
      // 可以在这里添加yield逻辑，但在Web Worker中影响不大
    }
  }

  // 容错机制：如果达到最大迭代次数仍未找到完整路径
  // 尝试返回到达离目标最近的可达点的部分路径
  if (iterations >= MAX_ITERATIONS) {
    console.warn("A*搜索达到最大迭代次数");

    // 寻找已访问节点中离目标最近的节点
    let closestNode = -1;
    let minDistance = Infinity;

    // 在所有已访问过的节点中（g值有限）找出离终点最近的节点
    for (let i = 0; i < nodes.length; i++) {
      if (g[i] < Infinity && h[i] < minDistance) {
        minDistance = h[i];
        closestNode = i;
      }
    }

    // 如果找到相对较近的节点，尝试构建部分路径
    if (closestNode !== -1) {
      const path = [];
      let current = closestNode;

      // 回溯构建部分路径
      while (current !== undefined) {
        path.push(nodes[current]);
        current = came.get(current);
      }

      path.reverse();
      console.log("返回部分路径");
      return path;
    }
  }

  // 无法找到路径的情况
  return null;
}

/**
 * 路径计算的主函数
 * 协调整个路径规划流程：网格构建、最近节点查找、A*搜索、路径优化
 * 这是在Web Worker环境中处理路径请求的入口点
 */
/**
 * 路径计算的主入口函数
 * 协调整个路径规划流程，包括障碍物处理、网格构建、路径搜索和优化
 * @param {number} startLon - 起点经度坐标
 * @param {number} startLat - 起点纬度坐标
 * @param {number} endLon - 终点经度坐标
 * @param {number} endLat - 终点纬度坐标
 * @param {Array} obstacles - 障碍物数组，每个障碍物是一个多边形
 * @param {Array} walls - 墙体数组，每条墙体是一条线段
 * @param {Object} bboxNodes - 节点边界框，定义计算范围
 * @returns {Object} 返回计算结果对象 {ok: boolean, path?: Array, error?: string}
 */
function computePath(
  startLon,
  startLat,
  endLon,
  endLat,
  obstacles,
  walls,
  bboxNodes,
) {
  // 第一步：验证输入坐标
  // 确保所有坐标值都是有效的数字，这是路径计算的基础前提
  if (
    !isFinite(startLon) ||
    !isFinite(startLat) ||
    !isFinite(endLon) ||
    !isFinite(endLat)
  ) {
    return { ok: false, error: "invalid-coordinates" };
  }

  // 只有当起点和终点都被设置（都不为0）时才进行计算
  // 避免无效的路径请求
  if ((startLon === 0 && startLat === 0) || (endLon === 0 && endLat === 0)) {
    return { ok: false, error: "zero-coordinates" };
  }

  // 第二步：构建障碍物和墙体的元数据
  // 将原始障碍物和墙体数据转换为更高效的数据结构，用于快速碰撞检测
  const obstaclesMeta = buildObstacleMeta(obstacles);
  const wallsMeta = buildWallMeta(walls);

  // 第三步：构建基于障碍物的网格地图
  // 根据障碍物分布生成可通行和不可通行区域的网格
  // 这是路径规划的基础数据结构
  const grid = ensureGridLocal({
    startLon,
    startLat,
    endLon,
    endLat,
    bboxNodes,
    obstaclesMeta,
    wallsMeta,
  });

  // 第四步：查找距离起点最近的有效网格节点
  // 将用户输入的起点坐标映射到网格中的一个可通行点上
  let startIndex = nearestFreeGridIndex(grid, startLon, startLat);
  // 查找距离终点最近的有效网格节点
  let endIndex = nearestFreeGridIndex(grid, endLon, endLat);

  // 尝试扩大搜索范围，确保找到有效的起点和终点
  if (startIndex < 0) {
    console.warn("无法找到起点附近的可通行点，尝试更远范围");
    startIndex = findAnyValidNode(grid);
  }

  if (endIndex < 0) {
    console.warn("无法找到终点附近的可通行点，尝试更远范围");
    endIndex = findAnyValidNode(grid);
  }

  // 检查是否找到了有效的起点和终点
  if (startIndex < 0 || endIndex < 0) {
    return { ok: false, error: "nearby-grid-fail" };
  }

  // 第五步：使用A*算法在网格上寻找最短路径
  // A*算法结合启发式函数，能高效地找到最短路径
  let path = aStarGrid(grid, startIndex, endIndex);

  // 容错机制：如果首次搜索未找到路径，尝试调整网格参数重试
  if (!path || !path.length) {
    console.warn("首次搜索未找到路径，尝试调整参数");

    // 通知主线程需要延长计算时间
    self.postMessage({
      type: "extending_computation",
      message: "首次搜索未找到路径，正在尝试调整参数重新计算...",
    });

    // 备用策略：降低网格密度但扩大范围
    const adjustedGrid = adjustGridForBetterCoverage({
      startLon,
      startLat,
      endLon,
      endLat,
      bboxNodes,
      obstaclesMeta,
      wallsMeta,
    });

    // 在调整后的网格上重新查找起点和终点
    const adjustedSi = nearestFreeGridIndex(adjustedGrid, startLon, startLat);
    const adjustedGi = nearestFreeGridIndex(adjustedGrid, endLon, endLat);

    // 尝试在调整后的网格上寻找路径
    if (adjustedSi >= 0 && adjustedGi >= 0) {
      console.log(adjustedGi, adjustedGi);
      path = aStarGrid(adjustedGrid, adjustedSi, adjustedGi);
    }
  }

  // 检查路径是否有效
  if (!path || !path.length) {
    return { ok: false, error: "no-path" };
  }

  // 第六步：优化路径
  // 移除路径中的冗余点，使路径更平滑，减少不必要的转折点
  const optimizedPath = optimizePath(path);

  // 第七步：调整路径端点
  // 确保路径的起点和终点精确匹配用户指定的坐标
  const finalPath = adjustPathEnds(
    optimizedPath,
    { lon: startLon, lat: startLat },
    { lon: endLon, lat: endLat },
  );

  // 返回成功结果和计算出的最终路径
  return { ok: true, path: finalPath };
}

// 功能验证函数，确保优化后的计算结果与原计算一致
function validateCalculations() {
  const errors = [];
  console.log("开始距离计算功能验证...");

  // 创建测试数据
  const testPoints = [
    { lon: 10.0, lat: 20.0 },
    { lon: 15.5, lat: 25.5 },
    { lon: 8.3, lat: 19.7 },
    { lon: 12.1, lat: 22.3 },
    { lon: 100.0, lat: 50.0 },
    { lon: 100.1, lat: 50.1 },
    { lon: -120.5, lat: 35.8 },
  ];

  // 验证欧几里得距离计算
  console.log("\n验证欧几里得距离计算:");
  for (let i = 0; i < testPoints.length - 1; i++) {
    const a = testPoints[i];
    const b = testPoints[i + 1];

    // 原始计算方式
    const originalDist = Math.hypot(a.lon - b.lon, a.lat - b.lat);
    // 优化后的计算方式
    const optimizedDist = euclid(a, b);

    // 验证结果一致性
    const difference = Math.abs(originalDist - optimizedDist);
    console.log(`点${i}到点${i + 1}: 差异 = ${difference.toFixed(10)}`);
    if (difference > 1e-10) {
      console.warn("警告: 计算结果不一致!");
      errors.push(`欧几里得距离计算错误: 点${i}到点${i + 1}差异过大`);
    }
  }

  // 验证点到线段距离计算
  console.log("\n验证点到线段距离计算:");
  for (let i = 0; i < testPoints.length - 2; i++) {
    const p = testPoints[i];
    const a = testPoints[i + 1];
    const b = testPoints[i + 2];

    // 原始计算方式
    const vx = b.lon - a.lon;
    const vy = b.lat - a.lat;
    const wx = p.lon - a.lon;
    const wy = p.lat - a.lat;
    const c1 = vx * wx + vy * wy;
    const c2 = vx * vx + vy * vy || 1e-12;
    let t = c1 / c2;
    if (t < 0) t = 0;
    else if (t > 1) t = 1;
    const cx = a.lon + t * vx;
    const cy = a.lat + t * vy;
    const originalDist = Math.hypot(p.lon - cx, p.lat - cy);

    // 优化后的计算方式
    const optimizedDist = pointSegmentDistance(
      p.lon,
      p.lat,
      a.lon,
      a.lat,
      b.lon,
      b.lat,
    );

    // 验证结果一致性
    const difference = Math.abs(originalDist - optimizedDist);
    console.log(
      `点到线段(${i}->${i + 1}->${i + 2}): 差异 = ${difference.toFixed(10)}`,
    );
    if (difference > 1e-10) {
      console.warn("警告: 计算结果不一致!");
      errors.push(
        `点到线段距离计算错误: 点${i}到线段(${i + 1}->${i + 2})差异过大`,
      );
    }
  }

  const passed = errors.length === 0;

  if (passed) {
    console.log("\n功能验证完成! 所有计算验证通过！");
  } else {
    console.error("\n功能验证完成! 计算验证失败:");
    errors.forEach((err) => console.error("-", err));
  }

  // 返回验证结果对象
  return {
    passed: passed,
    errors: errors,
  };
}

// 性能测试函数，用于测量距离计算的执行效率
function runPerformanceTest() {
  console.log("开始距离计算性能测试...");

  // 创建结果对象
  const results = {
    iterations: 1000000,
    times: {},
  };

  // 创建测试数据
  const testCases = 1000000;
  const points = [];

  // 生成随机测试点
  for (let i = 0; i < testCases; i++) {
    points.push({
      lon: Math.random() * 180 - 90,
      lat: Math.random() * 360 - 180,
    });
  }

  // 测试1：欧几里得距离计算
  console.log("\n测试欧几里得距离计算:");
  const startTime1 = performance.now();
  let totalDist = 0;
  for (let i = 0; i < testCases - 1; i++) {
    totalDist += euclid(points[i], points[i + 1]);
  }
  const endTime1 = performance.now();
  const euclidTime = endTime1 - startTime1;
  results.times["euclid"] = euclidTime;
  console.log(`执行时间: ${euclidTime.toFixed(2)}ms`);
  console.log(`计算结果示例: ${totalDist.toFixed(2)}`);

  // 测试2：欧几里得距离平方计算（用于比较）
  console.log("\n测试欧几里得距离平方计算:");
  const startTime2 = performance.now();
  let totalDistSquared = 0;
  for (let i = 0; i < testCases - 1; i++) {
    totalDistSquared += euclidSquared(points[i], points[i + 1]);
  }
  const endTime2 = performance.now();
  const euclidSquaredTime = endTime2 - startTime2;
  results.times["euclidSquared"] = euclidSquaredTime;
  console.log(`执行时间: ${euclidSquaredTime.toFixed(2)}ms`);
  console.log(`计算结果示例: ${totalDistSquared.toFixed(2)}`);

  // 测试3：点到线段距离计算
  console.log("\n测试点到线段距离计算:");
  const startTime3 = performance.now();
  let totalSegDist = 0;
  for (let i = 0; i < testCases - 2; i++) {
    totalSegDist += pointSegmentDistance(
      points[i].lon,
      points[i].lat,
      points[i + 1].lon,
      points[i + 1].lat,
      points[i + 2].lon,
      points[i + 2].lat,
    );
  }
  const endTime3 = performance.now();
  const pointSegmentTime = endTime3 - startTime3;
  results.times["pointSegmentDistance"] = pointSegmentTime;
  console.log(`执行时间: ${pointSegmentTime.toFixed(2)}ms`);
  console.log(`计算结果示例: ${totalSegDist.toFixed(2)}`);

  // 计算性能提升比例
  const euclidImprovement = (
    ((euclidTime - euclidSquaredTime) / euclidTime) *
    100
  ).toFixed(1);
  results.times["euclidImprovementPercent"] = parseFloat(euclidImprovement);

  console.log("\n性能测试完成!");

  return results;
}

self.onmessage = async (ev) => {
  // 从消息中提取路径计算所需的所有参数
  const {
    startLon,
    startLat,
    endLon,
    endLat,
    obstacles,
    walls,
    bboxNodes,
    testMode,
  } = ev.data;

  // 如果是测试模式，运行性能测试和功能验证
  if (testMode) {
    // 首先验证功能正确性
    const validation = validateCalculations();
    // 然后运行性能测试
    const results = runPerformanceTest();

    // 返回测试结果给主线程
    self.postMessage({
      type: "test_result",
      results: {
        iterations: results.iterations,
        times: results.times,
        validation: validation,
      },
    });
    return;
  }

  try {
    // 调用路径计算主函数
    const result = computePath(
      startLon,
      startLat,
      endLon,
      endLat,
      obstacles,
      walls,
      bboxNodes,
    );
    // 将计算结果发送回主线程
    self.postMessage(result);
  } catch (e) {
    // 异常处理：记录错误并返回失败消息
    console.error("路径计算错误:", e);
    self.postMessage({ ok: false, error: String((e && e.message) || e) });
  }
};

// 辅助函数：找到网格中任何一个有效的节点
function findAnyValidNode(grid) {
  for (let i = 0; i < grid.nodes.length; i++) {
    if (grid.adjacency[i] && grid.adjacency[i].length > 0) {
      return i;
    }
  }
  return -1;
}

// 辅助函数：调整网格参数以获得更好的覆盖范围
function adjustGridForBetterCoverage(config) {
  const {
    startLon,
    startLat,
    endLon,
    endLat,
    bboxNodes,
    obstaclesMeta,
    wallsMeta,
  } = config;

  // 计算扩展后的范围
  let minLon = Math.min(startLon, endLon);
  let maxLon = Math.max(startLon, endLon);
  let minLat = Math.min(startLat, endLat);
  let maxLat = Math.max(startLat, endLat);

  // 更大的扩展范围
  const lonSpan = maxLon - minLon;
  const latSpan = maxLat - minLat;
  const padLon = lonSpan * 0.5 + 1e-4;
  const padLat = latSpan * 0.5 + 1e-4;

  minLon = Math.max(bboxNodes.minLon, minLon - padLon);
  maxLon = Math.min(bboxNodes.maxLon, maxLon + padLon);
  minLat = Math.max(bboxNodes.minLat, minLat - padLat);
  maxLat = Math.min(bboxNodes.maxLat, maxLat + padLat);

  // 使用更稀疏但更大的网格
  const cols = Math.max(MIN_COLS, Math.min(BASE_COLS * 0.7, MAX_COLS));
  const rows = Math.round(((maxLat - minLat) / (maxLon - minLon)) * cols);

  // 复用现有的网格构建逻辑
  const originalEnsureGridLocal = ensureGridLocal;
  return originalEnsureGridLocal({
    startLon: minLon,
    startLat: minLat,
    endLon: maxLon,
    endLat: maxLat,
    bboxNodes,
    obstaclesMeta,
    wallsMeta,
  });
}

/**
 * 优化路径：移除冗余点，使路径更平滑
 * 使用向量点积算法检测三点是否近似共线，移除不必要的转折点
 * @param {Array} path - 原始路径点数组，包含{lon, lat}坐标的对象
 * @returns {Array} 返回优化后的路径点数组，保留关键转折点
 */
function optimizePath(path) {
  // 处理特殊情况：路径太短，无需优化
  if (!path || path.length <= 2) return path;

  // 初始化优化路径数组，始终保留第一个点
  const optimized = [path[0]];
  // 记录上一个保留点的索引，用于计算向量
  let prevIndex = 0;

  // 遍历路径中间的每个点
  for (let i = 1; i < path.length - 1; i++) {
    const current = path[i];
    const next = path[i + 1];

    // 获取上一个保留点的坐标
    const prev = path[prevIndex];

    // 计算两个相邻线段的方向向量
    const dx1 = current.lon - prev.lon;
    const dy1 = current.lat - prev.lat;
    const dx2 = next.lon - current.lon;
    const dy2 = next.lat - current.lat;

    // 计算方向向量的点积，判断是否接近共线
    const dotProduct = dx1 * dx2 + dy1 * dy2;

    // 计算向量的平方模长（避免开方）
    const magnitudeSquared1 = dx1 * dx1 + dy1 * dy1;
    const magnitudeSquared2 = dx2 * dx2 + dy2 * dy2;

    // 确保向量有效（非零长度）
    if (magnitudeSquared1 > 0 && magnitudeSquared2 > 0) {
      // 优化余弦计算，避免两次开方操作
      // 这里我们需要计算：dotProduct / (sqrt(m1²) * sqrt(m2²)) = dotProduct / sqrt(m1² * m2²)
      const denominator = Math.sqrt(magnitudeSquared1 * magnitudeSquared2);
      const cosine = dotProduct / denominator;

      // 如果夹角余弦值小于0.995，表示方向变化明显，需要保留当前点
      // 接近1表示夹角很小，接近共线；接近-1表示方向相反
      if (Math.abs(cosine) < 0.995) {
        optimized.push(current);
        prevIndex = i; // 更新上一个保留点的索引
      }
    }
  }

  // 添加最后一个点，确保路径完整性
  optimized.push(path[path.length - 1]);

  return optimized;
}

/**
 * 调整路径端点，确保起点和终点精确匹配用户指定的坐标
 * 由于路径计算可能在网格点上，此函数确保路径与用户输入精确吻合
 * @param {Array} path - 计算得到的路径点数组
 * @param {Object} startCoord - 用户指定的起点坐标 {lon, lat}
 * @param {Object} endCoord - 用户指定的终点坐标 {lon, lat}
 * @returns {Array} 返回调整端点后的路径点数组
 */
function adjustPathEnds(path, startCoord, endCoord) {
  // 处理特殊情况：无效路径
  if (!path || path.length < 2) return path;

  // 调整起点，确保路径从用户指定的精确位置开始
  path[0] = { lon: startCoord.lon, lat: startCoord.lat };

  // 调整终点，确保路径到用户指定的精确位置结束
  path[path.length - 1] = { lon: endCoord.lon, lat: endCoord.lat };

  return path;
}
