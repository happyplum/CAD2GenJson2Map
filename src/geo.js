/**
 * 地理坐标处理模块
 * 
 * 该模块提供地理坐标相关的工具函数，包括距离计算、坐标处理和投影转换。
 * 主要用于障碍图构建中的距离计算和地图可视化中的坐标转换。
 * 
 * 主要功能：
 * 1. 使用Haversine公式计算两点间的大圆距离（用于边权重计算）
 * 2. 坐标精度控制和键值生成（用于节点去重）
 * 3. 节点边界框计算（用于画布适配）
 * 4. 画布适配变换计算（用于障碍图可视化）
 * 
 * 坐标系统：使用经纬度坐标 [longitude, latitude]
 * 距离单位：米(m)
 */

// Geographic utilities: haversine distance and simple projection
const R = 6371000; // Earth radius in meters

/**
 * 使用Haversine公式计算两点间的大圆距离
 * Haversine公式考虑了地球的曲率，适用于计算任意两点间的球面距离
 * 
 * @param {Array<number>} a - 第一个点的坐标 [经度, 纬度]
 * @param {Array<number>} b - 第二个点的坐标 [经度, 纬度]
 * @returns {number} 两点间的距离，单位为米
 */
export function haversineDistance(a, b) {
  // 解析坐标
  const [lon1, lat1] = a;
  const [lon2, lat2] = b;
  
  // 角度转弧度的转换因子
  const toRad = Math.PI / 180;
  
  // 将纬度转换为弧度
  const φ1 = lat1 * toRad;
  const φ2 = lat2 * toRad;
  
  // 计算纬度和经度的差值（弧度）
  const Δφ = (lat2 - lat1) * toRad;
  const Δλ = (lon2 - lon1) * toRad;
  
  // Haversine公式核心计算
  // s = sin²(Δφ/2) + cos(φ1) * cos(φ2) * sin²(Δλ/2)
  const s = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  
  // 计算最终距离：2 * R * arcsin(min(1, sqrt(s)))
  // 使用min确保数值稳定性，避免浮点误差导致sqrt(s) > 1
  return 2 * R * Math.asin(Math.min(1, Math.sqrt(s)));
}

/**
 * 将地理坐标四舍五入到指定精度并生成键值
 * 用于节点去重和快速查找，相同位置的坐标将生成相同的键
 * 
 * @param {Array<number>} coord - 地理坐标 [经度, 纬度]
 * @param {number} [precision=6] - 小数点后保留的位数，默认为6位
 * @returns {string} 格式化的坐标键，格式为"经度,纬度"
 */
export function roundCoordKey([lon, lat], precision = 6) {
  return `${lon.toFixed(precision)},${lat.toFixed(precision)}`;
}

/**
 * 从障碍图节点数组计算边界框
 * 遍历所有节点，找出最小和最大的经纬度值，形成包围所有节点的矩形区域
 * 
 * @param {Array<Object>} nodes - 障碍图节点数组，每个节点应包含lon和lat属性
 * @returns {Object} 边界框对象，包含 { minLon, minLat, maxLon, maxLat }
 */
export function bboxFromNodes(nodes) {
  // 初始化边界值为极值
  let minLon = Infinity, minLat = Infinity, maxLon = -Infinity, maxLat = -Infinity;
  
  // 遍历所有节点，更新边界值
  for (const n of nodes) {
    if (n.lon < minLon) minLon = n.lon;
    if (n.lon > maxLon) maxLon = n.lon;
    if (n.lat < minLat) minLat = n.lat;
    if (n.lat > maxLat) maxLat = n.lat;
  }
  
  return { minLon, minLat, maxLon, maxLat };
}

/**
 * 计算将地理边界框适配到画布的变换参数（用于障碍图可视化）
 * 计算缩放比例和平移参数，使障碍图能够完整显示在指定大小的画布中
 * 
 * @param {Object} bbox - 障碍图地理边界框，包含 { minLon, minLat, maxLon, maxLat }
 * @param {number} width - 画布宽度（像素）
 * @param {number} height - 画布高度（像素）
 * @param {number} [padding=20] - 画布边距（像素），默认为20
 * @returns {Object} 变换参数，包含 { scale, tx, ty }
 *                   scale: 缩放比例
 *                   tx: x方向平移量
 *                   ty: y方向平移量
 */
export function fitToCanvas(bbox, width, height, padding = 20) {
  // 计算可用画布区域（减去边距）
  const w = width - padding * 2;
  const h = height - padding * 2;
  
  // 计算地理边界框的经纬度跨度
  const lonSpan = bbox.maxLon - bbox.minLon || 1; // 避免除以0
  const latSpan = bbox.maxLat - bbox.minLat || 1; // 避免除以0
  
  // 计算缩放比例，选择较小的比例确保完整显示
  const scale = Math.min(w / lonSpan, h / latSpan);
  
  // 计算平移量，使边界框左上角对齐到画布边距位置
  const tx = padding - bbox.minLon * scale;
  const ty = padding - bbox.minLat * scale;
  
  return { scale, tx, ty };
}