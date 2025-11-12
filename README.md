# 障碍图构建工具

这是一个专注于从GeoJSON数据构建障碍图的Web应用。该工具能够识别障碍物区域，构建可通行路径的图结构，并提供交互式界面进行路径选择。

## 核心功能

- 从GeoJSON线段数据构建障碍图
- 识别并处理障碍物多边形
- 构建可通行区域的图结构
- 使用空间索引实现高效的节点查询
- 交互式选择起点和终点
- 可视化展示障碍图结构

## 技术实现

### 主要模块

1. **障碍图构建 (graph.js)**
   - `buildObstacleGraph`: 从GeoJSON数据构建障碍图
   - `buildSpatialIndex`: 构建空间索引用于快速节点查询

2. **障碍物处理 (obstacles.js)**
   - `extractObstaclesFromGeoJSON`: 从GeoJSON提取障碍物多边形
   - `pointInPolygon`: 判断点是否在障碍物内部
   - `segmentIntersectsAnyObstacle`: 判断路径是否穿过障碍物

3. **地理计算 (geo.js)**
   - 距离计算、坐标处理、边界框计算等

4. **交互界面 (App.vue)**
   - 障碍图可视化
   - 使用空间索引选择实际图节点

### 障碍物识别标准

系统通过以下规则识别障碍物：
- `properties.walkable === false`
- `properties.blocked === true`
- `properties.type === 'obstacle'` (不区分大小写)
- `properties.obstacle === true`

## 使用方法

1. 运行应用
2. 点击"构建障碍图"按钮加载GeoJSON数据
3. 在画布上选择起点和终点（系统会自动选择最近的实际图节点）
4. 查看构建的障碍图统计信息

## 开发

```bash
# 安装依赖
npm install

# 启动开发服务器
npm run dev

# 构建生产版本
npm run build
```

## 项目结构

- `src/graph.js`: 障碍图构建核心模块
- `src/obstacles.js`: 障碍物识别和处理模块
- `src/geo.js`: 地理计算工具模块
- `src/App.vue`: 应用主组件
- `src/data/lines.geojson`: 示例GeoJSON数据
