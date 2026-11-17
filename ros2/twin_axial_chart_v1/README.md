# twin_axial_chart_v1 — synthetic chart adapter skeleton

这是 **chart adapter 的最小骨架**，不是 ENC/S-57/S-100 完整解析器，也不是 Gazebo 海图启动器。目标是把“合成海图 → WGS84 局部 ENU → `WorldModel`/障碍 → SDF 骨架 + 深度栅格”的链路先跑通并测试。

## 输入：`chart-v1` JSON

```json
{
  "schema": "chart-v1",
  "name": "synthetic_harbor_v1",
  "crs": "EPSG:4326",
  "origin": {"latitude": 1.25, "longitude": 103.75, "altitude_m": 0.0},
  "bounds": {"west": 103.744, "east": 103.756, "south": 1.244, "north": 1.256},
  "features": [
    {"id": "west_land", "kind": "land", "coordinates": [[lon, lat], "..."]},
    {"id": "east_no_go", "kind": "no_go", "coordinates": [[lon, lat], "..."]}
  ],
  "depth": {
    "resolution_m": 2.0,
    "soundings": [[lon, lat, depth_m], "..."]
  }
}
```

- 坐标必须是 WGS84 经纬度（`lon, lat`），多边形首尾可闭合，也可不闭合。
- `kind` 当前支持 `land`、`shore`、`no_go`、`obstacle`。
- `bounds` 可选；不提供时由多边形顶点外接矩形生成。
- 深度为向下的正水深；当前用反距离加权生成规则栅格。

## 处理链

1. `geo.py`：WGS84 → 局部 ENU `(x, z)`，`x` 向东、`z` 向北；小区域切线平面近似。
2. `chart.py`：验证 schema、原点、边界、多边形、深度点；生成：
   - `Boundary`（矩形工作区）；
   - 精确 Shapely `Obstacle` 多边形；
   - `DepthGrid`（规则网格、双线性查询）；
   - `WorldModel`。
3. `Chart.to_world_xml()`：输出 SDF 骨架：
   - `<spherical_coordinates>` WGS84/ENU；
   - 每个障碍的 **轴对齐包围盒** collision（合成矩形与真实多边形精确一致；非轴对齐海图只是近似）；
   - `chart_seabed` heightmap 引用 `depth.png`。
4. `Chart.write()`：输出 `chart.json`、`minimal.world`、`depth.png`。

## 输出

```text
chart.json      规范化清单、原点、边界、障碍包围盒、深度范围
minimal.world   SDF 骨架，未启动 Gazebo
depth.png       深度栅格图（宽度=x，高度=z）
```

## 明确限制

- **不支持** S-57/S-100/GeoJSON/Shapefile/GeoTIFF 直接读取；这些是下一层 parser。
- SDF 障碍使用包围盒，非轴对齐多边形会被放大；真实海图应改为挤出 mesh 或精确 collision。
- 深度是 IDW 栅格，不是潮位/ chart datum 校正后的通航水深。
- 没有海流、波浪、潮汐；没有 Gazebo 启动或验收。
- 当前没有 chart datum、tide、license/provenance 的完整字段；真实接入前必须补。

## 测试

```bash
python -m pytest -q tests/test_twin_axial_chart_v1.py
```

覆盖：ENU 往返、schema/原点/重复 ID 拒绝、多边形越界、障碍碰撞、深度查询、SDF 纬度/经度/包围盒/heightmap、PNG 尺寸、确定性 manifest。

全量离线回归：`1225 passed`（原有 1217 + 本骨架 8）。
