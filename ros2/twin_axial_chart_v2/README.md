# twin_axial_chart_v2 — real-chart ingestion skeleton

独立于 `twin_axial_chart_v1` 的 chart-v2 适配层。**不修改 v1、冻结 G12/S3、真实五条记录或门禁。**

## 支持

- `chart-v2` metadata：`license`、`provenance`、`datum`（chart datum、单位、深度符号、tide）；
- GeoJSON coastline：`Polygon` / `MultiPolygon`，可带 holes；`kind` 默认 `land`；
- CSV bathymetry：`longitude,latitude,depth_m`，正深度向下；
- 输出：`chart.json`、`minimal.world`、`depth.png`、精确 visual OBJ，以及 `source/` 快照。
- `minimal.world`：visual 用精确挤出 mesh；collision 用沿每条岸线边的 0.2 m 薄盒墙（Dartsim 不支持 SDF mesh/polyline collision）。

## 运行

```bash
python -m ros2.twin_axial_chart_v2 \
  --metadata artifacts/chart_v2_20261116_01/pilot_portsmouth_2km/metadata.json \
  --output artifacts/chart_v2_20261116_01/pilot_portsmouth_2km/build
```

不启动 Gazebo/ROS/xdyn。

## 输入 schema

```json
{
  "schema": "chart-v2",
  "name": "...",
  "crs": "EPSG:4326",
  "origin": {"latitude": 0, "longitude": 0, "altitude_m": 0},
  "bounds": {"west": 0, "east": 1, "south": 0, "north": 1},
  "license": {"name": "...", "url": "...", "public_domain": true, "redistributable": true, "attribution": "..."},
  "provenance": {"dataset": "...", "edition": "...", "issue_date": "...", "status": "...", "source_url": "...", "source_sha256": "64 hex", "downloaded_utc": "...", "notes": "..."},
  "datum": {"chart_datum": "MLLW", "vertical_unit": "m", "depth_sign": "positive_down", "tide": "none"},
  "coastline": "coastline.geojson",
  "bathymetry": {"path": "bathymetry.csv", "resolution_m": 25}
}
```

## 限制

- 不是完整 S-57/ENC loader；NOAA ENC 已经离线转换为 GeoJSON/CSV。
- 不使用单个轴对齐 bbox 代替岸线；碰撞中心线跟随精确多边形边，墙厚 0.2 m。
- Dartsim 不能创建 SDF mesh/polyline collision；薄墙是受支持引擎下的折中。
- 薄墙不填充陆地内部；起点/终点需另做陆地 preflight。
- 深度栅格由 CSV 点 IDW 生成；NOAA pilot 的 CSV 是 **DEPARE 深度区中点采样**，不是原始 SOUNDG，也不是可导航水深。
- 没有潮位/datum 动态改正；`tide=none` 明确记录。
- 已完成一次不控制 world-load smoke；没有海流、闭环控制或导航验收。
- NOAA pilot 使用的是 **US5NH01M edition 30.1，状态 obsolete**；仅用于工程链验证，不能作为航海数据。

## 测试

```bash
python -m pytest -q tests/test_twin_axial_chart_v2.py
```

不控制 world-load smoke：

```bash
bash artifacts/chart_v2_20261116_01/smoke_20261117_01/smoke.sh
```
