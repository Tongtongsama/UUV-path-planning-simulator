# 合成海图适配器骨架（离线）

日期：2026-11-16；时区 Asia/Singapore。

本轮只做 **chart adapter 最小骨架 + 合成海图测试**，没有启动 Gazebo/ROS/xdyn，也没有接真实海图或海流。目的：先证明“海图 JSON → WGS84 局部 ENU → WorldModel/障碍 → SDF 骨架 + 深度栅格”的链路可验证。

## 代码

`ros2/twin_axial_chart_v1/`

- `geo.py`：WGS84 局部切线平面，ENU `(x,z)` 与经纬度往返。
- `chart.py`：`chart-v1` 校验、Shapely 障碍、深度 IDW 栅格、`WorldModel`、SDF/PNG 输出。
- `__main__.py`：`--input --output` 最小入口。
- `README.md`：输入 schema、输出、限制。

## 合成输入

`tests/fixtures/chart_synthetic_harbor_v1.json`

- WGS84 原点 1.25 N / 103.75 E；
- 西侧 `land`、东侧 `no_go` 两个矩形多边形；
- 7 个深度点；
- 显式工作区边界。

## 输出

`artifacts/chart_adapter_20261116_01/`

- `synthetic_harbor_v1/chart.json`；
- `synthetic_harbor_v1/minimal.world`；
- `synthetic_harbor_v1/depth.png`；
- `chart_map.png`；
- `manifest.json`（输入/输出哈希、测试计数）；
- `README.md`。

`minimal.world` 含 `<spherical_coordinates>`、障碍包围盒 collision、`chart_seabed` heightmap；**没有启动 Gazebo**，也不声称已通过 LOTUSim 世界加载。

## 测试

```bash
python -m pytest -q tests/test_twin_axial_chart_v1.py
```

结果：`8 passed`。

全量离线回归：

```bash
python -m pytest -q
```

结果：`1225 passed`（原 1217 + 新 8）。

覆盖：

- ENU 往返、原点零位；
- schema/原点/重复 ID/非法 kind 拒绝；
- 多边形/深度点越界拒绝；
- 障碍精确 Shapely 碰撞；
- 深度网格与双线性查询；
- SDF 经纬度、障碍包围盒尺寸、heightmap URI、PNG 尺寸；
- `chart.json` 确定性。

## 明确限制

- 不读取 S-57/S-100/GeoJSON/Shapefile/GeoTIFF；这些仍需独立 parser。
- SDF 障碍当前是轴对齐包围盒；合成矩形精确，真实非轴对齐海图会放大，需改 mesh/挤出 collision。
- 深度是 IDW 栅格，不是 chart datum + 潮位后的通航水深。
- 没有 chart licence/provenance、潮位、海流、波浪或 Gazebo smoke test。
- 不能把本结果当成“真实海图 LOTUSim 已跑通”。

## 下一步

1. 增加 `chart-v1` 的显式许可/版本/datum 字段；
2. 做一个真实小区域的 **离线** GeoJSON 岸线 + 公开 bathymetry 转换；
3. 增加 `test_chart_world_smoke.py`，只验证 SDF 结构/加载前置条件，不启动 Gazebo；
4. 再单独决定是否接 `ekman current` 或 Copernicus/Gauss-Markov 流。
