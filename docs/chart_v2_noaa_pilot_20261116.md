# chart-v2 + NOAA ENC Portsmouth 2 km pilot（离线）

日期：2026-11-16；时区 Asia/Singapore。

本轮做 chart-v2 schema、GeoJSON coastline、CSV bathymetry、NOAA 2×2 km pilot、离线结构测试，以及一次**不控制**的 LOTUSim 世界加载 smoke test。**没有启动控制/xdyn，没有接海流，没有跑闭环导航。**

## 隔离

只新增：

- `ros2/twin_axial_chart_v2/`
- `tests/test_twin_axial_chart_v2.py`
- `artifacts/chart_v2_20261116_01/`

未修改 `twin_axial_chart_v1`、冻结 G12/S3、真实五条记录、门禁和旧结果。`C:\Users\Songyuan\Desktop\path planning v6` 里的法国勒旺岛数据本轮未使用、未移动、未修改，留作后续。

## NOAA 数据

- 数据集：**NOAA ENC US5NH01M — Portsmouth to Dover and Exeter**
- Edition：**30.1**
- 状态：**obsolete**
- 来源：<https://www.charts.noaa.gov/ENCs/US5NH01M.zip>
- 许可：NOAA ENC，美国政府作品，public domain
- 原始 `.000` SHA-256：`a8bc02ecc729db595a21f1ef9fab985b6d251dcc638f22d36c5e618694d8ff78`
- Pilot bbox：西 `-70.777`、东 `-70.753`、南 `43.076`、北 `43.094`（约 2×2 km）
- 原点：`43.085 N, -70.765 E`

处理：

1. GDAL 从 S-57 读取 `LNDARE`，按 bbox `-clipdst` 裁切，输出 GeoJSON；
2. 从 `DEPARE` 深度区按约 30–40 m 网格采样，`depth_m=(DRVAL1+DRVAL2)/2`，输出 CSV；
3. 生成 `chart-v2` metadata，记录 license、provenance、datum、source hash；
4. 适配器读取 GeoJSON/CSV，生成 `WorldModel`、`minimal.world`、`depth.png`。

## 输出

`artifacts/chart_v2_20261116_01/pilot_portsmouth_2km/`

- `metadata.json`：chart-v2 输入；
- `coastline.geojson`：4 个 LNDARE 岸线/陆地多边形；
- `bathymetry.csv`：558 个深度采样点；
- `chart_v2_map.png`：ENU 障碍 + 深度栅格；
- `build/chart.json`：规范化清单；
- `build/minimal.world`：SDF 骨架；
- `build/depth.png`：79×81 深度栅格。

`minimal.world` 包含：

- `<spherical_coordinates>` 43.085 N / -70.765 E；
- 4 个 land obstacle 的碰撞岸线：沿每个多边形边生成 0.2 m 厚薄盒墙，中心线精确跟随岸线；
- 每个 obstacle 的 visual 使用精确挤出 OBJ mesh；
- `chart_seabed` heightmap 引用 `depth.png`。
- 不使用单个轴对齐 bbox 代替岸线。

## 如何运行

在本仓库根目录：

```bash
python -m pytest -q tests/test_twin_axial_chart_v2.py
```

预期：`6 passed`。

跑 chart-v1 + environment 回归：

```bash
python -m pytest -q tests/test_twin_axial_chart_v1.py tests/environment
```

预期：`97 passed`。

全量离线回归：

```bash
python -m pytest -q
```

预期：`1239 passed`。

重新生成 pilot 输出（不会启动 Gazebo）：

```bash
python -m ros2.twin_axial_chart_v2 \
  --metadata artifacts/chart_v2_20261116_01/pilot_portsmouth_2km/metadata.json \
  --output artifacts/chart_v2_20261116_01/pilot_portsmouth_2km/build
```

注意：`build` 已存在时会覆盖该输出目录；不要把它指向旧记录或 v1 目录。

## 不控制 world-load smoke

```bash
bash artifacts/chart_v2_20261116_01/smoke_20261117_01/smoke.sh
```

该脚本只运行 `lotusim run --debug minimal.world` 15 秒；不启动 xdyn、ROS controller 或导航。日志在 `smoke_20261117_01/smoke.log`，摘要见 `smoke.json`。

结果：world initialized；`[Err]` 计数 0；RC=124（timeout 终止 server，属于预期）。

## Headless entity smoke（本轮新增）

`artifacts/chart_v2_20261117_01/entity_smoke/`：

- 两个 0.5 m 动态球，无 G12/xdyn/controller；
- `smoke_ball` 受 `-2 N` 向岸线墙运动；
- `drop_ball` 自由落体；
- 下方是显式平面测试地板 `y=-10`，不是海图海床；
- 记录 `/world/noaa_portsmouth_2km_entity/pose/info`。

结果（总体 **partial**）：

- world initialized；`[Err]` 0；262 个姿态样本/球；位置全部有限；
- 岸线墙 x=802.966 m；`smoke_ball` 最小中心 x=803.500 m，未穿越；
- 测试地板：最小中心 y=-9.668 m，未无限下落，但有约 0.168 m 接触穿透；
- `control_started=false`、`navigation_started=false`、`xdyn_started=false`；
- server RC=124（预期 timeout）；
- 岸线墙和平面地板通过，但 chart seabed/heightmap collision **未验证**，因此总体不是完整通过。

限制：Dartsim 仍不支持 SDF heightmap collision，`chart_seabed` 仍只是 visual；平面地板不是海图海床，不能当作潮位/水深碰撞验证。详见 `entity_smoke/README.md`。

## 验收标准

本任务验收离线链路 + 不控制 world-load smoke，标准如下：

1. **schema 完整**：`chart.json` 有 `license`、`provenance`、`datum`；source SHA-256 为 64 hex；pilot 记录 `public_domain=true`、`MLLW`、`positive_down`、`vertical_unit=m`、`tide=none`。
2. **空间正确**：WGS84 bbox 转 ENU 后约 2×2 km；4 个 land 多边形在 boundary 内；`WorldModel.collides` 命中陆地、不命中外部水域。
3. **深度链路**：CSV 558 点生成 79×81、25 m 深度栅格；最小/最大深度有限；`depth.png` 尺寸与栅格一致。
4. **world 结构**：`minimal.world` 含正确经纬度、4 个障碍模型和 `chart_seabed` heightmap；XML 可解析。
5. **world-load smoke**：`smoke_20261117_01` 在 `lotusim run --debug minimal.world` 下出现 `World [noaa_portsmouth_2km_v1] initialized`，`[Err]` 为 0；15 s 后由 `timeout` 终止，RC=124 是 server smoke 的预期退出，不是加载失败。
6. **隔离**：没有修改 v1、冻结 G12/S3、真实记录或门禁；smoke 只启动 Gazebo server，没有启动 xdyn、ROS controller 或导航。
7. **测试**：`tests/test_twin_axial_chart_v2.py` 6 passed；全量 `1239 passed`。

通过以上只代表 **chart-v2 离线适配链通过**，不代表 LOTUSim 能加载该 world，也不代表海图/水深可用于导航。

## 明确限制

- NOAA ENC 原始格式是 S-57；**GDAL 转换是离线一次性步骤，不是适配器运行时能力**。
- 视觉使用精确 OBJ mesh，但 **Dartsim 不支持 SDF mesh/polyline collision**；碰撞用沿每条岸线边的薄盒墙，中心线精确、厚度 0.2 m。
- 薄墙只阻止穿越岸线，不填充陆地内部；起点若已在陆地内部仍需单独 preflight。
- 深度 CSV 来自 `DEPARE` 深度区中点，不是原始 `SOUNDG`，也不是 chart datum + 潮位后的可导航水深。
- US5NH01M 状态为 **obsolete**；不能作为航海数据。
- 有无控制 Gazebo world-load smoke（`smoke_20261117_01`）；没有海流、波浪、控制或导航验收。
- 没有使用法国勒旺岛数据；后续接入前要先确认其格式和许可。

## 当前海流研究的海床边界（已接受）

为避免把 Dartsim 不支持的 heightmap collision 混进海流实验，当前阶段接受：

- `artifacts/chart_v2_20261118_01/current_flat_seabed/` 中使用 **y=-15 m 平面**作为海流实验的工程边界；
- `chart_seabed` 仍只是 pilot 的 visual/advisory 数据，不参与当前海流 collision；
- 岸线仍使用精确边墙 collision；
- 所有海流结果必须写明“平面工程海床，不是海图 bathymetry”。

该边界 world 的 load smoke：world initialized、`[Err]` 0、RC=124（预期 timeout）。这只证明 world 能加载，不证明海流。

F0/F2 xdyn-only drift smoke：F0 无漂移；F2 NED Δz=-0.771 m、final w≈-0.0493 m/s。

Gazebo-coupled flat-seabed F0/F2 smoke 也已完成：vessel depth 5 m、flat plane z=-15 m、局部 gateway depth envelope 0–45 m、零命令、无 G12/controller/navigation。F0 309/309 valid、Δz=0；F2 329/329 valid、state Δz=-0.5974 m、final w=-0.04855 m/s、Gazebo pose Δz=+0.5966 m；`[Err]` 0。平面在 15 m、艇在 5 m，因此没有发生 flat-seabed contact；不能声称海床 collision 已验证。

## 下一步

1. 增加起点/终点不落在陆地的 preflight；
2. 单独决定接 `ekman current` 或 Copernicus/Gauss-Markov；
3. 不修改冻结 G12/S3，不把本 pilot 结果混入既有五条/53 条结论。
