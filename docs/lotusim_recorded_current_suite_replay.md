# 恒流单障碍导航的 Gazebo 可视化

日期：2026-10-28。对象：`artifacts/twin_axial_current/navigation_20261027_final`（以及同结构的 `navigation_20261028_004323`）。入口仍是 `ros2/recorded_replay`，但显示内容从“只有 UUV”扩展为“UUV + 单挡板 + 计划路径 + 起点/终点 + 水流箭头 + 随流参照球”。

本文件说明怎样把新增的恒流 S1 导航记录变成 Gazebo 场景，以及哪些东西是记录中的真实几何、哪些只是显示辅助。验收结论仍由 `suite.json` 和每个子目录的 `acceptance.json` 给出，本文件不重新定义水流方程或导航阈值。

相关文档：

- [恒流模型、数值检查和 S1 导航验收](lotusim_twin_axial_current.md)
- [双障碍缩比导航](lotusim_twin_axial_current_double.md)
- [无流单障碍导航](lotusim_twin_axial_navigation.md)
- [回放的安全边界](lotusim_recorded_pose_replay.md)
- [上一轮 PID 套件回放](lotusim_recorded_pid_suite_replay.md)

---

## 1. 新增记录里有什么

每个导航子目录（例如 `F2_P0/`）不仅有位姿记录，还有：

| 文件 | 用途 |
|---|---|
| `gazebo_poses.jsonl` | Gazebo 实际回写的 UUV 位姿；回放的运动来源 |
| `worlds/minimal.world` | UUV 和 `single_baffle_obstacle` 的 SDF；挡板是记录中的真实场景几何 |
| `plan.json` | 规划器输出的 `path`（规划 x-z 坐标）、`goal`、障碍/边界/水流配置 |
| `current.json` | 本次流 F0/F1/F2 的规划坐标、ENU、NED 流速 |
| `navigation.jsonl` | LOS 目标、相位、净距、路径误差、水相对速度诊断 |
| `metadata.json` | 模型、case、flow、dt、时长 |
| `run_status.json` | 完成状态、到达时间、进程退出码 |
| `manifest.json` | 原始证据哈希 |

`suite.json` 共 13 次：F0/P0 对照；F1、F2 各 P0–P4 五组初态；F1/P0、F2/P0 各一次重复。回放入口按这个顺序逐例校验和切换。

---

## 2. 显示场景是怎么构造的

### 2.1 UUV 和挡板

- UUV 继续从记录的 `worlds/minimal.world` 中按白名单提取：只保留 link pose 和 visual，移除 `lotus_param`、插件、传感器、碰撞体和原物理接口。
- 新增 `static_models()`：把记录中**非 UUV 模型**也按同样的白名单提取，标记为 `static=true`。当前就是 `single_baffle_obstacle`：
  - 保留 `pose`（`9.5 0 -34 0 0 0`）；
  - 保留 visual box（1×4×12 m）和材质；
  - 删除 collision、插件、传感器。
- 因此 Gazebo 里看到的是记录中实际加载的挡板，不是另画一个“近似障碍”。挡板尺寸/位置与 `plan.json` 的 obstacle 配置一致。
- 套件模式会检查所有用例的静态场景是否一致；如果以后一个 `suite.json` 混有不同障碍（例如 S1 和 S2），回放会拒绝整套播放，要求逐例单独启动。因为 Gazebo 世界在启动时加载，静态障碍不能在 `next` 切换时替换。

### 2.2 计划路径、起点和终点

`mission_overlay()` 读取 `plan.json`：

- 规划坐标 `(x, z_plan)` 的 z 向上；Gazebo ENU 的 z = `z_plan - 20`（起点规划 z=-10 → ENU z=-30）。
- `path` 的每一段画成一条细 box（绿色），保持 x-z 平面走向；不是重新规划，只是把记录中的路径画出来。
- 起点画青色球，终点画黄色球；终点来自 `plan['config']['goal']`。
- 覆盖模型名 `recorded_mission_overlay`，`static=true`；只用于显示，不参与动力学或验收。

当前 P0 的路径是：

```text
规划 (2, -10) → (9.5, -6) → (18, -10)
ENU  (2, -30) → (9.5, -26) → (18, -30)
```

`P1–P4` 只是起点有小扰动（例如 x=1.9/2.1 或 z=-9.9/-10.1），plan 的第一个点会略有不同；回放默认使用 **P0 的名义路径**作为静态叠加。实际轨迹仍由当前选中的记录驱动，路径线只是参考。

### 2.3 水流箭头与受流参照物

当前版本在 Gazebo 中有两个**运行时动态叠加**（不是静态世界的一部分）：

1. **蓝色水流箭头**：一个由 MAS 动态创建的箭头模型（shaft + cone），局部 +X 为箭头方向。回放根据 `current.json.enu_mps` 计算方向，放在起点附近（约 `start + (1, 2, 2)`）：
   - F1 的 ENU 流速是 `(-0.05,0,0)`，箭头指向 -X；
   - F2 是 `(0,0,+0.05)`，箭头指向 +Z（向上）；
   - F0 没有非零流，箭头不显示。
2. **品红色随流参照球**：一个没有碰撞、没有推进的显示用小球，起点在 `start + (2,-2,0)` 附近。它的位置按记录流速积分：
   ```text
   marker_position = marker_anchor + current_enu * elapsed
   ```
   - F1 时向 -X 漂移；
   - F2 时向 +Z 漂移；
   - F0 时参照球不显示。
   50 s 仿真时间、0.05 m/s 流速对应约 2.5 m 位移，足以在画面里看出方向。

这两个对象通过 MAS 的 `CREATE/MOVE/DELETE` 在运行时管理，切换用例时：

- F0 → F1/F2：创建箭头和参照球；
- F1 ↔ F2：箭头只改姿态，参照球回到新起点；
- F1/F2 → F0：删除箭头和参照球。

**重要边界**：参照球是“记录流速的可视化积分”，不是被 xdyn 积分的刚体，也不是流速传感器；箭头只表示该次记录的均匀恒流方向，不代表局部流场。它们都不参与验收，也不改变位姿记录。`--prepare-only` 只写静态 world，不包含这两个运行时模型。

水流信息仍可从三处确认：

- 终端用例名前缀：`[F0]`、`[F1]`、`[F2]`；
- `summary.json` 的 `active_flow`；
- 记录中的 `current.json` 与 `navigation.jsonl` 的 `water` 字段（世界流、艇体流、对地速度、相对水速、流速艇体系导数）。

箭头/参照球的创建、切换、删除也会写入 `events.jsonl` 的 `flow_overlay` 事件。

### 2.4 镜头

默认 `--camera fit`：按全部用例的 UUV 位姿取景，同时让挡板顶部进入画面。

当前 13 例的 UUV 范围约为 x 2–18 m、z -30–-26 m；挡板顶部在 z=-28 m，正好在画面内；挡板向下延伸到 z=-40 m，超出画面的部分不会被强行压缩。这样既能看到“从挡板上方绕过”，又不会为了容纳整块 12 m 挡板把 UUV 缩得太小。需要看细节时用鼠标缩放，或加 `--camera close` 近看起点附近。

### 2.5 双障碍缩比场景（`twin_axial_current_double`）

同一回放入口也读取 `artifacts/twin_axial_current_double/navigation_20261028_final`：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current_double/navigation_20261028_final
```

场景中有两个 `static=true` 的有限高挡板（`baffle_lower`、`baffle_upper`）和一条双障碍计划路径；静态白名单、路径叠加、蓝色流向箭头和品红参照球逻辑与 S1 相同。两条挡板都来自记录的实际 SDF，不是回放代码另画的近似障碍。场景说明和缩比原因见 [双障碍恒流导航](lotusim_twin_axial_current_double.md)。

---

## 3. 启动与操作

在 WSL/NixOS 中：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator

# 整个 13 例套件，先完整校验，再打开 Gazebo
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current/navigation_20261027_final

# 只从 F2/P0 开始
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current/navigation_20261027_final --case F2_P0

# 只校验，不启动 Gazebo
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current/navigation_20261027_final --validate-only

# 只生成净化后的 world/SDF，不启动 Gazebo
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_current/navigation_20261027_final --prepare-only \
  --output "artifacts/twin_axial_current_replays/prepare_$(date +%Y%m%d_%H%M%S)"
```

默认暂停在首帧、0.25x、全景侧视。终端命令：

| 命令 | 功能 |
|---|---|
| `p` | 暂停/继续 |
| `pause` / `play` | 明确暂停/继续 |
| `speed 0.5` | 半速；允许 0.05–4 倍 |
| `r` | 当前用例从首帧重启 |
| `l` | 当前用例循环 |
| `next` / `n` | 下一个用例（F0→F1→…→F2 重复） |
| `prev` / `b` | 上一个用例 |
| `case F2_P0` | 按名称切换 |
| `list` | 列出 13 例、时长、验收状态 |
| `q` | 退出并清理显示进程 |

切换会更换位姿记录，并同步更新水流箭头/随流参照球；UUV 模型、挡板和 `recorded_mission_overlay` 保持同一份静态场景。`--auto-close` 会在当前用例播放完后关闭；`--headless` 可无界面跑完整段位姿流。

---

## 4. 本次实测

### 4.1 校验与场景生成

- `--validate-only` 读取 `navigation_20261027_final`：**13 个用例全部通过**，每个子目录 `acceptance=True`，`suite passed=True`。
- `--prepare-only` 生成的世界包含：
  - 2 个非物理插件（MAS + SceneBroadcaster）；
  - `single_baffle_obstacle`，`static=true`，只有一个 visual，没有 collision；
  - `recorded_mission_overlay`，包含两段绿色路径、青色起点、黄色终点；
  - 默认镜头约为 `9.92, -15.99, -27.95`，水平半视场 30°。
- `--prepare-only` **不包含**箭头和随流参照球：它们依赖 MAS 运行时创建，只有在真正启动 Gazebo 时才会出现。

### 4.2 无界面 Gazebo 回放

以 `F2_P0`、4x、`--headless --auto-close` 实测：

- **188 条 UUV MOVE、267 条 Gazebo 观测**；
- 首条观测 `[2.000, 0, -30.000]`，末条观测约 `[17.943, 0, -29.725]`，与记录轨迹一致；
- `status=completed`、`source_unchanged=true`、`dynamics_launched=false`；
- `gazebo.log` 显示 `single_baffle_obstacle`、`recorded_mission_overlay`、UUV、`flow_arrow` 和 `current_drifter` 都被创建；没有 SDF/几何加载错误；
- `events.jsonl` 的 `flow_overlay` 记录 `active=true, flow=[0,0,0.05]`。

输出在 `artifacts/twin_axial_current_replays/verify_flow_retry/`。

带 GUI 的 `F2_P0` 也以 4x 播放并自动关闭，`status=completed`，`gazebo.log` 同样创建了箭头和参照球；输出在 `artifacts/twin_axial_current_replays/verify_gui_flow_F2_P0/`。自动截屏仍受 WSLg/EGL 限制，不把空白截图当作可见性结论。

运行中从 `F2_P0` 切到 `F0_P0`：`summary.active_case=F0_P0`、`active_flow=F0`、`source_unchanged=true`，`events.jsonl` 记录 `flow_overlay active=false`；输出在 `artifacts/twin_axial_current_replays/verify_flow_switch/`。切到 F0 时箭头和参照球被删除；切回 F1/F2 会重新创建。

### 4.3 记录本身的验收结果

来自 [恒流验收文档](lotusim_twin_axial_current.md)：

| 工况 | 成功初态 | 到达时间 / s | 最小中心净距 / m | 最大 RMS / m | 最大路径误差 / m |
|---|---:|---:|---:|---:|---:|
| F0 无流 P0 | 1/1 | 49.75 | 1.6225 | 0.2401 | 0.5997 |
| F1 逆向流 P0–P4 | 5/5 | 49.85–50.30 | 1.6469 | 0.2329 | 0.5540 |
| F2 向上流 P0–P4 | 5/5 | 49.50–50.00 | 1.7429 | 0.2717 | 0.7060 |

F1/P0、F2/P0 重复的位置差和到达时间差均为零；所有运行 Gazebo 位姿覆盖率 100%。这些是导航验收数据，回放只负责把它们显示出来。

---

## 5. 边界

- 回放发送 MOVE，不运行 xdyn/PID/LOS；不能作为动力学或导航验收。
- 路径线来自 `plan.json` 的**名义 P0 路径**；P1–P4 的起点扰动会让第一段略有差异，实际轨迹以 UUV 为准。
- 挡板按记录完整显示（12 m 高），默认镜头只保证顶部和 UUV 动作进入画面；需要看全高时用鼠标缩放。
- 蓝色箭头只表示记录的均匀恒流方向；品红参照球是按 `current_enu * elapsed` 做的显示积分，不是 xdyn 刚体、不是传感器、不参与验收。F0 时两者都不显示。
- 箭头/参照球是运行时 MAS 模型，`--prepare-only` 不会生成它们；只有实际启动 Gazebo 才能看到。
- GUI 的最终可见性仍需在具体 WSLg/桌面确认；自动截屏受 EGL/WSLg 限制，本次不以空白截图判断模型可见或不可见。

---

## 6. 输出

默认输出到 `artifacts/twin_axial_current_replays/<UTC时间_随机ID>/`：

| 文件 | 内容 |
|---|---|
| `state/worlds/replay.world` | UUV + 挡板 + 路径/起点/终点 + 全景镜头 |
| `vehicle.sdf` | 白名单提取的 UUV |
| `source_record.json` | 13 个用例的路径/哈希、`suite.json` 哈希 |
| `commands.jsonl` / `observations.jsonl` | MOVE 与 Gazebo 观测 |
| `events.jsonl` | 就绪、切换、循环、`flow_overlay` 创建/切换/删除等 |
| `gazebo.log` | Gazebo/MAS 日志 |
| `summary.json` | `active_case`、`active_flow`、`mission`、各用例验收、`source_unchanged`、`dynamics_launched=false` |

`commands.jsonl` 只记录 UUV 的显示位姿；水流箭头和参照球的 MOVE 不写入该文件，也不进入验收。`--prepare-only` 只写 world、SDF、来源记录和 summary，`status=prepared`。
