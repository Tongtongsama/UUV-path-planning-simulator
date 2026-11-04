# S2/F0 双挡板无流闭环导航

本阶段使用 `double_baffle_scaled_v1`：规划平面边界 `[0,20] × [-20,0]` m，障碍为 `[6,7] × [-20,-7]` 和 `[13,14] × [-13,0]` m。SDF 同时提供真实碰撞盒和规划地图来源；安全检查读取原始 xdyn 状态并计算连续两个记录点之间的扫掠线段净距。F0 为无流。

在 `codex/s2-f0` 分支上，S1 入口仍默认运行。S2 显式使用 `--scene S2`。S2 任务层设置航速 0.2 m/s、LOS 分段切换半径 0.8 m、目标俯仰上限 ±70°；PID 增益、双推进器分配、xdyn 动力学和 0.9 m 安全阈值均未改。较低航速与扩大任务俯仰范围经过单独真实试验；它们是 S2 的开发配置，不能把当前 S1/F0 与 S2/F0 记录直接拼成要求同一控制配置的正式 45 次矩阵。

## 1. 真实仿真及验收

在 PowerShell 中启动 NixOS：

```powershell
wsl -d NixOS
```

在 NixOS 终端执行：

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash ros2/twin_axial_navigation/run-nix.sh suite --scene S2 \
  --output "artifacts/twin_axial_navigation/s2_f0_$(date +%Y%m%d_%H%M%S)"
```

程序逐次运行 P0–P4 与 P0_repeat。输出目录必须是新目录；不要与占用端口 12356/12357 或 ROS domain 74 的其他试验同时运行。最后查看 `suite.json` 中的 `passed`、`all_runs_safe` 和 `repeatability.passed`。单次目录包含 `model.yml`、实际加载的 `worlds/minimal.world`、规划结果、xdyn 事务、ROS 命令、Gazebo 位姿、导航日志、manifest 与 acceptance。

正式复验套件保存在 `artifacts/twin_axial_navigation/s2_f0_release_20261031`。五组初态全部通过，P0 重复也通过；到达时间 140.60–141.35 s，最小净距 1.2174–1.2193 m，路径 RMS 0.3018–0.3025 m，最大路径误差 0.6734–0.6752 m。P0 两次 2822 个匹配状态的最大位置差为 0 m，到达时间差 0 s。另有开发套件 `s2_f0_suite_20261031` 保留作为早期证据。正式记录经过独立只读重算，`suite.json` 中 `passed`、`all_runs_safe` 与 `repeatability.passed` 均为 `true`；全量 Python 回归测试 977 项通过。

只读复核，不启动仿真：

```bash
bash ros2/twin_axial_navigation/run-nix.sh analyze \
  artifacts/twin_axial_navigation/s2_f0_release_20261031
```

验收使用原 N1–N6/R1–R2 条件：目标 0.5 m 内连续 2 s、300 s 内到达；扫掠净距 ≥0.9 m；路径 RMS ≤0.35 m、最大 ≤0.8 m；饱和比例 ≤20%；五初态至少 4/5 且 P0 必须通过、所有运行安全、P0 重复性通过。此处观测为 5/5。

## 2. 离线交互式可视化

该播放器只读已保存记录，显示两块挡板、A* 与执行路径、实际轨迹、艇体、LOS 前视点、净距、路径误差和俯仰；不启动 LOTUSim 或 xdyn。PowerShell 中打开已生成的正式 P0 播放器：

```powershell
cd C:\Users\Songyuan\uuv_simulator
Start-Process (Resolve-Path 'artifacts\twin_axial_navigation_visual\s2_f0_release_p0\index.html')
```

点击“播放”或“暂停”，拖动进度条定位，选择 0.25×–4× 速度；“重头”回到首帧。HTML 自包含，浏览器无需联网。生成另一次记录的播放器：

```powershell
.\.venv\Scripts\python.exe -m visualization.twin_axial_navigation_export `
  artifacts\twin_axial_navigation\s2_f0_release_20261031\P1 `
  --output artifacts\twin_axial_navigation_visual\s2_p1
Start-Process (Resolve-Path 'artifacts\twin_axial_navigation_visual\s2_p1\index.html')
```

生成器校验原始 manifest，输出 `source.json` 记录来源、哈希与 `display_only=true`。任何现有输出目录都不会被覆盖。正式 P0 HTML 已在 Chrome 无界面渲染检查中显示 2822 帧、两块障碍；播放、暂停、拖动进度条也经过浏览器脚本实际操作检查。

## 3. LOTUSim/Gazebo 实时 GUI

在 NixOS 终端运行一场新的真实闭环：

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash ros2/twin_axial_navigation/run-nix.sh run --scene S2 --case P0 --gui \
  --output "artifacts/twin_axial_navigation/s2_gui_$(date +%Y%m%d_%H%M%S)"
```

WSLg 会打开 Gazebo Sim。S2 的 GUI 模式设置侧视镜头；第一块挡板为蓝色，第二块为紫红色。UUV 已恢复原本的艇体、艇首和双推进器外形，并保留这些部件的颜色；真实试验中不再附加跟踪球。建议最大化窗口查看场景。GUI 随该次试验结束自动关闭；原始记录和 `acceptance.json` 保留在指定目录。GUI 显示是观察工具，数值验收仍以 xdyn 原始记录与只读重算为准。恒流对照进展另见 [S2 F1/F2 说明](lotusim_twin_axial_current_f1_f2.md)。

### GUI 调速和重播已验收轨迹

真实闭环仍按固定的 0.05 s 控制步长运行。需要慢放、快放、暂停或从头查看时，在 NixOS 终端启动只读 GUI 回放：

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
bash ros2/twin_axial_navigation/run-nix.sh replay \
  artifacts/twin_axial_navigation/s2_f0_release_20261031/P0 \
  --output "artifacts/twin_axial_navigation_visual/s2_gui_replay_$(date +%Y%m%d_%H%M%S)"
```

保持启动终端处于前台，输入指令并按 Enter：`p` 暂停/继续、`r` 从头重播、`0.25`/`0.5`/`1`/`2`/`4`/`8` 调整倍速、`q` 退出。青色球表示当前记录。可加 `--reference <另一个 S2 记录目录>` 同时显示橙色参考球；S2/F1 已验收记录可用于无流/恒流对照，F2 失败记录需加 `--include-failed-reference`，橙球会停在原失败点。回放先校验 manifest 并独立只读重算验收结果；插值只用于显示，不生成新的仿真数据。`--rate 2` 可以指定启动倍速。Gazebo 窗口保持运行状态，不要在其中点击暂停；回放控制使用终端指令。

此前带青色标记的 GUI P0 试验 `s2_p0_gui_color_marker_20261031` 作为历史记录保留；当前新试验只显示原本的 UUV 外形。青色/橙色球仅存在于只读 GUI 对照回放，不属于真实动力学模型。

## 4. 开发记录与范围

`s2_p0_development_20261031_01` 至 `_04` 是保留的失败试验：早期 LOS 分段切换停滞，随后第二块挡板前安全停止；一次扩大俯仰参考但未同步控制输入范围，触发显式拒绝。`_05` 是通过的 P0 开发试验。正式套件与 GUI 试验使用最终配置，失败未被覆盖或替换。

本阶段仅证明固定 S2/F0 合成模型在五组初始状态下运行；S2 恒流对照见 [S2 F1/F2 说明](lotusim_twin_axial_current_f1_f2.md)。未完成 S3、其他流速/流向、在线感知或 45 次正式矩阵。
