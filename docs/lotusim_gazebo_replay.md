# LOTUSim Gazebo 回放适配层

## 2026-10-19：回放记录

正常运行命令不变，每次自动创建 `artifacts/lotusim_recordings/<UTC时间_随机编号>/`。
也可用 `--record-dir /path/to/new_directory` 指定新目录；拒绝覆盖已有目录。
`--validate-only` 不创建记录。

记录内容：

| 文件 | 含义 |
|---|---|
| environment.json / scene.sdf | 边界、挡板、目标路径、起终点与实际生成的展示场景 |
| *_states.csv / *_controls.csv | 冻结历史轨迹、pitch/q、施加力/力矩、阶段与安全检查字段 |
| historical_results.json | 原实验结果和指标，不是本次控制实验 |
| commands.csv | 发起 MOVE 前记录的位姿、序号和源时间；不等于动作已成功 |
| gazebo_poses.csv | `/lotusim/poses` 返回的本次艇位姿、消息仿真时间、接收墙钟时间 |
| events.csv | 实体创建、批次确认、结束与异常 |
| summary.json | completed/interrupted/failed、计数、有无 Gazebo 观测 |
| metadata.json / manifest.json | 运行设置、原证据哈希、输出 SHA-256 |
| runtime_*（若存在） | 本地世界文件、Nix 锁文件快照；不保证服务器使用了该文件版本 |

播放结束后额外采集约 1 秒终态并自动封存记录，不必等 Ctrl+C。
中断/异常也会封存。强制终止或断电可能只有部分 CSV、没有 summary/manifest，
这类目录视为未完成记录。记录退出前封存，不包含随后删除实体的清理过程。
Gazebo 暂停仍不受支持；记录不改变原有墙钟回放合同。

### 生成轨迹和控制器历史曲线

回放完成后，在 Windows PowerShell 运行（把最后的目录名换成终端打印的实际值）：

```powershell
cd C:\Users\刘松原\uuv_simulator
.\.venv\Scripts\python.exe -m ros2.lotusim_replay.lotusim_replay.recording_report artifacts/lotusim_recordings/<实际目录名>
```

使用已有 Windows 虚拟环境的 matplotlib，不增加 Nix GUI 依赖。
生成 `plots/recording.png` 与 `plots/recording.pdf`，包括环境/轨迹叠加、历史 pitch、q、
施加 surge force、pitch moment 和回放时间曲线。导出先验证 manifest，拒绝覆盖已有 plots。
没有 Gazebo 观测时会明确显示警告，不把发送轨迹冒充实际观测。

三种时钟没有锁步，消息也没有命令序号，因此本版不计算假定一一对应的“跟踪误差”。
控制器没有在回放中运行：控制曲线来自冻结实验；Gazebo 只提供显示位姿观测。
尚不包含屏幕视频、相机图像、实时控制器输出或完整 rosbag。

2026-10-17。只读取冻结数据，不运行或修改算法。原 `player.py` 发布的是外部 renderer
话题，不能在 Gazebo 创建模型；本实现 `gazebo_player.py` 使用 `/lotusim/mas_cmd_array`
的 CREATE/MOVE/DELETE 动作，等待每批操作确认。无需修改 LOTUSim C++ 或启动 xdyn。

## 已实现范围

- 复用既有 exporter 验证原始 manifest、运行历史、场景与几何绑定。
- double_baffle_detour 的挡板、边界、固定 P1 路径、起终点；障碍沿 y 加厚 0.6 m，仅展示。
- 默认 APPROACH，也可双臂同一物理时间播放；橙色 APPROACH、蓝色 P1、白色艇首。
- 自带 +X 艇首简化外形，不依赖 LRAUV 网格，不声称真实艇型或真实碰撞轮廓。
- ENU 位置不变，四元数为 R_y(-pitch)，保留原始展开俯仰的插值。
- 短臂结束后保持最后状态；整段结束后等待 Ctrl+C，仅删除本次随机前缀的实体。
- 无 waypoint、动力学插件或碰撞几何；原实验日志是成功/碰撞判定的唯一来源。

## Windows 验证（无需 ROS）

```powershell
cd C:\Users\刘松原\uuv_simulator
.\.venv\Scripts\python.exe -m ros2.lotusim_replay.lotusim_replay.gazebo_player --validate-only --arm P1 --arm APPROACH
.\.venv\Scripts\python.exe -m pytest tests/test_lotusim_gazebo_replay.py tests/test_lotusim_replay.py tests/test_replay_export.py -q
```

可追加 `--export-dir artifacts/lotusim_gazebo_preview` 导出场景 SDF 和摘要。
输出目录必须不存在，避免覆盖现有数据。

## NixOS / WSL 运行

两个终端都先在 PowerShell 执行 `wsl -d NixOS`。

终端 1：

```bash
nix run github:naval-group/LOTUSim -- run --gui
```

沿用已修复 SceneBroadcaster 的 lotusim.world。保持播放状态，关闭网页里的 Demo，
只留一个 Gazebo server。无需启动网页 UI。

终端 2：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
bash ros2/lotusim_replay/run-nix.sh --arm APPROACH
```

脚本建立独立 Nix 开发环境，提供 Python、rclpy 和 LOTUSim 自定义消息，不改系统 Python。
首次需要联网下载依赖。启动脚本把 flake 定义复制到
`${XDG_CACHE_HOME:-$HOME/.cache}/uuv-lotusim-replay-nix`，从该目录运行 `nix develop .`，
避免中文 Windows 路径的 flake URI 解析问题和 Git 未跟踪文件过滤。
flake.lock 在该缓存目录生成并保留；Python 仍从原项目目录运行，不复制或修改冻结数据。
此环境定义尚未在本机完成 Nix 构建验证；若出现依赖错误，保留完整终端输出。
首次运行可先追加 `--validate-only` 验证环境和冻结数据。

Gazebo 实体树会出现 `replay_<随机编号>_scene` 和 `replay_<随机编号>_approach`。
右键 scene → Move To，调整相机从 y 方向看 x-z 挡板平面；查看运动可定位或跟随艇。
默认播放约 98.3 秒，然后停在终态。第一版无启动等待，模型生成后立即播放；
若调整相机错过开头，Ctrl+C 后重新运行。

双臂比较（原坐标重叠，未人为横移）：

```bash
bash ros2/lotusim_replay/run-nix.sh --arm P1 --arm APPROACH --rate 2
```

共同原始时间约 120 秒；APPROACH 在 98.3 秒 SUCCESS 后保持不动，P1 到 120 秒 TIMEOUT。
这些是历史结果，不是本次新增实验。

## 时钟、退出和限制

这是墙钟驱动的显示回放，不是实时动力学，也不宣称与 Gazebo 仿真时钟锁步。
默认请求 10 Hz，受 Gazebo 步长与动作延迟限制，落后时跳过中间显示帧，不修改源样本。
保持 Gazebo 运行，不用 Gazebo 暂停按钮暂停回放。动作超时 15 秒后报错；
已被接收的超时动作仍可能执行，因此应重启世界后重试，避免残留实体。
正常按回放终端 Ctrl+C 清理本次实体，再在 Gazebo 终端 Ctrl+C 退出。

本版提供固定目标路径，尚未实现逐帧累计轨迹、拖动进度条、与网页曲线同步或传感器闭环。
原网页回放可独立查看完整曲线。运行环境的 SceneBroadcaster 必须已加载。
本机 WSL 调用报 HCS_E_SERVICE_NOT_AVAILABLE，无法完成 Gazebo 端到端验收；
纯 Python 测试不代表 ROS/Gazebo 已实际验证。
