# 双推进器 PID 验证套件的离线可视化

日期：2026-10-27。对象：`artifacts/twin_axial_pid/validation_20261027_final`。入口：`ros2/recorded_replay`。

## 1. 性质与边界

本入口把已完成试验中 Gazebo 实际回写的位姿重新送进一个独立的展示世界，只做可视化：

- 只读 `gazebo_poses.jsonl`，位置线性插值、姿态沿最短旋转弧插值；不放大角度、不改变轨迹、不虚构未记录的状态。
- 不启动 xdyn、网关、原 PID 或原试验，不加载动力学接口；展示世界只含 `MultiAgentSystem` 和 `SceneBroadcaster`。
- 不修改原记录，不把播放结果当作新的实验或验收。套件的验收结论仍以 `suite.json` 和每个子目录的 `acceptance.json` 为准。
- 显示时通过 MAS 发送 MOVE 属于离线可视化，不是新的控制闭环试验。

相关单记录说明见 [离线位姿回放](lotusim_recorded_pose_replay.md)；本文件只补充套件播放和本套件的场景参数。

## 2. 启动

打开 PowerShell：

```powershell
wsl -d NixOS
```

在 NixOS 中进入项目并启动整个套件：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_pid/validation_20261027_final
```

程序先按 `suite.json` 的顺序读取四个子目录，逐个校验 `manifest.json` 的全部哈希、`run_status.json` 的 `completed`、位姿有限性、单位四元数和严格递增时间。任一用例失败就不启动 Gazebo，不会只显示“看起来正常”的部分。

默认从第一个用例 `speed_step` 开始，暂停在首帧，速度 0.25 倍。只想先看某一例：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_pid/validation_20261027_final --case pitch_positive
```

只校验、不启动 Gazebo：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_pid/validation_20261027_final --validate-only
```

只生成净化后的展示场景和来源记录、不启动 Gazebo（输出 `status=prepared`，不是一次实际显示运行）：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial_pid/validation_20261027_final --prepare-only \
  --output "artifacts/twin_axial_pid_replays/prepare_$(date +%Y%m%d_%H%M%S)"
```

## 3. 操作

所有命令都在**启动回放的终端**输入并按 Enter，不在 Gazebo 菜单里操作。

| 命令 | 功能 |
|---|---|
| `p` | 暂停/继续 |
| `pause` / `play` | 明确暂停/继续 |
| `speed 0.25` | 慢放；允许 0.05–4 倍 |
| `speed 0.5` / `speed 1` | 半速 / 原速 |
| `r` | 当前用例从首帧重新播放 |
| `l` | 当前用例循环开关 |
| `next` / `n` | 切换到下一用例（末例回到首例） |
| `prev` / `b` | 切换到上一用例 |
| `case pitch_negative` | 按名称切换用例 |
| `list` | 打印四个用例、时长、位姿数和验收状态 |
| `q` | 退出并关闭本次展示进程 |

切换用例时沿用当前暂停状态、播放速度和循环设置，并从新用例首帧重新计时；显示模型不重建，只改变位姿数据源。暂停只停止记录时间推进，展示服务器继续运行以接收 MOVE 指令；不要暂停 Gazebo 服务器。

默认镜头以整段套件的全部记录位姿取景，因此播放过程中不会跟踪特写，而是一次显示全部轨迹范围。需要近距离看姿态时，可用鼠标在 Gazebo 视图中缩放/旋转，或启动时加 `--camera close`（近距侧视，长距离后会驶出画面）。

## 4. 本套件的数据

`suite.json` 的四个用例顺序为：

| 用例 | 记录位姿数 | 记录时间跨度 | 验收 |
|---|---|---|---|
| `speed_step` | 643 | 0.05–32.15 s | 通过 |
| `pitch_positive` | 643 | 0.05–32.15 s | 通过 |
| `pitch_negative` | 643 | 0.05–32.15 s | 通过 |
| `allocation_recovery` | 803 | 0.05–40.15 s | 通过 |

四个用例使用同一个 `uuv_twin_axial_v2` 模型和相同的 `worlds/minimal.world`（各子目录中该文件哈希一致），所以回放程序只加载一个展示模型，切换用例不会重建模型。若以后某套件的用例视觉模型不一致，程序会明确拒绝整套播放，要求逐例单独回放，而不是悄悄显示错误模型。

`suite.json` 本身没有外部签名或上层 manifest；程序只做结构检查，并把加载时的 SHA-256 写入输出再于退出时复核。真正可独立校验的是每个用例目录内的 `manifest.json`：其中列出的文件（含 `gazebo_poses.jsonl`、`worlds/minimal.world`、`run_status.json`、`metadata.json`、`model.yml` 等）都会逐个比对哈希。

这些用例的 `acceptance.json` 并不在各自的 `manifest.json` 中。程序只在加载时读取它、快照其哈希，并在退出时复核，因此能发现播放期间被改动，但不能证明加载前未被改动；验收结论仍应回到原套件的证据链，回放入口不重新计算 PID 指标。

## 5. 显示场景

- 展示世界由当前记录的白名单提取：只复制模型 link pose 和 visual；移除原世界、物理接口、`lotus_param`、碰撞、传感器和插件。当前模型是自包含内联几何，不依赖外部 mesh URI；若记录依赖 mesh，程序会拒绝。
- 默认全景侧视由全部用例的位姿包围盒计算：中心约 `(8.84, 0, -29.96)`，相机在 `y≈-13.62` 处朝 +y，水平半视场 30°、按 16:9 估算的垂直半视场约 18°。实际套件的水平最大偏角约 26.7°，垂直约 14.7°，留有边距；超宽窗口下极端俯仰点可能贴近边缘，用鼠标缩放即可。
- 模型外观来自记录本身（缺失材质时程序补显示用材质和低强度自发光），不按用例改色。当前活动用例显示在终端状态行和 `events.jsonl`/`summary.json` 中。
- 展示世界的窗口标题固定为
  `Twin-axial recorded suite — validation_20261027_final — visualization only`；
  为保持既有布局，`showTitleBar` 仍为 `false`，标题栏默认隐藏。

## 6. 输出

默认输出到 `artifacts/twin_axial_pid_replays/<UTC时间_随机ID>/`，与原始记录分开，不覆盖已有目录，也拒绝写入原记录或套件目录。

| 文件 | 内容 |
|---|---|
| `state/worlds/replay.world` | 净化后的展示世界和全景镜头 |
| `vehicle.sdf` | 白名单提取的展示模型 |
| `source_record.json` | 套件路径、`suite.json` 哈希、四个用例的路径/文件哈希/时间范围 |
| `commands.jsonl` | 每次 MOVE 的用例、记录时间和位姿 |
| `observations.jsonl` | Gazebo 返回的显示位姿 |
| `events.jsonl` | 就绪、控制、切换用例、循环结束等事件 |
| `gazebo.log` | 展示服务器和 GUI 日志 |
| `summary.json` | 命令数、观测数、各用例验收、`active_case`、`source_unchanged`、`dynamics_launched=false` |

`--prepare-only` 只生成 `replay.world`、`vehicle.sdf`、`source_record.json` 和 `summary.json`（`status=prepared`），不产生命令、观测、事件和 Gazebo 日志。

正常手动退出状态为 `stopped`，`--auto-close` 为 `completed`，`--prepare-only` 为 `prepared`（未启动 Gazebo）。`source_unchanged=true` 表示套件 `suite.json` 和四个用例的全部受校验文件在退出时未变。回放成功不替代原套件的 `acceptance.json`。

## 7. 实际运行验证与未完成项

2026-10-27 已在 Windows + WSL/NixOS 上实际运行：

- `--validate-only` 读取本套件：4 个用例、643/643/643/803 个位姿全部通过哈希和格式校验，四个 `acceptance.json` 均为 `passed: true`。
- 无界面 Gazebo 完整回放 `speed_step`：4 倍速播放 32.10 s，记录 161 条 MOVE、175 条位姿观测；末条观测为 `[13.471, 0, -30]`，与末帧记录一致；退出状态 `completed`，`source_unchanged=true`，输出在 `artifacts/twin_axial_pid_replays/verify_headless_speed/`。
- 用例切换：无界面运行中执行 `next`，从 `speed_step` 切到 `pitch_positive`；`summary.json` 的 `active_case` 为 `pitch_positive`，`source_unchanged=true`，输出在 `artifacts/twin_axial_pid_replays/verify_switch_clean/`。`case NAME`、`list` 也在同一会话中实测有效。
- 带 GUI 的 `pitch_negative` 回放以 4 倍速播放结束并自动关闭，状态 `completed`，输出在 `artifacts/twin_axial_pid_replays/verify_gui_pitch/`。
- 用真实记录生成场景检查：展示世界只含 2 个非物理插件，模型不含插件/传感器/`lotus_param`，全景镜头覆盖全部位姿。
- `tests/test_recorded_replay.py` 10 项通过，覆盖套件加载顺序、越界用例名拒绝、用例被篡改检测、全轨迹取景、带镜头/标题的展示世界，以及不导入 ROS 的 `--validate-only`/`--prepare-only` 子进程调用。

未完成/环境限制：本机 WSLg 会话中的 Gazebo 窗口能启动并完成回放，但自动截屏只得到空白 3D 区域；`gazebo.log` 有 `libEGL warning: failed to create dri2 screen` 和回退到 `kms_swrast` 的记录。因此无法用截图证明模型在窗口中的可见性，也不能据此断言 3D 画面一定正常。请在实际 NixOS 桌面确认窗口内模型可见；若不可见，先处理 WSLg/EGL，再检查 `gazebo.log`，不要以“窗口是否显示”替代套件验收。

`gazebo.log` 还会在首帧停留期间、以及暂停后重复发送同一末帧时打印
`EntitySpawner::moveEntity: SetComponentData failed (vessel='...', entity=0)`：这些时刻送出的位姿与上一帧相同，播放时间开始推进、位姿发生变化后该告警停止。它沿用的是现有 MAS 显示链路；本次回放中 MOVE 被接受，观测位姿随命令从首帧变化到末帧。若后续修复 WSLg 显示，可一并确认该无变化告警是否仍出现。

`--headless` 可用于无界面跑完整段位姿流；`--auto-close` 会在当前用例播放结束后退出，二者都不改变任何记录数据。
