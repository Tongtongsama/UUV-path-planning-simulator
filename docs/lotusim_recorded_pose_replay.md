# Gazebo 离线位姿回放

入口：`ros2/recorded_replay`。只读取完成的试验记录，不启动 xdyn、网关、PID 或原试验，不执行记录中的源码。不修改原始记录。显示时发送 MOVE 是离线可视化行为，不是新的动力学实验。

## 1. 启动本次俯仰记录

打开 PowerShell：

```powershell
wsl -d NixOS
```

在 NixOS 输入：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial/pitch_20261026_070605
```

无需另外启动 LOTUSim、网页 Demo 或 xdyn。程序先校验原记录，再启动独立展示服务器；模型创建且收到状态反馈后才打开 GUI，避免初次场景同步漏掉模型。

带 `suite.json` 的验证套件可整体回放，按用例切换，见 [双推进器 PID 验证套件的离线可视化](lotusim_recorded_pid_suite_replay.md)。套件模式仍是只读可视化，不启动动力学。

默认镜头在UUV侧面近距离观察，初始暂停，速度为0.25倍。回到上述终端输入 `p` 并按 Enter 开始。原实验最大俯仰约5度，不会显示长距离前进；不人为放大角度。

## 2. 控制方法

所有命令均在**启动回放的终端中输入，然后按 Enter**。不在 Gazebo 菜单里操作。

| 命令 | 功能 |
|---|---|
| `p` | 暂停/继续 |
| `pause` / `play` | 明确暂停/继续 |
| `speed 0.25` | 四分之一速度慢放 |
| `speed 0.5` / `speed 1` | 半速/原速；允许0.05–4倍 |
| `r` | 从首帧重新开始播放 |
| `l` | 开关循环播放 |
| `q` | 退出，关闭本次展示进程 |

默认播放结束后保持末帧，窗口不自动关闭。Ctrl+C也可退出。暂停停止记录时间推进，展示服务器继续运行以接收位姿指令；不要暂停 Gazebo 服务器，否则 MOVE 请求可能超时。

循环慢放：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial/pitch_20261026_070605 --rate 0.25 --loop
```

同样初始暂停，输入 `p` 开始。若希望启动后直接播放，追加 `--play`；慢电脑推荐保留默认暂停，等待窗口加载。

一次播放后自动退出：追加 `--auto-close`（自动开始播放，不能与 `--loop` 同用）。

## 3. 记录与显示的区别

- 使用 `gazebo_poses.jsonl` 中原始 Gazebo 实际返回的 ENU 位置、xyzw四元数和时间。
- 本记录有160个样本，原时间0.05–8.00 s；显示时间以第一样本为0，所以跨度为7.95 s。没有虚构未记录的0秒状态。
- 位置线性插值、姿态沿最短旋转弧插值；速度选项改变播放时间，不改变几何轨迹或角度幅值。
- 从记录的 `worlds/minimal.world` 按白名单提取模型link pose和visual。移除原世界、物理接口、lotus_param、碰撞和传感器，生成独立展示世界。
- 当前支持本试验的内联几何模型；若记录依赖外部mesh URI，明确拒绝，不能声称已经支持任意模型资产包。
- 展示世界只加载 MAS 和 SceneBroadcaster；时间参数用于限制展示消息频率，不加载动力学求解插件。
- Nix入口复用已有固定环境，因此环境中可以安装有xdyn，但回放程序不启动它。

只靠试验目录的数据足以重现这次模型和位姿；运行仍需要本回放程序及兼容的 Gazebo/LOTUSim 环境。目录不是可直接双击的独立播放器。

## 4. 原记录保护与输出

读取前校验manifest的全部条目，拒绝越界路径、损坏文件、非法四元数及不递增时间。退出后复核原文件哈希。

输出默认位于 `artifacts/twin_axial_replays/<UTC时间_随机ID>/`，与原实验目录分开。`--output` 可指定新的展示输出目录，但禁止放入原记录目录，也不覆盖已有目录。

输出：`source_record.json`（来源及哈希）、净化后的 `vehicle.sdf` 和展示world、`commands.jsonl`、`observations.jsonl`、`events.jsonl`、`gazebo.log`、`summary.json`。

summary的 `source_unchanged=true` 表示受校验源文件未变；`dynamics_launched=false` 表示此入口未启动动力学进程。正常手动退出状态为stopped，`--auto-close`结束为completed。回放成功不替代原实验的acceptance.json。

验证数据但不启动 Gazebo：

```bash
bash ros2/recorded_replay/run-nix.sh \
  artifacts/twin_axial/pitch_20261026_070605 --validate-only
```

只生成净化后的展示场景、不启动 Gazebo：追加 `--prepare-only --output <新目录>`，summary 状态为 `prepared`，不是一次实际显示运行。

## 5. 当前闭环进度与下一步

（本节保留回放首次完成时的表述；2026-10-27 的无流速度/正负俯仰 PID 阶跃验收已另行完成，见 [PID 阶跃验收](lotusim_twin_axial_pid.md)。）

已经完成：双推进器模型、分配、输入限幅、开环方向/时间/坐标、实际求解状态反馈、命令失效归零，以及实验记录回放。

尚未完成：LOTUSim上根据误差计算执行器输入的速度/俯仰闭环、无障碍跟踪、LOS路径跟踪及有流导航。原Python PID代码存在，但尚未适配和验收于新执行器/xdyn链路。开环试验成功不能称最小控制闭环已完成。

建议下一阶段先完成无流总能量/步长收敛及Python/xdyn响应对照，明确模型一致性，再适配现有PID的分配后抗饱和，依次验收速度阶跃、正负俯仰阶跃、无障碍直线和深度变化。含水流一致性仍需在正式有流导航评价前完成。回放入口仍不加入 PID 或动力学；套件播放只是只读可视化，后续闭环试验应在独立入口完成。
