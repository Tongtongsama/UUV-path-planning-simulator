# uuv_minimal_xdyn_v1：开环动力学试验

本试验独立于 `lotusim_replay`。不读取冻结轨迹，不调用 MOVE，不接 PID、LOS 或
TerminalApproach，不修改现有算法和冻结数据。Gazebo 的 physics interface 将状态交给
真实 xdyn 求解器积分，返回结果写回 Gazebo；独立网关记录这条链路并提供状态反馈。
网关本身没有积分器。

## 运行

在 NixOS WSL 中，进入项目。只需一个终端；脚本会启动并关闭本次试验自己的求解器、
网关和 LOTUSim，无需另开回放脚本或网页 Demo。

建议先运行整套自动验收（八个工况，另加一次前向脉冲复现检查）：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
bash ros2/minimal_xdyn/run-nix.sh suite \
  --output artifacts/minimal_xdyn/acceptance_01
```

通过时根目录 `suite.json` 为 `passed=true`。任一工况失败即停止，保留全部日志。
整套试验默认不打开 GUI。单独运行工况：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
bash ros2/minimal_xdyn/run-nix.sh run --case zero \
  --output artifacts/minimal_xdyn/zero_01
bash ros2/minimal_xdyn/run-nix.sh run --case surge_positive \
  --output artifacts/minimal_xdyn/surge_positive_01
bash ros2/minimal_xdyn/run-nix.sh run --case surge_negative \
  --output artifacts/minimal_xdyn/surge_negative_01
bash ros2/minimal_xdyn/run-nix.sh run --case heave_positive \
  --output artifacts/minimal_xdyn/heave_positive_01
bash ros2/minimal_xdyn/run-nix.sh run --case heave_negative \
  --output artifacts/minimal_xdyn/heave_negative_01
bash ros2/minimal_xdyn/run-nix.sh run --case pitch_positive \
  --output artifacts/minimal_xdyn/pitch_positive_01
bash ros2/minimal_xdyn/run-nix.sh run --case pitch_negative \
  --output artifacts/minimal_xdyn/pitch_negative_01
bash ros2/minimal_xdyn/run-nix.sh run --case watchdog \
  --output artifacts/minimal_xdyn/watchdog_01
```

输出目录必须是新目录，重复试验请换成 `_02` 等后缀。每次从静止重新启动。
默认无 GUI，避免渲染影响第一次动力学检查；需要观看时，在任一 `run` 命令末尾加
`--gui --keep-open`。8 秒仿真完成后保持窗口，Ctrl+C 结束并生成验收结果。
初始载具位于 ENU `(0,0,-30)` 米，可在 Entity Tree 选择载具定位视角。

第一次 Nix 下载可能很久。固定 LOTUSim revision 为
`2034754b2140006fd615bcb1c420d6a7c83f0642`，其 xdyn 及依赖由 flake lock 固定。
脚本在 Linux 缓存目录准备 Nix 环境，规避中文 Windows 路径中的 flake URI 问题。

试验使用 ROS_DOMAIN_ID=72、独立 GZ_PARTITION 和独立状态目录。
12346 是网关端口，12347 是 xdyn 端口；占用时脚本拒绝启动，不会杀掉其他进程。
xdyn 此版本的 `--address` 仅影响启动提示，服务器实际监听所有接口。

## 模型与输入约定

这是用于接口验证的合成 6 自由度载具，不是经辨识的真实 LRAUV。
没有把侧向或姿态状态每步强制归零。默认无流、无风浪，中性浮力；使用独立的人工
滚转/俯仰恢复力矩 `-6*sin(angle)` Nm。

| 参数 | 值 |
|---|---|
| 刚体质量/转动惯量对角线 | `[20,20,20,8,8,8]`，平移 kg、转动 kg·m² |
| 附加质量/惯量对角线 | `[5,5,8,2,2,2]` |
| 线性阻尼对角线 | `[4,6,6,3,3,3]`，对应力/速度及力矩/角速度 |
| 二次阻尼对角线 | `[1,1.5,1.5,.5,.5,.5]` |
| 时间步长 / 求解器 | 0.05 s / RK4 |
| 初始位置与姿态 | ENU `(0,0,-30)`，FLU 朝向 +X，静止 |

输入是理想可逆推力 N，不是 RPM，也不包含电机动态：

| 输入键 | 含义 | 限幅 |
|---|---|---|
| `surge(T)` | 向前推力，body FRD +X | ±5 N |
| `fore(T)` | 前端 x=+0.5 m 的向下推力 | ±2 N |
| `aft(T)` | 后端 x=-0.5 m 的向下推力 | ±2 N |

合力/力矩为 `X=surge(T)`、`Z=fore(T)+aft(T)`、
`M=0.5*(aft(T)-fore(T))`。模型以质心处等效 wrench 实现，避免力臂被重复计算。
FRD 的 +Z 向下、+q 为抬头；Gazebo 使用 ENU/FLU。

每个试验 8 秒仿真时间：前 2 秒零输入，2–4 秒脉冲，4–8 秒撤去输入观察衰减。
正/反向航行分别 ±2 N；正俯仰为 fore=-0.5 N、aft=+0.5 N，负俯仰相反。
正升沉为 fore=aft=+0.5 N，body +Z 向下，因此 Gazebo 高度下降；负升沉相反。
watchdog 在 2–3 秒发送 +2 N，此后停止命令，验证独立网关归零。
命令超过 0.25 秒仿真时间或 1 秒墙钟时间即失效；暂停不会继续积分。
出现非连续时间、非法状态或求解器错误时本次试验失败，必须完整重新启动。

## 状态反馈和记录

`/uuv_minimal/state` 为 `std_msgs/String` JSON。每个求解步发布：

- `solver_state_ned_frd`：`t,x,y,z,qr,qi,qj,qk,u,v,w,p,q,r`；
  位置为世界 NED，四元数为 NED←FRD、wxyz，速度和角速度为 body FRD。
- ENU 位置、ENU←FLU 四元数、body FRD 线速度/角速度、抬头为正的俯仰角。
- 实际施加的三个输入、命令序号、watchdog 状态、步起止时间、步序号及有效性。

这是求解器真值状态，不是含噪声的传感器估计。
`transactions.jsonl` 同时记录 LOTUSim 发来的初始状态和 xdyn 返回的完整状态；
下一步必须接续上一求解结果，检查位置、四元数及速度，禁止用轨迹重置替代动力学。
`gazebo_poses.jsonl` 独立订阅 LOTUSim 位姿，验证结果确实回写到了 Gazebo。

每个目录还保存 `commands.jsonl`、模型与 world、副本源码、flake.lock、
三个进程日志、`metadata.json`、`run_status.json`、`manifest.json` SHA-256 和
`acceptance.json`。仿真时间用于试验调度，墙钟仅用于超时监控。

## 验收

每次程序退出码 0 且 `acceptance.json` 的 `passed=true` 才算该工况通过。
重新检查已有记录（不启动仿真）：

```bash
python3 -m ros2.minimal_xdyn.cli analyze artifacts/minimal_xdyn/surge_positive_01
```

验收检查记录完整性、进程正常清理、连续 0.05 秒时间步、8 秒持续时间、合法状态、执行器限幅、
至少 20 条可匹配的 Gazebo 位姿及误差小于 2 cm。
零输入要求保持静止；正反脉冲要求速度和运动方向一致，撤去输入后能量衰减；
watchdog 要求停止发送后实际求解器输入归零。

应将八个工况全部跑通，并至少再跑一次正向航行以检查复现性。
`suite` 对两次前向试验匹配至少 160 个时间步，最大位置差要求小于 2 cm，
最大体坐标速度分量差小于 0.01（线速度 m/s，角速度 rad/s）。视觉上有运动但
验收失败时，应保留输出目录检查日志，不能仅凭画面宣告通过。
本阶段只验收输入→动力学→状态反馈和方向/时间；不证明真实水动力参数准确，
也不证明 PID、路径跟踪或 TerminalApproach 性能。

## 已完成的实测验收

2026-10-21 复核 `artifacts/minimal_xdyn/validation_20261020_01`：八个工况及一次
前向重复试验全部通过。重新分析所有原始记录及 SHA-256 后仍全部通过；每次均为
160 个有效求解步、160 条 Gazebo 位姿观测、8 秒仿真时间，时间步 0.05 秒。
两次前向试验匹配的 160 个状态点最大位置差和体坐标速度差均为 0。

下表取脉冲期间相对仿真时间 3.8 秒，作为符号核查的实测样本：

| 工况 | 响应 |
|---|---|
| 前向 / 反向 | u = +0.121762 / -0.121762 m/s |
| 下潜 / 上浮 | w = +0.052034 / -0.052034 m/s（FRD） |
| 抬头 / 低头 | q = +0.049066 / -0.049066 rad/s |
| 抬头 / 低头姿态 | pitch = +0.055709 / -0.055709 rad |
| 命令中断 | 3 秒停止发送，3.25 秒首次因超时施加零输入 |

本次验证使用 headless Gazebo；GUI 观察模式提供运行入口，但不作为本次已验证的
渲染效果承诺。相关新增及回放回归测试共 30 项通过。早期 `dev_*` 目录是调试记录，
包含已定位的失败，不属于上述通过的验收集。
