# 冻结 G12 的小规模真实闭环验证

日期：2026-11-10。入口：`ros2/twin_axial_g12_real`。模型、G12 参数、PID、分配器、S3 场景和全部验收阈值未改；S3 仍是 `double_baffle_tight_v1` 名义场景。默认 GUI，未关闭已有 Gazebo，也没有启动完整真实矩阵。

## 1. 门禁复核

先重新执行只读门禁：

```powershell
wsl -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/ros2/twin_axial_g12_real/run-nix.sh real-matrix --check-only --python-matrix artifacts/twin_axial_g12_matrix/python_20261110_01 --output artifacts/twin_axial_g12_real/xdyn_check_NEW --report artifacts/twin_axial_g12_real/gate_NEW.json
```

本次报告 `artifacts/twin_axial_g12_real/gate_20261110_03.json`：`passed=true`、`ready_to_run=true`、`real_started=false`。Python 证据、54 条回放、适配器等价性、Nix 运行文件和端口检查均通过；报告也明确限定为“启动前置条件”，不代表真实闭环验收。

## 2. 传输诊断与失败保留

真实试验在启动前又逐次重算门禁，但第一次 GUI F0/P0 没有得到控制状态：`commands.jsonl=0`、`navigation.jsonl=0`，最终 `Trial wall-clock timeout (paused or stalled)`。随后做的独立 ROS 探针（不是仿真）得到：

| 传输 | 500 B | 1000 B | 1500 B | 2000 B | 2500 B | 3000 B |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Fast DDS UDPv4 | 30/30 | 30/30 | 0/30 | 0/30 | 0/30 | 0/30 |
| SHM | — | — | — | — | — | 30/30 |

网关的 state 记录约 2.4 KB，因此 UDPv4 下控制器收不到状态；这是传输层失败，不是导航失败。改用 `FASTDDS_BUILTIN_TRANSPORTS=SHM` 后状态可以到达，但 Gazebo 的位姿/动作通道不再可见，`gazebo_poses.jsonl` 为 0、MAS 清理也不可用。默认传输（SHM+UDP）下则出现“只收到一条状态后导航超时”的不稳定行为。上述失败目录全部保留：

- `real_20261110_01/S3/F0/P0`：UDPv4，无状态，超时。
- `real_20261110_02/S3/F0/P0`：SHM，控制完成，但无 Gazebo 位姿、清理失败。
- `real_20261110_03/S3/F0/P0`：默认传输，1 条命令后导航超时。
- `real_20261110_04/S3/{F1,F2}/P0`、`real_20261110_04/S3/F2/P2`：SHM，控制完成，但无 Gazebo 位姿、清理失败。

这些记录说明最终门禁虽然 `ready_to_run=true`，但没有覆盖 ROS 大消息、Gazebo 位姿回写和动作服务这三项运行条件。

## 3. 四组真实控制结果

在 SHM 下完成了用户要求的优先级覆盖：三种水流的 P0，以及最紧的 F2/P2。四组都使用 GUI，控制闭环真实运行，全部到达且未越过 0.9 m 安全线：

| 工况 | 到达 (s) | 最小净距 (m) | RMS (m) | 最大误差 (m) | 最大误差判定 | Gazebo 位姿覆盖 | 清理 |
| --- | ---: | ---: | ---: | ---: | --- | ---: | --- |
| S3/F0/P0 | 136.50 | 1.12756 | 0.28945 | 0.93567 | 失败 | 0 | MAS 清理不可用 |
| S3/F1/P0 | 134.75 | 1.10037 | 0.28855 | 0.95477 | 失败 | 0 | MAS 清理不可用 |
| S3/F2/P0 | 129.35 | 0.91868 | 0.34059 | 0.97876 | 失败 | 0 | MAS 清理不可用 |
| S3/F2/P2 | 129.65 | 0.91617 | 0.34007 | 0.98129 | 失败 | 0 | MAS 清理不可用 |

F2/P2 的净距 `0.9161667 m`，只比 0.9 m 高 **16.17 mm**，与 Python 的 16.37 mm 很接近；但它不是当前的主要失败原因。四组的 **最大路径误差全部超过 0.8 m**，比 Python G12 的对应值高约 0.25–0.28 m：

| 工况 | Python 最大误差 | 真实最大误差 | 超出 0.8 m |
| --- | ---: | ---: | ---: |
| F0/P0 | 0.6728 | 0.9357 | 0.1357 |
| F1/P0 | 0.6773 | 0.9548 | 0.1548 |
| F2/P0 | 0.7236 | 0.9788 | 0.1788 |
| F2/P2 | 0.7261 | 0.9813 | 0.1813 |

四组的 RMS 仍低于 0.35 m，到达和安全也通过；因此失败集中在真实 xdyn 闭环的最大跟踪误差，不是传输造成的数值假象，也不是随机发散。`navigation_coverage` 另因到达后仍有约 40 条无导航更新的滑行记录而为 false；这是记录尾部与验收覆盖率的独立问题，不改变最大误差结论。

## 4. 是否值得继续

**在冻结 G12 下，不值得继续扩展真实闭环，也不值得进入完整真实矩阵。** 依据：

1. 四组真实控制已经覆盖三流 P0 和最紧的 F2/P2，全部到达且安全，但四组最大误差都超过未修改的 0.8 m 阈值；F2/P2 也没有因为净距接近阈值而出现更严重的失败。
2. 失败不是传输层能解释的：传输层导致的是无状态、无位姿或清理失败；四组 SHM 运行的 xdyn 状态、控制回放和到达/净距/RMS 都已记录，最大误差是独立指标。
3. 传输层仍有未解决的阻塞：UDPv4 不能承载 2.4 KB 的 state 消息，SHM 又丢失 Gazebo 位姿/动作；即使忽略最大误差，当前也没有一组能给出完整的真实验收记录。
4. 要保持模型、PID、分配器和 0.8 m 阈值不变，就不能通过调参或放宽阈值把 0.935–0.981 m 写成通过。若继续，应另立新的引导候选（并重新做 Python 与真实门禁），或明确把该结果作为失败边界；不能直接扩展真实矩阵。
5. S3 仍是名义场景；本轮没有把 S3 改称边界场景，也没有改 S3 几何、安全线、RMS 或最大误差阈值。

建议下一步：先修复 ROS 大消息/pose/action 传输并把三项探针纳入门禁；再决定是否需要新的引导候选。当前 `real_trial_20261110_01.json` 的 `worth_continuing_real=false` 是对冻结 G12 的判断，不是对 S3 场景定位的改写。

## 5. 记录

- 门禁：`artifacts/twin_axial_g12_real/gate_20261110_03.json`
- 失败与四组控制汇总：`artifacts/twin_axial_g12_real/real_trial_20261110_01.json`
- 四组真实记录：`artifacts/twin_axial_g12_real/real_20261110_02/S3/F0/P0`、`real_20261110_04/S3/{F1,F2}/P0`、`real_20261110_04/S3/F2/P2`
- 传输失败记录：`real_20261110_01`、`real_20261110_03`
- 本轮没有修改 `ros2/twin_axial_g12_real` 的适配器/门禁源码，没有修改模型、PID、分配器、场景或阈值；没有启动完整真实矩阵。
- 全量 Python 回归 1068 项通过：`artifacts/twin_axial_g12_real/regression_real_20261110.xml`；证据校验清单：`SHA256SUMS_real_20261110.txt`。
