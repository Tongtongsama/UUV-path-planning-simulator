# G12 独立传输/运行器修订：当前阻塞记录

日期：2026-11-10（下午）。入口原型：`ros2/twin_axial_g12_real_v2`。

本文件记录本轮尝试的独立传输、运行器和门禁修订，以及**尚未通过传输门禁**的事实。没有启动新的 GUI 实验，没有覆盖旧记录，没有改 G12、模型、PID、分配器、S3 场景或验收阈值；S3 仍为名义场景。

## 1. 已完成的只读门禁

`gate_20261110_03.json`：`passed=true`、`ready_to_run=true`、`real_started=false`。该门禁覆盖 Python 54 条证据、适配器等价性、Nix 文件、端口和 GUI 环境，但**不覆盖实际 ROS 大消息、Gazebo 位姿和 MAS 动作通道**。

## 2. 传输探针结论

使用实际 ROS 消息和独立进程做探针：

| 传输 | 500 B | 1000 B | 1500 B | 2000 B | 2500 B | 3000 B |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Fast DDS UDPv4 | 30/30 | 30/30 | 0/30 | 0/30 | 0/30 | 0/30 |
| SHM | — | — | — | — | — | 30/30 |

网关原 state 记录约 2.4 KB，因此 UDPv4 下控制器收不到状态；SHM 可以承载大消息，但 Gazebo 位姿/动作通道不可见。默认传输曾只收到 1 条状态后导航超时。由此可以确认：**旧四组真实记录里的“状态丢失/超时”是传输层问题，不能与控制性能混为一谈。**

## 3. 独立修订原型

已起草但不作为通过证据：

- `ros2/twin_axial_g12_real_v2/gateway.py`：保留完整 `transactions.jsonl`，但把 ROS state 改为约 374 B 的紧凑消息（只含控制器需要的字段），避免 UDPv4 大消息阈值。
- `ros2/twin_axial_g12_real_v2/trial.py`：在控制前等待实际 Gazebo 位姿和 MAS action server；到达后确认零输入，在 `arrival+0.15 s` 的有界时刻先停止 gateway/xdyn，再做 MAS 清理。保留全部记录，不裁剪旧尾段。
- `ros2/twin_axial_g12_real_v2/transport_probe.py`：实际类型/独立进程探针原型。
- `ros2/twin_axial_g12_real_v2/transport_gate.sh`：按已知可用的启动顺序运行三项探针。

当前探针结果：紧凑 state（374 B）30/30、实际 `VesselPositionArray` pose 30/30 均通过；MAS action 的测试 server 在独立进程中仍不能被 client 发现（`server unavailable`），因此 action 通道还没有可用的通过记录。该原型尚未完成独立门禁、release 绑定和全量测试，也没有通过真实 GUI 验证；因此**不能称为已修复**。在 action 探针和实际 trial preflight 通过前，不应启动新的四组 GUI 实验。

进一步核对 LOTUSim 源码和运行日志后，之前的“端点缺失”判断需要修正：MAS action 端点**存在**。`MultiAgentSystem` 用世界名作 namespace 创建 `mas_cmd_array` action server；Gazebo 日志明确出现 `MultiAgentSystem::handleMASCmdArrayGoal: Received MASCmdArray.`，且 `dev_v2b` 的清理 `cleanup_error=null`，说明 `/uuv_twin_axial/mas_cmd_array` 可以接收删除 goal。实际问题是 rclpy 的 `ActionClient.wait_for_server()` 在该运行时里不可靠：trial preflight 会阻塞/超时，但后面的清理 goal 仍能到达。

真正尚未解决的是 **Gazebo 同时运行时 controller 收不到紧凑 state**：`dev_v2b` 的 pose 已出现，但 `commands=0`；`dev_v2d`（UDPv4）和 `dev_v2e`（默认传输）在控制前仍为 `commands=0`、`navigation=0`，最终只能终止。

Gazebo-in-the-loop 图探针进一步定位到 trial 进程一侧：在 Gazebo、xdyn、gateway 已运行时，独立探针在 domain 74 能看到 `/uuv_twin_axial/state` 的 1 个 publisher，并收到 **201 条 376–389 B 的紧凑 state**；同时能看到 `mas_node`、`physics_plugin`、`/uuv_twin_axial/mas_cmd_array/_action/*` 和 `/uuv_twin_axial/poses`。但在同一时刻，trial 内部打印 `DOMAIN 74`、`STATE_PUBLISHERS 0`；图探针只列出 `uuv_twin_axial_gateway`，没有 trial 的 `uuv_twin_axial_navigation` 节点。也就是说：gateway 发布正常，Gazebo 也正常，失败在于 trial 进程自己的 ROS 节点没有被同一 DDS 图发现/订阅未匹配。

这不是消息格式或 controller 逻辑问题；下一步应检查 trial 进程的 `rclpy` context/participant、节点创建时机与 domain 环境，避免把“probe 能收到”误当成“trial 能收到”。

## 4. 当前结论

- 旧四组真实记录仍按原验收失败；最大误差峰值发生在到达后的清理滑行阶段，不能据此否决 G12 的受控导航段，但也不能裁掉尾段改判通过。
- 当前真实链路仍不合格：UDPv4 大消息、SHM 下的 Gazebo 位姿/MAS 动作、以及实际进程发现都需要修复并纳入门禁。
- 在传输门禁通过前，不扩展真实矩阵，不另立引导候选，不修改模型/PID/阈值或 S3 名义定位。

下一步：完成独立 transport probe 的稳定发现与 release/门禁绑定，先用无控制的实际类型探针通过，再启动三流 P0 与 F2/P2 的 GUI 复验。
