# LOTUSim/xdyn 接入、实现与当前阶段说明

日期：2026-10-27。本文解释项目从纯 Python 3-DoF 仿真推进到 LOTUSim + xdyn 联合仿真后，**每一层做了什么、为什么这样做、LOTUSim 是怎样接进来的、目前走到哪一步**。它是一份实现与阶段说明，不替代各阶段的 `acceptance.json`，也不重新定义验收阈值。本文对应 2026-10-27 的代码和记录；后续接入 LOS/恒流/导航时应新增版本，而不是覆盖本文的阶段结论。

阅读时建议配合：

- [研究范围与完成标准](dissertation_scope_baseline_v1.md)
- [模型与数值验收基线](dissertation_model_acceptance_v1.md)
- [最小开环](lotusim_minimal_xdyn_open_loop.md)、[双推进器 v1](lotusim_twin_axial_open_loop.md)、[v2](lotusim_twin_axial_v2.md)
- [PID 阶跃](lotusim_twin_axial_pid.md)、[运动与故障](lotusim_twin_axial_motion.md)
- [回放与可视化](lotusim_recorded_pid_suite_replay.md)

---

## 1. 一页结论

这个项目现在已经不是“在 Gazebo 里播放一条轨迹”的展示工程，而是：

> **Python 负责场景/控制/调度/记录/验收，LOTUSim 负责实体、传输和显示，xdyn 是唯一的 6-DoF 动力学积分器。**

已经完成并留有记录的主链路：

```text
目标 (u, pitch)
   │
   ▼
原 Python 级联 PID（只保留内环）
   │  X, M（请求广义力；控制器内部只为抗饱和预览分配）
   ▼
ROS 2 VesselCmdArray（带仿真时间戳、序号）
   │
   ▼
Gateway（校验、看门狗、再次执行同一套双推进器分配）
   │  upper(T), lower(T)（实际施加）
   │  WebSocket
   ▼
xdyn RK4 6-DoF（0.05 s）──► 下一状态
   │                         │
   │◄────────────────────────┘
   ▼
LOTUSim/Gazebo physics_interface（写回实体位姿）
   │
   ├─► /uuv_twin_axial/poses（独立 Gazebo 位姿，用于回写核对）
   └─► /uuv_twin_axial/state（求解器真值事务，用于控制与验收）
   │
   ▼
控制下一步 / 记录 / acceptance
```

控制器只把 `X/M` 请求发出；真正决定 `upper/lower` 的是网关这一层的分配器。控制器内部的分配调用只用于计算抗饱和回算，不把“分配后的推力”当作命令直接发出。

当前完成度用论文标准概括：

| 标准 | 状态 | 说明 |
|---|---|---|
| C1 模型方程、坐标、执行器与欠驱动一致 | **基本完成（无流纵向）** | M4–M6 数值检查及开环的分配/坐标/时间/回写检查通过；未做单一 M1–M7 签署，含流和全 6-DoF 未验证 |
| C2 可配置障碍、起终点、恒流 | **部分** | 旧 Python 环境已有该能力；尚未绑定到新模型/LOTUSim 并验收 |
| C3 实际动力学闭环 | **部分** | 阶跃、直线、深度、消息边界故障已通过；尚未在完整导航任务中复验 |
| C4 名义工况安全到达 | **未开始** | 还没有新模型的障碍任务和到达判定 |
| C5 对照与适用边界 | **未开始** | 无流/恒流对照、45 次导航矩阵未做 |
| C6 可复现系统与论文 | **部分** | 依赖、源码快照、manifest、测试较完整；论文/最终打包未完成 |

一句话概括当前阶段：

> **模型—执行器—内环—深度外环—消息边界故障这条“无流、无障碍”的闭环已经打通并有真实 xdyn/LOTUSim 记录；导航层（LOS、静态障碍、恒流、45 次矩阵）尚未接上。**

---

## 2. 系统分层：谁负责什么

| 层 | 位置/技术 | 负责 | 不负责 |
|---|---|---|---|
| 研究与算法层 | `core/`、`physics/`、`controller/`、`planner/`、`guidance/`、`navigation/` | 数据结构、3-DoF 参考方程、级联 PID、规划/LOS、指标 | 不冒充 xdyn，不把 Python 参考物理当作新模型证据 |
| 试验编排层 | `ros2/*/trial.py`、`cli.py` | 起进程、发目标、按求解器时间调度、记录、清理、验收 | 不积分动力学，不伪造状态 |
| ROS 2 总线 | `rclpy` + `lotusim_msgs` | 命令、状态、位姿、实体动作的传输 | 不是控制器，不是求解器 |
| 网关层 | `ros2/twin_axial_v2/gateway.py` | WebSocket 代理、执行器分配复算、看门狗、原始事务记录 | 不做 RK4，不产生“预测状态” |
| 物理服务层 | `xdyn-for-cs`（锁定版本） | 6-DoF 刚体/附加质量/阻尼/恢复力积分 | 不知道 ROS、不知道 Gazebo 实体 |
| 仿真环境层 | LOTUSim（Gazebo Harmonic） | 世界、实体、MAS、物理接口、场景广播、GUI | 载具状态由 xdyn 写回，不用 ODE 受力推进；不负责控制 |
| 记录/验收层 | `manifest.json`、`transactions.jsonl`、`acceptance.json` | 证据固化、只读重算、套件汇总 | 不重新跑动力学（`analyze` 模式） |

系统边界的关键点是**唯一状态所有权**：

- 动力学状态只能由 xdyn 产生；
- Gazebo 只把 xdyn 返回的状态写回实体并显示；
- 控制器只根据求解器反馈计算下一步；
- 不使用 MOVE 回放推进动力学试验；
- 不用命令值或 Python 预测状态替代求解器真值。

这条原则来自早期的[阶段合同](lotusim_minimal_dynamics_plan.md)，也是后面所有模块不互相“作弊”的基础。

---

## 3. LOTUSim 是怎么接入项目的

### 3.1 依赖方式：固定外部版本，不改 LOTUSim 源码

项目没有把 LOTUSim 复制进仓库改 C++，而是通过 Nix flake 固定一个外部 revision：

- LOTUSim：`github:naval-group/LOTUSim/2034754b2140006fd615bcb1c420d6a7c83f0642`
- xdyn：`naval-group/lxdyn@363668b2b1f97e3177d2d617586fee6c4a727262`（由该 LOTUSim 输入引用并锁定）
- ROS 2：Jazzy 的 `rclpy`、`action-msgs`、`std_msgs` 等
- Python：`websockets`，以及 LOTUSim 提供的 `lotusim_msgs`

实际定义在 [`ros2/minimal_xdyn/nix/flake.nix`](../ros2/minimal_xdyn/nix/flake.nix)。各入口的 `run-nix.sh` 会把 flake 和 lock 复制到 Linux 缓存目录再 `nix develop`，原因是 Windows 中文路径会让 flake URI 解析和 Git 未跟踪文件过滤出问题。这样做的结果：

- 同一条命令在不同时间拿到同一套 LOTUSim/xdyn/ROS 依赖；
- 不需要在 Windows 上装 ROS 2 或 Gazebo；
- 试验记录可以带 `flake.lock` 作为复现依据；
- 项目代码与上游 LOTUSim 解耦，升级时用新的 revision。

### 3.2 世界和模型是 Python 生成的

每个试验目录里有两份由 Python 写出的东西：

1. `model.yml`：xdyn 的 6-DoF 模型配置；
2. `worlds/minimal.world`：Gazebo/LOTUSim 的世界和载具 SDF。

世界骨架包含三个关键插件：

```xml
<plugin filename="physics_interface_plugin"
        name="lotusim::gazebo::PhysicsInterfacePlugin"/>
<plugin filename="multi_agent_system_plugin"
        name="lotusim::gazebo::MultiAgentSystem"/>
<plugin filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster"/>
```

它们的分工：

| 插件 | 作用 |
|---|---|
| `physics_interface_plugin` | 读取模型里的 `<lotus_param>`，作为 WebSocket 客户端连接外部动力学服务；每个求解步把 Gazebo 当前状态和命令送出去，再把 xdyn 返回的状态写回实体 |
| `multi_agent_system_plugin` | 管理载具实体：动态 CREATE/DELETE、发布位姿、处理 `MASCmdArray` 动作；动力学试验中用于实体存在、位姿发布和清理，回放中用于 CREATE/MOVE/DELETE |
| `gz-sim-scene-broadcaster-system` | 把场景发给 GUI/客户端；没有它，GUI 看不到动态实体 |

载具模型中的关键声明是：

```xml
<lotus_param>
  <physics_engine_interface>
    <init_state>Underwater</init_state>
    <underwater>
      <interface_type>XDynWebSocket</interface_type>
      <uri>ws://127.0.0.1:12356</uri>
      <control_surfaces>
        <upper>upper(T)</upper>
        <lower>lower(T)</lower>
      </control_surfaces>
    </underwater>
  </physics_engine_interface>
</lotus_param>
```

这段不是装饰：它告诉 physics interface “这个载具的水动力学不在 Gazebo 里算，去连 12356 的 WebSocket，并把 `upper(T)`、`lower(T)` 当作控制面”。世界本身 `gravity 0 0 0`，载具没有碰撞体；载具位姿由 physics interface 按 xdyn 状态写回，Gazebo 的 ODE 不承担载具水动力学。

### 3.3 一次联合仿真步到底发生了什么

以双推进器为例，三个进程和两个端口：

- xdyn：`xdyn-for-cs --solver rk4 --port 12357 --dt 0.05`
- gateway：监听 `12356`（给 Gazebo），同时作为客户端连 `12357`（给 xdyn）
- LOTUSim：加载 `worlds/minimal.world`

一个 0.05 s 步的顺序：

1. LOTUSim 的 physics interface 通过 `ws://127.0.0.1:12356` 发来请求：
   `{Dt: 0.05, states: [当前状态], commands: {...}}`。
2. gateway 先检查：
   - `Dt` 是否为 0.05、状态字段是否有限；
   - 与上一步返回的末端状态是否连续（位置/速度/四元数不跳变）；
   - 命令时间戳/序号是否前进；
   - 命令是否过期（0.25 s 仿真时间或 1 s 墙钟），过期就归零；
   - xdyn 返回后，再检查四元数是否单位长度、深度/速度是否在包络内。
3. gateway 对 `X/M` 请求重复一次**同一套双推进器分配**，得到实际 `upper/lower` 推力；把这份“要施加的命令”写进请求。
4. gateway 把请求转给 xdyn；xdyn 用 RK4 积分 0.05 s，返回下一状态。
5. gateway 把 xdyn 的**原始回复**不修改地发回 Gazebo；同时把完整事务写成 `transactions.jsonl`，并发布到 ROS 话题 `/uuv_twin_axial/state`。
6. Gazebo 把新状态写回实体；MAS 发布 `/uuv_twin_axial/poses`，供独立回写核对。
7. 控制节点读到新的 `state`，用反馈仿真时间和上一帧状态计算下一步，发布新的 `X/M`。

请求/回复的形状可以简化为：

```text
request = {
  "Dt": 0.05,
  "states": [{ "t": 12.35, "x": ..., "y": ..., "z": ...,
               "qr": ..., "qi": ..., "qj": ..., "qk": ...,
               "u": ..., "v": ..., "w": ..., "p": ..., "q": ..., "r": ... }],
  "commands": { "upper(T)": ..., "lower(T)": ... }
}
reply = { "t": 12.40, "x": ..., "y": ..., "z": ...,
          "qr": ..., "qi": ..., "qj": ..., "qk": ...,
          "u": ..., "v": ..., "w": ..., "p": ..., "q": ..., "r": ... }
```

`states` 是 Gazebo 送来的当前状态；`commands` 是 gateway 复算后的实际执行器输入；`reply` 是 xdyn 返回的下一状态。gateway 记录的是这条请求/回复的完整事务，不是只记录控制器的意图。

注意第 3 步：控制器发的是**请求的 X/M**，不是直接发两个推力。这样：
- 物理限幅只在分配器一处实施；
- 网关可以独立复算，记录“请求—分配—实际”三者；
- 抗饱和可以使用实际分配后的 X/M，而不是各通道独立裁剪的结果。

### 3.4 ROS 2 接口

| 接口 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/uuv_twin_axial/vessel_cmd_array` | `lotusim_msgs/VesselCmdArray` | 控制节点 → gateway | `cmd_string` 是 `{"X":...,"M":...}`；`header.stamp` 是求解器时间；`header.frame_id` 是命令序号 |
| `/uuv_twin_axial/state` | `std_msgs/String`（JSON） | gateway → 控制/记录 | 每个求解步的完整事务：请求状态、实际命令、xdyn 状态、坐标转换、watchdog 标志 |
| `/uuv_twin_axial/poses` | `lotusim_msgs/VesselPositionArray` | MAS → 记录/验收 | Gazebo 实体位姿，用来看“求解器状态是否真的写回并显示” |
| `/uuv_twin_axial/mas_cmd_array` | `lotusim_msgs/action/MASCmdArray` | trial/replay → MAS | `CREATE_CMD`/`DELETE_CMD`（动力学试验只用 DELETE；回放用 CREATE/MOVE/DELETE） |

`lotusim_msgs` 是 LOTUSim 提供的外部消息包，不是本项目自造的“假接口”。`VesselCmdArray` 的 `cmd_string` 是 JSON，这是项目在 ROS 消息之上定义的轻量协议；真正的求解器协议在 `xdyn` 的请求/回复里。

### 3.5 坐标、时间和正负号

这是最容易出错、也最值得单独说明的部分。

| 概念 | 约定 | 项目中的处理 |
|---|---|---|
| 世界/显示 | Gazebo ENU，x 东、y 北、z 上；水下 z 为负 | 初始位置 `(2, 0, -30)` m |
| 求解器 | xdyn NED，x 北、y 东、z 下；FRD 机体系 | 初始 NED `(0, 2, 30)`，航向 `π/2` |
| 规划平面 | x 向前/东，z 向上；起点 `(2, -10)` | `planning_z = ENU_z + 20`，所以 -30 m 对应 -10 |
| 俯仰 | FRD 中 `q`、`theta` 抬头为正 | `pitch_nose_up_rad` 由四元数计算；ENU/FLU 的 Euler pitch 符号相反，不能直接照抄 |
| 位置反馈 | ENU | `[y, x, -z]`（NED→ENU） |
| 姿态反馈 | ENU←FLU | `q_ENU_FLU = q_ENU_NED * q_NED_FRD * q_FRD_FLU` |
| 时间 | 求解器仿真时间 | 0.05 s 固定步长；控制、验收、故障截止都用仿真时间；墙钟只做超时/看门狗 |

代码位置：[`protocol.py`](../ros2/minimal_xdyn/protocol.py) 的 `feedback()` 和 [`ros2/twin_axial_v2/model.py`](../ros2/twin_axial_v2/model.py) 的初始状态。不要把这段当成“坐标转换小工具”：如果 NED/ENU 或 FRD/FLU 错一个符号，俯仰方向、深度方向、Gazebo 显示和验收会同时错，但日志仍然“看起来连续”。

### 3.6 进程隔离与生命周期

每次试验都是独立的一组进程，避免互相污染：

| 隔离项 | 双推进器试验 | 最小开环 |
|---|---|---|
| ROS domain | 74 | 72 |
| Gazebo partition | `uuv_twin_axial_<输出目录名>` | `uuv_minimal_<输出目录名>` |
| LOTUSim 状态目录 | 输出目录（`LOTUSIM_STATE_HOME`） | 输出目录 |
| gateway / xdyn 端口 | 12356 / 12357 | 12346 / 12347 |
| 数值检查端口 | 12358 | — |
| 回放 | 独立 `recorded_<tag>` partition，ROS domain 83 | — |

启动顺序是 xdyn → gateway → ROS 节点 → LOTUSim。清理顺序相反：先通过 MAS `DELETE_CMD` 删除本次实体（趁 Gazebo 还在运行，释放 xdyn socket），再按进程组终止 Gazebo、gateway、xdyn。这样不会杀掉别人的进程；端口被占用时直接报错，不尝试清理未知进程。

### 3.7 设计取舍：为什么不是“直接在 Gazebo 里跑”

| 选择 | 原因 | 代价/边界 |
|---|---|---|
| 用 xdyn 而不是 Gazebo 内置 ODE | 需要 6-DoF 刚体、附加质量、阻尼、外力表达式；xdyn 是船舶动力学求解器，便于和 Python 参考方程逐项对照 | 多一个进程、多一层 WebSocket；必须验证状态所有权和坐标 |
| 用 physics interface + gateway，而不是改 LOTUSim C++ | 上游 LOTUSim 保持原样，升级只换 revision；gateway 可以做校验、看门狗、原始事务记录 | gateway 成为必须正确的一层；它不是求解器，不能“补算”状态 |
| Python 只做控制/调度/记录 | 避免 Python 自己积分后又被误称为 xdyn 验证；控制算法可以独立测试 | 需要严格的仿真时间/反馈接口；Python 不能碰动力学状态 |
| 用 ROS 2 做总线 | 命令、状态、位姿、实体动作有明确消息边界；控制节点可独立启动/停止 | 需要 domain/partition/port 隔离，避免和别的试验串扰 |
| 用 Nix 固定依赖 | Windows 上不装 ROS/Gazebo；LOTUSim/xdyn/ROS 版本可复现 | 首次下载慢；中文路径需要缓存目录规避 |
| 用合成模型而不是真实艇 | 没有实艇辨识数据；合成参数可以完全公开、可重复 | 所有结论只适用于这个合成平台，不能称为真实 LRAUV/BlueROV2 |

这条链路的“真实”来自 xdyn 积分和 LOTUSim 回写，而不是来自 Gazebo 画面。画面只能证明实体存在/位姿在动；是否真的是动力学，要看 `transactions.jsonl`、`gazebo_poses.jsonl` 和 `acceptance.json` 的对齐。

---

## 4. 每个部分是怎么完成的

下面按时间/依赖顺序说明。每一小节都回答四个问题：目标是什么、怎么实现、怎么验收、边界在哪里。

### 4.0 起点：已有的纯 Python 3-DoF 研究系统

**已经有的东西**：

- `core/`：Pose、Twist、ControlInput、VehicleState、Path、Trajectory 等数据结构；
- `environment/`：x-z 障碍、边界、净距检查、无流/恒流场景；
- `physics/`：非线性欠驱动 3-DoF 参考方程（质量/附加质量/阻尼/恢复力/海流），Euler/RK4，数值对照；
- `controller/`：级联 PID、姿态恢复前馈、限幅与抗饱和；
- `planner/`、`guidance/`、`trajectory/`、`navigation/`：A* 基线、LOS/进度、时间参数化、任务编排与安全/终止；
- `visualization/`、离线回放：冻结历史的三维展示。

**已经达到的程度**：2026-09 完成了 3-DoF 仿真导航基线与展示冻结；历史结果包括 LOS 10/16、P1 7/12、TerminalApproach 10/12 等。它们证明了规划/引导/控制/动力学在同一套 Python 框架内可以闭环，且失败可以被分类；但**不是**新双推进器模型或 LOTUSim/xdyn 的证据。

**为什么还要迁到 LOTUSim/xdyn**：

- Python 物理是“参考方程”，不是独立的 6-DoF 求解器；用它验证自己容易循环论证；
- 旧模型是 3-DoF 纵向，无法直接代表真实的 6-DoF 出平面耦合；
- 论文要求“仿真系统”具备 Gazebo 展示、真实动力学、ROS 2 接口和可复现证据；
- 旧的 3-DoF 算法（尤其 PID、LOS、规划）仍可复用，但必须换到一个外部求解器上重新验收。

因此后续工作选择：**保留算法，替换动力学后端；新模型、新记录、新验收，旧结果不覆盖、不混用。**

### 4.1 可复现环境（2026-10 初）

**目标**：让“同一份代码 + 同一份锁文件”能重新生成同一套 LOTUSim/xdyn/ROS 环境。

**实现**：

- `nix/flake.nix` 固定 LOTUSim revision、xdyn 输入、ROS Jazzy 包、Python `websockets`；
- 每个入口的 `run-nix.sh` 把 flake/lock 复制到 `~/.cache` 下的固定目录，绕开中文路径；
- 每个试验目录保存 `flake.lock`、源码快照、`model.yml`、`worlds/minimal.world`；
- 输出目录只能新建，不能覆盖旧记录；失败记录也保留。

**验收**：`run_status.json` 中三个子进程退出码正常；`manifest.json` 覆盖除自身和 `acceptance.json` 之外的文件；只读 `analyze` 重新核对哈希。

**边界**：这是“可复现”而不是“跨机器绝对一致”；浮点重算允许 1e-12 级舍入差，不因此放宽物理阈值。

### 4.2 最小开环桥：`uuv_minimal_xdyn_v1`（2026-10-20/21）

**目标**：先证明 LOTUSim ↔ gateway ↔ xdyn ↔ ROS 这条桥本身是通的，且方向、时间、坐标、watchdog 正确。此时不接 PID，也不追求欠驱动。

**实现**：

- 合成 6-DoF 模型：质量/惯量 `[20,20,20,8,8,8]`，附加质量 `[5,5,8,2,2,2]`，线性阻尼 `[4,6,6,3,3,3]`，二次阻尼 `[1,1.5,1.5,.5,.5,.5]`，中性浮力，人工 `-6 sin(angle)` 滚转/俯仰恢复；
- 三个理想可逆推力：`surge(T)` ±5 N、`fore(T)`/`aft(T)` ±2 N；`X=surge`、`Z=fore+aft`、`M=0.5*(aft-fore)`；这是“纵向全驱动”的接口测试台，不是最终 UUV；
- 8 个工况：零输入、正/负前进、正/负升沉、正/负俯仰、命令中断；另加一次前进重复；
- 每 8 s 仿真：前 2 s 零、2–4 s 脉冲、4–8 s 释放；控制节点按求解器反馈时间调度；
- watchdog：命令超过 0.25 s 仿真时间或 1 s 墙钟即失效，网关把实际输入归零。

**验收**：

- 8 个工况 + 重复全部通过；
- 每次 160 个有效求解步、160 条 Gazebo 位姿观测、0.05 s 连续；
- 两次前进的位置/速度差为 0；
- 前向 `u=+0.121762`、反向 `u=-0.121762`；升沉 `w=±0.052034`；俯仰 `q=±0.049066`、`pitch=±0.055709`；
- 命令中断在 3.25 s 首次归零；
- 相关测试 30 项通过。

**边界**：这里没有 PID、没有 LOS、没有恒流；它证明“桥和符号”，不证明控制性能。

### 4.3 双推进器 v1：`uuv_twin_axial_v1`（2026-10-25）

**目标**：把“纵向全驱动测试台”换成论文真正要研究的**欠驱动**平台：两台上下分置、沿 +X 的轴向推进器；没有直接升沉执行器。

**实现**：

- 上推进器 FRD `(0,0,-0.25)`，下推进器 `(0,0,+0.25)`，各 ±15 N；
- `X=Tu+Tl`、`Z=0`、`M=0.25*(Tl-Tu)`；输入矩阵秩 2；
- 分配：`Tu=(X-4M)/2`、`Tl=(X+4M)/2`；先算未限幅值，再用统一 `alpha=min(1, 15/max(|Tu|,|Tl|))` 缩放两路，保持广义力方向；不允许直接 Z；
- 初始 ENU `(2,0,-30)`、朝东；规划 `(2,-10)`；
- 工况：零输入 30 s、正/负前进、正/负俯仰、联合饱和、watchdog、stale_stamp，再加前进重复；
- 验收：分配与实际推力一致、Z=0、15 N 限幅、首步加速度符号/幅值、释放后能量衰减、Gazebo 位姿回写 ≤0.01 m/1°、覆盖率 ≥95%、重复性。

**结果**：v1 套件 8+1 全部通过；饱和工况 `alpha=0.75`，实际 `X=15 N`、`M=3.75 Nm`，与解析值一致；前进重复 160 步位置/速度差为 0。

**边界**：这是开环验收，不是 PID；后来数值检查发现 v1 的 xdyn 外力配置漏了附加质量 Coriolis 项，因此在 v2 修正，而不是修改旧结果。

### 4.4 无流数值验证：M4/M5/M6 与“漏项”定位（2026-10-26）

**目标**：在接 PID 前，先回答“Python 参考 3-DoF 和 xdyn 6-DoF 是否在数值上足够一致”。

**实现**：

- 直接调用锁定的 `xdyn-for-cs`，通过 WebSocket 交换输入和完整状态；不启动 Gazebo/ROS，所以这是求解器层验证；
- 输入时间表：0–2 s 零、2–7 s 上/下各 2 N（X=4 N）、7–12 s 上 -1/下 +1（M=0.5 Nm）、12–22 s 零；
- 三种步长 0.05/0.025/0.0125 s；
- Python 参考：原有非线性 3-DoF 模型 + RK4；
- M4：撤除输入后 10 s 总机械能不增加超过 1%；
- M5：步长减半末端差 ≤0.02 m、0.5°、0.01 m/s，且第二次减半误差不增加；
- M6：同初态/输入逐时刻位置 ≤0.05 m、俯仰 ≤1°、u/w ≤0.02 m/s。

**发现**：

- M4/M5/M6 的有限容差检查都通过，但 M6 残差（0.022464 m）**不随步长减小而下降**，说明不是积分误差；
- 锁定 xdyn 源码中 `solid_body_inertia` 只算刚体 Coriolis，而外力配置没有 `-C_A(ν)ν`；原 Python 模型有这一项；
- 仅在诊断包装器中抵消该项后，dt=0.05 的 Python/xdyn 位置差从 0.022464 m 降到 8.87e-11 m，俯仰差从 0.36122° 降到 3.02e-10°。这证明漏项就是主因；
- 正式 M6 始终比较**未修改的 Python 参考**，没有用诊断结果提高通过率。

**边界**：能量检查通过不能排除不做功的耦合项；因此 v1 阶段同时保留“有限容差通过”和“方程尚未完全对齐”两个结论。下一节的 v2 才把该漏项补进 xdyn 并重跑。

### 4.5 v2：把附加质量 Coriolis 放进 xdyn（2026-10-26）

**目标**：在不改 xdyn 源码、不删除 Python 参考项的前提下，补齐 v1 漏掉的 `-C_A(ν)ν`。

**实现**：

- 新建 `ros2/twin_axial_v2/`，v1 和旧记录原样保留；
- 通过 xdyn 的 maneuvering 表达式外力接口，在积分阶段读取当前机体系速度，加入：

```text
X =  5*v*r - 8*w*q
Y =  8*w*p - 5*u*r
Z =  5*u*q - 5*v*p
K = -3*v*w
M =  3*u*w
N =  0
```

- 该外力满足 `ν·force=0`，不产生机械能；纵向退化为 `(-8*w*q, 5*u*q, 3*u*w)`，与原 Python 3-DoF 一致；
- 执行器、阻尼、恢复力、质量和分配逻辑保持 v1 数值；
- 重跑数值检查 + 九次 LOTUSim 开环（零、正/负前进、正/负俯仰、饱和、watchdog、stale_stamp、前进重复）。

**结果**：

| 检查 | 结果 |
|---|---|
| 三步长能量 | 全通过；dt=.05 撤力后 10 s 从 0.6661642096 降到 0.02550008258 J |
| xdyn .05→.025 末端差 | 位置 2.73e-10 m、俯仰 1.89e-7°、速度 1.91e-10 m/s |
| xdyn .025→.0125 末端差 | 位置 2.07e-11 m、俯仰 1.20e-8°、速度 1.18e-11 m/s |
| Python/xdyn 最大差 | 位置 9.32e-11 m、俯仰 3.43e-10°、速度 7.63e-13 m/s、q 5.85e-12 rad/s |
| 九次 LOTUSim 开环 | 全通过；重复前进 160 步位置/速度差为 0 |
| 相关测试 | 159 项通过 |

**边界**：这是**无流、纵向运动**的对照；公式是六维的，但不代表任意 6-DoF 工况或含流工况已验证。加入洋流时必须重新处理相对速度、附加质量和流速导数，不能直接说“含流也成立”。

### 4.6 PID 闭环：C1/C2 阶跃与分配后抗饱和（2026-10-27）

**目标**：在不改原控制器实现、不改 v2 模型和分配器的前提下，把已有 Python 内环 PID 接到真实 xdyn 反馈上，验证速度/俯仰阶跃和联合饱和恢复。

**实现**：

1. **复用原控制器**：`ros2/twin_axial_pid/control.py` 继承 `CascadedPID3DOFController`，只保留内环；位置/深度外环增益设为 0，目标直接通过 `BodyGuidance3DOF(u, pitch)` 传入。
2. **固定候选参数**：速度 `Kp=25, Ki=3`；俯仰 `Kp=20, Ki=1, Kd=22`；恢复前馈系数 6；积分限幅 ±4/±1；内部 `maximum_tau_x/m=1e6`，让物理限幅只发生在双推进器分配器。
3. **分配**：每个控制步先算原始 `X/M`，调用 `allocate()` 得到 `alpha` 和实际 `upper/lower`，再由实际分配后的 X/M 计算抗饱和回算：
   ```text
   Iu_next     = clip(Iu_candidate     + dt*(X_alloc-X_request)/(Ki_u*Tt), ±4)
   Itheta_next = clip(Itheta_candidate + dt*(M_alloc-M_request)/(Ki_theta*Tt), ±1)
   Tt = 1 s, dt = 0.05 s
   ```
   修正影响下一步，而不是“本步悄悄改输出”。
4. **网关复核**：控制器发原始 `X/M`；gateway 独立重复同一分配，记录 `requested_wrench`、`allocated_commands`、`applied_wrench`。验收会把请求、分配、实际施加三者对齐。
5. **记录**：`commands.jsonl` 保存控制器诊断（目标、反馈序号/时间、原始请求、分配、积分前后、回算项）；`transactions.jsonl` 保存 xdyn 真值和实际推力。

**验收**：

- 从记录的反馈逐步**重放** PID：重新计算原始 X/M 和全部分配/抗饱和诊断，要求与记录在 1e-12 相对/绝对容差内一致；
- 请求与实际施加一致；无控制掉线；末段零输入；时间连续；Gazebo 位姿覆盖率 ≥95%；状态包络有效；
- C1/C2 阈值固定：速度 15 s 内进 ±0.04 m/s 并保持、末 5 s MAE ≤0.02 m/s、超调 ≤0.08 m/s；俯仰 15 s 内进 ±3°、末 5 s MAE ≤2°、超调 ≤5°。

**结果**：

| 用例 | 进入误差带并保持 | 末 5 s MAE | 超调 |
|---|---:|---:|---:|
| speed_step 0→0.4 m/s | 2.70 s | 0.001274 m/s | 0 |
| pitch_positive 0→+20° | 2.10 s | 0.2291° | 0.9967° |
| pitch_negative 0→-20° | 2.10 s | 0.2291° | 0.9967° |

补充 `allocation_recovery`：2–10 s 给 `u=0.6 m/s, pitch=60°`，10–40 s 改回 `0.4/0°`；26 个控制步触发联合饱和和 26 次回算，速度 4.70 s、俯仰 2.80 s 进入并保持误差带，末 5 s MAE 0.002512 m/s 和 0.1301°，末段 `alpha` 退出饱和。四个用例共 643/643/643/803 步，Gazebo 位姿覆盖率 100%。

**边界**：这是无流、无障碍、固定目标的内环阶跃；没有 LOS、路径、深度外环（下一节才有）、恒流；`allocation_recovery` 是补充抗饱和试验，不替代正式 C1/C2；候选增益不是实艇辨识结果。

### 4.7 运动与故障：直线、深度、状态/命令断流（2026-10-27）

**目标**：在无流、无障碍条件下，验证“能沿直线走、能改变深度、故障时能安全停下”，为后续导航做动力学闭环准备。

**实现**：

- 复用 v2 模型、PID 适配器、分配器和网关，不改旧参数；
- 新深度外环只生成俯仰参考，不增加升沉执行器：
  ```text
  vertical_up = u*sin(theta) - w*cos(theta)
  pitch_raw = 0.35*(actual_depth - target_depth) - 0.8*vertical_up
  pitch_target = clip(pitch_raw, -25°, +25°)
  u_target = 0.4 m/s
  ```
  外环无积分；深度误差用真实 xdyn 状态；`w` 保持动态，不强制置零；
- 六个工况：`straight`（42 s，30 m）、`depth_up`/`depth_down`（68 s，8 s 时 30→28/32 m）、`state_loss`/`stale_state`/`command_loss`（8 s 注入，12 s 结束）；
- 故障注入在**控制器状态递交边界**和 **ROS 发布边界**：
  - `state_loss`：不再把新状态交给控制器；
  - `stale_state`：持续递交最后一帧旧状态；
  - `command_loss`：抑制实际 ROS 发布（非零和后续零命令都不发），必须由网关独立 watchdog 归零；
- `FeedbackGuard` 监测 0.25 s 墙钟超时、无效状态、序号/时间戳倒退或跳跃，触发后锁存停止；控制器积分清零，不再生成 PID 输出；
- 网关保留 0.25 s 仿真时间/1 s 墙钟的独立命令过期保护；
- 故障用例正常结束状态是 `fault_observed`：表示“预期故障被正确处理”，不是“完成原运动任务”。

**验收**：

- 运动：直线全程深度偏差 ≤0.10 m、末 5 s ≤0.05 m、前进 >10 m；深度变化 40 s 内进入 ±0.10 m 并保持、末 5 s ≤0.05 m、超调 ≤0.20 m；速度/俯仰内环阈值同上一节；无反向前进；包络有效；Gazebo 位姿覆盖率 ≥95%；
- 故障：故障前必须有实际非零闭环输入；归零不晚于最后有效控制时间戳 +0.25 s + 一个积分步；首次零输入后至少 2 s 持续为零；控制器在注入后 0.35 s 内停止并清积分；观测支路继续记录真值，但绝不把真值送回 PID 代替丢失状态。

**结果**：

| 试验 | 实测 |
|---|---|
| 直线 | 前进 15.4062 m；深度偏差 0；末 5 s 速度 MAE 0.000421 m/s |
| 上浮 30→28 m | 20.9 s 进入并保持 ±0.10 m；末 5 s 深度 MAE 0.007156 m；超调 0.01504 m |
| 下潜 30→32 m | 同上，方向相反 |
| 状态断流 | 最后有效控制时间戳后 0.25 s 施加零推力；`state_timeout` 停止，积分清零 |
| 过期状态 | 最后有效控制时间戳后 0.05 s 施加零推力；`stale_state` 停止，积分清零 |
| 命令断流 | 最后有效控制时间戳后 0.30 s 由独立 watchdog 施加零推力；`command_watchdog` 停止 |

两项深度变化末 5 s 最大俯仰绝对值约 0.06287°，前进约 25.6084 m；三类故障首次归零后持续零输入超过 2 s；六项 Gazebo 位姿覆盖率为 100%；相关回归测试 181 项通过。

**边界**：这是消息边界的故障注入，不修改网络、不停止求解器、不伪造求解器返回值；没有覆盖网关崩溃、全硬件故障或暂停仿真时钟的整链路墙钟试验。故障通过不等于完成所有 C3 场景，更不等于导航任务中的故障处理已经验收。

### 4.8 回放与可视化：把“证据”变成可看的 Gazebo 场景

**目标**：让已完成的试验记录可以在 Gazebo 中按原始位姿回放，用于检查模型外观、方向和轨迹；**不运行 xdyn、不接 PID、不产生新的动力学结论**。

**演化**：

1. 早期 `ros2/lotusim_replay/` 把冻结的 3-DoF 历史轨迹映射到 LOTUSim：先发 renderer 话题，后来用 MAS 的 CREATE/MOVE/DELETE 在 Gazebo 里创建实体；这只是显示，不重新积分。
2. 新链路完成后，`ros2/recorded_replay/` 改为读取**已完成的 LOTUSim/xdyn 试验目录**：
   - 校验 `manifest.json` 的全部哈希、`run_status=completed`、位姿有限性、单位四元数、时间严格递增；
   - 从 `worlds/minimal.world` 按白名单提取模型 link pose 和 visual，移除原世界、物理接口、`lotus_param`、碰撞、传感器和插件；
   - 生成只含 MAS + SceneBroadcaster 的展示世界；GUI 启动后通过 MAS action 创建实体；
   - 位置线性插值、姿态沿最短弧插值；终端命令控制暂停、速度、循环、重启；
   - 退出后复核源文件哈希，输出到原记录之外的新目录。
3. 本轮新增**套件模式**：识别 `suite.json`，按顺序校验四个用例，`next`/`prev`/`case NAME`/`list` 切换；因为四例共享同一个内联视觉模型，切换只换位姿源，不重建模型；镜头按全部用例的位姿包围盒取景（`--camera fit`，可用 `--camera close` 近看）。

**实测**：

- `--validate-only` 读取本套件：4 个用例、643/643/643/803 个位姿全部通过，四个 `acceptance.json` 均 `passed: true`；
- 无界面 Gazebo 完整回放 `speed_step`：161 条 MOVE、175 条观测；末条观测 `[13.471, 0, -30]` 与末帧记录一致；`status=completed`、`source_unchanged=true`；
- 运行中 `next` 切到 `pitch_positive`，`summary.active_case` 正确；
- 带 GUI 的 `pitch_negative` 回放以 4x 播放完成并自动关闭；
- 回放相关测试 10 项通过。

**边界**：回放发送的是 MOVE，不是动力学；它不能作为模型、控制或导航验收。`gazebo_poses.jsonl`/`observations.jsonl` 是显示反馈，不是求解器真值。GUI 的最终画面可见性仍需在具体 WSLg/图形环境确认；本次自动截屏受 WSLg/EGL 限制，只得到空白 3D 区域，未把截图当作可见性证据。

---

## 5. 证据链：为什么这些结论不是“看窗口看起来对”

各阶段的动力学试验共享以下证据骨架；`events.jsonl` 和 `feedback_delivery.jsonl` 只在运动/故障阶段出现。

| 文件 | 说明 | 证明什么 |
|---|---|---|
| `model.yml` | xdyn 模型配置 | 用的是哪套方程/参数 |
| `worlds/minimal.world` | Gazebo 世界和载具 SDF | 用的是哪个实体/插件/接口 |
| `source/` | 本次运行相关源码快照 | 代码不是“事后改过” |
| `flake.lock` | 依赖锁定 | LOTUSim/xdyn 版本可复现 |
| `metadata.json` | 工况、dt、时长、可执行文件路径 | 运行配置 |
| `commands.jsonl` | 控制节点发布的请求/目标/时间戳/序号 | 控制器想做什么 |
| `transactions.jsonl` | 请求状态、实际命令、xdyn 状态、分配、watchdog、坐标转换 | 求解器实际做了什么 |
| `gazebo_poses.jsonl` | 独立订阅的 Gazebo 位姿 | 结果是否真的写回并显示 |
| `events.jsonl` | 故障注入、停止原因、积分前后（运动/故障阶段） | 故障处理过程 |
| `feedback_delivery.jsonl` | 观测序号 vs 实际递交控制器的状态（运动/故障阶段） | 断流/冻结是否真的发生 |
| `run_status.json` | 完成/故障/中断、进程退出码、清理错误 | 进程生命周期 |
| `manifest.json` | 除自身和 `acceptance.json` 外全部文件的 SHA-256 | 原始证据未被改动 |
| `acceptance.json` | 由记录重算的指标/检查 | 本阶段结论 |

几个重要的“防自欺”设计：

1. **请求、分配、实际三分开**：控制器只发 X/M；网关重新分配；`transactions.jsonl` 记录 `requested_wrench`、`allocation`、`applied_commands`、`applied_wrench`。验收要求三者一致，而不是只看控制器的自我报告。
2. **求解器真值与 Gazebo 位姿分开**：`/state` 是 xdyn 真值；`/poses` 是 Gazebo 实体位姿。验收要求两者在 0.01 m/1° 内匹配且覆盖率 ≥95%，防止“求解器在算，但 Gazebo 没写回”。
3. **连续性检查**：gateway 每一步要求当前请求状态等于上一步返回状态（位置/速度/四元数差 ≤1e-5），否则报错关闭。这防止把重置/回放当成连续动力学。
4. **看门狗在网关**：控制进程崩溃或命令丢失时，gateway 仍能把实际输入归零；不是“控制器自己保证”。
5. **只读重算**：`analyze` 重新读记录、核对哈希、逐步重放对应阶段的控制器/分配（阶跃与运动记录），不启动 xdyn/Gazebo；所以“重新分析通过”不是“重新跑一遍碰巧通过”。
6. **不覆盖旧记录**：每次新目录；失败也保留；suite 逐 case 记录退出码。
7. **阈值先固定**：`protocol.py`/`controller.json`/`acceptance.py` 中固定工况、目标、阈值；验收会核对 `controller.json` 与当前参数完全一致，防止“看到结果后调参”。

一个容易误解的点：`manifest.json` 保护的是原始证据文件，`acceptance.json` 是**派生报告**，不在 manifest 中；`analyze` 会重新读记录、核对哈希、逐步重放控制器并打印重算结果，但不写回原目录、不改 `acceptance.json`。因此引用结论时应以原始记录 + 重算结果为准，而不是只信某次运行留下的一个 `passed` 字段。

---

## 6. 当前阶段：已经到哪一步

### 6.1 时间线

| 时间 | 里程碑 | 关键结果 |
|---|---|---|
| 2026-09 | 纯 Python 3-DoF 仿真导航基线冻结 | Physics/Planner/Guidance/Controller/Navigation 完成各自范围；LOS 10/16、P1 7/12 等历史结果保留 |
| 2026-10-17/19 | 冻结历史 → LOTUSim 显示回放 | 证明历史位姿可进 Gazebo；不证明实时动力学 |
| 2026-10-20/21 | 最小开环桥 `uuv_minimal_xdyn_v1` | 8+1 工况通过；桥、坐标、时间、watchdog 正确 |
| 2026-10-25 | 双推进器 v1 开环 | 分配、欠驱动、方向、限幅、回写通过 |
| 2026-10-26 | M4/M5/M6 数值检查 → 发现漏项 → v2 | 补附加质量 Coriolis；数值和九次开环通过 |
| 2026-10-27 | PID C1/C2 + 分配恢复 | 速度/正负俯仰阶跃通过；抗饱和有效 |
| 2026-10-27 | 直线/深度/故障边界 | 直线 15.4062 m；深度 20.9 s 稳定；三类故障按截止归零 |
| 2026-10-27 | 只读套件回放 | 四例全校验；无界面完整回放/切换/GUI 启动 |

各阶段文档记录的回归测试数为：最小开环 30、v2 159、PID 169、motion 181；本次回放新增 10 项。它们分别是各自阶段的回归集合，不能简单相加成“总测试数”。

### 6.2 用验收编号看

| 编号 | 状态 | 依据/缺口 |
|---|---|---|
| M1 分配与欠驱动 | **通过（开环检查覆盖）** | 输入矩阵秩 2、Z=0、±15 N、X/M 重构误差检查 |
| M2 零输入静止 | **通过（开环检查覆盖）** | 30 s 零输入漂移检查 |
| M3 零状态瞬时导数 | **通过（开环检查覆盖）** | 首步加速度与解析值 1% 内 |
| M4 无流机械能 | **通过（v2 数值）** | 撤力 10 s 能量不增 |
| M5 步长收敛 | **通过（v2 数值）** | 三步长末端差远小于阈值 |
| M6 Python/xdyn 配对 | **无流纵向通过** | v2 严格容差；含流未做 |
| M7 坐标往返/时间 | **通过（开环检查覆盖）** | 开环与回写检查 |
| C1 速度阶跃 | **通过** | 2.70 s 稳定，末 5 s MAE 0.001274 |
| C2 正负俯仰 | **通过** | 2.10 s 稳定，MAE 0.2291°，超调 0.9967° |
| C3 状态/指令断流 | **部分通过** | 消息边界三类故障；未在导航任务中复验，未覆盖网关崩溃/硬件故障 |
| C4 安全到达 | **未开始** | 无新模型障碍任务 |
| C5 对照/边界 | **未开始** | 无流/恒流对照、45 次矩阵 |
| C6 可复现+论文 | **部分** | 依赖/记录/测试强；论文和最终打包未完成 |
| N1–N6 | **未开始** | 需要点到点导航任务 |
| R1/R2 | **开环版本通过** | 位姿回写、重复性；导航版本未做 |

### 6.3 一句话定位

> **已经完成“模型 + 执行器 + 内环 + 深度 + 消息边界故障”的无流闭环，并通过真实 xdyn/LOTUSim 记录验收；尚未完成“规划/LOS + 静态障碍 + 恒流 + 点到点导航”的论文后半段。**

这意味着：

- 现在可以可靠地说“这个合成双推进器 UUV 在无流、无障碍条件下能以真实 6-DoF 动力学完成速度/俯仰/深度控制和故障停机”；
- **不能**说“已完成 UUV 导航系统”；
- **不能**说“含流模型已验证”；
- **不能**用回放或 Gazebo 画面替代上述验收。

---

## 7. 尚未完成的关键工作与建议顺序

### 7.1 先接 LOS/规划，不先调 PID

下一步不是继续调增益，而是把已有的 LOS/Planner 参考转换并接入 `BodyGuidance3DOF(u, pitch)`，在无流直线/平缓路径上验证“规划—引导—控制—xdyn”全链路。建议先做：

1. 固定直线/单折线路径，起点 `(2,-10)`，目标在 x-z 平面；不接障碍；
2. 把现有 `Guidance v1.0` 的进度/LOS 参考转换为 `BodyGuidance3DOF(u, pitch)`；控制器和分配器不变；
3. 验收：实际轨迹连续、无 MOVE、位置/姿态回写、速度/俯仰包络、路径跟踪误差、无控制掉线；
4. 失败时先区分“几何参考不可行”“执行器饱和”“控制器跟踪不足”，不要直接改阈值或加增益。

### 7.2 再接静态障碍与 C2/C4

在无流直线通过后，把旧 Python 环境里的三个静态场景（单挡板、双挡板、紧双挡板）绑定到新模型：

- 统一规划平面、ENU 偏移、障碍 y 向厚度、碰撞包络（半径 0.70 m，安全裕度 0.20 m，净距 ≥0.90 m）；
- 先做几何连通性与包络预检，再做真实动力学；
- 验收到达（N1）、超时（N2）、安全（N3）、路径跟踪（N4）、饱和比例（N5）、名义成功率（N6）。

这一步完成前，不能把“直线/深度通过”当作导航能力。

### 7.3 再接恒流与 C5

恒流不是“在位置上加偏移”，而是进入相对速度、阻尼、附加质量和恢复力的计算。需要：

- 在 xdyn 模型中用相对速度 `ν_r = ν - ν_c` 重写阻尼/附加质量耦合；
- 处理惯性系恒流在旋转艇体系下的导数；
- 重跑 M6 的含流对照，再做无流/两种非零恒流的同场景比较；
- 检查 `Z` 执行器仍为 0，欠驱动关系不被“流水推力”绕过。

当前 v2 文档明确只支持无流；不要直接把绝对机体速度公式称为“含流已验证”。

### 7.4 最后做 45 次导航矩阵与论文

按范围基线：

- 3 场景 × 3 水流 × 5 初态 = 45 次正式导航；
- 同一控制器/规划/分配配置，固定种子，失败不重试替换；
- 每次保存完整状态、命令、路径、安全距离、到达/失败原因；
- 先冻结 release manifest，再跑矩阵；不能看完结果放宽阈值；
- 同时补齐 C6 的论文、图表、复现说明。

### 7.5 展示层

回放已经能用于检查和演示，但：

- 不要在动力学试验中使用 MOVE；
- 不要让回放输出参与 `acceptance.json`；
- GUI 可见性要在实际 NixOS/WSLg 桌面确认；本会话的自动截屏受 EGL/WSLg 限制，只得到空白 3D 区域，不能据此判断模型不可见，也不能据此宣布可见。

---

## 8. 如何运行（速查）

所有命令都在 WSL/NixOS 中、项目根目录执行；输出目录必须是全新的。

| 目的 | 命令 | 说明 |
|---|---|---|
| 双推进器开环套件 | `bash ros2/twin_axial_v2/run-nix.sh suite --output artifacts/twin_axial_v2/suite_$(date +%Y%m%d_%H%M%S)` | 9 次，无 GUI，自动起停三个进程 |
| 无流数值检查 | `bash ros2/twin_axial_v2/run-nix.sh numerics --output artifacts/twin_axial_v2_numerics/check_$(date +%Y%m%d_%H%M%S)` | 三种步长，不启动 Gazebo/ROS |
| PID 阶跃套件 | `bash ros2/twin_axial_pid/run-nix.sh suite --output artifacts/twin_axial_pid/suite_$(date +%Y%m%d_%H%M%S)` | 四个用例 |
| 直线/深度/故障套件 | `bash ros2/twin_axial_motion/run-nix.sh suite --output artifacts/twin_axial_motion/suite_$(date +%Y%m%d_%H%M%S)` | 六个用例 |
| 只读复核 | `bash ros2/twin_axial_pid/run-nix.sh analyze artifacts/twin_axial_pid/validation_20261027_final` | 不跑动力学；motion 入口同样支持套件，v2 需指定到单一 case 子目录 |
| 套件回放 | `bash ros2/recorded_replay/run-nix.sh artifacts/twin_axial_pid/validation_20261027_final` | 默认暂停 0.25x；`p` 播放，`next`/`prev`/`case`/`list` 切换 |
| 只校验回放 | `... --validate-only` | 不启动 Gazebo |
| 只生成场景 | `... --prepare-only --output <新目录>` | 写 world/SDF/来源记录，不启动 Gazebo |

常用隔离/端口：

- 双推进器/PID/motion：ROS domain 74，端口 12356/12357；
- 最小开环：ROS domain 72，端口 12346/12347；
- 数值检查：端口 12358；
- 回放：独立 Gazebo partition + ROS domain 83，无 xdyn。

---

## 9. 术语与文件索引

| 术语 | 含义 |
|---|---|
| `uuv_minimal_xdyn_v1` | 三输入、纵向全驱动的接口测试台，不是最终欠驱动模型 |
| `uuv_twin_axial_v1` | 双推进器、无直接升沉的开环模型 |
| `uuv_twin_axial_v2` | v1 + xdyn 内计算的附加质量 Coriolis；本轮闭环使用 |
| M1–M7 | 分配/欠驱动、零输入、首步导数、能量、步长收敛、Python-xdyn 对照、坐标/时间检查 |
| C1–C3 | 速度阶跃、俯仰阶跃、状态/指令断流 |
| C4–C6 | 安全到达、对照与边界、可复现系统与论文 |
| N1–N6 / R1–R2 | 导航到达/超时/安全/跟踪/饱和/成功率；位姿回写/重复性 |
| MAS | LOTUSim 的 MultiAgentSystem 插件，管理实体的创建、删除、位姿 |
| physics interface | LOTUSim 中把外部动力学（xdyn）接入 Gazebo 的插件 |
| gateway | 本项目的 WebSocket 代理 + 看门狗 + 记录器；不是求解器 |

关键代码：

- LOTUSim 最小接入：[`ros2/minimal_xdyn/trial.py`](../ros2/minimal_xdyn/trial.py)、[`gateway.py`](../ros2/minimal_xdyn/gateway.py)、[`model.py`](../ros2/minimal_xdyn/model.py)
- 双推进器模型/分配/网关：[`ros2/twin_axial_v2/model.py`](../ros2/twin_axial_v2/model.py)、[`allocation.py`](../ros2/twin_axial_v2/allocation.py)、[`gateway.py`](../ros2/twin_axial_v2/gateway.py)
- PID 适配与抗饱和：[`ros2/twin_axial_pid/control.py`](../ros2/twin_axial_pid/control.py)
- 深度外环/故障：[`ros2/twin_axial_motion/protocol.py`](../ros2/twin_axial_motion/protocol.py)、[`safety.py`](../ros2/twin_axial_motion/safety.py)、[`trial.py`](../ros2/twin_axial_motion/trial.py)
- 只读回放：[`ros2/recorded_replay/core.py`](../ros2/recorded_replay/core.py)、[`cli.py`](../ros2/recorded_replay/cli.py)

关键记录：

- `artifacts/minimal_xdyn/validation_20261020_01`
- `artifacts/twin_axial_v2_numerics/validation_20261026_01`
- `artifacts/twin_axial_v2/validation_20261026_01`
- `artifacts/twin_axial_pid/validation_20261027_final`
- `artifacts/twin_axial_motion/validation_20261027_final`
- `artifacts/twin_axial_pid_replays/verify_headless_speed`、`verify_switch_clean`、`verify_gui_pitch`

---

## 10. 结论

项目已经完成了一次重要的“平台迁移”：

- 从**纯 Python 3-DoF 参考仿真**迁移到 **LOTUSim + 真实 xdyn 6-DoF 外部积分**；
- 从“播放历史轨迹”迁移到“控制节点驱动真实动力学、Gazebo 只负责实体和显示”；
- 从“看曲线”迁移到“请求/分配/实际/求解器真值/Gazebo 回写”五层证据；
- 从单记录回放扩展到套件校验与切换。

目前的准确表述是：

> **无流、无障碍条件下的双推进器 UUV 动力学闭环（含速度/俯仰/深度控制和消息边界故障停机）已经实现并通过验收；导航层尚未接入，含流模型尚未验证，C4–C6 尚未完成。**

下一步应把已有的 LOS/Planner 接到这条真实动力学链路上，再做静态障碍、恒流和 45 次导航矩阵；在此之前，不应把开环、阶跃或回放结果扩大为“导航系统已完成”的结论。
