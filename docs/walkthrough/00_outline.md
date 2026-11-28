# UUV Simulator 讲解大纲（00 总目录）

生成日期：2026-11-28
用途：把整个 `uuv_simulator` 仓库拆成可逐块讲解的单元。本文件是目录与索引，
后续每一部分单独成文（`01_architecture.md`、`02_physics.md` …）。

---

## 0. 一句话概括

这个仓库不是"一个仿真器"，而是**同一套算法内核 + 两套运行后端**：

| 栈 | 物理/渲染后端 | 语言/通信 | 用途 | 代码位置 |
|---|---|---|---|---|
| **A. 纯 Python 3-DOF 栈** | 自研纵平面 3-DOF 方程 + RK4 | 进程内 Python 调用 | 学术基线、可复现、单测/验收 | `core/ environment/ physics/ controller/ planner/ trajectory/ guidance/ navigation/ simulation/` |
| **B. LOTUSim + xdyn 6-DOF 栈** | xdyn 6-DOF 求解器 + Gazebo/LOTUSim 场景 | ROS 2 DDS + WebSocket + JSONL IPC | 工程闭环、真实链路、证据矩阵 | `ros2/`、`external/LOTUSim`、`delivery/` |

两栈共用 A 栈的 `core / environment / planner / trajectory / controller / guidance`
（B 栈直接 import 它们），差别只在**动力学从 3-DOF 自研方程换成 xdyn 6-DOF、
控制回路由进程内函数调用换成 ROS 2 话题 + WebSocket 网关**。

---

## 1. 分层与模块清单

数据流（A 栈，`navigation/navigator.py` 实际执行的顺序）：

```
WorldModel ──► A* Planner ──► Path ──► Trajectory Generator ──► Trajectory
                                                                    │
                            ┌───────────────────────────────────────┘
                            ▼
      (可选) Startup / LOS Guidance / TerminalApproach / TerminalCapture
                            │  BodyGuidance3DOF(u, pitch, q) 或 VehicleState 参考
                            ▼
                    Cascaded PID 3-DOF ──► ControlInput(τx, 0, τm)
                            │
                            ▼
                    Physics Backend.step ──► 新 VehicleState
                            │
                            ▼
               WorldModel 连续碰撞/边界检查 ──► 下一步
```

### 1.1 Core（数据结构，v0.2）

| 文件 | 内容 | 要点 |
|---|---|---|
| `core/pose.py` | `Pose(x,y,z,roll,pitch,yaw)` | ENU 惯性系；不可变 frozen dataclass |
| `core/twist.py` | `Twist(u,v,w,p,q,r)` | SNAME 船体系 |
| `core/control_input.py` | `ControlInput(X,Y,Z,K,M,N)` | 广义力/力矩 |
| `core/vehicle_state.py` | `Pose + Twist + timestamp` | `to_numpy()` 返回 12 维 `[η;ν]`，时间戳不进向量 |
| `core/path.py` | 不可变 `Pose` 序列 | 纯几何，无时间 |
| `core/trajectory.py` | 不可变 `VehicleState` 序列 | 时间只存在 `VehicleState.timestamp` |

注：`core/controlinput.py` 是历史遗留的空/兼容文件，真实实现在 `control_input.py`。

### 1.2 Environment（规划世界，v0.3）

| 文件 | 内容 |
|---|---|
| `environment/obstacle.py` | Shapely 支撑的圆/矩形/多边形静态障碍，`distance_to`、`segment_clearance` |
| `environment/boundary.py` | 多边形/矩形作业边界，`contains_segment` |
| `environment/current.py` | `CurrentField` 抽象；`NoCurrent`、`ConstantCurrent` |
| `environment/world.py` | `WorldModel`：contains / collides / clearance / nearby_obstacles / current_at / segment_clearance / is_segment_valid |
| `environment/scenario.py` | Python/YAML 场景加载（唯一规划世界来源） |
| `environment/coordinates.py` | Core `(x,z)` ↔ 规划 `[x,z]` 显式映射 |

### 1.3 Physics（自研 3-DOF，v0.4）

| 文件 | 内容 |
|---|---|
| `physics/parameters.py` | `UUV3DOFParameters`：对角质量/附加质量/线性+二次阻尼/输入选择矩阵/恢复力矩系数，带正定与非负校验 |
| `physics/frames.py` | `body_to_world_velocity_matrix(θ)`、`world_current_to_body` |
| `physics/uuv_3dof.py` | `Python3DOFDynamics.derivatives()`（连续方程）+ `Python3DOFBackend.step()`（定步长） |
| `physics/integrators.py` | `EulerIntegrator`、默认 `RK4Integrator` |
| `physics/backend.py` | 12 维 ↔ 6 维约简：索引 `[0,2,4,6,8,10] = x,z,θ,u,w,q` |

### 1.4 Controller（v0.5 / v0.5.1）

| 文件 | 内容 |
|---|---|
| `controller/cascaded_pid_3dof.py` | 串级 PID 主体、条件积分抗饱和、前馈、诊断快照 |
| `controller/parameters.py` | 增益与限幅参数（全部非负校验） |
| `controller/limits.py` | `ControlLimits3DOF`：本步可用 τx/τm 上限（先于积分更新） |
| `controller/guidance.py` | `BodyGuidance3DOF(u, pitch, q)`：显式内环目标，绕过外环 |
| `controller/backend.py` | 三个协议：`Controller3DOF`、`LimitAwareController3DOF`、`GuidanceController3DOF` |

### 1.5 Planner（v0.6a/b/c）

| 文件 | 内容 |
|---|---|
| `planner/space.py` | `PlanningSpace` / `BoundedPlanningSpace` 协议 + `EnvironmentPlanningSpace` 适配器 |
| `planner/algorithms/grid/occupancy.py` | x-z 占用栅格，节点级 clearance 认证，4/8 邻接，禁止对角切角 |
| `planner/algorithms/grid/astar.py` | 欧氏 A*，目标区域、虚拟精确终点 `None`、路径长度剪枝、资源预算 |
| `planner/candidates.py` | `simplify_path`（共线消除/安全捷径）+ `CandidatePlanner`（多 buffer 生成 + 几何代价排序） |
| `planner/validation.py` | `validate_request` / `validate_path`（逐点 + 逐段 + 端点 + 长度） |

### 1.6 Trajectory（v0.7）

| 文件 | 内容 |
|---|---|
| `trajectory/generator.py` | 去重 → 贪心捷径 → 按 `sample_spacing` 细分 → 等速时间参数化 |
| `trajectory/sampling.py` | `sample_reference()`：线性插值 + 出向姿态，端点保持零速 |
| `trajectory/circular_fillet.py` | 圆角平滑（C1 切线连续、sagitta 误差上界、胶囊体安全认证） |

### 1.7 Guidance（v1.0 Phase A/B）

| 文件 | 内容 |
|---|---|
| `guidance/progress_projection.py` | 有限段投影、单调 committed_s、前向窗口、位移+slack 夹紧、切换门限 |
| `guidance/los_guidance.py` | 固定前视 LOS：`target = point_at_s(s + lookahead)`，`pitch = atan2(dz,dx)` |
| `guidance/guidance_command.py` | `GuidanceCommand`（pitch_ref / speed_ref / terminal_ready / projection 证据） |

### 1.8 Navigation（v0.8）

| 文件 | 内容 |
|---|---|
| `navigation/navigator.py` | 唯一编排器：plan → trajectory → 循环（startup/approach/LOS/capture → controller → physics → safety） |
| `navigation/los_adapter.py` | LOS ↔ `BodyGuidance3DOF` 适配 + `spatial_metrics` |
| `navigation/startup.py` | `ControlledStartupPolicy`：制动 → 对准首向 → 合格后释放参考时钟 |
| `navigation/terminal_capture.py` | `TerminalCapturePolicy`：BRAKE → ALIGN → MOVE 状态机 |
| `navigation/terminal_approach.py` | `TerminalApproachPolicy`：有限候选 roll-out 预测 + 状态偏差守卫 |
| `navigation/models.py` | `NavigationStatus`（12 种终态）、`NavigationRequest/Config/Step/Result` |

### 1.9 Simulation（编排演示，非生产 Simulation Manager）

`simulation/vehicle_config.py`、`controller_config.py`、`core_integration_demo.py`、
`environment_smoke_demo.py`、`physics_smoke_demo.py`、`control_tracking_demo.py`、
`planner_smoke_demo.py`、`trajectory_demo.py`。

### 1.10 ROS 2 / LOTUSim 栈（B 栈）

分三代演进，**只保留最新可用版本作为主线**：

| 代 | 目录 | 角色 |
|---|---|---|
| 数值/协议基线 | `ros2/twin_axial/`、`twin_axial_v2/` | xdyn 模型生成、协议校验、双轴分配 |
| 制导筛选 | `twin_axial_feasible*/`、`twin_axial_s3_transient*/` | 场景、候选路径、12 组 guidance 筛选 → 冻结 G12 |
| 真实链路 | `twin_axial_g12_*`（boundary → link_v4/v5/v6 → real_v3 → current_v10） | 网关、IPC、DDS 配置、trial、gate、acceptance |
| 场景扩展 | `twin_axial_s4_*` | 五障碍 S4、A/B/C/D 路径对照、终端 4 m 段 |
| 联合 | `twin_axial_s4_current_v1` | S4 D 路径 + v10 实测流 |

关键文件：

- `ros2/twin_axial_v2/model.py`：xdyn `model.yml`（6-DOF、双推进器 ±0.25 m、15 N 限幅、阻尼/附加质量/恢复力矩）
- `ros2/twin_axial_v2/allocation.py`：`(X,M) → upper/lower`，方向保持缩放 `α = min(1, 15/peak)`
- `ros2/twin_axial_g12_link_v6/gateway.py`：12356 收 LOTUSim、转发 12357 给 xdyn、原样回传、watchdog 归零
- `ros2/twin_axial_g12_link_v6/ipc.py`：有界追加读，ENODATA 不提交 offset
- `ros2/twin_axial_g12_link_v6/trial.py`：ROS 2 主循环、状态连续性校验、异常清理
- `ros2/twin_axial_feasible_v2/guidance.py`：G12 制导内核（预瞄弯道、限速、电流补偿、发射对齐）
- `ros2/twin_axial_pid/control.py`：`AllocatedPID`，分配后反算积分修正
- `ros2/twin_axial_g12_current_v10/schedule.py`：NOAA 实测流 6 样本 + smooth gate

### 1.11 证据与验收

- `validation/`：30 个验收/实验 runner（planner、navigation、control、guidance matched 16、terminal、envelope…）
- `tests/`：106 个测试文件；历史记录 1256 passed（2026-11-20）
- `artifacts/`：每次运行的不可变输出（`*_YYYYMMDD_NN`）
- `deliverables/`：论文/答辩用的 docx、pptx、PNG/SVG 与 manifest

---

## 2. 算法清单（"有哪些不同的算法"）

### 2.1 动力学

| # | 算法 | 位置 | 说明 |
|---|---|---|---|
| P1 | 纵平面 3-DOF 刚体 + 附加质量 + 阻尼 + 恢复力矩 | `physics/uuv_3dof.py` | `Mν̇ = τ − Cν − C_A ν_r − D(ν_r)ν_r − g + M_A ν̇_c` |
| P2 | 定步长 Euler / RK4 | `physics/integrators.py` | RK4 默认；每步验证有限性 |
| P3 | 世界流 → 船体流 + 旋转附加质量修正 | `physics/frames.py` + `uuv_3dof.py` | `ν_r = ν − [u_c, w_c, 0]`，再补 `M_A ν̇_c` |
| P4 | xdyn 6-DOF 全模型 | `ros2/twin_axial_v2/model.py` | RK4 内每级求值，无状态裁剪 |
| P5 | 6-DOF 电流"水动力增量"注入 | `ros2/twin_axial_current/model.py` | 不改内置流，只加 `Δ` 力 |

### 2.2 控制与分配

| # | 算法 | 位置 | 说明 |
|---|---|---|---|
| C1 | 串级 PID（位置→速度、深度→俯仰） | `controller/cascaded_pid_3dof.py` | 外环 P，内环 PD + I |
| C2 | 条件积分抗饱和 | 同上 `_conditional_integral` | 饱和且误差继续同号时冻结积分 |
| C3 | 恢复力矩前馈 | 同上 | `ff = k_ff · sin(θ)`，默认 0，候选 6 |
| C4 | 分配后反算（back-calculation） | `ros2/twin_axial_pid/control.py` | 用实际分配力矩修正积分项 |
| C5 | 双轴推进器方向保持分配 | `ros2/twin_axial_v2/allocation.py` | 保方向缩放，不裁剪单轴 |

### 2.3 制导

| # | 算法 | 位置 | 说明 |
|---|---|---|---|
| G1 | 有限段投影 + 单调进度 | `guidance/progress_projection.py` | 不回溯、前向窗口、位移+slack 双夹紧 |
| G2 | 固定前视 LOS | `guidance/los_guidance.py` | 恒速、无减速、`terminal_ready` 只是移交资格 |
| G3 | 预瞄弯道 G12 | `ros2/twin_axial_feasible_v2/guidance.py` | `bend = max−min` 于 2 m 预瞄 → 插值限速 |
| G4 | 电流/侧滑补偿 | 同上 | `pitch = χ + atan2(w_c + w_res, max(0.08, u))` |
| G5 | 发射对齐（ARMING/ALIGN/TRACK） | 同上 | 姿态对齐并 dwell 0.5 s 才放行速度 |
| G6 | 终端捕获状态机 | `navigation/terminal_capture.py` | BRAKE → ALIGN → MOVE → BRAKE |
| G7 | 终端接近预测（roll-out） | `navigation/terminal_approach.py` | 3 速度 × 2 刹车距离 = 6 候选，前瞻 30 s |
| G8 | 受控启动 | `navigation/startup.py` | 制动 → 转首向 → 合格释放时钟 |

### 2.4 规划与轨迹

| # | 算法 | 位置 | 说明 |
|---|---|---|---|
| A1 | 占用栅格 + 4/8 邻接 | `planner/algorithms/grid/occupancy.py` | 节点认证、禁切角、网格预算 |
| A2 | 欧氏 A*（目标区域 + 虚拟终点） | `planner/algorithms/grid/astar.py` | `h = max(0, d − tol)`，`g` 冲突时丢弃陈旧堆项 |
| A3 | 安全捷径 / 共线消除 | `planner/candidates.py::simplify_path` | 不消除 U 形回转 |
| A4 | 多 buffer 候选 + 几何代价排序 | `planner/candidates.py::CandidatePlanner` | 5 档 buffer，代价 = 长度+转角+净空+间距 |
| A5 | 贪心 shortcut 重采样 | `trajectory/generator.py` | 逐步退让直到可见 |
| A6 | 圆角 fillet（C1） | `trajectory/circular_fillet.py` | 半径固定、sagitta 上界、胶囊体认证 |
| A7 | 二次 Bezier 转角平滑 | `ros2/twin_axial_s4_planning_v1/study.py::smooth` | 4 档 trim 回退，失败保留拒绝 |
| A8 | 解析候选路径生成 | `ros2/twin_axial_feasible_v2/guidance.py::build_path` | 双交替挡板 / 单挡板绕行 + 瓶颈角 |
| A9 | 等速时间参数化 | `trajectory/generator.py` | 每个采样点一个时间戳，端点零速 |

### 2.5 环境与水流

| # | 算法 | 位置 | 说明 |
|---|---|---|---|
| E1 | Shapely 连续碰撞 + 净空 | `environment/obstacle.py`、`world.py` | 段-圆/段-矩形最小距离 |
| E2 | 常流 F1/F2 | `environment/current.py`、`ros2/*/current_model.py` | F0 零流、F1 −0.05 x、F2 +0.05 z |
| E3 | 分段时间流 v4/v5 | `ros2/twin_axial_g12_current_v4/v5` | 斜坡/限幅/弱化等变体 |
| E4 | NOAA 实测流 v10 | `ros2/twin_axial_g12_current_v10/schedule.py` | 6 样本线性插值 × 平滑门（60–150 s） |
| E5 | 电流数据契约 | `ros2/twin_axial_current_data_v1/schema.py` | measured / synthetic / estimated 三类显式分离 |

### 2.6 评估与门禁

| # | 机制 | 位置 | 说明 |
|---|---|---|---|
| V1 | 轨迹指标 | `navigation/navigator.py::finish` | RMSE、max、终端误差、饱和率、控制能量代理 |
| V2 | 空间指标 | `navigation/los_adapter.py::spatial_metrics` | 路径偏差、进度单调性、切换次数、夹紧次数 |
| V3 | 验收阈值 | `ros2/twin_axial_s3_transient/study.py::THRESHOLDS` | 净空 0.9 m、RMS 0.35、max 0.8、饱和 20% … |
| V4 | 运行门禁 | `ros2/twin_axial_g12_link_v6/gate.py` | release 绑定、600 s 新鲜度、PilotProof |
| V5 | 工况矩阵 + repeat | `ros2/twin_axial_g12_matrix/`、`scripts/s4_joint18_*` | 3 流 × 6 初始 = 18；补充至 45 条 |

---

## 3. 接口清单（"接口怎么联通"）

### 3.1 进程内（A 栈）——Python Protocol

| 接口 | 定义位置 | 方法签名 |
|---|---|---|
| `PhysicsBackend` | `physics/backend.py` | `step(state, control, current_velocity, dt) -> VehicleState` |
| `Controller3DOF` | `controller/backend.py` | `step(current, reference, dt) -> ControlInput`、`reset()` |
| `LimitAwareController3DOF` | 同上 | `step_with_limits(current, reference, dt, limits)` |
| `GuidanceController3DOF` | 同上 | `step_guidance(current, BodyGuidance3DOF, dt, limits)` |
| `GlobalPlanner` | `planner/base.py` | `plan(request, space) -> PlanningResult` |
| `PlanningSpace` | `planner/space.py` | `contains` / `clearance` / `is_pose_valid` / `is_segment_valid` |
| `BoundedPlanningSpace` | 同上 | `bounds -> (xmin, zmin, xmax, zmax)` |
| `CurrentField` | `environment/current.py` | `velocity_at(position, time) -> [vx, vz]` |

### 3.2 数据契约

- 12 维状态 `[x,y,z,φ,θ,ψ,u,v,w,p,q,r]` ↔ 6 维约简 `[x,z,θ,u,w,q]`（索引 `0,2,4,6,8,10`）
- 非活跃分量必须严格为 0（`1e-12` 容差），否则 `ValueError`
- 控制必须落在选择矩阵子空间：`diag(1,0,1)` → `tau_y = tau_z = tau_k = tau_n = 0`
- `Path` 无时间；`Trajectory` 时间只在 `VehicleState.timestamp`

### 3.3 B 栈链路（端到端）

```
ROS 2 节点 (trial.py)
  │  订阅 /uuv_twin_axial/state      (std_msgs/String, JSON 紧凑状态)
  │  订阅 /uuv_twin_axial/poses      (VesselPositionArray, Gazebo 位姿)
  │  发布 /uuv_twin_axial/vessel_cmd_array (VesselCmdArray, JSON {"X":…, "M":…})
  ▼
commands.ipc.jsonl  ──(JsonlReader)──►  gateway.py :12356
                                          │  Gate：seq/stamp 单调 + 0.25 s 仿真龄 + 1 s 墙钟龄
                                          ▼
                                    xdyn :12357 (WebSocket)
                                          │  返回 NED/FRD 状态 + ENU/FLU 位姿
                                          ▼
                                   原样回传 LOTUSim ── Gazebo 位姿 ──► /poses
```

- **传输**：Fast DDS UDP，`maxMessageSize=1200`（DDS 分片），`ROS_DOMAIN_ID=74`，`useBuiltinTransports=false`
- **时序**：`dt = 0.05`；`sequence` 严格 +1；`stamp` 严格 +0.05；偏差即停止控制器
- **故障语义**：watchdog 超龄 → 双推进器严格 0；异常 → 清零 + 记录 + 拒绝尾段
- **证据文件**（每个 trial 输出目录）：
  `navigation.jsonl`、`state.jsonl`、`commands.jsonl`、`gazebo_poses.jsonl`、
  `transactions.jsonl`、`acceptance.json`、`gate.json`、`adapter_release.json`、`manifest.json`

### 3.4 外部依赖边界

- `external/LOTUSim`：仅作为仿真器/渲染，不承载控制律
- xdyn：唯一 6-DOF 动力学真值；LOTUSim 不做动力学
- NOAA CO-OPS：唯一"实测"水流来源；北向分量只进 metadata，不进二维 `(x,z)`

---

## 4. 讲解顺序（后续每一节单独成文）

1. **架构与数据流**：坐标系、时间步、两条栈如何共用内核、`Navigator.run` 逐行
2. **物理**：3-DOF 方程逐项推导与符号、附加质量科氏项、RK4、与 xdyn 6-DOF 的差异
3. **控制**：串级 PID 公式、条件积分、前馈、`AllocatedPID` 反算、双轴分配几何
4. **制导与导航**：投影/LOS/G12 预瞄、三种终端策略、状态机与失败分类
5. **规划与轨迹**：A*、候选、捷径、fillet/Bezier、时间参数化——算法对比最集中
6. **接口联通**：Protocol 层、ROS 2/DDS、WebSocket/IPC、证据格式、门禁
7. **实验与验收**：工况矩阵、指标定义、18/45/5 证据分层、已知失败
8. **局限与扩展**：6-DOF 迁移清单、RRT/RL 位置、真实海试边界

---

## 5. 阅读覆盖声明

- 已逐行通读：`core/`（除兼容文件）、`physics/`、`controller/`、`guidance/`、
  `navigation/`、`planner/`（含 grid）、`trajectory/`、`environment/` 主要文件、
  `ros2/twin_axial_v2/model.py`、`allocation.py`、`g12_link_v6/gateway.py`、`ipc.py`、`trial.py`、
  `feasible_v2/guidance.py`、`twin_axial_pid/control.py`、`current_v10/schedule.py`、
  `current_data_v1/schema.py`、`s4_current_v1/mission.py`、`s3_transient/study.py` 前 140 行
- 未逐行读：各代 `acceptance.py` / `gate.py` / `probe.py` 全文、`s4_planning/study.py` 全文、
  `scripts/` 与 `visualization/` 绘图代码、`build/`、`ref/`、`artifacts/` 内数据文件
- 数值结论均取自仓库内文档（`docs/*.md`）与 README，未重新运行实验
