# 01 架构与数据流

日期：2026-11-28
范围：Core 数据模型、坐标系、时间离散化、模块责任边界、`Navigator.run` 单步循环、两栈同构性。
不涉及：具体动力学推导（见 02）、PID 公式（见 03）、规划算法细节（见 05）。

---

## 1. 通俗版：把"仿真"拆成五个问题

任何水下航行器仿真，每一帧都在回答五个问题：

| 问题 | 谁来回答 | 模块 |
|---|---|---|
| ① 我在哪、朝哪、多快？ | 状态 | `core` |
| ② 周围有什么、水在往哪流？ | 世界 | `environment` |
| ③ 我该往哪走？ | 决策 | `planner` + `trajectory` + `guidance` + `navigation` |
| ④ 我该出多大力？ | 控制 | `controller` |
| ⑤ 出了这个力，下一步我在哪？ | 物理 | `physics`（或 xdyn） |

这五个问题的答案被串成一个环：

```
②世界 ──► ③决策 ──► ④控制 ──► ⑤物理 ──► ①新状态 ──┐
   ▲                                                  │
   └──────────────────────────────────────────────────┘
```

项目的全部工程努力，本质上是在保证这个环**每一段都有明确契约、每一帧都可复现、每个失败都可归因**。

### 1.1 两套坐标系，一个容易踩的坑

- **位置/姿态用 ENU**（东-北-天）：`x` 向东、`y` 向北、`z` **向上**。
- **速度/力用 SNAME 船体系**：`u` 纵向（前）、`v` 横向（右）、`w` 垂向（**向下**）。

所以 `z` 向上、`w` 向下，两者符号相反。这不是笔误，是海洋工程的惯例：世界系"天向为正"符合大地测量，船体系"下为正"符合耐压壳与重力方向。

由此产生一个必须记住的矩阵（见 `physics/frames.py:12-22`）：

```
ẋ =  u·cosθ + w·sinθ
ż =  u·sinθ − w·cosθ      ← 负号来自 w 向下、z 向上
θ̇ =  q
```

**为什么不让 Core 做这个变换？** 因为 Core 只管"数据长什么样"，不管"数据怎么换算"。一旦 Core 里出现三角函数，6-DOF 迁移时就要改数据类；把变换留在 Physics，未来换 6-DOF 只需换 Physics。

### 1.2 固定步长：0.05 秒

`NavigationConfig.dt = 0.05`（20 Hz），`max_duration = 120 s` → 单次任务固定 2400 步。

**为什么必须固定步长？**

1. **可比性**：A*、Bezier、LOS、G12 四组实验要能逐帧对齐比较，步长必须一样。
2. **可复现**：没有自适应步长，就没有"这次和上次为什么不一样"的争论。
3. **证据性**：每一步都落一条记录，步数就是证据长度。

代价是：**没有误差控制**。RK4 在 0.05 s 下对这套慢速 UUV（0.5 m/s）足够，但对高速或强流不保证。项目用"与 SciPy `solve_ivp` 离线对比"来兜底，而不是内置自适应。

### 1.3 全部 frozen：每一帧都是一张不可篡改的照片

`Pose`、`Twist`、`ControlInput`、`VehicleState`、`Path`、`Trajectory` 全是 `@dataclass(frozen=True)`。

好处很实际：

- 历史轨迹存下来后，**不可能被后续代码改掉**——这正是一篇论文需要的"证据不可变性"。
- 模块之间传对象时不用担心"我传给你之后你改了它"。
- `tuple` 而非 `list` 保存序列，从容器层面也禁止原地修改。

代价是每步都新建对象，性能不高；但 2400 步的量级完全可以接受，换来的是可审计性。

### 1.4 12 维状态，6 维内核

`VehicleState.to_numpy()` 永远返回 12 维：

```
[x, y, z, roll, pitch, yaw, u, v, w, p, q, r]
```

但 3-DOF 纵平面只用 6 个（`physics/backend.py:34`）：

```
索引 [0,2,4,6,8,10] → [x, z, pitch, u, w, q]
```

**为什么保留 12 维而不直接定义 6 维类？**

- 未来 6-DOF 不需要改 Core；
- `VehicleState` 可以同时被 3-DOF 和 6-DOF 后端消费；
- 代价是必须显式断言"非活跃分量严格为 0"（容差 `1e-12`），否则抛 `ValueError`。

这个断言很关键：它把"维度不匹配"从**静默错误**变成**显式异常**。如果允许 `y=0.3` 却悄悄丢掉，你会得到一条看似正常、实则错误的轨迹。

### 1.5 责任边界：谁不许做什么

| 模块 | 拥有 | **明确不做** |
|---|---|---|
| Core | 数据结构、序列不可变性 | 插值、平滑、重采样、坐标变换 |
| Environment | 几何、边界、水流查询 | 不 import Planner，不碰动力学 |
| Physics | 坐标变换、连续方程、积分 | 不做推力分配，不知道推进器存在 |
| Controller | 广义力 `(τx, 0, τm)` | **不输出推进器转速/推力** |
| Planner | 几何路径 | 不管时间、不管动力学可行性 |
| Trajectory | 时间参数化 | 不做动力学平滑（等速，非最小时间） |
| Navigation | 相位编排、安全检查 | 不实现规划/动力学/PID |
| ROS 2 | 传输、分配、门禁、证据 | 不改控制律 |

"Controller 不输出推进器指令"这条最容易被误解。它输出的是**广义力**，因为：

- 3-DOF 模型里根本没有推进器几何；
- 有推进器几何的是 `ros2/twin_axial_v2/allocation.py`；
- 一旦 Controller 里出现"左推进器/右推进器"，控制律就和某个具体艇绑死了。

### 1.6 两条栈：同一个大脑，两台发动机

```
                    ┌──────────────────────────────┐
                    │  共用内核（Python 包）        │
                    │  core / environment / planner │
                    │  trajectory / controller      │
                    │  guidance / navigation        │
                    └───────────┬──────────────────┘
                                │
              ┌─────────────────┴──────────────────┐
              ▼                                    ▼
   ┌─────────────────────┐              ┌──────────────────────┐
   │ A：自研 3-DOF 后端   │              │ B：xdyn 6-DOF 后端    │
   │ Python3DOFBackend   │              │ gateway + xdyn       │
   │ 进程内函数调用       │              │ ROS 2 DDS + WebSocket│
   │ 无渲染               │              │ Gazebo/LOTUSim 渲染   │
   └─────────────────────┘              └──────────────────────┘
```

**同构性**是理解整个仓库的钥匙：

| 环节 | A 栈 | B 栈 | 是否共用 |
|---|---|---|---|
| 世界模型 | `WorldModel` + Shapely | 从 SDF 解析出同一个 `WorldModel` | ✅ 共用 |
| 规划 | `AStarPlanner` | 同一 `AStarPlanner` | ✅ 共用 |
| 轨迹 | `generate_trajectory` | 同一函数 | ✅ 共用 |
| 制导 | `LOSNavigationAdapter` / `G12 Guide` | `G12 Guide` | ⚠️ G12 另有实现（预瞄+补偿） |
| 控制 | `CascadedPID3DOFController` | `AllocatedPID`（继承同一类） | ✅ 继承复用 |
| 动力学 | `Python3DOFBackend` | xdyn 6-DOF | ❌ 不同 |
| 传输 | 函数返回 | ROS 2 + WebSocket + JSONL | ❌ 不同 |
| 安全 | `WorldModel.segment_clearance` | 同一函数 + 0.9 m 阈值 | ✅ 共用 |

所以"B 栈的算法"并不是另一套算法，而是**同一控制律接在更高保真的动力学上**。这也解释了为什么 G12 的所有结论都能用 A 栈的 Python 试验先筛一遍（54 组），再上 xdyn 验证——筛的是控制律，不是物理。

### 1.7 一轮循环：`Navigator.run` 的值班表

`navigation/navigator.py:180-295`，每一步按固定顺序执行，**顺序本身就是设计**：

```
第 k 步：
  1. 若启用 startup 且未到位 → ControlledStartupPolicy.update
       失败 → 立即返回 STARTUP_FAILED
  2. 若启用 approach 且 startup 未接管 → TerminalApproachPolicy.update
       失败 → APPROACH_FAILED；预测合格 → capture_latched = True
  3. 相位判定：
       STARTUP           → 用 startup 给的目标
       TERMINAL_APPROACH → 用 approach 给的目标
       TERMINAL_CAPTURE  → 用 capture 状态机的目标
       PATH_FOLLOWING    → LOS 给 GuidanceCommand → BodyGuidance3DOF
       TIMED_REFERENCE   → 用预先采样的轨迹参考
  4. controller.step_guidance / step_with_limits / step
  5. 输出裁剪 + 非活跃分量必须为 0 + 诊断必须与本次输入匹配
  6. world.current_at(pose, t) → 当前流速
  7. physics.step(state, applied, current, dt)
  8. 断言时间戳严格 +dt
  9. world.segment_clearance / boundary.contains_segment
 10. 记录 NavigationStep；碰撞/越界立即终止
 11. 到位判定（位置 + 速度 + 俯仰角速度 + settle_time）
```

几个细节值得单独说：

**(a) 为什么"读流速"在"算控制"之后？**

因为控制器用的是**当前时刻**的流速（如果它需要），而物理积分需要的是**本步**的流速。二者同属第 k 步，顺序不影响结果，但代码上把"控制决策"和"物理推进"分开，便于在第 6 步插入不同的流场实现（常流/分段流/实测流），而不用动控制器。

**(b) 为什么安全检查在积分之后？**

因为碰撞是**轨迹**的性质，不是**点**的性质。只有拿到 `nxt.pose`，才能检查 `state.pose → nxt.pose` 这条线段扫过的胶囊体。这是"事后检查"（post-hoc），不是"预测式安全屏障"（barrier function）——这是本项目的已知局限，论文里必须写清楚。

**(c) 为什么 `timestamp` 要断言严格 `+dt`？**

防止后端偷偷改变了时间语义。xdyn 那边同样有 `abs(stamp - last_stamp - 0.05) > 1e-6 → RuntimeError`（`trial.py:149-151`）。两栈用同一条时间纪律。

**(d) `capture_latched` 为什么"下一步才生效"？**

`capture_latched = los_command.terminal_ready` 之后，本步仍然按 LOS 走完控制与积分；下一步才进入 TERMINAL_CAPTURE。这是**显式的一步延迟**，目的是保证相位切换永远发生在两个完整物理步之间，不会出现"半个 LOS + 半个捕获"的混合指令。

### 1.8 失败不是异常，是返回值

`NavigationStatus` 有 12 个终态（`navigation/models.py:13-27`）：

```
SUCCESS / PLANNING_FAILED / TRAJECTORY_FAILED / COLLISION / TIMEOUT /
OUT_OF_BOUNDS / NUMERICAL_FAILURE / CONTROLLER_FAILURE / ENVIRONMENT_FAILURE /
CAPTURE_FAILED / STARTUP_FAILED / APPROACH_FAILED
```

所有 `try/except` 都转成 `finish(status, message)`，返回一个**完整的历史 + 明确的原因**，而不是抛异常中断。

这对论文极其重要：**失败也是数据**。18 工况矩阵里如果某条失败，你要能说清"是规划不可行、还是执行碰撞、还是超时"，而不是"程序崩了"。

---

## 2. 严谨版：形式化总结

### 2.1 状态空间与约简映射

设惯性系为 ENU，船体系为 SNAME。完整状态

```
x = [η; ν] ∈ R^12,   η = [x,y,z,φ,θ,ψ]^T,   ν = [u,v,w,p,q,r]^T
```

本项目的 3-DOF 纵平面运行域由线性投影算子定义：

```
Π : R^12 → R^6,   Π x = [x, z, θ, u, w, q]^T
```

其合法性由**硬约束**保证：`‖(I − ΠᵀΠ) x‖_∞ ≤ 10⁻¹²`，否则拒绝（`physics/backend.py:31-32`）。
即运行域是 R^12 的一个**坐标子空间**，而非一个低维近似。这一点使 3-DOF 结果可以被
6-DOF 后端以"零化非活跃分量"的方式无损嵌入，而不需要重新定义状态类型。

### 2.2 输入子空间

控制输入 τ = [X,Y,Z,K,M,N]^T ∈ R^6，物理可行域由选择矩阵 S = diag(1,0,1) 定义：

```
U = { τ ∈ R^6 : (I − S) τ = 0 } = span{e_X, e_M}
```

即 `τ = [τx, 0, 0, 0, τm, 0]^T`。这是**欠驱动**（underactuated）的数学表述：
2 个独立执行维度控制 3 个自由度（x, z, θ）。横向（y）与横滚/艏向被结构性排除，
不是"没调好"，而是模型定义的一部分。

### 2.3 时间离散化与转移算子

固定步长 h = 0.05 s，离散转移算子

```
x_{k+1} = Φ_h(x_k, τ_k, v_c,k),   Φ_h = RK4( f(·, τ_k, v_c,k), h )
```

其中 f 为连续向量场（见 02）。由于 h 固定且 Φ_h 无随机项、无自适应容差，
对给定 (x_0, {τ_k}, {v_c,k}) 的轨迹是**逐位确定**的（bitwise deterministic，在
同一 BLAS/浮点环境下）。这是全部"可复现"主张的技术基础。

时间戳 t_k 是**元数据**，不属于状态向量：`x` 与 `t` 在类型层面分离
（`VehicleState.timestamp` 不进入 `to_numpy()`），避免"时间是状态"的范畴错误。

### 2.4 模块契约与单向依赖

系统是一个分层组合，依赖关系为有向无环图：

```
Core ← Environment ← Planner ← Trajectory ← Guidance ← Navigation
Core ← Physics ←───────────────────────────────────────┘
Core ← Controller ←────────────────────────────────────┘
```

每个边界由结构化协议（`typing.Protocol`）而非基类定义，满足"结构化子类型"
（structural subtyping）：实现者不需要继承，只需方法签名兼容。
`Navigator.__init__` 在构造期做能力检查（capability check），把"运行到一半才发现
控制器不支持 `step_guidance`"提前到"装配期拒绝"，属于 fail-fast 设计。

组合语义可写为：

```
Navigator.run = Safety ∘ Physics ∘ Controller ∘ Phase ∘ Planner ∘ Trajectory
```

其中 `Phase` 是一个显式状态机（TIMED_REFERENCE / STARTUP / PATH_FOLLOWING /
TERMINAL_APPROACH / TERMINAL_CAPTURE），状态转移只在整步边界发生。

### 2.5 两栈同构性命题

**命题**：A 栈与 B 栈在控制律层面同构。

存在映射 ψ：(自研 3-DOF 状态) → (xdyn 6-DOF 状态)，使得对同一控制律 π，

```
π_A(x_k) = π_B(ψ(x_k))
```

成立，因为：

1. 两栈共用同一 `CascadedPID3DOFController`（B 栈的 `AllocatedPID` 是其子类，
   仅覆盖积分更新与限幅策略）；
2. 两栈共用同一 `WorldModel` 与 `segment_clearance`；
3. 差异仅在 Φ（RK4 自研方程 vs xdyn 6-DOF）与传输语义（函数返回 vs DDS/WebSocket）。

**推论**：A 栈的 54 组 Python 试验可作为 B 栈的**必要非充分**筛选条件——它筛除的是
控制律与几何层面的不可行，不能筛除 6-DOF 耦合、推进器饱和、通信时延导致的失败。
这正是仓库中"Python 证据 ≠ 真实链路证据"分层陈述的理论依据。

### 2.6 可复现性的三个支柱

| 支柱 | 机制 | 失效条件 |
|---|---|---|
| 确定性 | 固定步长 + 无随机项 + 纯函数 `step()` | 换 BLAS/编译器/浮点环境 |
| 不可变证据 | frozen dataclass + tuple 序列 | 反序列化后重新构造 |
| 显式失败 | 12 态枚举 + `message` + 保存失败指令/状态 | 异常被上层吞掉 |

### 2.7 已识别的局限

1. **安全是事后判定**：碰撞在积分后发现，不是预测式约束。因此"通过验收"不等于
   "安全"，只等于"在这条离散轨迹上未越界"。
2. **一步延迟的相位切换**：`capture_latched` 的显式延迟降低了混合指令风险，
   但也意味着终端切换点比理论最优晚一个步长。
3. **3-DOF 运行域是子空间而非近似**：好处是无损嵌入，代价是无法表达任何
   横向/横滚耦合，因此 3-DOF 的"鲁棒性"结论不能外推到 6-DOF。
4. **通信时延未建模**：A 栈是零延迟函数调用；B 栈有 0.25 s 仿真龄/1 s 墙钟龄的
   watchdog，但控制律本身不感知时延，属于"检测并归零"而非"补偿"。
