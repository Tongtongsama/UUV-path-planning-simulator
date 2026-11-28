# 03 控制算法

日期：2026-11-28
范围：串级 PID、条件积分抗饱和、恢复力矩前馈、限幅协商、分配后反算、双轴推力分配。
不涉及：制导目标如何产生（见 04）、规划（见 05）、传输（见 06）。

---

## 1. 通俗版

### 1.1 根本矛盾：两个执行器，三个自由度

3-DOF 模型有 3 个状态要控制：`x`（前后）、`z`（上下）、`θ`（俯仰）。
但输入只有 2 个：`τx`（推力）、`τm`（俯仰力矩）。

```
2 个输入  →  3 个输出      欠驱动（underactuated）
```

这不是缺陷，是所有鱼雷形 UUV 的物理现实：没有侧推，横向靠"转个身再走"。
所以控制律必须先**决定怎么分配**这 2 个自由度去换 3 个目标。

分配策略是：

```
用「俯仰」换「深度」—— 想下潜就低头，靠 u·sinθ 产生垂向速度
用「推力」换「前进」—— 想快就加大 u
```

这就是"串级"的起源：**深度不直接给推力，而是先转成俯仰角目标**。

### 1.2 两个环，各管一件事

```
        ┌──────────── 外环（位置 → 速度/姿态）────────────┐
        │                                                 │
  位置误差 e_x ──投影到船头──► e_xb ──► u_c（期望纵速）   │
  深度误差 e_z ──乘符号 s_u─► Δθ_c ──► θ_c（期望俯仰）    │
        │                                                 │
        └─────────────────────┬───────────────────────────┘
                              ▼
        ┌──────────── 内环（速度/姿态 → 力）─────────────┐
        │  e_u = u_c − u       → τx = Kp·e_u + Ki·I_u    │
        │  e_θ = wrap(θ_c − θ)  → τm = Kp·e_θ + Ki·I_θ   │
        │  e_q = q_c − q              + Kd·e_q           │
        └─────────────────────┬───────────────────────────┘
                              ▼
                       (τx, 0, τm) → Physics
```

**为什么要分两级？**

1. **时间尺度分离**：位置变化慢（秒级），速度/姿态变化快（亚秒级）。外环慢、内环快，符合奇异摄动假设，两级可以独立整定。
2. **欠驱动解耦**：外环负责"把 3 个目标压成 2 个可执行目标"，内环只管"跟踪这 2 个目标"，不关心任务。
3. **可替换性**：换制导算法（第 4 段）只影响外环输入；换执行器（第 6 段）只影响内环输出。

### 1.3 外环公式，以及那个必须记住的符号

**(a) 位置误差要投影到船头方向**

```
e_xb = cosθ·(x_d − x) + sinθ·(z_d − z)
```

为什么？因为推力只能沿船头方向出。船低头 30° 时，"向前 1 m"这个目标里有 `cos30°` 的分量能靠推力完成，剩下 `sin30°` 的分量要靠"先抬头"。这个投影就是把世界系误差换成"我沿着自己轴线该走多远"。

然后

```
u_c = clip(u_d + k_x · e_xb,  −1.5, +1.5)
```

基线 `k_x = 0.35`，最大纵速参考 `1.5 m/s`。

**(b) 深度误差要转成俯仰修正，而且要带方向符号**

```
s_u  = +1  若 u_c ≥ 0，否则 −1
Δθ_c = clip(s_u · k_z · e_z,  −0.5, +0.5)
θ_c  = θ_d + Δθ_c
```

基线 `k_z = 0.20`，最大俯仰修正 `0.5 rad ≈ 28.6°`。

`s_u` 这一项**极其容易漏**。想清楚：前进时想下潜要低头（负俯仰）；倒车时想下潜要**抬头**。因为垂向速度是 `u·sinθ`——`u` 变号，`sinθ` 也必须变号，`u·sinθ` 才能仍然指向下方。

漏掉 `s_u` 的后果：倒车下潜会变成倒车上浮，控制器"越努力越错"。

**(c) 俯仰误差必须 wrap**

```
e_θ = wrap_[−π,π)(θ_c − θ)
```

因为 `θ_c = +179°`、`θ = −179°` 时，真实误差只有 2°，不是 358°。
不 wrap 会让控制器朝错误方向转一整圈。

### 1.4 内环：为什么俯仰用 `q` 而不是 `de_θ/dt`

```
τm = Kp_θ·e_θ + Ki_θ·I_θ + Kd_q·(q_c − q)
```

教科书 PID 的 D 项是 `d(error)/dt`。这里用的是**测量到的角速度误差** `q_c − q`。

三个理由：

1. **避免微分噪声**：数值微分会把离散量化和传感器噪声放大。
2. **避免参考阶跃冲击**："微分冲击"（derivative kick）——参考突变时 `d(error)/dt` 会瞬间巨大，产生一个没人要的力矩脉冲。用 `q` 反馈就没有这个问题，因为 `q` 是物理量，不会因为参考跳变而跳变。
3. **物理意义清晰**：`q_c − q` 就是"角速度还差多少"，直接对应阻尼。

这也是为什么参数名是 `pitch_rate_kd` 而不是 `pitch_kd`——命名把语义写进去了。

### 1.5 抗饱和：积分器"憋气"问题

**问题场景**：目标很远，推力从头到尾顶在 20 N 上限。此时误差一直是正的，积分项 `I` 每步都在涨。等真的接近目标、误差反号了，`I` 已经涨到 4.0 的上限，要花很久才能"吐"回来——结果就是**冲过头再慢慢荡回来**（超调、振荡）。

这叫积分饱和（integrator windup）。

**这个项目的解法：条件积分（conditional integration）**

每一步先"试算"：

```
I_candidate = clip(I_old + error·dt, ±I_max)
raw_candidate = PD + Ki · I_candidate        ← 用全部非积分项合成
```

然后按三条规则决定要不要接受：

| 情形 | 处理 | 理由 |
|---|---|---|
| `|raw_candidate| ≤ L`（没饱和） | **接受** | 正常积分 |
| `raw_candidate > L` 且 `error < 0` | **接受** | 误差已在往回拉，积分该退 |
| `raw_candidate < −L` 且 `error > 0` | **接受** | 同上，反向 |
| 其它（饱和且误差同向） | **冻结**（保留旧 `I`） | 再加只会更饱和 |

代码在 [controller/cascaded_pid_3dof.py](../../controller/cascaded_pid_3dof.py#L46-L63)：

```python
candidate = clip(previous + error*dt, -integral_limit, integral_limit)
candidate_raw = proportional_and_derivative + integral_gain * candidate
if abs(candidate_raw) <= output_limit:      return candidate   # 未饱和
if candidate_raw >  output_limit and error < 0: return candidate
if candidate_raw < -output_limit and error > 0: return candidate
return previous                                               # 冻结
```

**注意这不是 back-calculation。** 区别是：

| 方法 | 机制 | 特点 |
|---|---|---|
| 条件积分（本项目 3-DOF） | 冻结/放行 | 简单、无额外参数；解饱和可能偏慢 |
| 反算 back-calculation（本项目 6-DOF） | `I += Kt·(u_applied − u_raw)` | 连续修正；需整定跟踪时间常数 |
| 积分限幅 clamping | 只限 `I` 的幅度 | 最弱，饱和期间仍会积到上限 |

项目**两种都用了**，用在不同层：3-DOF 用条件积分（因为没有"分配后实际值"可言），6-DOF 分配后用反算（因为分配器会真的削减力矩）。

**一个诚实的局限**：硬冻结策略在"刚好轻微饱和"时会整步不积分，可能比"接受裁剪后的积分"收敛更慢。代码选择了保守的一方，代价是低速末端可能有稳态残差——这正是仓库里 `docs/terminal_residual_classification_v051.md` 讨论的问题。

### 1.6 前馈：为什么默认是关的

物理里有恢复力矩 `+k·sinθ`（第 2 段）。控制器如果知道 `k`，就可以直接加一个 `ff = k·sinθ` 把它抵消掉，让 PID 不用靠积分慢慢攒。

但基线配置里 `restoring_pitch_coefficient` **默认 0**，也就是关闭。理由：

1. **模型无关性**：控制器不应该偷看被控对象的参数。真实艇的 `k` 需要辨识，不能假设。
2. **不能自动读取**：文档明确写"旧 YAML 缺省字段必须按 0 加载，**不可**自动从 Physics 偷读系数"。这是一个架构纪律：跨层读取会破坏可替换性。
3. **实验可归因**：默认关 + 显式候选开（`cascaded_pid_3dof_restoring_candidate.yaml` 里 `6.0`），才能回答"加了前馈到底改善了多少"。如果默认就开，历史结果无法对比。

### 1.7 合成顺序：先加再限，不是先限再加

文档给了一个能一眼看懂的数值反例：

```
PD = −10,  ff = +6,  力矩上限 L = 8
```

| 做法 | 结果 |
|---|---|
| ✅ 先合成再限：`clip(−10 + 6, ±8) = −4` | 正确 |
| ❌ 先限 PD 再加 ff：`clip(−10, ±8) + 6 = −2` | 错误 |

错在哪？先限幅等于**假装 PD 只出了 −8**，但物理上 PD 想出 −10。叠加前馈后力矩被高估了 2 N·m，等于凭空多给了力矩。

所以代码里写成一行：

```python
tau_m = clip((pitch_pd + feedforward) + Ki*I,  -L, +L)
```

### 1.8 限幅协商：`min()` 与"响亮地失败"

控制器自己有一套上限（`maximum_tau_x = 20`、`maximum_tau_m = 8`），任务层（`NavigationConfig`）也有。两者取小：

```python
effective = ControlLimits3DOF(min(p.maximum_tau_x, limits.maximum_tau_x),
                              min(p.maximum_tau_m, limits.maximum_tau_m))
```

**为什么必须协商？** 因为抗饱和判据里的 `L` 必须是**真正会用到的那个上限**。如果控制器按 20 N 判断"没饱和"，但任务层只给 14 N，那积分照样会 windup——抗饱和就白做了。

配套的接口是 `LimitAwareController3DOF.step_with_limits(...)`。旧控制器不实现它，就退回 `step(...)`，文档会明确标注"没有 applied-limit-aware 保证"，而不是假装有。

**最后一层是"响亮地失败"而不是"悄悄修正"**（[navigation/navigator.py](../../navigation/navigator.py#L246-L249)）：

```python
applied = clip(command, ±nav_limits)
if limits_aware and applied != command:
    raise ValueError('limits-aware controller exceeded supplied limits')
```

如果控制器声称自己"限幅感知"却仍然超限，Navigator 不是帮它裁掉，而是**抛异常终止**。因为"悄悄裁掉"会掩盖一个真实的契约破坏，让错误进入证据链。

### 1.9 6-DOF 侧：关掉外环、放开内环、分配后反算

B 栈的 `AllocatedPID`（[ros2/twin_axial_pid/control.py](../../ros2/twin_axial_pid/control.py)）继承同一个控制器，但参数完全不同：

| 参数 | 3-DOF 基线 | G12（6-DOF） | 为什么 |
|---|---|---|---|
| `position_to_surge_gain` | 0.35 | **0** | 外环关闭 |
| `depth_to_pitch_gain` | 0.20 | **0** | 外环关闭 |
| `surge_kp / ki` | 15 / 1.5 | **25 / 3** | 内环更硬 |
| `pitch_kp / ki / kd` | 12 / 0.5 / 6 | **20 / 1 / 22** | 内环更硬 |
| `maximum_tau_x / τm` | 20 / 8 | **1e6 / 1e6** | 禁用控制器内限幅 |
| `restoring_pitch_coefficient` | 0 | **6** | 前馈打开 |

三处关键差异：

**(a) 外环增益为 0**：因为 G12 制导直接给出 `BodyGuidance3DOF(u, pitch, q)`，走 `step_guidance()`，本来就不经过外环。增益设 0 是"明确禁用"，而不是"靠调用路径绕过"。

**(b) 限幅设成 1e6**：物理上限交给分配器（`±15 N`）。这样控制器内部不会先削一刀，导致"分配器看到的是一个已经被削过的请求"，无法反算真实饱和量。

**(c) 分配后反算**（[control.py](../../ros2/twin_axial_pid/control.py#L50-L60)）：

```python
allocation = allocate({'X': raw_X, 'M': raw_M})
allocated  = allocation['allocated_wrench']
I += dt * (allocated - raw) / (Ki * T_t)        # T_t = 1.0 s
```

这就是标准 back-calculation：分配器实际给出的力矩比请求少多少，就按这个差把积分往回拉多少。`T_t` 是跟踪时间常数，`T_t` 越小修正越快。

**注意一个刻意的"双算"**：控制器返回的是 `raw`（未分配），网关再独立跑一遍同样的 `allocate()`。目的是让 `transactions.jsonl` 里能同时看到"控制器请求"和"网关实际执行"，两者必须一致——这是一个对账机制，不是冗余。

### 1.10 分配器：为什么必须"等比缩放"而不是"逐轴裁剪"

双推进器几何（[ros2/twin_axial_v2/model.py](../../ros2/twin_axial_v2/model.py)）：

```
upper 在 +0.25 m，产生推力 T_u
lower 在 −0.25 m，产生推力 T_l
X = T_u + T_l
M = 0.25·(T_l − T_u)
```

反解：

```
T_u = (X − M/0.25)/2
T_l = (X + M/0.25)/2
```

如果某个推进器超过 ±15 N，两种处理方式：

| 方法 | 结果 |
|---|---|
| ❌ 逐轴裁剪：`T_u = clip(T_u, ±15)`，`T_l = clip(T_l, ±15)` | `X/M` 比例被改变 → 力矩方向偏转 |
| ✅ 等比缩放：`α = 15 / max(|T_u|,|T_l|)`，两个都乘 `α` | 比例保持，整体变弱 |

代码选后者：

```python
peak = max(abs(raw))
alpha = min(1.0, 15.0/peak) if peak else 1.0
thrust = {k: v*alpha for k, v in zip(KEYS, raw)}
```

**为什么比例重要？** 因为 `M/X` 就是"每单位推力换多少力矩"。逐轴裁剪会让这个比值突变，等于在饱和瞬间偷偷改了控制增益——控制器还以为自己在执行 45° 的力矩向量，实际被执行成 30°。

代价是：**饱和时推力和力矩同时损失**。这是双轴布局的固有限制——没有独立的垂向推进器，就无法在饱和时"保力矩、舍推力"。文档里 `allocated_wrench` 与 `requested_wrench` 并列保存，就是为了让这个损失可量化。

---

## 2. 严谨版：学术总结

### 2.1 控制律形式化

**外环**（位置→参考）：

```
e_xb = cos θ · e_x + sin θ · e_z                       （船头投影）
u_c  = clip(u_d + k_x e_xb, −u_max, u_max)
s_u  = sgn(u_c)  （u_c = 0 时取 +1）
Δθ_c = clip(s_u k_z e_z, −Δθ_max, Δθ_max)
θ_c  = θ_d + Δθ_c
q_c  = q_d
```

**内环**（参考→广义力）：

```
e_u = u_c − u,          e_θ = wrap_[−π,π)(θ_c − θ),     e_q = q_c − q
τx  = K_{p,u} e_u + K_{i,u} I_u
τm  = K_{p,θ} e_θ + K_{i,θ} I_θ + K_{d,q} e_q + k_ff sin θ
τ   = clip(τ, −L, +L),   L_i = min(L_i^ctrl, L_i^nav)
```

结构与经典"级联位置-姿态控制"一致，但有两处偏离标准形式：

1. **D 项作用于测量角速度而非误差微分**：`K_{d,q}(q_c − q)` 而非 `K_d d(e_θ)/dt`。这等价于"参考不参与微分"的实现（setpoint weighting 的一个特例），消除了 derivative kick。
2. **俯仰修正带方向符号** `s_u`：因 `ż = u sin θ − w cos θ`，垂向控制效能与纵速符号耦合。这是欠驱动纵平面模型的特有结构，标准级联律不包含此项。

### 2.2 抗饱和：类别与比较

设饱和函数 `sat_L(x) = min(L, max(−L, x))`，则

```
u = sat_L( K_p e + K_i I + K_d ė )
```

三种抗饱和策略：

| 策略 | 更新律 | 本项目使用 |
|---|---|---|
| Clamping | `İ = sat_{I_max}(e)` | 仅作为积分容量上界 |
| Conditional integration | `İ = e` 若 `(u_raw − sat_L(u_raw))·e ≤ 0`，否则 `0` | ✅ 3-DOF 内环 |
| Back-calculation | `İ = e + K_t (u_applied − u_raw)` | ✅ 6-DOF 分配后 |

本项目 3-DOF 的条件积分判据可写成等价形式：

```
接受积分  ⟺  |K_i I_cand + PD| ≤ L   ∨   (K_i I_cand + PD − L)·e < 0
```

即"未饱和，或误差方向与饱和方向相反"。这是一个**硬冻结**（hard freeze）变体：不采用"接受裁剪后的积分"这一较温和的形式。

**已知理论性质**：

- 条件积分保证 `I` 有界（双重保障：`I_max` 与冻结判据），从而排除 windup 导致的持续超调。
- 它**不保证**受限输出立即解饱和（与 back-calculation 不同），因此低速末端的稳态残差需要在验收中单独分类——仓库中 `docs/terminal_residual_classification_v051.md` 即为此。

### 2.3 限幅一致性的形式条件

要求控制器输出满足"可执行域"：

```
τ_applied = sat_{L_nav}(τ_cmd),   且   τ_cmd = sat_{L_ctrl}(τ_raw)
```

若 `L_ctrl > L_nav`，则存在 `τ_cmd` 使 `sat_{L_nav}(τ_cmd) ≠ τ_cmd`，即"控制器认为自己没饱和、实际被削"。此时抗饱和判据使用的 `L` 与真实 `L` 不一致，windup 判据失效。

因此项目强制：

```
L_i = min(L_i^ctrl, L_i^nav)      （构造期/每步传入）
```

并要求在 `L_ctrl ≤ L_nav` 时 `sat_{L_nav} ∘ τ_cmd = τ_cmd`（幂等）。Navigator 用 `applied != command → ValueError` 检验这一幂等性，属于**契约断言**而非补偿。

### 2.4 前馈的模型依赖性与可归因性

前馈项 `k_ff sin θ` 精确抵消对象模型中的 `+k_θ sin θ`，故当 `k_ff = k_θ` 时该通道的稳态力矩需求降为零，PID 仅需处理残余与耦合。

但项目将 `k_ff` 默认置 0，并要求：

- 控制器不得从 Physics 读取 `k_θ`（跨层耦合禁止）；
- 配置层显式提供并记录来源（合成实验 `6.0`，真实艇需辨识）；
- 关闭时输出与记忆须与旧基线**逐位一致**（回归测试 `tests/controller/test_feedforward_v051.py`）。

这是一个**可归因性**（attributability）设计：默认关闭使得"开启前馈带来的性能变化"可被单独测量，而不是混入基线。

### 2.5 分配的反问题与保比性质

推进器到广义力的映射为线性：

```
[X]   [ 1    1 ] [T_u]
[M] = [−λ    λ ] [T_l],     λ = 0.25 m
```

其逆为

```
[T_u]   1  [ 1   −1/λ ] [X]
[T_l] = 2  [ 1    1/λ ] [M]
```

可行域为 `‖T‖_∞ ≤ T_max`。当解越界时，采用**保方向缩放**：

```
T ← α T,   α = T_max / ‖T‖_∞ < 1
```

**保比性质**：`M/X` 不变，即广义力向量的方向不变，仅模长缩短。

对比逐轴饱和 `T_i ← sat(T_i)`：后者使 `(X, M)` 落在可行域的**边界折线**上，方向改变，等效于在饱和瞬间改变了控制增益矩阵——破坏了"控制器输出语义 = 执行器输入语义"这一契约。

代价是**联合饱和**：推力与力矩不可独立饱和，`X` 的可用上限随 `|M|` 增大而下降。这是双轴布局的结构性限制，需在任务规划（第 5 段）与终端策略（第 4 段）中显式考虑。

该缩放同时解释了为什么 xdyn 侧验收要统计 `saturation_fraction = #{α < 0.999}/N`：`α` 是"可用控制权限"的直接度量，比 `|τ|` 是否触限更早、更连续地反映饱和。

### 2.6 参数表

**3-DOF 基线**（`cascaded_pid_3dof_baseline.yaml`）：

| 参数 | 值 | 含义 |
|---|---|---|
| `position_to_surge_gain` | 0.35 | 外环位置→速度 |
| `depth_to_pitch_gain` | 0.20 | 外环深度→俯仰 |
| `maximum_surge_reference` | 1.5 m/s | 纵速参考上限 |
| `maximum_pitch_correction` | 0.5 rad | 俯仰修正上限 |
| `surge_kp / ki` | 15 / 1.5 | 纵速环 |
| `pitch_kp / ki / kd` | 12 / 0.5 / 6 | 俯仰环 |
| `maximum_tau_x / tau_m` | 20 N / 8 N·m | 广义力上限 |
| `surge_integral_limit` | 4.0 | 积分容量 |
| `pitch_integral_limit` | 1.0 | 积分容量 |
| `restoring_pitch_coefficient` | 0（候选 6.0） | 前馈 |

**G12 / 6-DOF**（`ros2/twin_axial_pid/control.py::parameters()`）：

| 参数 | 值 | 说明 |
|---|---|---|
| 外环两增益 | 0 / 0 | 由制导直接给体轴目标 |
| `surge_kp / ki` | 25 / 3 | |
| `pitch_kp / ki / kd` | 20 / 1 / 22 | |
| `maximum_tau_x / tau_m` | 1e6 / 1e6 | 禁用内限幅 |
| `restoring_pitch_coefficient` | 6 | 前馈开启 |
| `tracking_time_s` | 1.0 | 反算时间常数 |

### 2.7 局限

1. **无稳定性证明**：增益为实验整定，只有局部收敛与饱和行为证据；未做 Lyapunov 或 passivity 分析，未给鲁棒性界。
2. **无执行器动力学**：无转速-推力曲线、无电机滞后、无死区、无速率限制；`τm` 可瞬时跳变。
3. **无非对称限幅**：`ControlLimits3DOF` 只支持对称上限；真实推进器正反向推力常不同。
4. **倒车工况薄弱**：`s_u` 只处理了符号，未处理倒车时 `u` 接近 0 的效能塌陷（`u·sinθ → 0`），该区间 `θ` 控制对深度几乎无效。
5. **反算依赖 `T_t` 整定**：`tracking_time_s = 1.0` 未做灵敏度分析；过大则解饱和慢，过小则可能引入振荡。
