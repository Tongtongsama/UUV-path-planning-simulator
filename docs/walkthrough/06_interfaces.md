# 06 接口联通

日期：2026-11-28
范围：三种通信介质、一次事务的生命周期、时间纪律与看门狗、DDS 配置、门禁、证据落盘。
不涉及：控制律（见 03）、制导（见 04）、规划（见 05）、实验结论（见 07）。

---

## 1. 通俗版

### 1.1 一个闭环里为什么有三种通信

| 介质 | 连接谁 | 为什么必须用它 |
|---|---|---|
| **Python Protocol**（进程内） | Core / Physics / Controller / Planner | 零开销、可静态检查、无序列化 |
| **ROS 2 话题**（DDS） | trial.py ↔ LOTUSim/Gazebo 插件 | Gazebo 插件只认 ROS 消息，没有别的选择 |
| **JSONL 文件 + WebSocket** | trial.py ↔ gateway ↔ xdyn | xdyn 只提供 WebSocket 服务端；JSONL 用于**解耦时序** |

第三种最容易被问"为什么不直接传"。答案是**时序模型不匹配**：

- LOTUSim 是**拉模型**：每个物理步主动发一个 request，网关只能"在那一刻"回应；
- 控制器是**异步**算出命令的：它读到状态、算一会儿、再发命令，时刻不确定。

两者之间必须有缓冲。`commands.ipc.jsonl` 就是这个**信箱**：控制器往里投，网关按需取。
用追加写（append-only）而不是覆盖写，是为了让"谁先启动、谁慢一点"都不影响正确性——网关可以启动时从头读，也可以运行中增量读。

### 1.2 一次事务的完整生命周期

```
① trial.py：控制器算出 {"X":…, "M":…}
      ├─ 发布 ROS 话题 /uuv_twin_axial/vessel_cmd_array（给 Gazebo 看，非必需）
      └─ 追加写 commands.ipc.jsonl {sequence, stamp, commands}

② gateway（JsonlReader 增量读）
      └─ Gate.update(commands, stamp, seq, now)
            校验：stamp 不倒退、seq 严格递增
            有界读：只提交完整行，ENODATA 不提交 offset

③ LOTUSim → WebSocket :12356，发 request（含当前状态 + 上一步结果）
      └─ check_request：Dt 必须 0.05、单船、t 与上一步连续
      └─ 状态反馈连续性：|Δstate| ≤ 1e-5 且 |q·q_prev| ≈ 1
      └─ Gate.applied(sim_time, now)：判断命令是否"新鲜"
             新鲜 → 用该命令；不新鲜 → 双推进器强制 0（watchdog）

④ gateway → xdyn :12357，转发同一 request（命令已替换为本步实际值）

⑤ xdyn 返回 → decode_reply
      └─ t 必须 = start + 0.05
      └─ 四元数模长 ≈ 1（±1e-3）
      └─ 深度 ∈ (15, 45) m，速度 ≤ 1.5 m/s，角速度 ≤ 0.8 rad/s   ← 台架包络

⑥ 落盘：transactions.jsonl（全量）+ state.jsonl（紧凑）
  原样回传 LOTUSim（不改一个字节）

⑦ trial.py 订阅 /uuv_twin_axial/state（std_msgs/String）
      └─ JsonlReader 读 state.jsonl → 下一轮控制
```

**注意第 ⑥ 步的"原样回传"**：gateway 把 xdyn 的回复**一字不改**交给 LOTUSim。
这是刻意的——如果网关"顺手修正"一下状态，那么"Gazebo 里看到的位置"就不再是"xdyn 算出来的位置"，整条证据链断掉。

### 1.3 三处"不信任"

这条链路上没有一处假设"对方一定对"。

**(a) 不信任命令的新鲜度 → watchdog**

```python
fresh = (not invalid
         and 0 <= command_age(sim_time, stamp) <= 0.25 + 1e-6
         and now - received <= 1.0)
command = self.command if fresh else zeros()
```

两个时间尺度：

- **仿真龄 ≤ 0.25 s**（5 个控制周期）：命令不能"太旧"；
- **墙钟龄 ≤ 1.0 s**：命令不能"卡住"。

任一超限 → 双推进器输出**严格 0**。注意是"归零"，不是"保持上一帧"——保持上一帧在通信中断时会产生持续推力，归零才是安全的。

**(b) 不信任状态的连续性**

```python
mismatch = max|state − previous_state|
dot = |q · q_prev|
if mismatch > 1e-5 or |dot − 1| > 1e-5:
    raise ValueError('Gazebo state feedback discontinuity')
```

这行代码在验证什么？验证 **LOTUSim 真的把上一步 xdyn 的结果喂回来了**，而不是重放、重置或送了个旧状态。如果 Gazebo 侧把模型重置了，状态会跳变，这里立刻报错。

**(c) 不信任证据的完整性**

每个 trial 目录都有一份 `manifest.json`：文件名 → SHA256。
门禁会逐文件重算哈希，任何一个字节变了就拒绝。

### 1.4 时间纪律：为什么这么严

| 约束 | 位置 | 违反后果 |
|---|---|---|
| `Dt == 0.05` | `check_request` | 拒绝请求，试验失败 |
| `t_k+1 == t_k + 0.05` | `decode_reply` | 拒绝回复 |
| `sequence` 严格 +1 | trial 主循环 | `Skipped or reordered state feedback` |
| `stamp` 严格 +0.05 | trial 主循环 | `Non-contiguous controller simulation time` |
| 命令龄 ≤ 0.25 s / 1 s | `Gate.applied` | 推力归零 |

**为什么不能"容忍"一点跳步？** 因为控制器的积分项是按"每步 dt"累加的。如果中间丢了一步，积分就少算了一次，而控制器自己不知道——它会以为"误差已经积了这么多次"，实际不是。这类误差不会立刻爆炸，但会让"可复现"彻底失效。

所以这里的选择是：**宁可整条试验失败，也不接受一条时间上有洞的轨迹**。

### 1.5 DDS 的坑：为什么要把包切成 1200 字节

`loopback.xml`：

```xml
<transport_descriptor>
  <transport_id>uuv_local_udp</transport_id>
  <type>UDPv4</type><maxMessageSize>1200</maxMessageSize>
  <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
</transport_descriptor>
<participant profile_name="uuv_local" is_default_profile="true">
  <rtps>
    <userTransports><transport_id>uuv_local_udp</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports>
  </rtps>
</participant>
```

三件事各有原因：

| 配置 | 解决什么 |
|---|---|
| `maxMessageSize = 1200` | 让 DDS 主动分片，避免超过 WSL/防火墙的 UDP 报文上限 |
| `interfaceWhiteList = 127.0.0.1` | 只走回环，排除多网卡选路的不确定性 |
| `useBuiltinTransports = false` | 防止 LOTUSim 包装器设置 `UDPv4` 时**绕过**上面的 profile 回到默认传输 |

文档里给了一个对照实验：不含仿真的 `dds_ab.json` 测试中，默认大小配置**未完成匹配**，必须算失败；而 1200 字节配置在匹配后发送 500/1500/3000 字节消息，各 30/30 送达。

**并且明确写了边界**：没有抓包证明更底层一定是哪条 WSL/防火墙规则。这是诚实的做法——"工程证据支持默认大报文是阻塞环节"，不等于"已定位到 TCP/IP 栈某一层"。

### 1.6 门禁（gate）：运行前放行 + 运行后审计

门禁不是"检查一下能不能跑"，而是**两条独立的证明链**。

**（a）运行前：`PilotProof`**

```python
pilot_evaluate(normal, repeat, abort)
```

要求三个**不同**的探针记录：

| 探针 | 要求 |
|---|---|
| normal | 完成的 260 步联合探针 |
| repeat | 同上，且**必须在 600 s 新鲜度窗口内** |
| abort | 主动在 sequence=120 注入异常，验证清理路径 |

再叠加：release 里的每个源文件哈希一致、Python 证据 54 条完整、9 组重复性、运行时检查通过。

**（b）运行后：`analyze()` 逐项审计**

25 个必需文件、逐文件哈希、状态序列连续、命令与 IPC 载荷逐字节一致、`allocate()` 复算一致、位姿覆盖率 ≥ 95%、watchdog 从未误触发、最后 4 步推力为 0、日志无 `Segmentation fault`……

**这里的设计哲学是**：门禁不是"打个勾"，而是"把每一条能验证的因果链都验一遍"。所以它慢、它啰嗦，但它能在你写论文之前告诉你"这条数据不可用"。

### 1.7 证据落盘：为什么一次试验要 20 多个文件

| 文件 | 回答什么问题 |
|---|---|
| `transactions.jsonl` | 每一步的完整事务（请求/命令/分配/结果/时间戳） |
| `state.jsonl` | 控制器实际读到的状态（紧凑，< 1 KB，走 UDP） |
| `commands.jsonl` / `commands.ipc.jsonl` | 控制器发了什么 / 网关收到什么（对账） |
| `gazebo_poses.jsonl` | Gazebo 里船在哪（与 xdyn 对账） |
| `navigation.jsonl` | 制导给了什么目标 |
| `run_status.json` | 成功/失败/中断，零推力是否确认 |
| `acceptance.json` | 数值判据逐项 |
| `manifest.json` | 上面所有文件的哈希 |
| `gate.json` | 运行前放行条件 |

**核心思想**：**同一件事至少用两条独立路径记录**。
控制器说"我发了 5 N"，网关记录"我收到 5 N"，xdyn 记录"我用了 5 N"，Gazebo 显示"船动了"。四者必须能对上。对不上就是证据链断裂，而不是"某一方的小误差"。

### 1.8 失败也要留下完整证据

`trial.py` 的 `finally` 块有一句很关键：

```python
# On failure/stale state, reset memory and send only zero, never old thrust.
controller.reset()
publish({'X': 0.0, 'M': 0.0}, ...)
```

失败时**先归零、再清理**，而不是"抛出异常、让进程死掉、推力留在最后一个值"。

清理顺序也有讲究：

```
① 停 gateway / xdyn（先断积分）
② MAS DELETE（趁服务器还活着，删掉本试验的实体）
③ 停 Gazebo 进程组（只杀自己启动的）
```

先停积分再删实体，是因为"删实体"需要 Gazebo 还活着；先停积分是因为"删实体"需要时间，期间不能再有推力。

---

## 2. 严谨版：学术总结

### 2.1 三种介质的语义对比

| 介质 | 有序性 | 可靠性 | 时延 | 失败语义 |
|---|---|---|---|---|
| Python 调用 | 严格 | 异常传播 | 零 | `TypeError` / `ValueError` |
| DDS（UDPv4，回环） | 按 topic QoS | 尽力/可靠（配置相关） | 亚毫秒~毫秒 | 静默丢失 → 靠超时检测 |
| JSONL 文件 | 追加写严格有序 | 行级原子（完整行才提交） | 轮询周期（2 ms） | ENODATA → 重试 3 次后上抛 |
| WebSocket | 严格（TCP） | 可靠 | 毫秒 | 断连 → 关闭 1011 |

**关键设计**：文件介质承担"**状态暂存**"而非"实时传输"，因此它的毫秒级延迟不影响控制周期；真正的实时耦合发生在 gateway ↔ xdyn 的 TCP 回环上。

### 2.2 事务不变量

设第 k 步状态 `x_k`、命令 `τ_k`、时间 `t_k = k·h`（`h = 0.05`），则网关强制：

```
(I1) t_{k+1} = t_k + h                    （decode_reply 与 trial 双重校验）
(I2) seq_{k+1} = seq_k + 1                 （严格递增，重复/倒退即拒）
(I3) stamp_{k+1} > stamp_k                 （仿真时间单调）
(I4) ‖q_k‖₂ = 1 ± 1e-3                     （四元数单位化）
(I5) ‖x_k − x_{k−1}‖_∞ ≤ 1e-5  ∧  |q_k·q_{k−1} − 1| ≤ 1e-5   （反馈连续）
(I6) 0 ≤ t_k − stamp_k ≤ 0.25  ∧  Δwall ≤ 1.0                （新鲜度）
```

**失败处理**：`(I1)–(I5)` 违反 → 关闭连接（1011），整条试验作废；`(I6)` 违反 → 不关闭，双推进器归零，记录 `watchdog_zero=true`。

这体现了**两级失败语义**：

- **结构性失败**（时间/状态不连续）→ 不可恢复，必须重跑；
- **时效性失败**（命令过期）→ 可恢复，降级为零推力继续运行并留证。

### 2.3 看门狗的形式化与安全语义

```
τ_applied(k) = τ_cmd(k)   若 fresh(k)
             = 0          否则
fresh(k) = ¬invalid ∧ 0 ≤ t_k − stamp(τ_cmd) ≤ 0.25 ∧ Δwall ≤ 1.0
```

**安全性质（fail-safe，非 fail-operational）**：

```
通信中断 ⇒ 推力为零 ⇒ 无主动加速
```

**但不能推出"安全"**：

1. 零推力下 UUV 仍有惯性 + 水流，会继续漂移（可能撞障碍）；
2. 深度方向无主动控制（`Z = 0`），只能靠恢复力矩与俯仰维持；
3. 因此语义是"**移除危险源**"，不是"**保证安全状态**"。

### 2.4 证据不可抵赖性的构造

```
E = { file_i, ... }
M = { (name_i, SHA256(file_i)) }          ← manifest.json
R = { (src_j, SHA256(src_j)) }            ← release_*.json
验证：∀i: SHA256(file_i) = M[i]  ∧  ∀j: SHA256(snapshot_j) = R[j]  ∧  SHA256(M) 记录在 gate 报告中
```

**性质**：任一字节改动都会被检测（碰撞阻力由 SHA256 提供）。

**边界**：

- 不能证明"文件是诚实产生的"——只能证明"文件未被事后修改"；
- 不能防"生成时就伪造"——防伪造靠的是"源快照 + release 绑定 + 外部时间戳（`completed_unix`）"三者的组合；
- 时间戳本身可被改（本地时钟），所以有"600 s 新鲜度窗口"这一折中：承认无法绝对防伪，但要求"证据必须是刚生成的"。

### 2.5 门禁的充分性边界

`gate.analyze()` 的检查项可归为五类：

| 类别 | 代表检查 | 证明了什么 |
|---|---|---|
| 完整性 | 25 个文件齐备、manifest 一致 | 证据未缺失、未篡改 |
| 一致性 | `state.jsonl` == `transactions.jsonl` 投影；`allocate()` 复算一致 | 记录之间自洽 |
| 时序性 | seq 连续、delivery ≤ 0.25 s、watchdog 未误触发 | 时间纪律被遵守 |
| 物理映射 | 位姿覆盖率 ≥ 95%、消息类型正确 | Gazebo 确实在跟随 xdyn |
| 清洁性 | 3 个进程返回码 ∈ {0, −15}、无原生崩溃 | 退出路径干净 |

**它证明了**：这条链路在**传输与映射**层面是通的、干净的、可审计的。

**它没有证明**：

1. 导航精度（探针是零输入，不含控制器闭环）；
2. 鲁棒性（单次或两次探针，不是分布）；
3. 实时性上限（只证明了"≤ 0.25 s 内送达"，没证明"最坏情况有界"）；
4. 安全性（零推力不等于安全）。

文档对此的表述很明确：`scope='Actual 13 s zero-input integration only; no navigation or four-case acceptance'`，`formal_trials_started=False`。

### 2.6 进程生命周期与清理

```
正常路径：  完成 → 归零确认（applied == 0）→ 停 gateway/xdyn → MAS DELETE → 停 Gazebo
异常路径：  捕获 → 归零 → 停 gateway/xdyn → MAS DELETE → 停 Gazebo → 写 run_status
```

**顺序约束的因果**：

1. 先停积分，防止清理期间继续产生推力；
2. 再 MAS DELETE，因为该服务需要 Gazebo 存活；
3. 最后停 Gazebo，因为 DELETE 是异步 action，需要服务器响应。

**只杀自己的进程组**（`os.killpg(process.pid, SIGTERM)`），不杀他人进程。这是"共享机器"场景下的必要纪律——否则并行跑试验会互相踩。

### 2.7 局限

1. **单船**：`len(request['states']) != 1 → ValueError`；无多艇协同。
2. **无时钟同步协议**：依赖回环 + 单调时钟，未用 PTP/NTP；跨机部署未验证。
3. **无 QoS 显式配置**：DDS 用默认 QoS，未声明 `RELIABLE`/`TRANSIENT_LOCAL` 等语义。
4. **无端到端时延补偿**：控制律不感知时延，只做"检测并归零"。
5. **强耦合于 xdyn 的 WebSocket 协议**：`request['Dt']`、`states`、四元数字段名均为 xdyn 私有格式，换求解器需重写 protocol 层。
6. **`state.jsonl` 走 UDP 有 1 KB 限制**：故只能传紧凑字段；扩展诊断会撞上这个边界。
7. **清理依赖 Gazebo 存活**：若 Gazebo 先崩，MAS DELETE 失败，实体残留需人工清理。
