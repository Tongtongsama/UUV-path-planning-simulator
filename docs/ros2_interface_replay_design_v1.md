# ROS 2 Jazzy：接口与历史回放设计 v1

2026-09-30。**设计完成，尚未实现或在 ROS 中验证。**
独立分支 `codex/ros2-replay-design`；上阶段版本见[冻结记录](3dof_simulation_display_freeze.md)。
当前机器的 PATH 未发现 `ros2`，仅发现 WSL 启动器；这不能证明 WSL 内是否装有 ROS。
本轮不安装 ROS、不修改系统环境、不连接实验室网络或机器人。

## 1. 首轮目标与边界

先验证：冻结 JSON → 平台无关映射 → ROS 消息/TF 与 rosbag2 → 回读 → 现有离线展示。
输入复用 `visualization/replay_export.py` 的哈希核验与原始历史；不重新生成路径或执行物理。
现有 HTML 仍可完全离线使用。ROS 端首先提供 RViz 几何显示；bag 回读生成相同展示数据，
而不是在第一版给浏览器新增 WebSocket/rosbridge 或开放网络控制面。

不包含：实机命令、Nav2 控制器、ROS 版 PID、Gazebo、6-DoF、current-aware terminal，
也不在 ROS timer 中直接运行有多秒预测开销的 approach。实机类型/命令接口仍未知。

计划依赖方向：`ros2_adapter → transport-neutral mapping → Core / frozen logs`。
Core、Planner、Guidance、Physics、PID 不导入 rclpy、ROS 消息或 launch。
ROS workspace/package 独立于当前 Python 虚拟环境；Jazzy 宿主与安装方案在实施前确认。

## 2. 坐标合同：只支持当前 vertical_3dof profile

局部固定世界 frame 命名 `sim_world`，ENU，原点沿用场景配置，不声称具有地理定位。
每臂拥有唯一 FLU 机体系，例如 `replay_p1/base_link`、`replay_approach/base_link`。
不凭空添加 map→odom 或定位漂移。ROS 常用 body 为前/左/上（REP-103），而本项目为
前/右/下（SNAME）；保持同一原点，向量转换为 `T=diag(1,-1,-1)`。

当前 Physics 权威约定而非泛化 Euler docstring：
`x_dot=u*cos(theta)+w*sin(theta)`，`z_dot=u*sin(theta)-w*cos(theta)`，正 theta 抬头。
因此 child 在 world 中的姿态旋转（将 body FLU 向量表达在世界）为 `R_y(-theta)`：

```text
position_world = (x, 0, z)
quaternion_xyzw = (0, -sin(theta/2), 0, cos(theta/2))
linear_body_FLU = (u, 0, -w)
angular_body_FLU = (0, -q, 0)
force_body_FLU = (tau_x, 0, -tau_z)
moment_body_FLU = (0, -tau_m, 0)
```

基准 tau_z 必须为零，接口不能将它变成新执行器。体速度仍是 body 表达的对地速度，
不是相对水速度。映射限于 y/roll/yaw/v/p/r 等非活动量为零的当前合同；其他 profile 拒绝，
不默默置零或假装支持完整 6-DoF。换原点需额外速度/力矩变换，首轮不支持。

四元数不能保留未折叠 pitch 的完整转数；权威回放样本必须另存 `raw_pitch_rad`、
原始 u/w/q、控制和时间。四元数只供标准姿态显示，不从 quaternion 重建论文的绕圈曲线。
Quaternion 归一化并在序列内选择连续符号；q 与 -q 表示相同姿态，测试不强求分量逐位相同。

先做纯函数测试：0、±π/2 方向，向下 w、正 q、正 tau_m、世界速度重建，与
`physics/frames.py` 一致；多圈 pitch 保留、非有限值/非法 frame 拒绝。
这是项目推导，不从 ROS 的通用 pitch 标签反向改写已冻结 Physics。

## 3. 消息与只读话题（拟定接口，不是已安装 API）

命名空间 `/uuv_replay/<session>/<arm>/`，arm 为 p1 或 approach。

| 后缀 | 类型/语义 | frame |
| --- | --- | --- |
| `state/pose` | geometry_msgs/PoseStamped，观测姿态 | sim_world |
| `state/twist` | geometry_msgs/TwistStamped，对地体速度 | 对应 base_link（FLU） |
| `observed/commanded_wrench` | WrenchStamped，日志请求值，非执行命令 | base_link |
| `observed/applied_wrench` | WrenchStamped，日志施加值 | base_link |
| `planned_path` | nav_msgs/Path，固定 P1 几何，不赋予速度/时间参数化 | sim_world |
| `scene` | visualization_msgs/MarkerArray，障碍/边界/S/G | sim_world |
| `sample` | 拟定 uuv_replay_msgs/ReplaySample，权威原始记录 | 明确 profile/version |
| `metadata`、`result` | 拟定实验身份/终止记录，含哈希与原终止时刻 | 不冒充状态 |

PoseStamped 与 TwistStamped 避免为确定性 ground truth 编造 Odometry covariance。
未来若增加 Odometry，要单独定义 covariance，不能把全零误称为已知零不确定度。
Path waypoint 没有原几何姿态语义时使用显式标记的占位 identity quaternion；
guidance attitude 不能由 Path 消息暗中重新计算。

ReplaySample 最小字段：session_id、arm、uint64 step_index、source_time(float64)、ROS stamp、
raw x/z/pitch/u/w/q、phase/capture/startup、has_control、原始 commanded/applied、
collision/in_bounds、对应 transition 的起止原时间。N+1 个状态与 N 个控制保持区别：
state[k] 配 control[k] 的前向区间，最后一个 state 没有新的 control；不能前移碰撞标签。
缺失/重复/乱序按 `(session,arm,step_index)` 检查，不能仅按浮点时间拼接各话题。

物理控制入口默认不存在，不发布 `/cmd_vel`、thruster 或 driver 命令；observed 数据禁止
被 launch 重映射为实机输入。硬件执行另立 capability profile 与授权入口。

## 4. 同一回放时钟与结束语义

- 一个 session、一个 `/clock` owner、两臂共同时间；节点使用 `use_sim_time`。
- 约定 `stamp_ns = 1_000_000_000 + round((source_time-common_start)*1e9)`，首样本映射到
  非零 epoch，避免零时间与未初始化混淆；metadata 保存 common_start/epoch/profile。
  保留原始 float64 source_time，不把纳秒量化结果回写原 JSON。
- 启动先发布初始非零 clock 并完成订阅握手，再发送数据；不能依靠跨 topic 发布顺序
  假定 clock 已到达。发布调度用单调 wall timer，暂停不能因 ROS time 冻结而无法恢复。
- 倍速只改变等待时长，不改变样本时间、dt 或两臂相对时序。包时间/header/source_time
  分别保存，不把 DDS 接收顺序视为物理顺序。
- 一方结束后不制造新的状态/控制样本。最终结果持久发布；世界系 Marker 保留末位置。
  TF 停在最后原时间，结束显示不依赖查询过期 dynamic TF，也不以新 stamp 冒充新观测。
- 碰撞在 transition 末状态检测，保存整个区间，截断后续运动；沿用离线播放器语义。
- 第一版 ROS 仅前进、暂停、单步、倍速。回退/重新播放通过停止当前 session 并新建 session，
  清空 TF/消费端缓存、删除旧 markers；不跨 session 拼接。HTML 既有拖动能力不受影响。

## 5. QoS、记录与反向回放

首轮在隔离的本地 ROS domain 中运行，不使用实验室正在控制机器人的 graph。
默认提案：sample/pose/twist/wrench 为 reliable + volatile + keep_last(100)；
静态 path/scene/metadata/result 为 reliable + transient_local + keep_last(1)。
clock 使用与运行端兼容的时钟 QoS（拟 best_effort + volatile + keep_last(1)），
TF 使用 tf2 的标准 QoS。实施时记录实际 RMW/QoS，验证订阅兼容；不要把深度 100
或 reliable 当作完整性保证。计数/序号/哈希检查不通过时，本次 bag 不验收。

只录制明确白名单话题，不使用 `-a` 接触无关 graph。两种独立入口：
1. JSON replay publisher 驱动 clock → recorder；
2. 关闭 JSON publisher，由 bag player 唯一驱动 clock。
不要同时播放保存的 clock 和另一份合成 clock。bag 播放参数以实际 Jazzy `--help` 验证，
本设计不提供未经执行确认的完整 launch/CLI 命令。

优先使用直接 bag writer 将每个 ReplaySample 的存储时间设为映射后的 stamp，
避免在线 recorder 的最近 clock 接收时刻污染物理排序；在线 recorder 作为独立通信验证。
即使接收时间有偏差，回读仍依据 source_time 和 step_index，与原 JSON 比较。
bag→replay_data 仅从权威 sample + metadata 重建，标准 pose/twist/TF 为互操作视图。
校验通过才能送入现有 HTML 导出；不能把丢包后插值的结果当作原始证据。

## 6. 几何绑定与 future capability

metadata 必须绑定 scene_hash、path_hash、源 manifest、算法提交、arm、profile、单位和
软件环境。P1 是唯一首轮 profile；ROS Path 不携带 collision certificate。
未来 primitive 与 dense safety path 若接入，必须同时声明 primitive_hash、safety_geometry_hash、
binding/version 和验证余量；缺失即拒绝标记为已验证路径，不接收任意外部曲线当安全证据。
P2b + approach 尚未组合验证，不作为回放默认。

实机前必须明确：机器人类型、运动平面、状态 frame 与定位来源、可控 DOF、命令是
velocity/wrench/actuator 哪一层、单位/频率/限幅、超时 watchdog、取消/急停及故障策略。
命令 watchdog 应基于独立单调时钟，不能因仿真时间暂停而失效。未知则保持 read-only。
不能把 x-z/pitch 改名成 x-y/yaw，或将 tau_m 发到速度接口。

## 7. 下一项实现与验收门槛

**R0：纯映射与证据 DTO**（不依赖 ROS 安装）：上述方向/往返测试，nanosecond clock、
N/N+1 对齐、原 pitch 与 phase/collision 保真，哈希错误拒绝。不得调用闭环模块。

**R1：Jazzy 历史桥接**：先 double_baffle，两臂 topic/TF 隔离、一个 clock；JSON→bag→JSON
样本数/顺序/原值一致、姿态旋转等价、最多 0.5 ns 的时间量化误差；暂停/倍速不修改时间。
测试迟订阅静态几何、QoS 不兼容、录制缺样拒绝、结束 marker 保留、只读话题隔离。
随后复用 horizontal、detour、两碰撞场景；生成独立 JUnit/bag manifest/回放截图。

**R2：模拟系统接口**：R1 通过后，再设计受控仿真任务接口；保持 Python 固定步长与
算法一致，不把 ROS timer 调度抖动直接当作 Physics dt。当前不实现此阶段。

阶段报告分开：原算法 847 passed；展示 6 项＋浏览器/视频；ROS 映射与 ROS 运行验收各自计数。
此文件不新增测试成绩，不声称现已运行 rclpy、RViz 或 rosbag2。

## 8. 官方来源（2026-09-30 阅读）

以下为接口/设计依据，未复制代码；branch URL 不是已固定运行依赖，实施时再记录具体包版本。
部分 docs.ros.org 页面访问受限，核对使用官方仓库 Jazzy 分支原文，而非搜索摘要。

- [REP-103 官方原文](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst)：SI、ENU、body FLU、姿态表示。
- [ROS 2 Clock and Time](https://design.ros2.org/articles/clock_and_time.html)：模拟时间、单一 clock、时间跳变和未初始化零时间。
- [Jazzy QoS 文档源码](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)：QoS 策略与兼容。
- [Jazzy rosbag2 README](https://raw.githubusercontent.com/ros2/rosbag2/jazzy/README.md)：模拟时钟录制、回放与 QoS 覆盖。
- [PoseStamped](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/geometry_msgs/msg/PoseStamped.msg)、
  [TwistStamped](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/geometry_msgs/msg/TwistStamped.msg)：标准带 frame/time 的容器。
