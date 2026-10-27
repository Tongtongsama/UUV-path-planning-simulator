# LOTUSim 冻结历史三维回放 v1

> Gazebo 窗口内回放请使用 [Gazebo 适配层运行指南](lotusim_gazebo_replay.md)。
> 本文的 `play` 发布到外部渲染器接口，不会在 Gazebo 中生成实体。

本阶段只把已经存在、已经冻结的 UUV 历史位姿送入 LOTUSim 渲染接口。它不会调用或替换
Planner、Navigator、Guidance、PID、Physics，也不发布推进器、速度、力或力矩命令。实时联合
仿真不在本阶段范围内。

## 数据与坐标合同

- 输入仍由 `visualization.replay_export.build_payload()` 读取；运行前会核验 terminal manifest、
  每个 run、规划几何、场景配置和 freeze 绑定。失败时不启动 ROS 发布。
- 原模型为 ENU 世界坐标、垂直 x-z 平面、body 前/右/下；LOTUSim 的
  `VesselPosition` 注明 body 前/左/上。因此位置映射为 `(x, 0, z)`，姿态映射为
  `R_y(-pitch)`，四元数 `(0, -sin(pitch/2), 0, cos(pitch/2))`。
- 动画在相邻原始状态间线性插值；这只是显示帧，不写回 evidence，也不产生新的仿真结果。
  碰撞截断和终态冻结继续由已有 exporter 的规则负责。

## 构建与运行

LOTUSim 与本包需要位于同一个 ROS 2 Jazzy workspace（或先安装 LOTUSim，使
`lotusim_msgs` 可发现）。从 workspace 根目录执行：

```bash
colcon build --packages-select lotusim_msgs lotusim_replay
source install/setup.bash
ros2 run lotusim_replay play --repo-root /path/to/uuv_simulator
```

在尚未安装 ROS 2 的开发机上，可以先验证证据链、场景和样本数：

```powershell
python -m ros2.lotusim_replay.lotusim_replay.player `
  --repo-root C:\path\to\uuv_simulator --validate-only
```

默认场景是 `double_baffle_detour`，默认只显示 `APPROACH`，模型类型为 `lrauv`，发布到
`/lotusim/renderer_cmd` 和 `/lotusim/renderer_poses`。这些话题与 LOTUSim `lotusim.world`
中的 ROS2 render interface 一致。常用选项：

```bash
# 同时比较两臂；横向错开 2 m 仅用于展示，0 表示保持原始位置
ros2 run lotusim_replay play --repo-root /path/to/uuv_simulator \
  --arm P1 --arm APPROACH --lateral-separation 2 --rate 2

# 循环回放另一冻结场景
ros2 run lotusim_replay play --repo-root /path/to/uuv_simulator \
  --scene detour --loop
```

播放器直接仿真 LOTUSim render plugin 的输出端。使用它时不得同时让另一个发布者向相同
`renderer_cmd` / `renderer_poses` 话题驱动同名对象。退出时发送 DELETE；正常播放结束后保留
终态，按 Ctrl+C 才删除对象。若前端使用不同 world namespace，可用 `--namespace` 调整。

## 已知边界与下一阶段门槛

此包不证明当前 Windows 主机已具备 ROS 2、Gazebo 或 LOTUSim UI 运行环境；无 ROS 环境时
只能运行纯映射测试。进入实时联合仿真前，至少需要另行确定状态所有权、固定步长调度、命令
接口层级、超时/watchdog、暂停与复位语义，并完成 JSON→LOTUSim 的端到端截图与消息计数验收。
