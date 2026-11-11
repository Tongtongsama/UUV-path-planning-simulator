# 冻结 G12 v3：实际联合探针与门禁复核（2026-11-11）

独立扩展：`ros2/twin_axial_g12_v3_joint`。原v3的模型、G12、PID、分配器、场景、阈值、网关、IPC解析器和watchdog源码均未修改。没有执行三流P0/F2-P2四组正式复验。

## 为什么独立扩展证据格式

旧v3 transport_check规定pose必须有发送端monotonic时间、sent==received。实际Gazebo位姿消息没有发送端墙钟字段；不能在接收后补造发送证据。本扩展保留旧门禁，按真实可观测证据检查：

- IPC state：网关完整transactions、state.jsonl与探针接收载荷逐条对应；状态延迟以事务开始墙钟至接收时间作为保守上界，非伪造发送时间。
- IPC command：探针实际写入与网关实际读取/采用的sequence、stamp、requested_wrench逐项对应，并核查分配、watchdog和投递延迟。
- pose：订阅实际VesselPositionArray，按原R1的一个dt匹配窗口、0.01 m/1°及95%覆盖条件对照真实xdyn事务。没有虚构Gazebo发送端时钟或用自发消息冒充Gazebo。
- MAS：对本探针自己的艇发送DELETE goal，保留实际accepted/result/status。不会删除其他实体。
- 结束协议：沿用v3先确认实际零输入、停止自己gateway/xdyn，再执行MAS清理的顺序；核查清理前后事务数不再增长。此处是验证既有协议，不是已证实可靠的改进协议。

探针为S3/F0/P0、GUI、目标260步（13秒）的零输入链路试验。准备配置来自原v3，未运行G12导航控制器，不能充当四组导航验收，也没有覆盖非零控制负载的所有时序情况。

## 本轮实际结果

| 检查 | probe_20261111_01 | probe_20261111_02 |
|---|---|---|
| 真实事务 | 260，13.00 s | 35，1.75 s后异常 |
| state接收 | 260 | 32 |
| 首轮实际采用command | 257个序号 | 不足完整探针，未判通过 |
| 首轮state交付上界最大值 | 33.82 ms | 未完成 |
| 首轮command交付最大值 | 9.15 ms | 未完成 |
| watchdog（2.2秒后） | 通过，跨过第161步 | 未到该阶段 |
| Gazebo pose | 0，失败 | 0，失败 |
| MAS | wait_for_server返回后，goal应答2秒超时 | 未进入动作阶段 |
| 结束协议整体 | 失败：未完成MAS清理 | 失败：运行中异常 |

首轮证明当前v3文件IPC能在这次真实零输入栈上运行260步，没有重现161步浮点误触发；不能推广为整个真实传输已经稳定。

第二轮在collector中记录 `OSError(61, 'No data available')`；日志未保存完整Python traceback，因此现有证据不足以定位是哪个具体I/O/图查询调用，不能直接归因于IPC、DDS或负载。Gazebo停止阶段日志出现physics_interface_plugin相关段错误；外层lotusim进程仍返回0，说明外层退出码本身不能证明内部插件正常退出。该运行已因状态未完成和异常被拒绝，不作为清理成功证据。

第二轮graph记录显示domain74、rmw_fastrtps_cpp，pose_publishers=1、action_ready=true，但节点列表当时只有探针自己、实际pose仍为0。因此“发现接口/ready标志”不能替代实际消息和goal/result往返；不能声称MAS端点不存在。

## 失败保留与冻结

首轮shell入口末行含CRLF，导致末个输出路径参数携带CR字符。运行完成后仅将新建探针目录和runtime报告重命名为无CR的可访问路径，未更改任何记录内容；两个清单复核均无哈希变化。操作脚本normalize_paths.py保留。首版源码/release保存在development_01及首轮source快照。

第二版将shell转为LF，在导入运行环境检查前显式设置domain74，并增加节点图诊断。当前扩展release为release_02.json；原release.json保留。全部输入源码绑定在扩展release中，原v3 release哈希仍为3cbf10bcd3ca3aded68a36c1bea11bfe847caadac22bb312b67b2062754d4844。

没有更改冻结G12，也没有在失败后增大watchdog、动作等待期限、R1误差或覆盖率。两轮都使用新目录；只清理自身Popen创建的进程组。开始前与结束后检查12356/12357；没有关闭用户已有Gazebo。

## 放行判断及下一步

当前不具备四组正式真实复验条件。以gate_20261111_02.json最终结果为准：Python证据与Nix运行环境通过也不能抵消联合探针失败。探针已启动真实GUI/xdyn，但正式导航试验数为0；gate报告中real_started=false只表示门禁检查自身未启动试验。

下一步只处理两个局部问题：

1. 为collector和文件IPC/ROS调用补充带调用点的异常堆栈、sequence与时间记录，定位ENODATA，禁止静默吞异常；同时单独标记Gazebo插件崩溃而不是只看外层returncode。
2. 在同一栈中核对pose publisher/subscriber的实际QoS、类型、匹配和MAS goal/result通道；严格区分发现、发送、接受和执行成功。保持零输入诊断，不启动G12矩阵。必要时验证停止gateway后插件错误循环是否阻碍MAS处理，不能预先认定是DDS或控制器问题。

先让这项短联合探针完整通过并复核来源，再讨论四组正式试验。无需调G12、PID或重做Python筛选。

## 验证入口

```powershell
wsl -d NixOS -- bash /mnt/c/Users/Songyuan/uuv_simulator/ros2/twin_axial_g12_v3_joint/run-nix.sh gate check --probe artifacts/twin_axial_g12_v3_joint/probe_20261111_02 --report artifacts/twin_axial_g12_v3_joint/gate_NEW.json
```

报告路径必须全新。探针证据时效为原v3定义的600秒，晚于此时间会明确拒绝作为新启动依据，历史只读analyze不使用时效约束。当前扩展只提供probe/check/analyze，不执行四组正式任务，也没有把旧v3门禁的失败报告覆盖为通过。

测试：67项针对性通过；最终全量1135项通过。保全：5965个清单内原有文件哈希不变，2418个Windows不可读历史资产路径明确排除。该清单不是声称覆盖所有外部文件。记录位于artifacts/twin_axial_g12_v3_joint。
