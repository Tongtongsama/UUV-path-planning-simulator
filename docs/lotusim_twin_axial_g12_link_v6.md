# G12 真实链路工程收口（2026-11-11）

本修订只处理传输、IPC 和退出流程，不改变 G12、模型、PID、分配器、场景或数值验收阈值。S3 仍为名义场景。最终代码位于 `ros2/twin_axial_g12_link_v6/`，证据位于 `artifacts/twin_axial_g12_link_v4/`；证据目录名称沿用本轮首个修订，不代表其中所有探针使用 v4。各记录有自己的源码快照和 release。

## 修复与证据

### IPC

新 `ipc.py` 使用无缓冲二进制读取，按 fstat 已观测长度读取，不再无界 read-to-EOF。仅完整且可解析的 JSON 对象行提交 offset。ENODATA 最多尝试三次、间隔 2 ms；每次重试记录事件，不提交 offset，不更新控制命令的时间戳。持续异常仍上抛，网关锁存失败、watchdog 归零。原 0.25 s 仿真时限及 1 s 墙钟时限不变。

离线测试覆盖半行 UTF-8、多行、非法完整记录、截断/消失、瞬时 ENODATA 恢复与持续 ENODATA 上抛；Gate 类与冻结 v3 逐字一致。真实最终探针未发生 ENODATA 或 IPC 重试。这证明本轮验证通过，不是对所有文件系统故障的永久排除。

### Gazebo 位姿和 MAS

仅限制回环接口的 v4 配置仍失败。v5/v6 增加 Fast DDS UDP `maxMessageSize=1200`，由 DDS 分片；不缩短、替换或伪造 ROS 消息，也不改任何控制/验收阈值。`useBuiltinTransports=false` 避免 LOTUSim 包装器设置 UDPv4 时绕回默认传输。

不含仿真的 `dds_ab.json` 对照中，默认大小配置未完成匹配，子进程报错，必须算失败，不能称为发送后丢失 90 条；1200 字节配置在匹配后发送 500/1500/3000 字节消息，各 30/30 送达。此前无匹配等待的开发探针是 0 对 99/100，不能把发现阶段损失解释成稳定链路丢包率。

最终真实探针同时恢复 physics_plugin/mas_node 的节点图、实际 VesselPositionArray 位姿、积分期间空 MAS goal 的应答、关停后对本试验实体的 DELETE 成功结果。工程证据支持 DDS 默认大数据报/发现交付路径是此次阻塞环节；没有抓包证明更底层一定是哪条 WSL/防火墙规则。

配置依据：[Fast DDS 2.14 接口白名单](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/transport/whitelist.html)及[传输配置定义](https://github.com/eProsima/Fast-DDS-docs/blob/master/docs/fastdds/xml_configuration/transports.rst)。是否有效以本项目实际联合证据为准。

### 正常和异常退出

两条路径均先停止自己启动的 gateway/xdyn，再等待自己的 MAS DELETE，最后关闭自己启动的 Gazebo 进程组。日志中的 Segmentation fault/core dumped/terminate called 会使验收失败，不能用外层返回码 0 掩盖。

主动在 sequence=120 抛出 collector 异常：真实事务保留 121 步，MAS 返回 accepted、status=4、result=[true]，删除对象为本试验艇；结束后仍为 121 步，无原生崩溃。该探针的运行状态及常规分析仍为 failed，只在独立异常清理检查中通过。未删尾、重标成功或以故障注入记录抵扣名义试验。

## 全部短探针

| 记录 | 修订 | 结果 |
|---|---|---|
| probe_01 | v4 | 260 步；IPC 通过，pose=0、MAS 超时，失败保留 |
| probe_02 | v5 | 260 步联合通过；位姿覆盖 100% |
| probe_03 | v6 | 260 步联合通过；263 条位姿，覆盖 100% |
| probe_abort_05 | v6 | 121 步故障注入；运行失败，异常清理检查通过 |
| probe_repeat_06 | v6 | 260 步联合通过；264 条位姿，覆盖 100% |

另有 `abort_preflight_04.json` 端口占用拒绝，没有启动仿真。后续 ss 检查看到 TIME-WAIT，等待端口自然释放，不杀进程。最终门禁辅助脚本的首次导入失败也保存在 `gate_launcher_failure_01.json`，改用独立辅助入口后重查，未覆盖失败。

共 5 次短 GUI 零输入探针、1161 个实际积分步，0 次导航小试、0 次正式矩阵。它们不验证 G12 导航精度或导航重复性。

## 冻结、运行器与门禁

`release_01.json` 绑定 v6 IPC、gateway、DDS XML、probe、trial、acceptance 及继承的冻结源码和运行文件。`trial.py` 使用原 v3 Mission/PID；离线 AST 比较确认 `process` 和 `publish` 控制计算不变。验收保留全部旧数值判据，额外拒绝原生崩溃。

运行器只接受 `PilotProof`。权威四组放行检查是 `pilot_evaluate(normal, repeat, abort)`，要求同一 release 的两份不同正常探针与一份主动异常清理证据，以及完整 54 条 Python 证据、9 组重复性和当前 Nix/端口检查。单个 `gate analyze/check` 仅用于诊断一个联合探针，不能提供运行器要求的 PilotProof。

最新正常重复探针在实际启动时必须仍处于 600 s 新鲜度窗口；旧报告不能永久授权启动。每次运行前 guard 重新核对源码、证据、运行环境。记录绑定会再次核验复制的三份探针与源码快照。

本轮全量回归 1187 项通过。`preservation_final.json` 核对 7384 个旧清单可读文件，0 修改、0 新读取错误。历史不可读资产不在该数量内；不能将此数量解释成所有历史文件均已校验。

最终机器判定见 `artifacts/twin_axial_g12_link_v4/gate_final.json`。只有其 passed/ready_to_run 以及三类联合证据全部为 true，才具备进入 S3/F0/P0、F1/P0、F2/P0、F2/P2 四组小试的工程条件。真实导航能否满足全部原阈值仍须小试确认，尤其 F2/P2 约 16 mm 的已知净距裕量。

## 收尾影响

本轮不增加引导候选、不重新调参。联合门禁通过后，下一步只做四组小试；四组通过才进入同一冻结修订的完整真实矩阵。有限边界评价仍按既定五个额外工况，复现与论文工作可开始整理已有证据。链路修复成功不能代替 C4 名义导航验收，也不能把 S3 改为边界案例。
