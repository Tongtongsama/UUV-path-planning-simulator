# 双推进器开环实测记录 — 2026-10-25

最终运行目录：`artifacts/twin_axial/validation_20261025_final`。
NixOS WSL、固定LOTUSim/xdyn依赖，真实外部求解器，无GUI、无PID、无MOVE。

| 工况 | 积分步数 | Gazebo观测 | 匹配覆盖率 | 结果 |
|---|---:|---:|---:|---|
| zero | 600 | 600 | 100% | PASS |
| surge_positive | 160 | 160 | 100% | PASS |
| surge_negative | 160 | 160 | 100% | PASS |
| pitch_positive | 160 | 160 | 100% | PASS |
| pitch_negative | 160 | 160 | 100% | PASS |
| saturation | 160 | 160 | 100% | PASS |
| watchdog | 160 | 160 | 100% | PASS |
| stale_stamp | 160 | 160 | 100% | PASS |
| surge_positive_repeat | 160 | 160 | 100% | PASS |

- 套件全部退出码0，重复前进脉冲匹配160步，位置及速度差为0。
- 两种失效注入（停止发包、冻结时间戳持续发包）均通过实际推力归零及截止时间检查。
- 所有运行的源码快照与交付Python文件逐字节一致；离线重分析全部通过。
- 122项动力学、控制器、旧开环与新开环相关测试通过；含时间跳变、错误GUI姿态和零输入异常运动的负向验收测试。
- 49个旧模型、算法和配置文件哈希保持不变；原冻结payload校验通过。没有修改旧模型或冻结结果。
- 仅验收当前首阶段。未运行GUI验证、恒流、PID、导航、正式总能量/步长收敛/跨求解器一致性实验，不将本套件误标为全部M1–M7通过。

运行方法见 [开环说明](lotusim_twin_axial_open_loop.md)。没有Git提交或上传。
